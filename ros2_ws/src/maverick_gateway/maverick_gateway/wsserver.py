"""Minimal RFC 6455 websocket server on asyncio (standard library only, text frames).

The sim image has no `websockets` package; the gateway needs only: handshake, text frames in and
out, ping/pong, close. `broadcast` is thread safe (called from ROS callbacks). A client whose
socket buffer grows past `max_buffer` bytes (it is not reading) is dropped, never waited for.
"""

from __future__ import annotations

import asyncio
import base64
import contextlib
import hashlib
import struct
import threading
from collections.abc import Callable

GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1(key.encode() + GUID).digest()).decode()


def encode_frame(payload: bytes, opcode: int = 0x1) -> bytes:
    """One unmasked, unfragmented server frame."""
    n = len(payload)
    if n < 126:
        head = struct.pack("!BB", 0x80 | opcode, n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x80 | opcode, 126, n)
    else:
        head = struct.pack("!BBQ", 0x80 | opcode, 127, n)
    return head + payload


async def read_frame(reader: asyncio.StreamReader, max_size: int) -> tuple[bool, int, bytes]:
    """(fin, opcode, unmasked payload) of the next frame."""
    b0, b1 = await reader.readexactly(2)
    n = b1 & 0x7F
    if n == 126:
        (n,) = struct.unpack("!H", await reader.readexactly(2))
    elif n == 127:
        (n,) = struct.unpack("!Q", await reader.readexactly(8))
    if n > max_size:
        raise ValueError("frame too large")
    mask = await reader.readexactly(4) if b1 & 0x80 else b""
    data = await reader.readexactly(n)
    if mask:
        data = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
    return bool(b0 & 0x80), b0 & 0x0F, data


class WebSocketServer:
    def __init__(
        self,
        host: str,
        port: int,
        on_connect: Callable[[], list[str]],
        on_message: Callable[[str], None],
        max_buffer: int = 8 << 20,
        max_message: int = 1 << 20,
    ) -> None:
        self.host, self.port = host, port
        self.on_connect, self.on_message = on_connect, on_message
        self.max_buffer, self.max_message = max_buffer, max_message
        self.clients: set[asyncio.StreamWriter] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._server: asyncio.base_events.Server | None = None

    # -- lifecycle ---------------------------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gateway-ws", daemon=True)
        self._thread.start()
        if not self._ready.wait(10.0):
            raise RuntimeError("websocket server did not start")

    def stop(self) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(2.0)

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._server = self._loop.run_until_complete(
            asyncio.start_server(self._handle, self.host, self.port)
        )
        self.port = self._server.sockets[0].getsockname()[1]
        self._ready.set()
        self._loop.run_forever()

    # -- sending -----------------------------------------------------------------------------
    def broadcast(self, text: str) -> None:
        """Queue one text frame for every client (any thread)."""
        if self._loop is not None and self.clients:
            self._loop.call_soon_threadsafe(self._send_all, encode_frame(text.encode()))

    def _send_all(self, frame: bytes) -> None:
        for w in list(self.clients):
            self._send(w, frame)

    def _send(self, w: asyncio.StreamWriter, frame: bytes) -> None:
        if w.transport.get_write_buffer_size() > self.max_buffer:
            self.clients.discard(w)
            w.close()
            return
        try:
            w.write(frame)
        except (ConnectionError, RuntimeError):
            self.clients.discard(w)

    # -- per connection ----------------------------------------------------------------------
    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            head = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), 5.0)
            headers = {}
            for line in head.decode("latin-1").split("\r\n")[1:]:
                if ":" in line:
                    k, v = line.split(":", 1)
                    headers[k.strip().lower()] = v.strip()
            key = headers.get("sec-websocket-key")
            if headers.get("upgrade", "").lower() != "websocket" or not key:
                body = b"maverick gateway (websocket)\n"
                writer.write(
                    b"HTTP/1.1 426 Upgrade Required\r\nContent-Length: %d\r\n\r\n" % len(body)
                    + body
                )
                await writer.drain()
                return
            writer.write(
                (
                    "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                    f"Connection: Upgrade\r\nSec-WebSocket-Accept: {accept_key(key)}\r\n\r\n"
                ).encode()
            )
            # state on connect, then join the broadcast set (no await in between: no gap)
            for frame in self.on_connect():
                writer.write(encode_frame(frame.encode()))
            self.clients.add(writer)
            await writer.drain()
            await self._read_loop(reader, writer)
        except (TimeoutError, asyncio.IncompleteReadError, ConnectionError, ValueError):
            pass
        finally:
            self.clients.discard(writer)
            with contextlib.suppress(Exception):
                writer.close()

    async def _read_loop(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        buf = b""
        while True:
            fin, op, data = await read_frame(reader, self.max_message)
            if op == 0x8:  # close
                writer.write(encode_frame(data[:2], 0x8))
                return
            if op == 0x9:  # ping
                writer.write(encode_frame(data, 0xA))
            elif op in (0x1, 0x0):  # text / continuation
                buf += data
                if len(buf) > self.max_message:
                    return
                if fin:
                    text, buf = buf.decode("utf-8", "replace"), b""
                    try:
                        self.on_message(text)
                    except Exception:  # a bad handler must not drop the console
                        pass
