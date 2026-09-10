import type { GatewayClient } from "../gateway/client";
import type { CameraFrameData, DetectionsData } from "../gateway/types";

const PALETTE = ["#4cc9f0", "#f4a261", "#b794f6", "#5eead4", "#fbbf24", "#f472b6", "#a3e635", "#fb7185"];
function colorFor(label: string): string {
  let h = 0;
  for (let i = 0; i < label.length; i++) h = (h * 31 + label.charCodeAt(i)) >>> 0;
  return PALETTE[h % PALETTE.length];
}

/** Drone camera: image plus detection boxes and labels drawn on one canvas. */
export class CameraPanel {
  private canvas = document.createElement("canvas");
  private ctx = this.canvas.getContext("2d")!;
  private hudRes = document.createElement("span");
  private hudFps = document.createElement("span");
  private nosignal = document.createElement("div");
  private img: HTMLImageElement | null = null;
  private frame: CameraFrameData | null = null;
  private det: DetectionsData | null = null;
  private lastFrameAt = 0;
  private fpsCount = 0;
  private fpsT = performance.now();
  private decoding = false;

  constructor(root: HTMLElement, tag: HTMLElement, gw: GatewayClient) {
    root.appendChild(this.canvas);
    const hud = document.createElement("div");
    hud.className = "hud";
    const live = document.createElement("span");
    live.className = "live";
    live.textContent = "LIVE";
    hud.append(live, this.hudRes, this.hudFps);
    this.nosignal.className = "nosignal";
    this.nosignal.textContent = "no signal";
    root.append(hud, this.nosignal);
    new ResizeObserver(() => this.draw()).observe(root);

    gw.on("camera_frame", (d) => this.onFrame(d));
    gw.on("detections", (d) => {
      this.det = d;
      tag.textContent = d.boxes.length ? `${d.boxes.length} detected` : "";
      this.draw();
    });
    setInterval(() => {
      const now = performance.now();
      this.hudFps.textContent = `${Math.round((this.fpsCount * 1000) / (now - this.fpsT))} fps`;
      this.fpsCount = 0;
      this.fpsT = now;
      this.nosignal.style.display = now - this.lastFrameAt > 2000 ? "flex" : "none";
      live.style.visibility = now - this.lastFrameAt > 2000 ? "hidden" : "visible";
    }, 1000);
  }

  private onFrame(d: CameraFrameData): void {
    this.frame = d;
    this.lastFrameAt = performance.now();
    this.fpsCount++;
    this.hudRes.textContent = `${d.width}x${d.height}`;
    if (this.decoding) return; // drop frames if the decoder is behind
    this.decoding = true;
    const img = new Image();
    img.onload = () => {
      this.img = img;
      this.decoding = false;
      this.draw();
    };
    img.onerror = () => (this.decoding = false);
    img.src = `data:${d.mime};base64,${d.data}`;
  }

  private draw(): void {
    const c = this.canvas;
    const r = c.parentElement!.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const W = Math.max(1, Math.round(r.width * dpr));
    const H = Math.max(1, Math.round(r.height * dpr));
    if (c.width !== W || c.height !== H) {
      c.width = W;
      c.height = H;
    }
    const ctx = this.ctx;
    ctx.fillStyle = "#04060a";
    ctx.fillRect(0, 0, W, H);
    if (!this.img || !this.frame) return;
    const iw = this.frame.width, ih = this.frame.height;
    const s = Math.min(W / iw, H / ih);
    const w = iw * s, h = ih * s, x0 = (W - w) / 2, y0 = (H - h) / 2;
    ctx.imageSmoothingQuality = "high";
    ctx.drawImage(this.img, x0, y0, w, h);
    if (!this.det || this.det.frame_seq < this.frame.seq - 3) return;
    ctx.lineWidth = 2 * dpr;
    ctx.font = `600 ${Math.round(11 * dpr)}px system-ui, sans-serif`;
    ctx.textBaseline = "middle";
    for (const b of this.det.boxes) {
      const col = colorFor(b.label);
      const [bx, by, bw, bh] = b.box;
      const x = x0 + bx * w, y = y0 + by * h, ww = bw * w, hh = bh * h;
      ctx.strokeStyle = col;
      ctx.fillStyle = col + "22";
      ctx.beginPath();
      ctx.roundRect(x, y, ww, hh, 4 * dpr);
      ctx.fill();
      ctx.stroke();
      const text = `${b.label} ${b.confidence.toFixed(2)}`;
      const tw = ctx.measureText(text).width + 12 * dpr;
      const th = 17 * dpr;
      const ly = y - th < y0 ? y : y - th;
      ctx.fillStyle = col;
      ctx.beginPath();
      ctx.roundRect(x, ly, tw, th, 4 * dpr);
      ctx.fill();
      ctx.fillStyle = "#06121a";
      ctx.fillText(text, x + 6 * dpr, ly + th / 2 + 0.5);
    }
  }
}
