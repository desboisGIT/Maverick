import type { GatewayClient } from "../gateway/client";
import type { TranscriptData } from "../gateway/types";

export class ConversationPanel {
  private chat = document.createElement("div");
  private nodes = new Map<string, HTMLElement>();
  private input!: HTMLInputElement;
  private sendBtn!: HTMLButtonElement;
  private ptt!: HTMLButtonElement;
  private pttOn = false;

  constructor(root: HTMLElement, private gw: GatewayClient) {
    root.innerHTML = `
      <div class="chatwrap"></div>
      <form class="composer">
        <input type="text" placeholder='Type a command, e.g. "check the kitchen for bananas"' autocomplete="off" spellcheck="false" />
        <button type="button" class="ptt" title="Hold to talk (voice is a stub in v1)"><span class="bars"><i></i><i></i><i></i><i></i></span><span class="lbl-t">Hold to talk</span><kbd>Space</kbd></button>
        <button type="submit" class="send">Send</button>
      </form>`;
    root.querySelector(".chatwrap")!.appendChild(this.chat);
    this.chat.className = "chat";
    this.input = root.querySelector("input")!;
    this.sendBtn = root.querySelector(".send")!;
    this.ptt = root.querySelector(".ptt")!;
    root.querySelector("form")!.addEventListener("submit", (e) => {
      e.preventDefault();
      const text = this.input.value.trim();
      if (!text) return;
      if (gw.send("text_command", { text })) this.input.value = "";
      else this.system("Gateway offline: command not sent.");
    });
    gw.onState((s) => (this.sendBtn.disabled = s !== "open"));

    this.ptt.addEventListener("pointerdown", (e) => {
      e.preventDefault();
      this.ptt.setPointerCapture(e.pointerId);
      this.pttStart();
    });
    this.ptt.addEventListener("pointerup", () => this.pttStop());
    this.ptt.addEventListener("pointercancel", () => this.pttStop());
    window.addEventListener("keydown", (e) => {
      if (e.code !== "Space" || e.repeat || document.activeElement === this.input) return;
      e.preventDefault();
      this.pttStart();
    });
    window.addEventListener("keyup", (e) => {
      if (e.code === "Space") this.pttStop();
    });
    gw.on("transcript", (d) => this.upsert(d));
  }

  private pttStart(): void {
    if (this.pttOn) return;
    this.pttOn = true;
    this.ptt.classList.add("active");
    this.ptt.querySelector(".lbl-t")!.textContent = "Listening...";
    this.gw.send("ptt_start", {});
  }
  private pttStop(): void {
    if (!this.pttOn) return;
    this.pttOn = false;
    this.ptt.classList.remove("active");
    this.ptt.querySelector(".lbl-t")!.textContent = "Hold to talk";
    this.gw.send("ptt_stop", {});
  }

  private system(text: string): void {
    this.upsert({ id: "local-" + Date.now(), role: "system", text });
  }

  private upsert(d: TranscriptData): void {
    let el = this.nodes.get(d.id);
    if (!el) {
      el = document.createElement("div");
      this.nodes.set(d.id, el);
      this.chat.appendChild(el);
    }
    el.className = `msg ${d.role}${d.partial ? " partial" : ""}`;
    el.textContent = "";
    if (d.role !== "system") {
      const who = document.createElement("span");
      who.className = "who";
      who.textContent = d.role === "user" ? (d.source === "voice" ? "You (voice)" : "You") : "Maverick";
      el.appendChild(who);
    }
    el.appendChild(document.createTextNode(d.text));
    for (const a of d.attachments ?? []) {
      const card = document.createElement("div");
      card.className = "photo";
      const img = document.createElement("img");
      img.src = `data:${a.mime};base64,${a.data}`;
      img.alt = a.caption ?? "photo";
      card.appendChild(img);
      if (a.caption) {
        const cap = document.createElement("div");
        cap.className = "cap";
        cap.textContent = a.caption;
        card.appendChild(cap);
      }
      el.appendChild(card);
    }
    this.chat.scrollTop = this.chat.scrollHeight;
    if ((d.attachments ?? []).length) setTimeout(() => (this.chat.scrollTop = this.chat.scrollHeight), 60);
  }
}
