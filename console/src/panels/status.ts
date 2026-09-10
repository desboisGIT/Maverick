import type { GatewayClient } from "../gateway/client";
import type { DirectorAction, StatusData } from "../gateway/types";

const DIRECTOR: { label: string; action: DirectorAction; params?: Record<string, unknown> }[] = [
  { label: "Remove the bananas", action: "remove_object", params: { label: "banana" } },
  { label: "Cut Wi-Fi for 5 s", action: "cut_wifi", params: { seconds: 5 } },
  { label: "Drain battery (-40%)", action: "drain_battery", params: { pct: 40 } },
  { label: "Add person: Adrien", action: "add_person", params: { name: "Adrien" } },
  { label: "Restart demo (mock only)", action: "reset_demo" },
];

export class StatusPanel {
  private el: HTMLElement;
  private s: StatusData | null = null;

  constructor(root: HTMLElement, gw: GatewayClient) {
    this.el = root;
    root.innerHTML = `
      <div class="kv" id="kv"></div>
      <div class="director" id="dir">
        <div class="menu">${DIRECTOR.map((d, i) => (d.action === "reset_demo" ? `<div class="sep"></div>` : "") + `<button data-i="${i}">${d.label}</button>`).join("")}</div>
        <button class="btn" id="dirbtn">Director &#9662;</button>
      </div>`;
    const dir = root.querySelector<HTMLElement>("#dir")!;
    root.querySelector("#dirbtn")!.addEventListener("click", (e) => {
      e.stopPropagation();
      dir.classList.toggle("open");
    });
    document.addEventListener("click", () => dir.classList.remove("open"));
    root.querySelectorAll<HTMLButtonElement>(".menu button").forEach((b) =>
      b.addEventListener("click", () => {
        const d = DIRECTOR[Number(b.dataset.i)];
        gw.send("director", { action: d.action, params: d.params });
      }),
    );
    gw.on("status", (d) => {
      this.s = d;
      this.render();
    });
    this.render();
  }

  private render(): void {
    const s = this.s;
    const kv = this.el.querySelector("#kv")!;
    if (!s) {
      kv.innerHTML = `<div class="empty" style="grid-column:1/-1">waiting for status...</div>`;
      return;
    }
    const batCol = s.battery_pct < 20 ? "var(--bad)" : s.battery_pct < 40 ? "var(--warn)" : "var(--good)";
    const linkCls = s.link.quality === "good" ? "good" : s.link.quality === "degraded" ? "warn" : "bad";
    const vram = s.vram_used_gb / s.vram_total_gb;
    const vramCol = vram > 0.9 ? "var(--bad)" : vram > 0.75 ? "var(--warn)" : "var(--accent)";
    kv.innerHTML = `
      <span class="k">Battery</span><span class="v">${Math.round(s.battery_pct)}%${s.battery_v ? ` <span class="k">${s.battery_v.toFixed(1)} V</span>` : ""}</span>
      <div class="meter"><div style="width:${s.battery_pct}%;background:${batCol}"></div></div>
      <span class="k">Link</span><span class="v"><i class="dot ${linkCls}"></i>${s.link.quality}${s.link.rtt_ms !== undefined && s.link.quality !== "lost" ? ` <span class="k">${Math.round(s.link.rtt_ms)} ms</span>` : ""}</span>
      <span class="k">Mode</span><span class="v">${esc(s.mode)}${s.armed ? "" : ` <span class="k">disarmed</span>`}</span>
      <span class="k">VRAM</span><span class="v">${s.vram_used_gb.toFixed(1)} / ${s.vram_total_gb.toFixed(0)} GB</span>
      <div class="meter"><div style="width:${vram * 100}%;background:${vramCol}"></div></div>`;
  }
}

function esc(s: string): string {
  return s.replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]!);
}
