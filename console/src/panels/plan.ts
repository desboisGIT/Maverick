import type { GatewayClient } from "../gateway/client";
import type { AgentStepData, SkillProgressData } from "../gateway/types";

const ICON = { done: "✓", running: "▶", pending: "○", failed: "✕", skipped: "–" } as const;

export class PlanPanel {
  private plan: AgentStepData | null = null;
  private skill: SkillProgressData | null = null;
  private rcvAt = 0;
  private el: HTMLElement;

  constructor(root: HTMLElement, gw: GatewayClient) {
    this.el = root;
    gw.on("agent_step", (d) => {
      this.plan = d.steps.length ? d : null;
      this.rcvAt = performance.now();
      this.render();
    });
    gw.on("skill_progress", (d) => {
      this.skill = d;
      this.render();
    });
    // running timers tick locally between gateway updates
    setInterval(() => this.tickTimers(), 500);
    this.render();
  }

  private fmt(s: number): string {
    return s < 60 ? `${Math.floor(s)} s` : `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
  }

  private tickTimers(): void {
    if (!this.plan) return;
    this.el.querySelectorAll<HTMLElement>(".el[data-base]").forEach((n) => {
      n.textContent = this.fmt(Number(n.dataset.base) + (performance.now() - this.rcvAt) / 1000);
    });
  }

  private render(): void {
    const p = this.plan;
    let html = "";
    if (!p) html += `<div class="empty">No active plan.<br>Give Maverick a command.</div>`;
    else {
      html += `<div class="goal"><small>Goal</small>${esc(p.goal)}</div><ul class="steps">`;
      for (const s of p.steps) {
        const running = s.status === "running";
        const el =
          s.elapsed_s === undefined ? "" : running ? `<span class="el" data-base="${s.elapsed_s}">${this.fmt(s.elapsed_s)}</span>` : `<span class="el">${this.fmt(s.elapsed_s)}</span>`;
        html += `<li class="step ${s.status}"><span class="ic">${ICON[s.status]}</span><span>${esc(s.label)}${s.detail ? `<span class="d">${esc(s.detail)}</span>` : ""}</span>${el}</li>`;
      }
      html += `</ul>`;
    }
    const k = this.skill;
    if (k && k.state === "running") {
      const pct = k.progress >= 0 ? Math.round(k.progress * 100) : 100;
      html += `<div class="skill"><div class="row"><b>${esc(k.skill)}</b><span>${k.progress >= 0 ? pct + "%" : ""}${k.eta_s !== undefined ? ` &middot; ~${Math.round(k.eta_s)} s` : ""}</span></div>
        <div class="bar"><div style="width:${pct}%"></div></div><div class="msg2">${esc(k.message)}</div></div>`;
    }
    this.el.innerHTML = html;
  }
}

function esc(s: string): string {
  return s.replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]!);
}
