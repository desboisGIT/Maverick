// Fake first-person camera: a column raycaster over the mock world, emitted as SVG.
import { CEILING, RES, World } from "./world.ts";

export const W = 640, H = 480, HFOV = Math.PI / 2; // 90 degrees
const F = W / 2 / Math.tan(HFOV / 2);
export const CAM_Z = 1.2;
const COLS = 96;

export interface Prop { x: number; y: number; z: number; w: number; h: number; color: string }
export interface DrawBox { label: string; confidence: number; box: [number, number, number, number] }

function shade(hex: string, k: number): string {
  const n = parseInt(hex.slice(1), 16);
  const c = (v: number) => Math.max(0, Math.min(255, Math.round(v * k))).toString(16).padStart(2, "0");
  return `#${c(n >> 16)}${c((n >> 8) & 255)}${c(n & 255)}`;
}

interface Hit { d: number; top: number; color: string; faceX: boolean }

export function renderView(w: World, x: number, y: number, yaw: number, props: Prop[], boxes: DrawBox[] = []): string {
  const horizon = H / 2;
  const fx = Math.cos(yaw), fy = Math.sin(yaw), rx = Math.sin(yaw), ry = -Math.cos(yaw); // right-hand side of the view (ENU, yaw CCW from +x)
  const out: string[] = [];
  out.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}" shape-rendering="crispEdges">`);
  out.push(`<defs><linearGradient id="c" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#1b212d"/><stop offset="1" stop-color="#3a4252"/></linearGradient>`);
  out.push(`<linearGradient id="f" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#4a3f35"/><stop offset="1" stop-color="#7a6a58"/></linearGradient>`);
  out.push(`<radialGradient id="v" cx="50%" cy="50%" r="75%"><stop offset="0.55" stop-color="#000" stop-opacity="0"/><stop offset="1" stop-color="#000" stop-opacity="0.55"/></radialGradient></defs>`);
  out.push(`<rect width="${W}" height="${horizon}" fill="url(#c)"/><rect y="${horizon}" width="${W}" height="${H - horizon}" fill="url(#f)"/>`);
  const cw = W / COLS;
  const wallDist: number[] = [];
  for (let c = 0; c < COLS; c++) {
    const a = Math.atan(((c + 0.5) / COLS * 2 - 1) * Math.tan(HFOV / 2));
    const dx = fx * Math.cos(a) + rx * Math.sin(a), dy = fy * Math.cos(a) + ry * Math.sin(a);
    const hits: Hit[] = [];
    let prevTop = 0, prevI = -1, prevJ = -1, wallD = 12;
    for (let d = 0.1; d < 12; d += 0.04) {
      const px = x + dx * d, py = y + dy * d;
      if (px < 0 || py < 0 || px >= w.nx * RES || py >= w.ny * RES) break;
      const i = Math.floor(px / RES), j = Math.floor(py / RES);
      const t = w.top[i * w.ny + j];
      if (t > 0.01 && (prevTop <= 0.01 || Math.abs(t - prevTop) > 0.05)) {
        const col = w.colorOf[i * w.ny + j];
        let faceX = prevI !== i && prevJ === j ? true : prevJ !== j && prevI === i ? false : true;
        if (!col) {
          // walls are one voxel thick: the face normal is across the wall's run direction
          const tall = (a2: number, b2: number) => w.top[a2 * w.ny + b2] >= CEILING - 0.01;
          const runY = (j > 0 && tall(i, j - 1)) || (j + 1 < w.ny && tall(i, j + 1));
          const runX = (i > 0 && tall(i - 1, j)) || (i + 1 < w.nx && tall(i + 1, j));
          if (runY !== runX) faceX = runY;
        }
        hits.push({ d: d * Math.cos(a), top: t, color: col || "#b9bdc8", faceX });
      }
      prevTop = t; prevI = i; prevJ = j;
      if (t >= CEILING - 0.01) { wallD = d * Math.cos(a); break; }
    }
    wallDist.push(wallD);
    hits.sort((p, q) => q.d - p.d);
    const x0 = (c * cw).toFixed(1);
    for (const h of hits) {
      const dd = Math.max(0.15, h.d);
      const yt = horizon - ((h.top - CAM_Z) / dd) * F, yb = horizon + (CAM_Z / dd) * F;
      const k = Math.max(0.3, 1 - dd / 13) * (h.faceX ? 1 : 0.84);
      out.push(`<rect x="${x0}" y="${yt.toFixed(1)}" width="${(cw + 0.6).toFixed(1)}" height="${Math.max(0, yb - yt).toFixed(1)}" fill="${shade(h.color, k)}"/>`);
      if (h.top < CEILING - 0.01) out.push(`<rect x="${x0}" y="${yt.toFixed(1)}" width="${(cw + 0.6).toFixed(1)}" height="3" fill="${shade(h.color, k * 1.35)}"/>`);
    }
  }
  // props (small items), occluded only by walls
  for (const p of props) {
    const rx2 = p.x - x, ry2 = p.y - y;
    const fwd = rx2 * fx + ry2 * fy, side = rx2 * rx + ry2 * ry;
    if (fwd < 0.3) continue;
    const sx = W / 2 + (F * side) / fwd, sy = horizon - (F * (p.z - CAM_Z)) / fwd;
    const col = Math.min(COLS - 1, Math.max(0, Math.floor(sx / cw)));
    if (sx < 0 || sx > W || fwd > wallDist[col]) continue;
    const pw = (F * p.w) / fwd, ph = (F * p.h) / fwd;
    out.push(`<ellipse cx="${sx.toFixed(1)}" cy="${sy.toFixed(1)}" rx="${(pw / 2).toFixed(1)}" ry="${(ph / 2).toFixed(1)}" fill="${p.color}" shape-rendering="auto"/>`);
  }
  out.push(`<rect width="${W}" height="${H}" fill="url(#v)"/>`);
  for (const b of boxes) {
    const [bx, by, bw, bh] = b.box;
    out.push(`<rect x="${bx * W}" y="${by * H}" width="${bw * W}" height="${bh * H}" fill="none" stroke="#4cc9f0" stroke-width="3" shape-rendering="auto"/>`);
    out.push(`<text x="${bx * W + 4}" y="${Math.max(14, by * H - 6)}" font-family="sans-serif" font-size="15" font-weight="700" fill="#4cc9f0">${b.label} ${b.confidence.toFixed(2)}</text>`);
  }
  out.push(`</svg>`);
  return out.join("");
}

/** Project a world-space axis-aligned box into normalised image coordinates; null if not visible. */
export function projectBox(x: number, y: number, yaw: number, lo: number[], hi: number[]): { box: [number, number, number, number]; fwd: number } | null {
  const fx = Math.cos(yaw), fy = Math.sin(yaw), rx = Math.sin(yaw), ry = -Math.cos(yaw);
  let minU = Infinity, maxU = -Infinity, minV = Infinity, maxV = -Infinity, minF = Infinity;
  for (const cx of [lo[0], hi[0]]) for (const cy of [lo[1], hi[1]]) for (const cz of [lo[2], hi[2]]) {
    const dx = cx - x, dy = cy - y;
    const fwd = Math.max(0.25, dx * fx + dy * fy), side = dx * rx + dy * ry;
    const u = W / 2 + (F * side) / fwd, v = H / 2 - (F * (cz - CAM_Z)) / fwd;
    minU = Math.min(minU, u); maxU = Math.max(maxU, u); minV = Math.min(minV, v); maxV = Math.max(maxV, v);
    minF = Math.min(minF, dx * fx + dy * fy);
  }
  const cxm = (lo[0] + hi[0]) / 2 - x, cym = (lo[1] + hi[1]) / 2 - y;
  const cf = cxm * fx + cym * fy;
  if (cf < 0.4) return null;
  const u0 = Math.max(0, minU), u1 = Math.min(W, maxU), v0 = Math.max(0, minV), v1 = Math.min(H, maxV);
  if (u1 - u0 < 6 || v1 - v0 < 6) return null;
  if ((u1 - u0) * (v1 - v0) < 0.35 * (maxU - minU) * (maxV - minV)) return null; // mostly outside the image
  return { box: [u0 / W, v0 / H, (u1 - u0) / W, (v1 - v0) / H], fwd: cf };
}
