// Voxel world derived from the apartment manifest (ground truth for the mock only).
import { readFileSync } from "node:fs";

export const RES = 0.1;
export const NONE = 0, WALL = 1, FLOOR = 2, FURN = 3;
export const CEILING = 2.55;

export interface ManifestRoom { id: string; type: string; polygon: [number, number][]; area_m2: number }
export interface ManifestDoor { id: string; rooms: [string, string]; kind: string; center: [number, number]; width: number }
export interface ManifestFurniture { id: string; class: string; room: string; bbox: [[number, number, number], [number, number, number]] }
interface Manifest { footprint: [number, number][]; rooms: ManifestRoom[]; doors: ManifestDoor[]; furniture: ManifestFurniture[]; spawn: { pose: number[] } }

export interface Rect { x0: number; y0: number; x1: number; y1: number }

export const FURN_COLORS: Record<string, string> = {
  sofa: "#5b7a99", bed: "#8a6fb0", wardrobe: "#8b6b4a", shelves: "#7a5c3e", plant: "#4f8a4a",
  toilet: "#dfe7ee", sink: "#dfe7ee", shower: "#c5d6e0", tv_unit: "#3a3f4a", coffee_table: "#9c7a55",
  kitchen_counter: "#b0b6bd", oven: "#666c75", fridge: "#cfd5db",
};

export class World {
  m: Manifest;
  nx: number; ny: number; nz: number;
  kind: Uint8Array;
  top: Float32Array; // height of the tallest non-floor voxel per column
  colorOf: string[]; // per column colour for furniture ("" = wall)
  rects = new Map<string, Rect>();

  constructor() {
    this.m = JSON.parse(readFileSync(new URL("./apt_7.json", import.meta.url), "utf8")) as Manifest;
    const xs = this.m.footprint.map((p) => p[0]), ys = this.m.footprint.map((p) => p[1]);
    this.nx = Math.round(Math.max(...xs) / RES);
    this.ny = Math.round(Math.max(...ys) / RES);
    this.nz = Math.ceil(CEILING / RES);
    this.kind = new Uint8Array(this.nx * this.ny * this.nz);
    this.top = new Float32Array(this.nx * this.ny);
    this.colorOf = new Array(this.nx * this.ny).fill("");
    for (let ix = 0; ix < this.nx; ix++) for (let iy = 0; iy < this.ny; iy++) this.kind[this.idx(ix, iy, 0)] = FLOOR;

    for (const r of this.m.rooms) {
      const px = r.polygon.map((p) => p[0]), py = r.polygon.map((p) => p[1]);
      this.rects.set(r.id, { x0: Math.min(...px), y0: Math.min(...py), x1: Math.max(...px), y1: Math.max(...py) });
    }
    // walls along every room edge
    for (const rc of this.rects.values()) {
      for (const x of [rc.x0, rc.x1]) for (let y = rc.y0; y < rc.y1 - 1e-9; y += RES / 2) this.wall(x, y);
      for (const y of [rc.y0, rc.y1]) for (let x = rc.x0; x < rc.x1 - 1e-9; x += RES / 2) this.wall(x, y);
    }
    // carve doors
    for (const d of this.m.doors) {
      const a = this.rects.get(d.rooms[0])!, b = this.rects.get(d.rooms[1])!;
      const h = d.kind === "open" ? CEILING : 2.0;
      const [cx, cy] = d.center, half = d.width / 2;
      const near = (u: number, v: number) => Math.abs(u - v) < 1e-6;
      const vertical = near(a.x1, b.x0) || near(b.x1, a.x0);
      for (let s = -half; s <= half; s += RES / 2) {
        const x = vertical ? cx : cx + s, y = vertical ? cy + s : cy;
        const ix = this.cx(x), iy = this.cy(y);
        for (let iz = 0; iz * RES < h; iz++) this.kind[this.idx(ix, iy, iz)] = iz === 0 ? FLOOR : NONE;
        this.top[ix * this.ny + iy] = 0;
      }
    }
    // furniture
    for (const f of this.m.furniture) {
      const [lo, hi] = f.bbox;
      for (let ix = this.cx(lo[0]); ix <= this.cx(hi[0] - 1e-6); ix++)
        for (let iy = this.cy(lo[1]); iy <= this.cy(hi[1] - 1e-6); iy++) {
          for (let iz = 0; iz * RES < hi[2]; iz++) this.kind[this.idx(ix, iy, iz)] = FURN;
          this.top[ix * this.ny + iy] = Math.max(this.top[ix * this.ny + iy], hi[2]);
          this.colorOf[ix * this.ny + iy] = FURN_COLORS[f.class] ?? "#777f8c";
        }
    }
  }

  idx(ix: number, iy: number, iz: number): number { return (ix * this.ny + iy) * this.nz + iz; }
  cx(x: number): number { return Math.min(this.nx - 1, Math.max(0, Math.floor(x / RES + 1e-6))); }
  cy(y: number): number { return Math.min(this.ny - 1, Math.max(0, Math.floor(y / RES + 1e-6))); }

  private wall(x: number, y: number): void {
    const ix = this.cx(x), iy = this.cy(y);
    for (let iz = 0; iz < this.nz; iz++) this.kind[this.idx(ix, iy, iz)] = WALL;
    this.top[ix * this.ny + iy] = CEILING;
    this.colorOf[ix * this.ny + iy] = "";
  }

  kindAt(x: number, y: number, z: number): number {
    if (x < 0 || y < 0 || z < 0 || x >= this.nx * RES || y >= this.ny * RES || z >= this.nz * RES) return -1;
    return this.kind[this.idx(Math.floor(x / RES), Math.floor(y / RES), Math.floor(z / RES))];
  }

  roomAt(x: number, y: number): string | null {
    for (const [id, r] of this.rects) if (x >= r.x0 && x < r.x1 && y >= r.y0 && y < r.y1) return id;
    return null;
  }
}
