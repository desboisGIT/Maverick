import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CSS2DObject, CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import type { GatewayClient } from "../gateway/client";
import type { Frontier, MapObject, MapUpdateData, PathData, PoseData, Room, RoomType, Vec3 } from "../gateway/types";

/** Map frame (ENU: x east, y north, z up) to three.js (x east, y up, z south). */
const M2T = (x: number, y: number, z: number) => new THREE.Vector3(x, z, -y);

const ROOM_COLORS: Record<RoomType, string> = {
  unknown: "#64748b",
  living: "#4cc9f0",
  kitchen: "#f4a261",
  bedroom: "#b794f6",
  bathroom: "#5eead4",
  hallway: "#94a3b8",
  office: "#a3e635",
  other: "#f472b6",
};

const HEIGHT_STOPS: [number, string][] = [
  [0.0, "#26354f"],
  [0.4, "#1fa9a0"],
  [1.0, "#5ec8f2"],
  [1.7, "#8f9bff"],
  [2.5, "#e0a3ff"],
];
const heightColor = (z: number, out: THREE.Color): THREE.Color => {
  for (let i = 1; i < HEIGHT_STOPS.length; i++) {
    if (z <= HEIGHT_STOPS[i][0] || i === HEIGHT_STOPS.length - 1) {
      const [z0, c0] = HEIGHT_STOPS[i - 1];
      const [z1, c1] = HEIGHT_STOPS[i];
      const k = Math.min(1, Math.max(0, (z - z0) / (z1 - z0)));
      return out.set(c0).lerp(new THREE.Color(c1), k);
    }
  }
  return out;
};

const key = (ix: number, iy: number, iz: number) => ((ix + 1024) * 2048 + (iy + 1024)) * 2048 + (iz + 1024);

function pointInPoly(x: number, y: number, poly: [number, number][]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

export class MindView {
  private renderer: THREE.WebGLRenderer;
  private labels = new CSS2DRenderer();
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
  private controls: OrbitControls;
  private host: HTMLElement;

  // voxels
  private res = 0.1;
  private origin: Vec3 = [0, 0, 0];
  private index = new Map<number, number>(); // key -> slot
  private vox: number[] = []; // flat ix,iy,iz per slot
  private mesh: THREE.InstancedMesh | null = null;
  private capacity = 0;
  private voxDirty = true;
  private colorMode: "height" | "room" = "height";
  private tmpM = new THREE.Matrix4();
  private tmpC = new THREE.Color();

  // layers
  private roomGroup = new THREE.Group();
  private objGroup = new THREE.Group();
  private frontGroup = new THREE.Group();
  private pathGroup = new THREE.Group();
  private rooms: Room[] = [];
  private roomPolys: { id: string; type: RoomType; poly: [number, number][] }[] = [];
  private objects: MapObject[] = [];
  private frontiers: Frontier[] = [];

  // drone
  private drone = new THREE.Group();
  private droneTarget = new THREE.Vector3(0, 0.5, 0);
  private droneYaw = 0;
  private droneYawTarget = 0;
  private haveDrone = false;
  private trail: THREE.Vector3[] = [];
  private trailLine: THREE.Line;
  private follow = false;
  private rotors: THREE.Object3D[] = [];

  // ui
  private stats!: HTMLElement;
  private showRooms = true;
  private showLabels = true;
  private showPath = true;
  private userMoved = false;

  constructor(root: HTMLElement, gw: GatewayClient) {
    this.host = root;
    root.innerHTML = `
      <div id="mind-3d"></div><div id="mind-labels"></div>
      <div class="stats"></div>
      <div class="toolbar">
        <span class="seg"><button class="btn on" data-cm="height">Height</button><button class="btn" data-cm="room">Room</button></span>
        <button class="btn on" data-t="rooms">Rooms</button>
        <button class="btn on" data-t="labels">Labels</button>
        <button class="btn on" data-t="path">Path</button>
        <button class="btn" data-t="follow">Follow drone</button>
        <button class="btn" data-t="reset">Reset view</button>
      </div>
      <div class="legend">
        <span><i style="background:#5ec8f2"></i>occupied voxel</span>
        <span><i style="background:#fbbf24"></i>frontier</span>
        <span><i style="background:#ffd166"></i>planned path</span>
        <span><i style="background:#f4a261;border-radius:50%"></i>object</span>
      </div>
      <div class="hint">drag: orbit &middot; right-drag: pan &middot; wheel: zoom</div>`;
    this.stats = root.querySelector(".stats")!;
    const gl = root.querySelector<HTMLElement>("#mind-3d")!;
    const lab = root.querySelector<HTMLElement>("#mind-labels")!;

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x080b11);
    gl.appendChild(this.renderer.domElement);
    this.labels.domElement.style.position = "absolute";
    this.labels.domElement.style.inset = "0";
    this.labels.domElement.style.pointerEvents = "none";
    lab.appendChild(this.labels.domElement);

    this.scene.fog = new THREE.Fog(0x080b11, 22, 60);
    this.scene.add(new THREE.HemisphereLight(0xcfe3ff, 0x151a26, 1.15));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(-4, 10, 6);
    this.scene.add(sun);
    const grid = new THREE.GridHelper(40, 80, 0x1c2636, 0x121a27);
    (grid.material as THREE.Material).transparent = true;
    (grid.material as THREE.Material).opacity = 0.7;
    grid.position.y = -0.002;
    this.scene.add(grid);
    this.scene.add(this.roomGroup, this.objGroup, this.frontGroup, this.pathGroup);

    this.buildDrone();
    this.scene.add(this.drone);
    const tg = new THREE.BufferGeometry();
    tg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(600 * 3), 3));
    tg.setDrawRange(0, 0);
    this.trailLine = new THREE.Line(tg, new THREE.LineBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.45 }));
    this.trailLine.frustumCulled = false;
    this.scene.add(this.trailLine);

    this.camera.position.set(0, 9, 8);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.minDistance = 1;
    this.controls.maxDistance = 40;
    this.controls.addEventListener("start", () => (this.userMoved = true));

    this.wireToolbar(root);
    new ResizeObserver(() => this.resize()).observe(gl);
    this.resize();

    gw.on("pose", (d) => this.onPose(d));
    gw.on("map_update", (d) => this.onMap(d));
    gw.on("rooms", (d) => {
      this.rooms = d.rooms;
      this.buildRooms();
      this.voxDirty = true;
      this.updateStats();
    });
    gw.on("objects", (d) => {
      this.objects = d.objects;
      this.buildObjects();
      this.updateStats();
    });
    gw.on("frontiers", (d) => {
      this.frontiers = d.frontiers;
      this.buildFrontiers();
      this.updateStats();
    });
    gw.on("path", (d) => this.buildPath(d));

    (window as unknown as { __mind?: MindView }).__mind = this; // debugging handle
    const clock = new THREE.Clock();
    const loop = () => {
      requestAnimationFrame(loop);
      this.frame(clock.getDelta(), clock.elapsedTime);
    };
    loop();
  }

  // ------------------------------------------------------------------ UI

  private wireToolbar(root: HTMLElement): void {
    root.querySelectorAll<HTMLButtonElement>("[data-cm]").forEach((b) =>
      b.addEventListener("click", () => {
        this.colorMode = b.dataset.cm as "height" | "room";
        root.querySelectorAll("[data-cm]").forEach((x) => x.classList.toggle("on", x === b));
        this.voxDirty = true;
      }),
    );
    root.querySelectorAll<HTMLButtonElement>("[data-t]").forEach((b) =>
      b.addEventListener("click", () => {
        const t = b.dataset.t!;
        if (t === "reset") return this.fitView(true);
        const on = !b.classList.contains("on");
        b.classList.toggle("on", on);
        if (t === "rooms") (this.showRooms = on), (this.roomGroup.visible = on);
        if (t === "labels") (this.showLabels = on), this.applyLabelVisibility();
        if (t === "path") (this.showPath = on), (this.pathGroup.visible = on);
        if (t === "follow") this.follow = on;
      }),
    );
  }

  private applyLabelVisibility(): void {
    this.labels.domElement.style.display = this.showLabels ? "" : "none";
  }

  private updateStats(): void {
    const visited = this.rooms.filter((r) => r.type !== "unknown").length;
    this.stats.innerHTML = `<span><b>${this.vox.length / 3 | 0}</b> voxels</span><span><b>${this.rooms.length}</b> rooms${this.rooms.length ? ` (${visited} typed)` : ""}</span><span><b>${this.objects.filter((o) => o.status !== "gone").length}</b> objects</span><span><b>${this.frontiers.length}</b> frontiers</span>`;
  }

  private resize(): void {
    const el = this.renderer.domElement.parentElement!;
    const w = Math.max(1, el.clientWidth), h = Math.max(1, el.clientHeight);
    this.renderer.setSize(w, h);
    this.labels.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  private fitGoal: { target: THREE.Vector3; pos: THREE.Vector3 } | null = null;

  /** Camera framing of everything mapped so far (or the drone if the map is empty). */
  private computeFit(): void {
    let min = new THREE.Vector3(Infinity, Infinity, Infinity), max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
    const n = this.vox.length / 3;
    for (let i = 0; i < n; i += Math.max(1, Math.floor(n / 800))) {
      const p = this.voxPos(this.vox[3 * i], this.vox[3 * i + 1], this.vox[3 * i + 2]);
      min.min(p);
      max.max(p);
    }
    if (n === 0) {
      min = this.droneTarget.clone().addScalar(-3);
      max = this.droneTarget.clone().addScalar(3);
    }
    const c = min.clone().add(max).multiplyScalar(0.5);
    c.y = 0.3;
    const size = max.clone().sub(min);
    const r = Math.max(3.5, Math.hypot(size.x, size.z) * 0.5);
    const dir = new THREE.Vector3(0.12, 0.9, 0.62).normalize();
    this.fitGoal = { target: c, pos: c.clone().addScalar(0).addScaledVector(dir, r * 2.0 / Math.min(1, Math.max(0.5, this.camera.aspect / 1.4))) };
  }

  private fitView(force: boolean): void {
    this.computeFit();
    this.controls.target.copy(this.fitGoal!.target);
    this.camera.position.copy(this.fitGoal!.pos);
    this.controls.update();
    if (force) {
      this.userMoved = false;
      this.follow = false;
      this.host.querySelector('[data-t="follow"]')?.classList.remove("on");
    }
  }

  // ------------------------------------------------------------------ data

  private voxPos(ix: number, iy: number, iz: number): THREE.Vector3 {
    const r = this.res, o = this.origin;
    return M2T(o[0] + (ix + 0.5) * r, o[1] + (iy + 0.5) * r, o[2] + (iz + 0.5) * r);
  }

  private onPose(d: PoseData): void {
    const [x, y, z] = d.position;
    this.droneTarget.copy(M2T(x, y, z));
    this.droneYawTarget = d.yaw - Math.PI / 2; // local forward is -Z
    if (!this.haveDrone) {
      this.haveDrone = true;
      this.drone.position.copy(this.droneTarget);
      this.droneYaw = this.droneYawTarget;
      this.fitView(false);
    }
    const last = this.trail[this.trail.length - 1];
    if (!last || last.distanceTo(this.droneTarget) > 0.08) {
      this.trail.push(this.droneTarget.clone());
      if (this.trail.length > 600) this.trail.shift();
      const a = this.trailLine.geometry.getAttribute("position") as THREE.BufferAttribute;
      this.trail.forEach((p, i) => a.setXYZ(i, p.x, p.y, p.z));
      a.needsUpdate = true;
      this.trailLine.geometry.setDrawRange(0, this.trail.length);
    }
  }

  private onMap(d: MapUpdateData): void {
    if (d.reset) {
      this.index.clear();
      this.vox = [];
      this.trail = [];
      this.trailLine.geometry.setDrawRange(0, 0);
    }
    this.res = d.resolution;
    this.origin = d.origin;
    for (let i = 0; i + 2 < d.add.length; i += 3) {
      const k = key(d.add[i], d.add[i + 1], d.add[i + 2]);
      if (this.index.has(k)) continue;
      this.index.set(k, this.vox.length / 3);
      this.vox.push(d.add[i], d.add[i + 1], d.add[i + 2]);
    }
    for (let i = 0; i + 2 < d.remove.length; i += 3) {
      const k = key(d.remove[i], d.remove[i + 1], d.remove[i + 2]);
      const slot = this.index.get(k);
      if (slot === undefined) continue;
      const last = this.vox.length / 3 - 1;
      if (slot !== last) {
        const a = this.vox[3 * last], b = this.vox[3 * last + 1], c = this.vox[3 * last + 2];
        this.vox[3 * slot] = a;
        this.vox[3 * slot + 1] = b;
        this.vox[3 * slot + 2] = c;
        this.index.set(key(a, b, c), slot);
      }
      this.vox.length -= 3;
      this.index.delete(k);
    }
    this.voxDirty = true;
    this.updateStats();
  }

  private ensureMesh(n: number): void {
    if (this.mesh && n <= this.capacity) return;
    const cap = Math.max(20000, Math.ceil((n * 1.5) / 10000) * 10000);
    const old = this.mesh;
    if (old) {
      this.scene.remove(old);
      old.dispose();
    }
    const geo = new THREE.BoxGeometry(1, 1, 1);
    const mat = new THREE.MeshLambertMaterial({ color: 0xffffff });
    this.mesh = new THREE.InstancedMesh(geo, mat, cap);
    this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.mesh.frustumCulled = false;
    this.mesh.setColorAt(0, this.tmpC.set(0xffffff)); // allocates instanceColor
    this.capacity = cap;
    this.scene.add(this.mesh);
  }

  private rebuildVoxels(): void {
    const n = this.vox.length / 3;
    this.ensureMesh(n);
    const mesh = this.mesh!;
    const r = this.res, o = this.origin;
    const m = this.tmpM, c = this.tmpC;
    const byRoom = this.colorMode === "room";
    const roomCache = new Map<number, string>();
    for (let i = 0; i < n; i++) {
      const ix = this.vox[3 * i], iy = this.vox[3 * i + 1], iz = this.vox[3 * i + 2];
      const z = o[2] + (iz + 0.5) * r;
      const floor = z < 0.5 * r + o[2] + 1e-6;
      const p = this.voxPos(ix, iy, iz);
      const sy = floor ? r * 0.3 : r * 0.96;
      m.makeScale(r * 0.96, sy, r * 0.96).setPosition(p.x, floor ? o[2] + r * 0.15 : p.y, p.z);
      mesh.setMatrixAt(i, m);
      if (byRoom) {
        const cell = Math.floor((o[0] + (ix + 0.5) * r) / 0.25) * 4096 + Math.floor((o[1] + (iy + 0.5) * r) / 0.25);
        let col = roomCache.get(cell);
        if (col === undefined) {
          const px = o[0] + (ix + 0.5) * r, py = o[1] + (iy + 0.5) * r;
          const room = this.roomPolys.find((rp) => pointInPoly(px, py, rp.poly));
          col = room ? ROOM_COLORS[room.type] : "#3b4658";
          roomCache.set(cell, col);
        }
        c.set(col);
        const t = 0.55 + 0.45 * Math.min(1, z / 2);
        c.multiplyScalar(floor ? 0.4 : t);
      } else {
        heightColor(z, c);
        if (floor) c.multiplyScalar(0.8);
      }
      mesh.setColorAt(i, c);
    }
    mesh.count = n;
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    this.voxDirty = false;
    if (!this.userMoved) this.computeFit();
  }

  private buildRooms(): void {
    clearGroup(this.roomGroup);
    this.roomPolys = this.rooms.map((r) => ({ id: r.id, type: r.type, poly: r.polygon as [number, number][] }));
    for (const r of this.rooms) {
      const col = new THREE.Color(ROOM_COLORS[r.type] ?? ROOM_COLORS.unknown);
      const shape = new THREE.Shape(r.polygon.map(([x, y]) => new THREE.Vector2(x, y)));
      const geo = new THREE.ShapeGeometry(shape);
      geo.rotateX(-Math.PI / 2);
      const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: r.type === "unknown" ? 0.1 : 0.2, depthWrite: false, side: THREE.DoubleSide }));
      mesh.position.y = 0.075;
      this.roomGroup.add(mesh);
      const pts = r.polygon.map(([x, y]) => M2T(x, y, 0.08));
      pts.push(pts[0]);
      const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: 0.6 }));
      this.roomGroup.add(line);
      const cx = r.polygon.reduce((s, p) => s + p[0], 0) / r.polygon.length;
      const cy = r.polygon.reduce((s, p) => s + p[1], 0) / r.polygon.length;
      const div = document.createElement("div");
      div.className = "lbl room";
      div.style.color = "#" + col.getHexString();
      div.innerHTML = `${esc(r.label)}${r.type === "unknown" ? "<small>unlabelled</small>" : `<small>${Math.round(r.confidence * 100)}% ${r.visited ? "visited" : ""}</small>`}`;
      const lbl = new CSS2DObject(div);
      lbl.position.copy(M2T(cx, cy, 0.12));
      this.roomGroup.add(lbl);
    }
    this.roomGroup.visible = this.showRooms;
  }

  private buildObjects(): void {
    clearGroup(this.objGroup);
    for (const o of this.objects) {
      const gone = o.status === "gone";
      const col = new THREE.Color(gone ? "#f87171" : o.status === "unverified" ? "#fbbf24" : "#f4a261");
      const [x, y, z] = o.position;
      const [sx, sy, sz] = o.size;
      // bounding box (three axes: x east = map x, y up = map z, z south = -map y)
      const box = new THREE.Mesh(new THREE.BoxGeometry(Math.max(sx, 0.03), Math.max(sz, 0.03), Math.max(sy, 0.03)), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: gone ? 0.05 : 0.09, depthWrite: false }));
      box.position.copy(M2T(x, y, z));
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(box.geometry), new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: gone ? 0.4 : 0.75 }));
      box.add(edges);
      this.objGroup.add(box);
      const top = z + sz / 2;
      const stem = new THREE.Line(new THREE.BufferGeometry().setFromPoints([M2T(x, y, top), M2T(x, y, top + 0.3)]), new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: 0.8 }));
      const pin = new THREE.Mesh(new THREE.SphereGeometry(0.05, 14, 10), new THREE.MeshBasicMaterial({ color: col }));
      pin.position.copy(M2T(x, y, top + 0.3));
      this.objGroup.add(stem, pin);
      const div = document.createElement("div");
      div.className = "lbl obj" + (gone ? " gone" : "");
      div.innerHTML = `${esc(o.label)}<em>${gone ? "gone" : Math.round(o.confidence * 100) + "%"}</em>`;
      const lbl = new CSS2DObject(div);
      lbl.position.copy(M2T(x, y, top + 0.38));
      this.objGroup.add(lbl);
    }
  }

  private buildFrontiers(): void {
    clearGroup(this.frontGroup);
    const geo = new THREE.OctahedronGeometry(0.1);
    for (const f of this.frontiers) {
      const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0xfbbf24 }));
      m.position.copy(M2T(...f.position));
      m.userData.base = m.position.y;
      this.frontGroup.add(m);
      const ring = new THREE.Mesh(new THREE.RingGeometry(0.16, 0.2, 28), new THREE.MeshBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.45, side: THREE.DoubleSide }));
      ring.rotation.x = -Math.PI / 2;
      ring.position.set(m.position.x, 0.09, m.position.z);
      this.frontGroup.add(ring);
    }
  }

  private buildPath(d: PathData): void {
    clearGroup(this.pathGroup);
    if (d.waypoints.length < 2) return;
    const pts = d.waypoints.map((w) => M2T(...w));
    const curve = new THREE.CatmullRomCurve3(pts, false, "centripetal", 0.3);
    const col = d.status === "failed" ? 0xf87171 : 0xffd166;
    const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, Math.min(400, pts.length * 12), 0.022, 6, false), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.85 }));
    this.pathGroup.add(tube);
    const dot = new THREE.SphereGeometry(0.045, 10, 8);
    for (const p of pts.slice(1, -1)) {
      const m = new THREE.Mesh(dot, new THREE.MeshBasicMaterial({ color: col }));
      m.position.copy(p);
      this.pathGroup.add(m);
    }
    const goal = d.goal ? M2T(...d.goal) : pts[pts.length - 1];
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.2, 0.26, 32), new THREE.MeshBasicMaterial({ color: col, side: THREE.DoubleSide, transparent: true, opacity: 0.9 }));
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(goal.x, 0.09, goal.z);
    this.pathGroup.add(ring);
    this.pathGroup.visible = this.showPath;
  }

  // ------------------------------------------------------------------ drone

  private buildDrone(): void {
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.05, 0.2), new THREE.MeshLambertMaterial({ color: 0x1b2433 }));
    this.drone.add(body);
    const armMat = new THREE.MeshLambertMaterial({ color: 0x2b3850 });
    for (const [sx, sz] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.02, 0.26), armMat);
      arm.position.set(sx * 0.1, 0, sz * 0.1);
      arm.rotation.y = (sx * sz * Math.PI) / 4;
      this.drone.add(arm);
      const rotor = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.01, 20), new THREE.MeshBasicMaterial({ color: sz > 0 ? 0xfbbf24 : 0x4cc9f0, transparent: true, opacity: 0.55 }));
      rotor.position.set(sx * 0.18, 0.035, sz * 0.18);
      this.drone.add(rotor);
      this.rotors.push(rotor);
    }
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.06, 0.2, 12), new THREE.MeshBasicMaterial({ color: 0xff7a3d }));
    nose.rotation.x = -Math.PI / 2; // +Y -> -Z (forward)
    nose.position.set(0, 0, -0.24);
    this.drone.add(nose);
    // camera field of view wedge (90 deg, 3 m), flat at drone height
    const R = 3, a = Math.PI / 4;
    const wedge = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(-R * Math.sin(a), 0, -R * Math.cos(a)), new THREE.Vector3(R * Math.sin(a), 0, -R * Math.cos(a))]);
    this.drone.add(new THREE.Mesh(wedge, new THREE.MeshBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.1, side: THREE.DoubleSide, depthWrite: false })));
    const edge = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(-R * Math.sin(a), 0, -R * Math.cos(a)), new THREE.Vector3(0, 0, 0), new THREE.Vector3(R * Math.sin(a), 0, -R * Math.cos(a))]), new THREE.LineBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.35 }));
    this.drone.add(edge);
    const div = document.createElement("div");
    div.className = "lbl drone";
    div.textContent = "MAVERICK";
    const lbl = new CSS2DObject(div);
    lbl.position.set(0, 0.28, 0);
    this.drone.add(lbl);
  }

  private shadow: THREE.Mesh | null = null;
  private stalk: THREE.Line | null = null;

  // ------------------------------------------------------------------ loop

  private frame(dt: number, time: number): void {
    if (this.voxDirty) this.rebuildVoxels();
    // drone smoothing
    const k = 1 - Math.exp(-dt * 14);
    this.drone.position.lerp(this.droneTarget, k);
    let dy = this.droneYawTarget - this.droneYaw;
    dy = Math.atan2(Math.sin(dy), Math.cos(dy));
    this.droneYaw += dy * k;
    this.drone.rotation.y = this.droneYaw;
    for (const r of this.rotors) r.rotation.y += dt * 30;
    this.drone.position.y += Math.sin(time * 3) * 0.0006;
    // shadow + altitude stalk
    if (!this.shadow) {
      this.shadow = new THREE.Mesh(new THREE.RingGeometry(0.1, 0.16, 28), new THREE.MeshBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.6, side: THREE.DoubleSide }));
      this.shadow.rotation.x = -Math.PI / 2;
      this.scene.add(this.shadow);
      this.stalk = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 1, 0)]), new THREE.LineBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.3 }));
      this.scene.add(this.stalk);
    }
    this.shadow.visible = this.stalk!.visible = this.haveDrone;
    this.shadow.position.set(this.drone.position.x, 0.09, this.drone.position.z);
    this.stalk!.position.set(this.drone.position.x, 0.09, this.drone.position.z);
    this.stalk!.scale.y = Math.max(0.01, this.drone.position.y - 0.09);
    // frontier animation
    for (const o of this.frontGroup.children) {
      if (o.userData.base !== undefined) {
        o.position.y = o.userData.base + Math.sin(time * 2.2 + o.position.x * 3) * 0.04;
        o.rotation.y = time * 1.5;
      }
    }
    // follow
    if (this.follow && this.haveDrone) {
      const toTarget = this.drone.position.clone().sub(this.controls.target).multiplyScalar(Math.min(1, dt * 3));
      this.controls.target.add(toTarget);
      this.camera.position.add(toTarget);
    }
    else if (!this.userMoved && this.fitGoal && this.vox.length > 0) {
      const k2 = 1 - Math.exp(-dt * 2.5);
      const dT = this.fitGoal.target.clone().sub(this.controls.target).multiplyScalar(k2);
      this.controls.target.add(dT);
      this.camera.position.add(this.fitGoal.pos.clone().sub(this.camera.position).multiplyScalar(k2));
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.labels.render(this.scene, this.camera);
  }
}

function clearGroup(g: THREE.Group): void {
  for (const c of [...g.children]) {
    g.remove(c);
    c.traverse((o) => {
      const m = o as THREE.Mesh;
      m.geometry?.dispose();
      const mat = m.material as THREE.Material | undefined;
      mat?.dispose?.();
      if (o instanceof CSS2DObject) o.element.remove();
    });
  }
}

function esc(s: string): string {
  return s.replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]!);
}
