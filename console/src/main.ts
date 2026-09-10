import "./style.css";
import { GatewayClient } from "./gateway/client";
import { CameraPanel } from "./panels/camera";
import { MindView } from "./panels/mind";
import { ConversationPanel } from "./panels/conversation";
import { PlanPanel } from "./panels/plan";
import { StatusPanel } from "./panels/status";

const params = new URLSearchParams(location.search);
const gatewayUrl = params.get("gateway") ?? `ws://${location.hostname || "localhost"}:8765`;

const app = document.getElementById("app")!;
app.innerHTML = `
<div class="app">
  <div class="topbar">
    <span class="brand">MAVER<b>I</b>CK</span>
    <span class="sub">Mission Console</span>
    <span class="spacer"></span>
    <span class="pill" id="conn"><i class="dot"></i><span>connecting</span></span>
    <span class="sub" id="gwurl"></span>
  </div>
  <main class="grid">
    <section class="panel cam"><h2>Drone camera <span class="tag" id="cam-tag"></span></h2><div class="body" id="cam-body"></div></section>
    <section class="panel mind"><h2>Maverick's mind <span class="tag">what the drone knows, not ground truth</span></h2><div class="body" id="mind-body"></div></section>
    <section class="panel world"><h2>World <span class="tag">Gazebo</span></h2><div class="body">
      <b>World view</b><span>The Gazebo third-person view will be streamed here in a later stage.</span></div></section>
    <section class="panel convo"><h2>Conversation</h2><div class="body" id="convo-body"></div></section>
    <section class="panel plan"><h2>Agent plan</h2><div class="body" id="plan-body"></div></section>
    <section class="panel status"><h2>Status</h2><div class="body" id="status-body"></div></section>
  </main>
</div>`;

const gw = new GatewayClient(gatewayUrl);
const q = (id: string) => document.getElementById(id) as HTMLElement;
q("gwurl").textContent = gatewayUrl;

new CameraPanel(q("cam-body"), q("cam-tag"), gw);
new MindView(q("mind-body"), gw);
new ConversationPanel(q("convo-body"), gw);
new PlanPanel(q("plan-body"), gw);
new StatusPanel(q("status-body"), gw);

const conn = q("conn");
gw.onState((s) => {
  const dot = conn.querySelector(".dot")!;
  dot.className = "dot " + (s === "open" ? "good" : s === "connecting" ? "warn" : "bad");
  conn.querySelector("span")!.textContent = s === "open" ? "gateway connected" : s === "connecting" ? "connecting" : "gateway offline, retrying";
});
gw.connect();
