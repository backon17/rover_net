// Drive the console page through Chrome DevTools Protocol (no puppeteer here).
// usage: node cdp_drive.mjs <debug-port> <out-dir> <script-name>
import { writeFileSync } from "node:fs";
const [port, out, which] = process.argv.slice(2);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
let ws, id = 0;
const pending = new Map();
async function connect() {
  for (let i = 0; i < 50; i++) {
    try {
      const list = await (await fetch(`http://127.0.0.1:${port}/json`)).json();
      const page = list.find((t) => t.type === "page");
      if (page) { ws = new WebSocket(page.webSocketDebuggerUrl); break; }
    } catch (_) {}
    await sleep(200);
  }
  await new Promise((r) => (ws.onopen = r));
  ws.onmessage = (m) => { const d = JSON.parse(m.data); if (d.id && pending.has(d.id)) { pending.get(d.id)(d); pending.delete(d.id); } };
}
const send = (method, params = {}) => new Promise((r) => { const i = ++id; pending.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
async function ev(expr) {
  const r = await send("Runtime.evaluate", { expression: expr, awaitPromise: true, returnByValue: true });
  if (r.result.exceptionDetails) throw new Error(JSON.stringify(r.result.exceptionDetails).slice(0, 600));
  return r.result.result.value;
}
async function shot(name) {
  const r = await send("Page.captureScreenshot", { format: "png" });
  writeFileSync(`${out}/${name}.png`, Buffer.from(r.result.data, "base64"));
  console.log("shot", name);
}
await connect();
await send("Runtime.enable"); await send("Page.enable");
await send("Emulation.setDeviceMetricsOverride", { width: 1880, height: 1250, deviceScaleFactor: 1, mobile: false });
const errs = [];
ws.addEventListener("message", (m) => { const d = JSON.parse(m.data); if (d.method === "Runtime.exceptionThrown") errs.push(d.params.exceptionDetails.text + " " + (d.params.exceptionDetails.exception || {}).description); });
await sleep(2500);
// wait until 7 cars are online
for (let i = 0; i < 60; i++) { if ((await ev("live ? live.cars.filter((c) => c.connected).length : 0")) >= 7) break; await sleep(300); }
await shot("idle");
if (which === "setup") {
  // select car2, template L, left, a=1 b=0.3, generate
  await ev(`document.querySelector('.tab[data-i="1"]').click()`);
  await ev(`document.querySelector('[data-tpl="L 形"]').click()`);
  await ev(`document.querySelector('[data-dir="1"]').click()`);
  await ev(`(() => { const a = document.querySelector('[data-p="a"]'), b = document.querySelector('[data-p="b"]'); a.value = 1; a.oninput(); b.value = 0.3; b.oninput(); })()`);
  await ev(`document.getElementById("tplgo").click()`);
  await ev(`document.getElementById("fb-all").click()`);
  await ev(`document.getElementById("fb-copy").click()`);
  await ev(`(() => { const g = document.getElementById("fb-gap"); g.value = 0.4; })()`);
  await ev(`document.getElementById("fb-line").click()`);
  await sleep(800);
  const st = await ev(`JSON.stringify({ cars: cfg.cars.map((c) => ({ id: c.id, en: c.enabled, steps: c.steps, speed: c.speed, start: c.start })), go: document.getElementById("go-sync").textContent, sum: document.getElementById("fbsum").textContent, warn: document.getElementById("warn").textContent, msg: document.getElementById("fb-msg").textContent })`);
  console.log("STATE " + st);
  await shot("configured");
  // one click: sync start (confirm auto-accepted, text captured)
  await ev(`window.__confirmText = ""; window.confirm = (t) => { window.__confirmText = t; return true; }; window.alert = (t) => { window.__alertText = t; };`);
  await ev(`document.getElementById("go-sync").click()`);
  await sleep(500);
  console.log("CONFIRM " + JSON.stringify(await ev("window.__confirmText")));
  console.log("ALERT " + JSON.stringify(await ev("window.__alertText || ''")));
  for (let i = 0; i < 40; i++) { await sleep(300); if (await ev(`live && live.cars.some((c) => c.phase === "straight")`)) break; }
  await sleep(4500);
  await shot("running");
  for (let i = 0; i < 200; i++) { await sleep(300); if (await ev(`live && !live.running`)) break; }
  await sleep(800);
  await shot("done");
}
console.log("JSERRORS " + JSON.stringify(errs));
ws.close();
process.exit(0);
