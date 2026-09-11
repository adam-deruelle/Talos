"use strict";
/* xArm · CONSOLE — logique frontend (vanilla JS) */

const TOKEN = document.querySelector('meta[name=xarm-token]').content;
const $  = (id) => document.getElementById(id);
const SERVOS = [1, 2, 3, 4, 5, 6];
const NAMES = {
  1: ["pince", "servo 1"], 2: ["poignet", "rotation"], 3: ["poignet", "flexion"],
  4: ["coude", "servo 4"], 5: ["épaule", "servo 5"], 6: ["base", "servo 6"],
};
const HUE = { 1: "#e8b04a", 2: "#8fb7ff", 3: "#7fd4c1", 4: "#c9a3ff", 5: "#f2a2b0", 6: "#a7d879" };

const setpoint = {}; SERVOS.forEach(s => setpoint[s] = 500);
const lastCmd = {}; SERVOS.forEach(s => lastCmd[s] = 0);  // horodatage dernière consigne locale
const stamp = (s) => { lastCmd[s] = performance.now(); };
let latest = null;          // dernier /state reçu
let lastSeq = -1;
let firstState = true;
let interacting = false;    // curseur en cours de manipulation
let selPose = null, selRec = null;
let armedAxis = 6;
let gripOpen = true;

/* ---------------- API ---------------- */
function post(path, body) {
  return fetch(path, { method: "POST",
    headers: { "Content-Type": "application/json", "X-XArm-Token": TOKEN },
    body: JSON.stringify(body || {}) }).catch(() => {});
}
function del(path) {
  return fetch(path, { method: "DELETE", headers: { "X-XArm-Token": TOKEN } }).catch(() => {});
}

/* ---------------- Mouvements ---------------- */
let moveBuf = {}, moveDur = 800, moveTimer = null;
function queueMove(targets, dur) {
  Object.assign(moveBuf, targets); moveDur = dur;
  if (!moveTimer) moveTimer = setTimeout(flushMove, 55);
}
function flushMove() {
  const t = moveBuf; moveBuf = {}; moveTimer = null;
  post("/move", { targets: t, duration_ms: moveDur });
}
const durVal = () => +$("dur").value;
const stepVal = () => Math.max(1, +$("step").value || 1);
function lim(s) { const l = latest && latest.limits ? latest.limits[s] : null; return l ? [l.min, l.max] : [200, 800]; }
function clampS(s, v) { const [lo, hi] = lim(s); return Math.max(lo, Math.min(hi, Math.round(v))); }

/* ---------------- Afficheur 7 segments ---------------- */
function horiz(cy) { const xl = 5.5, xr = 20.5, h = 2;
  return `${xl},${cy} ${xl + h},${cy - h} ${xr - h},${cy - h} ${xr},${cy} ${xr - h},${cy + h} ${xl + h},${cy + h}`; }
function vert(cx, yt, yb) { const h = 2; yt += 2; yb -= 2;
  return `${cx},${yt} ${cx + h},${yt + h} ${cx + h},${yb - h} ${cx},${yb} ${cx - h},${yb - h} ${cx - h},${yt + h}`; }
const SEGP = { a: horiz(3.5), g: horiz(23), d: horiz(42.5),
  f: vert(3.5, 3.5, 23), b: vert(22.5, 3.5, 23), e: vert(3.5, 23, 42.5), c: vert(22.5, 23, 42.5) };
const DIG = { "0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc",
  "5": "afgcd", "6": "afgecd", "7": "abc", "8": "abcdefg", "9": "abcdfg", " ": "", "-": "g" };
const SVGNS = "http://www.w3.org/2000/svg";
function makeDigit() {
  const svg = document.createElementNS(SVGNS, "svg");
  svg.setAttribute("viewBox", "0 0 26 46");
  for (const k of "abcdefg") {
    const p = document.createElementNS(SVGNS, "polygon");
    p.setAttribute("points", SEGP[k]); p.setAttribute("class", "seg7"); p.dataset.k = k;
    svg.appendChild(p);
  }
  return svg;
}
function setDigit(svg, ch) {
  const on = DIG[ch] || "";
  svg.querySelectorAll(".seg7").forEach(p => p.classList.toggle("on", on.includes(p.dataset.k)));
}
function setSetpoint(s) {
  const box = $("sp-" + s);
  const str = String(setpoint[s]).padStart(4, " ").slice(-4);
  [...box.children].forEach((svg, i) => setDigit(svg, str[i]));
}

/* ---------------- Construction des strips ---------------- */
function buildStrips() {
  const root = $("strips");
  SERVOS.forEach(s => {
    const [main, sub] = NAMES[s];
    const el = document.createElement("div");
    el.className = "strip"; el.dataset.s = s;
    el.innerHTML =
      `<div class="strip-head">
         <div class="chip" style="--hue:${HUE[s]}">${s}</div>
         <div class="names"><b>${main}</b><small>${sub}</small></div>
         <div class="readout">
           <div class="setpoint" id="sp-${s}"></div>
           <div class="live-wrap"><div class="live-val" id="lv-${s}">—</div><div class="delta" id="dl-${s}">Δ —</div></div>
           <button class="torque" id="tq-${s}" title="Couple : HOLD / FREE"><i></i></button>
         </div>
       </div>
       <div class="strip-jog">
         <button class="jbtn" data-dir="-1">−</button>
         <input type="range" id="sl-${s}" min="200" max="800" step="1" value="500">
         <button class="jbtn" data-dir="1">+</button>
       </div>
       <div class="seek" id="sk-${s}"><div class="fill"></div><div class="tc"></div><div class="tl"></div></div>`;
    root.appendChild(el);
    const box = $("sp-" + s); for (let i = 0; i < 4; i++) box.appendChild(makeDigit());
    setSetpoint(s);

    const sl = $("sl-" + s);
    sl.addEventListener("pointerdown", () => { interacting = true; armedAxis = s; renderArmed(); });
    sl.addEventListener("pointerup", () => { interacting = false; if (!$("live").checked) sendServo(s); });
    sl.addEventListener("input", () => {
      setpoint[s] = +sl.value; stamp(s); setSetpoint(s); renderArm();
      if ($("live").checked) queueMove({ [s]: setpoint[s] }, Math.min(durVal(), 150));
    });
    el.querySelectorAll(".jbtn").forEach(b =>
      b.addEventListener("click", () => nudge(s, +b.dataset.dir * stepVal())));
    $("tq-" + s).addEventListener("click", () => toggleTorque(s));
  });
}
function sendServo(s) { stamp(s); post("/move", { targets: { [s]: setpoint[s] }, duration_ms: durVal() }); }
function nudge(s, delta) {
  setpoint[s] = clampS(s, setpoint[s] + delta); $("sl-" + s).value = setpoint[s];
  setSetpoint(s); renderArm(); sendServo(s);
}
function toggleTorque(s) {
  const held = latest && latest.torque[s];
  if (held) { post("/release", { ids: [s] }); }
  else { const r = latest && latest.read[s]; const v = (r != null ? r : setpoint[s]);
    setpoint[s] = clampS(s, v); stamp(s); $("sl-" + s).value = setpoint[s]; setSetpoint(s);
    post("/move", { targets: { [s]: setpoint[s] }, duration_ms: 400 }); }
}

/* ---------------- Cinématique (SVG) ---------------- */
const O = { x: 210, y: 252 }, L1 = 74, L2 = 64, L3 = 30;
function fk(v) {
  const d = Math.PI / 180;
  const a5 = (90 - (v[5] - 500) / 500 * 62) * d;
  const fa = a5 - (v[4] - 500) / 500 * 66 * d;
  const wa = fa + (v[3] - 500) / 500 * 60 * d;
  const P1 = { x: O.x + L1 * Math.cos(a5), y: O.y - L1 * Math.sin(a5) };
  const P2 = { x: P1.x + L2 * Math.cos(fa), y: P1.y - L2 * Math.sin(fa) };
  const P3 = { x: P2.x + L3 * Math.cos(wa), y: P2.y - L3 * Math.sin(wa) };
  const open = (v[1] / 1000) * 12 + 3;
  return { P1, P2, P3, wa, open };
}
function chainSVG(v, color, op, w) {
  const { P1, P2, P3, wa, open } = fk(v);
  const jx = Math.cos(wa), jy = -Math.sin(wa), px = -jy, py = jx;
  const tip = { x: P3.x + jx * 9, y: P3.y + jy * 9 };
  const A = { x: tip.x + px * open, y: tip.y + py * open };
  const B = { x: tip.x - px * open, y: tip.y - py * open };
  return `<g stroke="${color}" opacity="${op}" fill="none" stroke-linecap="round" stroke-linejoin="round">
      <polyline stroke-width="${w}" points="${O.x},${O.y} ${P1.x.toFixed(1)},${P1.y.toFixed(1)} ${P2.x.toFixed(1)},${P2.y.toFixed(1)} ${P3.x.toFixed(1)},${P3.y.toFixed(1)}"/>
      <line stroke-width="${w - 3}" x1="${P3.x.toFixed(1)}" y1="${P3.y.toFixed(1)}" x2="${A.x.toFixed(1)}" y2="${A.y.toFixed(1)}"/>
      <line stroke-width="${w - 3}" x1="${P3.x.toFixed(1)}" y1="${P3.y.toFixed(1)}" x2="${B.x.toFixed(1)}" y2="${B.y.toFixed(1)}"/>
    </g>
    <g fill="${color}" opacity="${op}"><circle cx="${P1.x.toFixed(1)}" cy="${P1.y.toFixed(1)}" r="3.4"/><circle cx="${P2.x.toFixed(1)}" cy="${P2.y.toFixed(1)}" r="3.4"/></g>`;
}
let trail = [];
const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
function renderArm() {
  const svg = $("arm"); if (!svg) return;
  const reads = latest && Object.keys(latest.read || {}).length
    ? Object.fromEntries(SERVOS.map(s => [s, latest.read[s] != null ? latest.read[s] : setpoint[s]])) : null;
  const A = css("--accent"), G = css("--live"), FA = css("--faint");
  let out = `<line x1="16" y1="${O.y}" x2="404" y2="${O.y}" stroke="${FA}" stroke-width="1" opacity=".5"/>`;
  out += `<path d="M ${O.x - 150} ${O.y} A 150 150 0 0 1 ${O.x + 150} ${O.y}" fill="none" stroke="${FA}" stroke-width="1" stroke-dasharray="2 5" opacity=".5"/>`;
  out += `<rect x="${O.x - 26}" y="${O.y}" width="52" height="30" rx="4" fill="${css("--surface-2")}" stroke="${css("--border")}"/>`;
  if (reads && !reduceMotion && trail.length > 1) {
    out += `<polyline fill="none" stroke="${G}" stroke-width="1.5" opacity=".35" points="${trail.map(p => p.x.toFixed(0) + "," + p.y.toFixed(0)).join(" ")}"/>`;
  }
  out += chainSVG(setpoint, A, 0.42, 7);          // fantôme ambre = consigne
  if (reads) out += chainSVG(reads, G, 0.96, 6);  // solide vert = position lue
  out += `<circle cx="${O.x}" cy="${O.y}" r="5" fill="${reads ? G : A}"/>`;
  svg.innerHTML = out;
}
function css(v) { return getComputedStyle(document.documentElement).getPropertyValue(v).trim(); }

function renderCompass() {
  const svg = $("compass"); if (!svg) return;
  const C = { x: 60, y: 62 }, R = 46, A = css("--accent"), G = css("--live"), M = css("--muted"), F = css("--faint");
  const reads = latest && latest.read ? latest.read : {};
  function needle(cx, cy, r, val, color, w) {
    const ang = (-90 + (val - 500) / 500 * 150) * Math.PI / 180;
    return `<line x1="${cx}" y1="${cy}" x2="${(cx + r * Math.cos(ang)).toFixed(1)}" y2="${(cy + r * Math.sin(ang)).toFixed(1)}" stroke="${color}" stroke-width="${w}" stroke-linecap="round"/>`;
  }
  let out = `<circle cx="${C.x}" cy="${C.y}" r="${R}" fill="none" stroke="${css("--border")}"/>`;
  for (let a = 0; a < 360; a += 30) { const r1 = R - 4, r2 = R, rad = a * Math.PI / 180;
    out += `<line x1="${(C.x + r1 * Math.cos(rad)).toFixed(1)}" y1="${(C.y + r1 * Math.sin(rad)).toFixed(1)}" x2="${(C.x + r2 * Math.cos(rad)).toFixed(1)}" y2="${(C.y + r2 * Math.sin(rad)).toFixed(1)}" stroke="${F}" stroke-width="1"/>`; }
  out += `<circle cx="${C.x}" cy="${C.y}" r="24" fill="none" stroke="${F}" stroke-dasharray="2 3"/>`;
  out += needle(C.x, C.y, R - 8, setpoint[6], A, 2);
  if (reads[6] != null) out += needle(C.x, C.y, R - 8, reads[6], G, 2.5);
  out += needle(C.x, C.y, 20, setpoint[2], A, 2);
  if (reads[2] != null) out += needle(C.x, C.y, 20, reads[2], G, 2.5);
  out += `<circle cx="${C.x}" cy="${C.y}" r="3" fill="${M}"/>`;
  out += `<text x="60" y="14" fill="${M}" font-size="8" text-anchor="middle" letter-spacing="1">BASE · POIGNET</text>`;
  svg.innerHTML = out;
}

/* ---------------- Rendu depuis /state ---------------- */
function onState(st) {
  latest = st;
  // pastille de connexion
  const conn = $("conn"), txt = $("conn-txt");
  conn.className = "pill " + (st.connected ? "ok" : (st.error ? "bad" : "wait"));
  txt.textContent = st.connected ? "CONNECTÉ" : (st.error ? "HORS LIGNE" : "CONNEXION…");
  $("dev").textContent = st.connected ? st.info : (st.error || "");

  if (firstState) { SERVOS.forEach(s => { setpoint[s] = st.command[s]; $("sl-" + s).value = st.command[s]; setSetpoint(s); }); firstState = false; }

  // suivi de la lecture (curseurs alignés) : seulement quand la lecture est
  // FRAÎCHE (auto-lecture / apprentissage / rejeu), et jamais sur un axe qu'on
  // vient de commander à la main (sinon le jog est écrasé par une lecture périmée).
  const freshRead = (st.read_age_ms != null && st.read_age_ms < 1500) || st.recording || st.replaying;
  if ($("follow").checked && !interacting && freshRead) {
    const now = performance.now();
    SERVOS.forEach(s => {
      if (st.read[s] != null && now - lastCmd[s] > 1200) {
        setpoint[s] = st.read[s]; $("sl-" + s).value = st.read[s]; setSetpoint(s);
      }
    });
  }

  // limites (course) → bornes des curseurs
  SERVOS.forEach(s => { const [lo, hi] = [st.limits[s].min, st.limits[s].max];
    const sl = $("sl-" + s); if (+sl.min !== lo) sl.min = lo; if (+sl.max !== hi) sl.max = hi; });

  // readouts
  SERVOS.forEach(s => {
    const r = st.read[s]; const lv = $("lv-" + s), dl = $("dl-" + s);
    lv.textContent = (r != null ? r : "—");
    const d = (r != null ? setpoint[s] - r : null);
    dl.textContent = "Δ " + (d == null ? "—" : (d > 0 ? "+" : "") + d);
    dl.className = "delta" + (d != null && Math.abs(d) < 3 ? " zero" : (d != null && Math.abs(d) > 60 ? " big" : ""));
    $("tq-" + s).classList.toggle("free", !st.torque[s]);
    const [lo, hi] = [st.limits[s].min, st.limits[s].max]; const span = Math.max(1, hi - lo);
    const pc = (setpoint[s] - lo) / span * 100, pl = (r != null ? (r - lo) / span * 100 : pc);
    const sk = $("sk-" + s); sk.querySelector(".tc").style.left = pc + "%";
    sk.querySelector(".tl").style.left = pl + "%";
    const f = sk.querySelector(".fill"); f.style.left = Math.min(pc, pl) + "%"; f.style.width = Math.abs(pc - pl) + "%";
  });

  // trail de la pointe (position lue)
  if (st.read && st.read[1] != null && !reduceMotion) {
    const { P3, wa } = fk(Object.fromEntries(SERVOS.map(s => [s, st.read[s] != null ? st.read[s] : setpoint[s]])));
    const tip = { x: P3.x + Math.cos(wa) * 9, y: P3.y - Math.sin(wa) * 9 };
    const last = trail[trail.length - 1];
    if (!last || Math.hypot(last.x - tip.x, last.y - tip.y) > 1.5) { trail.push(tip); if (trail.length > 26) trail.shift(); }
  }
  renderArm(); renderCompass();

  // contrôles synchronisés depuis le serveur
  segSet("range", "mode", st.range_mode);
  segSet("profile", "p", st.profile);
  if (document.activeElement !== $("feed")) $("feed").value = st.feed;
  $("feed-out").textContent = $("feed").value + "%";
  $("autoread").checked = st.autoread;
  $("pause").textContent = st.paused ? "▶ REPRENDRE" : "⏸ PAUSE";
  const rb = $("rec-toggle"); rb.classList.toggle("recording", st.recording);
  rb.querySelector("i").nextSibling && (rb.lastChild.textContent = st.recording ? " ARRÊTER" : " ENREGISTRER");
  $("rec-info").textContent = st.recording ? (st.rec_frames + " img") : (st.has_recording ? "prêt à sauver" : "—");
  $("rec-save").disabled = !st.has_recording;

  renderLists(st); renderLog(st);
}

function renderLists(st) {
  const pl = $("pose-list");
  if (pl.dataset.sig !== st.poses.join("|") + "#" + selPose) {
    pl.dataset.sig = st.poses.join("|") + "#" + selPose; pl.innerHTML = "";
    st.poses.forEach(n => { const li = document.createElement("li"); li.textContent = n;
      if (n === selPose) li.classList.add("sel");
      li.addEventListener("click", () => { selPose = n; renderLists(latest); });
      li.addEventListener("dblclick", () => post("/poses/" + encodeURIComponent(n) + "/goto", { duration_ms: durVal() }));
      pl.appendChild(li); });
  }
  const rl = $("rec-list");
  if (rl.dataset.sig !== st.recordings.join("|") + "#" + selRec) {
    rl.dataset.sig = st.recordings.join("|") + "#" + selRec; rl.innerHTML = "";
    st.recordings.forEach(n => { const li = document.createElement("li"); li.textContent = n;
      if (n === selRec) li.classList.add("sel");
      li.addEventListener("click", () => { selRec = n; renderLists(latest); });
      li.addEventListener("dblclick", () => post("/replay", { name: n }));
      rl.appendChild(li); });
  }
}
let logSig = "";
function renderLog(st) {
  const sig = st.log.length + ":" + (st.log[st.log.length - 1] || {}).t;
  if (sig === logSig) return; logSig = sig;
  const box = $("log");
  box.innerHTML = st.log.map(l => `<div class="log-line ${l.n}"><span class="lt">${l.t}</span><span class="lm">${escapeHtml(l.m)}</span></div>`).join("");
  box.scrollTop = box.scrollHeight;
}
function escapeHtml(s) { return s.replace(/[&<>]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c])); }
function segSet(id, key, val) { $(id).querySelectorAll("button").forEach(b => b.classList.toggle("on", b.dataset[key] === val)); }
function renderArmed() { $("armed").textContent = "AXE ARMÉ : " + armedAxis + " · " + NAMES[armedAxis][0]; }

/* ---------------- Boutons ---------------- */
function wireControls() {
  $("home").onclick = () => post("/home");
  $("park").onclick = () => post("/park");
  $("read").onclick = () => post("/read");
  $("release").onclick = () => post("/release", {});
  $("estop").onclick = $("pause") && (() => post("/stop"));
  $("estop").onclick = () => post("/stop");
  $("pause").onclick = () => post("/pause", { paused: !(latest && latest.paused) });
  $("recaler").onclick = () => {
    const t = {}; SERVOS.forEach(s => { const r = latest && latest.read[s]; if (r != null) { setpoint[s] = clampS(s, r); stamp(s); $("sl-" + s).value = setpoint[s]; setSetpoint(s); t[s] = setpoint[s]; } });
    if (Object.keys(t).length) post("/move", { targets: t, duration_ms: durVal() }); renderArm();
  };
  $("mirror").onclick = () => {
    const t = {}; [2, 6].forEach(s => { setpoint[s] = clampS(s, 1000 - setpoint[s]); stamp(s); $("sl-" + s).value = setpoint[s]; setSetpoint(s); t[s] = setpoint[s]; });
    post("/move", { targets: t, duration_ms: durVal() }); renderArm();
  };
  ["open", "half", "close"].forEach(k => $("grip-" + k).onclick = () => {
    const [lo, hi] = lim(1); const v = k === "open" ? hi : k === "close" ? lo : 500;
    gripOpen = (k === "open"); setpoint[1] = v; stamp(1); $("sl-1").value = v; setSetpoint(1); renderArm();
    post("/move", { targets: { 1: v }, duration_ms: durVal() });
  });

  $("dur").oninput = () => $("dur-out").textContent = $("dur").value;
  let feedT = null;
  $("feed").oninput = () => { $("feed-out").textContent = $("feed").value + "%"; clearTimeout(feedT); feedT = setTimeout(() => post("/feed", { feed: +$("feed").value }), 120); };
  $("range").querySelectorAll("button").forEach(b => b.onclick = () => post("/range", { mode: b.dataset.mode }));
  $("profile").querySelectorAll("button").forEach(b => b.onclick = () => post("/profile", { profile: b.dataset.p }));
  $("autoread").onchange = () => post("/autoread", { enabled: $("autoread").checked, interval_ms: 1000 });

  // poses
  $("pose-save").onclick = () => { post("/poses", { name: $("pose-name").value }); $("pose-name").value = ""; };
  $("pose-goto").onclick = () => { if (selPose) post("/poses/" + encodeURIComponent(selPose) + "/goto", { duration_ms: durVal() }); };
  $("pose-del").onclick = () => { if (selPose) { del("/poses/" + encodeURIComponent(selPose)); selPose = null; } };
  $("pose-play").onclick = () => post("/poses/play", { duration_ms: durVal(), pause_ms: +$("pause-ms").value, loop: $("loop").checked });
  $("pose-export").onclick = exportPoses;
  $("pose-import").onclick = () => $("file-in").click();
  $("file-in").onchange = importPoses;

  // enregistrement gestuel
  $("rec-toggle").onclick = () => post(latest && latest.recording ? "/record/stop" : "/record/start");
  $("rec-save").onclick = () => { post("/recordings", { name: $("rec-name").value }); $("rec-name").value = ""; };
  $("rec-play").onclick = () => post("/replay", selRec ? { name: selRec } : {});
  $("rec-del").onclick = () => { if (selRec) { del("/recordings/" + encodeURIComponent(selRec)); selRec = null; } };
}

async function exportPoses() {
  const data = await fetch("/poses/export").then(r => r.json()).catch(() => null);
  if (!data) return;
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: "application/json" }));
  a.download = "xarm_poses.json"; a.click(); URL.revokeObjectURL(a.href);
}
function importPoses(e) {
  const f = e.target.files[0]; if (!f) return;
  const r = new FileReader();
  r.onload = () => { try { post("/poses/import", JSON.parse(r.result)); } catch (_) {} };
  r.readAsText(f); e.target.value = "";
}

/* ---------------- Thème ---------------- */
function wireTheme() {
  const html = document.documentElement, btn = $("theme");
  const glyph = { system: "◒", dark: "◑", light: "◐" };
  let t = localStorage.getItem("xarm-theme") || "system";
  const apply = () => { if (t === "system") html.removeAttribute("data-theme"); else html.setAttribute("data-theme", t);
    btn.textContent = glyph[t]; localStorage.setItem("xarm-theme", t); if (latest) { renderArm(); renderCompass(); } };
  btn.onclick = () => { t = t === "system" ? "dark" : t === "dark" ? "light" : "system"; apply(); };
  apply();
}

/* ---------------- Clavier ---------------- */
function wireKeyboard() {
  document.addEventListener("keydown", (e) => {
    const tag = (e.target.tagName || "").toLowerCase();
    // ne pas détourner les touches quand un contrôle a le focus (Espace = activer le bouton, etc.)
    if (["input", "textarea", "select", "button", "a"].includes(tag)) return;
    if (e.key >= "1" && e.key <= "6") { armedAxis = +e.key; renderArmed(); return; }
    let step = stepVal(); if (e.shiftKey) step = 1; if (e.ctrlKey) step = stepVal() * 5;
    if (e.key === "ArrowUp" || e.key === "ArrowRight") { nudge(armedAxis, step); e.preventDefault(); }
    else if (e.key === "ArrowDown" || e.key === "ArrowLeft") { nudge(armedAxis, -step); e.preventDefault(); }
    else if (e.key === " ") { const [lo, hi] = lim(1); const v = gripOpen ? lo : hi; gripOpen = !gripOpen;
      setpoint[1] = v; stamp(1); $("sl-1").value = v; setSetpoint(1); renderArm(); post("/move", { targets: { 1: v }, duration_ms: durVal() }); e.preventDefault(); }
    else if (e.key.toLowerCase() === "h") post("/home");
    else if (e.key === "Escape") post("/stop");
  });
  // navigation clavier des listes poses / trajectoires
  listKeyNav($("pose-list"), () => latest && latest.poses, (n) => selPose = n,
    () => { if (selPose) post("/poses/" + encodeURIComponent(selPose) + "/goto", { duration_ms: durVal() }); });
  listKeyNav($("rec-list"), () => latest && latest.recordings, (n) => selRec = n,
    () => post("/replay", selRec ? { name: selRec } : {}));
}

function listKeyNav(ul, itemsFn, setSel, activate) {
  ul.addEventListener("keydown", (e) => {
    const items = itemsFn() || []; if (!items.length) return;
    const cur = [...ul.children].findIndex(li => li.classList.contains("sel"));
    if (e.key === "ArrowDown" || e.key === "ArrowUp") {
      let i = cur + (e.key === "ArrowDown" ? 1 : -1);
      i = Math.max(0, Math.min(items.length - 1, i < 0 ? 0 : i));
      setSel(items[i]); renderLists(latest); e.preventDefault();
    } else if (e.key === "Enter") { activate(); e.preventDefault(); }
  });
}

/* ---------------- Boucle ---------------- */
async function poll() {
  try { const st = await fetch("/state").then(r => r.json()); if (st.seq !== lastSeq) { lastSeq = st.seq; onState(st); } }
  catch (_) {}
  setTimeout(poll, interacting || (latest && (latest.recording || latest.replaying || latest.playing)) ? 150 : 600);
}

window.addEventListener("pagehide", () => { try { navigator.sendBeacon("/shutdown"); } catch (_) {} });

buildStrips(); wireControls(); wireTheme(); wireKeyboard(); renderArmed(); renderArm(); renderCompass(); poll();
setTimeout(() => document.documentElement.removeAttribute("data-boot"), 1200);
