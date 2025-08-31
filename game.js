// game.js — rendering, input, HUD, spawning; imports physics
import { stepPhysics, SOFT2_SQRT } from './physics.js';

// --- Config ---
const WORLD_SIZE = 1_000_000;
let   DENSITY = 0.0045;
const D_MIN = 1e-7, D_MAX = 0.03;
const D_STEP = 1.1;

const ZOOM_STEP = 1.1;
const MAX_SCALE = 200;
const MIN_SCALE = 1;

const GRAVITY_ZOOM_GATE = 1; // gravity active from 1 px/unit
let   G = 10;                 // default gravity (was 15)
const G_MIN = 1e-6, G_MAX = 1e6;
const G_STEP = 1.1;

let   ALPHA = 0.01;
const ALPHA_MIN = 1e-6, ALPHA_MAX = 1.0;
const ALPHA_STEP = 1.5;

const THETA = 1.0;

// Visual radius base; rWorld = BASE_R_WORLD * sizeFactor
const BASE_R_WORLD = 0.45;

// Size variation (deterministic)
let   SIZE_VAR_MIN = 0.31 / BASE_R_WORLD;
let   SIZE_VAR_MAX = 1.50 / BASE_R_WORLD;
const S_MIN_MIN = 0.05 / BASE_R_WORLD;
const S_MIN_MAX = 15.0 / BASE_R_WORLD;
const S_MAX_MIN = 0.1  / BASE_R_WORLD;
const S_MAX_MAX = 20.0 / BASE_R_WORLD;
const S_STEP    = 1.1;

let   SIZE_BIAS = 7.0;
const BIAS_MIN = 0.2, BIAS_MAX = 10.0;
const BIAS_STEP = 1.1;

// Initial velocity cap (spawn-time only)
let   V0_MAX = 7.0;
const V0_MIN = 0.1, V0_MAX_CAP = 5000.0;
const V0_STEP = 1.1;

// Manual control accel (WASD)
const CONTROL_ACCEL = 25.0;

// Picking: minimum clickable area
const MIN_CLICK_AREA = 5;
const MIN_CLICK_RADIUS = Math.sqrt(MIN_CLICK_AREA / Math.PI);

// --- Spatial hashing params (used inside physics)
const BUCKET_SIZE = 2.0;

// --- Canvas / HUD ---
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d', { alpha: false, desynchronized: true });
const hudZ = document.getElementById('zv');
const hudV = document.getElementById('vv');
const perfVal = document.getElementById('perfVal');

const gVal        = document.getElementById('gval');
const gMinusBtn   = document.getElementById('gMinus');
const gPlusBtn    = document.getElementById('gPlus');

const alphaVal      = document.getElementById('alphaVal');
const alphaMinusBtn = document.getElementById('alphaMinus');
const alphaPlusBtn  = document.getElementById('alphaPlus');

const vmaxVal     = document.getElementById('vmaxVal');
const vmaxMinusBtn= document.getElementById('vmaxMinus');
const vmaxPlusBtn = document.getElementById('vmaxPlus');

const rmaxVal     = document.getElementById('rmaxVal');
const rmaxMinusBtn= document.getElementById('rmaxMinus');
const rmaxPlusBtn = document.getElementById('rmaxPlus');

const rminVal     = document.getElementById('rminVal');
const rminMinusBtn= document.getElementById('rminMinus');
const rminPlusBtn = document.getElementById('rminPlus');

const dVal        = document.getElementById('dVal');
const dMinusBtn   = document.getElementById('dMinus');
const dPlusBtn    = document.getElementById('dPlus');

const biasVal     = document.getElementById('biasVal');
const biasMinusBtn= document.getElementById('biasMinus');
const biasPlusBtn = document.getElementById('biasPlus');

const followVal   = document.getElementById('followVal');

// --- View state ---
let scale = 2; // starting zoom (px/unit)
let viewX = 0;
let viewY = 0;

function viewportWorldWidth ()  { return canvas.width  / scale; }
function viewportWorldHeight()  { return canvas.height / scale; }

function clampView() {
  const vw = viewportWorldWidth();
  const vh = viewportWorldHeight();
  const maxX = Math.max(0, WORLD_SIZE - vw);
  const maxY = Math.max(0, WORLD_SIZE - vh);
  viewX = Math.min(Math.max(0, viewX), maxX);
  viewY = Math.min(Math.max(0, viewY), maxY);
}

// --- Hashing ---
function hash2D_u32(x, y) {
  let n = Math.imul(x | 0, 374761393) ^ Math.imul(y | 0, 668265263);
  n = Math.imul(n ^ (n >>> 13), 1274126177);
  return (n ^ (n >>> 16)) >>> 0;
}
function hash01(x, y) { return hash2D_u32(x, y) / 4294967296; }

// Numeric cell key for spawned cells
function nkey(x, y) { return x * 1_000_000 + y; }

// Density threshold for integer compare
let DENSITY_THR = Math.floor(DENSITY * 4294967296);

function sizeMulAt(x, y) {
  const u = hash01(x + 9173, y + 12345);
  const ub = Math.pow(u, SIZE_BIAS);
  return SIZE_VAR_MIN + (SIZE_VAR_MAX - SIZE_VAR_MIN) * ub;
}

// --- Render / Simulation scheduling ---
let needsRender = false;
function requestRender() {
  if (!needsRender) {
    needsRender = true;
    requestAnimationFrame(render);
  }
}

// Persistent universe state
/** @type {Map<string, {id:string,x:number,y:number,vx:number,vy:number,m:number,rWorld:number,rRef:number,sources:number[]}>} */
const bodies = new Map();

const activatedCells = new Set();
const consumedCells  = new Set();

let lastSimTime = null;
let lastFrameTime = null;

// Perf metrics (ms)
let lastTreeBuildMs = 0.0;
let lastGravMs = 0.0;
let lastSpawnMs = 0.0;
let lastCollideMs = 0.0;
let lastIntegrateMs = 0.0;
let lastRenderMs = 0.0;
let lastTotalMs = 0.0;
let hudCounter = 0;

// Selection & control
let selectedId = null;
let keyW = false, keyA = false, keyS = false, keyD = false;
let followSelected = false;

function simActive() { return scale >= GRAVITY_ZOOM_GATE; }

function fmtG(g) { return (g >= 0.001 && g < 1000) ? g.toFixed(3) : g.toExponential(2); }
function fmtV0(x) { return (x < 10) ? x.toFixed(2) : (x < 1000 ? x.toFixed(1) : x.toExponential(1)); }
function fmtR(valUnits) { return valUnits.toFixed(valUnits < 10 ? 2 : 1); }
function fmtDensity(d) { return d < 0.001 ? d.toExponential(2) : d.toFixed(5); }
function fmtBias(k) { return k.toFixed(2); }
function fmtAlpha(a) {
  if (a >= 0.001 && a <= 1.0) return (a * 100).toFixed(2) + '%';
  return a.toExponential(2);
}

function cellId(x, y) { return `c:${x},${y}`; }

// Spawn helper with early-out by density
function addCellBodyIfNeeded(x, y) {
  if (hash2D_u32(x, y) >= DENSITY_THR) return;
  const nk = nkey(x, y);
  if (consumedCells.has(nk)) return;
  if (activatedCells.has(nk)) return;

  const sMul = sizeMulAt(x, y);
  const rWorld = BASE_R_WORLD * sMul;
  const mass = Math.pow(sMul, 2); // m ∝ s^2

  const ang = 2 * Math.PI * hash01(x + 4242, y + 7777);
  const speed = 1 + (V0_MAX - 1) * hash01(x + 31415, y + 2718);
  const vx0 = Math.cos(ang) * speed;
  const vy0 = Math.sin(ang) * speed;

  const id = cellId(x, y);
  bodies.set(id, {
    id,
    x: x + 0.5,
    y: y + 0.5,
    vx: vx0,
    vy: vy0,
    m: mass,
    rWorld,
    rRef: Math.max(rWorld, SOFT2_SQRT),
    sources: [nk]
  });
  activatedCells.add(nk);
}

// --- Spawn deltas (only newly revealed cells) ---
let prevX0 = null, prevY0 = null, prevX1 = null, prevY1 = null;

function spawnRect(x0, x1, y0, y1) {
  if (x0 >= x1 || y0 >= y1) return;
  for (let y = y0; y < y1; y++) {
    for (let x = x0; x < x1; x++) addCellBodyIfNeeded(x, y);
  }
}
function spawnNewlyRevealed(curr) {
  if (prevX0 === null) {
    spawnRect(curr.x0, curr.x1, curr.y0, curr.y1);
    prevX0 = curr.x0; prevY0 = curr.y0; prevX1 = curr.x1; prevY1 = curr.y1;
    return;
  }
  if (curr.x0 === prevX0 && curr.y0 === prevY0 && curr.x1 === prevX1 && curr.y1 === prevY1) return;

  const ix0 = Math.max(curr.x0, prevX0);
  const ix1 = Math.min(curr.x1, prevX1);
  const iy0 = Math.max(curr.y0, prevY0);
  const iy1 = Math.min(curr.y1, prevY1);

  if (curr.x0 < prevX0) spawnRect(curr.x0, Math.min(prevX0, curr.x1), curr.y0, curr.y1);
  if (curr.x1 > prevX1) spawnRect(Math.max(prevX1, curr.x0), curr.x1, curr.y0, curr.y1);
  if (curr.y0 < prevY0 && ix0 < ix1) spawnRect(ix0, ix1, curr.y0, Math.min(prevY0, curr.y1));
  if (curr.y1 > prevY1 && ix0 < ix1) spawnRect(ix0, ix1, Math.max(prevY1, curr.y0), curr.y1);

  prevX0 = curr.x0; prevY0 = curr.y0; prevX1 = curr.x1; prevY1 = curr.y1;
}

function render() {
  if (!needsRender) return;
  needsRender = false;

  const t0 = performance.now();

  const w = canvas.width, h = canvas.height;

  // Clear
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, w, h);

  // Visible bounds
  const x0 = Math.max(0, Math.floor(viewX));
  const y0 = Math.max(0, Math.floor(viewY));
  const x1 = Math.min(WORLD_SIZE, Math.ceil(viewX + w / scale));
  const y1 = Math.min(WORLD_SIZE, Math.ceil(viewY + h / scale));
  const viewBounds = { x0, y0, x1, y1 };

  // Frame dt (for off-gravity thrusters)
  const now = performance.now() * 0.001;
  let dtFrame = 0;
  if (lastFrameTime == null) lastFrameTime = now;
  else { dtFrame = Math.min(0.1, Math.max(0, now - lastFrameTime)); lastFrameTime = now; }

  const active = simActive();

  // Spawn only newly revealed cells
  if (active) {
    const tSpawn0 = performance.now();
    spawnNewlyRevealed(viewBounds);
    const tSpawn1 = performance.now();
    lastSpawnMs = tSpawn1 - tSpawn0;
  } else {
    lastSimTime = null;
    lastTreeBuildMs = 0;
    lastGravMs = 0;
    lastIntegrateMs = 0;
    lastCollideMs = 0;
    lastSpawnMs = 0;
    prevX0 = prevY0 = prevX1 = prevY1 = null;
  }

  // Physics step
  let dt = 0;
  if (active) {
    const nowS = performance.now() * 0.001;
    if (lastSimTime == null) lastSimTime = nowS;
    else { dt = Math.min(1/60, Math.max(0, nowS - lastSimTime)); lastSimTime = nowS; }
    if (dt > 0) {
      const controls = { selectedId, keyW, keyA, keyS, keyD, controlAccel: CONTROL_ACCEL };
      const params = {
        G, alpha: ALPHA, theta: THETA,
        worldSize: WORLD_SIZE, bucketSize: BUCKET_SIZE, baseRWorld: BASE_R_WORLD
      };
      const res = stepPhysics(bodies, dt, viewBounds, controls, params);
      selectedId = res.selectedId;
      lastTreeBuildMs = res.treeBuildMs;
      lastGravMs = res.gravMs;
      lastIntegrateMs = res.integrateMs;
      lastCollideMs = res.collideMs;
    }
  } else if (selectedId && (keyW || keyA || keyS || keyD)) {
    // Thrusters while gravity is inactive (kept as before)
    const b = bodies.get(selectedId);
    if (b) {
      let dx = 0, dy = 0;
      if (keyA) dx -= 1;
      if (keyD) dx += 1;
      if (keyW) dy -= 1;
      if (keyS) dy += 1;
      if (dx !== 0 || dy !== 0) {
        const inv = 1 / Math.hypot(dx, dy);
        dx *= inv; dy *= inv;
        b.vx += CONTROL_ACCEL * dx * dtFrame;
        b.vy += CONTROL_ACCEL * dy * dtFrame;
        b.x  += b.vx * dtFrame;
        b.y  += b.vy * dtFrame;
      }
    }
  }

  // Camera follow
  if (followSelected && selectedId) {
    const b = bodies.get(selectedId);
    if (b) {
      const vw = viewportWorldWidth();
      const vh = viewportWorldHeight();
      viewX = b.x - vw / 2;
      viewY = b.y - vh / 2;
      clampView();
    }
  }

  // Draw bodies
  const tDraw0 = performance.now();
  ctx.beginPath();
  for (const b of bodies.values()) {
    const cx = (b.x - viewX) * scale;
    const cy = (b.y - viewY) * scale;
    const rpx = b.rWorld * scale;
    if (cx + rpx < 0 || cy + rpx < 0 || cx - rpx > w || cy - rpx > h) continue;
    ctx.moveTo(cx + rpx, cy);
    ctx.arc(cx, cy, rpx, 0, Math.PI * 2);
  }
  ctx.fillStyle = '#fff';
  ctx.fill();

  // Highlight selected
  if (selectedId) {
    const b = bodies.get(selectedId);
    if (b) {
      const cx = (b.x - viewX) * scale;
      const cy = (b.y - viewY) * scale;
      const rpx = b.rWorld * scale;
      if (!(cx + rpx < 0 || cy + rpx < 0 || cx - rpx > w || cy - rpx > h)) {
        ctx.beginPath();
        ctx.arc(cx, cy, rpx, 0, Math.PI * 2);
        ctx.lineWidth = Math.max(2, 4 * (window.devicePixelRatio || 1));
        ctx.strokeStyle = '#33aaff';
        ctx.stroke();
      }
    } else {
      selectedId = null;
    }
  }

  const tDraw1 = performance.now();
  lastRenderMs = tDraw1 - tDraw0;

  // HUD (throttled ~10Hz)
  const t1 = performance.now();
  lastTotalMs = t1 - t0;
  if ((hudCounter = (hudCounter + 1) % 6) === 0) {
    hudZ.textContent = `${scale.toFixed(3)} px/unit`;
    hudV.textContent = `x:[${x0}..${x1}] y:[${y0}..${y1}]`;
    gVal.textContent = fmtG(G);
    alphaVal.textContent = fmtAlpha(ALPHA);
    vmaxVal.textContent = fmtV0(V0_MAX);
    rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
    rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
    dVal.textContent = fmtDensity(DENSITY);
    biasVal.textContent = fmtBias(SIZE_BIAS);
    followVal.textContent = followSelected ? 'On' : 'Off';

    perfVal.textContent =
      `TreeB ${lastTreeBuildMs.toFixed(1)} ms | ` +
      `Grav ${lastGravMs.toFixed(1)} ms | ` +
      `Spawn ${lastSpawnMs.toFixed(1)} ms | ` +
      `Collide ${lastCollideMs.toFixed(1)} ms | ` +
      `Integr ${lastIntegrateMs.toFixed(1)} ms | ` +
      `Render ${lastRenderMs.toFixed(1)} ms | ` +
      `Total ${lastTotalMs.toFixed(1)} ms`;
  }
}

// --- Sizing ---
function resizeCanvas() {
  const dpr = Math.max(1, Math.min(3, window.devicePixelRatio || 1));
  canvas.width  = Math.floor(canvas.clientWidth  * dpr);
  canvas.height = Math.floor(canvas.clientHeight * dpr);
  ctx.imageSmoothingEnabled = false;
  clampView();
  requestRender();
  prevX0 = prevY0 = prevX1 = prevY1 = null; // force spawn next active frame
}

// --- Picking ---
function pickBodyAt(worldX, worldY) {
  let best = null;
  let bestD2 = Infinity;
  for (const b of bodies.values()) {
    const dx = worldX - b.x;
    const dy = worldY - b.y;
    const rPick = Math.max(b.rWorld, MIN_CLICK_RADIUS);
    const d2 = dx*dx + dy*dy;
    if (d2 <= rPick * rPick && d2 < bestD2) {
      bestD2 = d2;
      best = b;
    }
  }
  return best ? best.id : null;
}

// --- Input: pan & zoom & click-to-select ---
let dragging = false;
let lastX = 0, lastY = 0;
let downX = 0, downY = 0;

canvas.addEventListener('mousedown', (e) => {
  if (e.button === 0 || e.button === 2) {
    dragging = true;
    lastX = downX = e.clientX;
    lastY = downY = e.clientY;
    canvas.classList.add('dragging');
  }
});
window.addEventListener('mousemove', (e) => {
  if (!dragging) return;
  const dx = e.clientX - lastX;
  const dy = e.clientY - lastY;
  lastX = e.clientX;
  lastY = e.clientY;
  viewX -= dx / scale;
  viewY -= dy / scale;
  clampView();
  requestRender();
});
window.addEventListener('mouseup', (e) => {
  if (dragging) {
    canvas.classList.remove('dragging');
    const moved2 = (e.clientX - downX) ** 2 + (e.clientY - downY) ** 2;
    const CLICK_TOL2 = 25;
    if (moved2 <= CLICK_TOL2 && e.button === 0) {
      const rect = canvas.getBoundingClientRect();
      const mx = (e.clientX - rect.left) * (canvas.width  / canvas.clientWidth);
      const my = (e.clientY - rect.top ) * (canvas.height / canvas.clientHeight);
      const worldX = viewX + (mx / scale);
      const worldY = viewY + (my / scale);
      const id = pickBodyAt(worldX, worldY);
      selectedId = id;
      requestRender();
    }
  }
  dragging = false;
});
canvas.addEventListener('contextmenu', (e) => e.preventDefault());

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const { left, top } = canvas.getBoundingClientRect();
  const mouseX = (e.clientX - left) * (canvas.width  / canvas.clientWidth);
  const mouseY = (e.clientY - top ) * (canvas.height / canvas.clientHeight);

  const worldBeforeX = viewX + (mouseX / scale);
  const worldBeforeY = viewY + (mouseY / scale);

  const direction = Math.sign(e.deltaY);
  const factor = direction > 0 ? (1/ZOOM_STEP) : ZOOM_STEP;
  scale *= factor;
  scale = Math.min(Math.max(scale, MIN_SCALE), MAX_SCALE);

  viewX = worldBeforeX - (mouseX / scale);
  viewY = worldBeforeY - (mouseY / scale);
  clampView();
  requestRender();

  prevX0 = prevY0 = prevX1 = prevY1 = null;
  if (simActive()) lastSimTime = null;
}, { passive: false });

// --- Keyboard control & follow toggle ---
function handleKey(e, down) {
  const k = e.key.toLowerCase();
  if (k === 'w') { keyW = down; if (selectedId) e.preventDefault(); }
  else if (k === 'a') { keyA = down; if (selectedId) e.preventDefault(); }
  else if (k === 's') { keyS = down; if (selectedId) e.preventDefault(); }
  else if (k === 'd') { keyD = down; if (selectedId) e.preventDefault(); }
  else if (k === 'escape' && down) { selectedId = null; requestRender(); }
  else if (k === 'f' && down) { followSelected = !followSelected; followVal.textContent = followSelected ? 'On' : 'Off'; requestRender(); }
}
window.addEventListener('keydown', (e) => handleKey(e, true), { passive: false });
window.addEventListener('keyup',   (e) => handleKey(e, false), { passive: false });

// --- HUD controls ---
function updateG(newG) {
  G = Math.min(G_MAX, Math.max(G_MIN, newG));
  gVal.textContent = fmtG(G);
  requestRender();
}
gMinusBtn.addEventListener('click', () => updateG(G / G_STEP));
gPlusBtn .addEventListener('click', () => updateG(G * G_STEP));

function updateAlpha(newA) {
  ALPHA = Math.min(ALPHA_MAX, Math.max(ALPHA_MIN, newA));
  alphaVal.textContent = fmtAlpha(ALPHA);
  requestRender();
}
alphaMinusBtn.addEventListener('click', () => updateAlpha(ALPHA / ALPHA_STEP));
alphaPlusBtn .addEventListener('click', () => updateAlpha(ALPHA * ALPHA_STEP));

function updateV0Max(newMax) {
  V0_MAX = Math.min(V0_MAX_CAP, Math.max(V0_MIN, newMax));
  vmaxVal.textContent = fmtV0(V0_MAX);
}
vmaxMinusBtn.addEventListener('click', () => updateV0Max(V0_MAX / V0_STEP));
vmaxPlusBtn .addEventListener('click', () => updateV0Max(V0_MAX * V0_STEP));

function updateRmax(multiplierNew) {
  SIZE_VAR_MAX = Math.min(S_MAX_MAX, Math.max(S_MAX_MIN, multiplierNew));
  if (SIZE_VAR_MAX < SIZE_VAR_MIN) SIZE_VAR_MIN = SIZE_VAR_MAX;
  rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
  rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
  requestRender();
}
rmaxMinusBtn.addEventListener('click', () => updateRmax(SIZE_VAR_MAX / S_STEP));
rmaxPlusBtn .addEventListener('click', () => updateRmax(SIZE_VAR_MAX * S_STEP));

function updateRmin(multiplierNew) {
  SIZE_VAR_MIN = Math.min(S_MIN_MAX, Math.max(S_MIN_MIN, multiplierNew));
  if (SIZE_VAR_MIN > SIZE_VAR_MAX) SIZE_VAR_MAX = SIZE_VAR_MIN;
  rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
  rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
  requestRender();
}
rminMinusBtn.addEventListener('click', () => updateRmin(SIZE_VAR_MIN / S_STEP));
rminPlusBtn  .addEventListener('click', () => updateRmin(SIZE_VAR_MIN * S_STEP));

function updateDensity(newD) {
  DENSITY = Math.min(D_MAX, Math.max(D_MIN, newD));
  DENSITY_THR = Math.floor(DENSITY * 4294967296);
  dVal.textContent = fmtDensity(DENSITY);
  requestRender();
}
dMinusBtn.addEventListener('click', () => updateDensity(DENSITY / D_STEP));
dPlusBtn .addEventListener('click', () => updateDensity(DENSITY * D_STEP));

function updateBias(newK) {
  SIZE_BIAS = Math.min(BIAS_MAX, Math.max(BIAS_MIN, newK));
  biasVal.textContent = fmtBias(SIZE_BIAS);
  requestRender();
}
biasMinusBtn.addEventListener('click', () => updateBias(SIZE_BIAS / BIAS_STEP));
biasPlusBtn  .addEventListener('click', () => updateBias(SIZE_BIAS * BIAS_STEP));

// --- Bootstrap ---
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

(function initView() {
  const vw = viewportWorldWidth();
  const vh = viewportWorldHeight();
  viewX = (WORLD_SIZE - vw) / 2;
  viewY = (WORLD_SIZE - vh) / 2;
  clampView();

  gVal.textContent = fmtG(G);
  alphaVal.textContent = fmtAlpha(ALPHA);
  vmaxVal.textContent = fmtV0(V0_MAX);
  rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
  rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
  dVal.textContent = fmtDensity(DENSITY);
  biasVal.textContent = fmtBias(SIZE_BIAS);
  followVal.textContent = followSelected ? 'On' : 'Off';

  perfVal.textContent =
    `TreeB ${lastTreeBuildMs.toFixed(1)} ms | ` +
    `Grav ${lastGravMs.toFixed(1)} ms | ` +
    `Spawn ${lastSpawnMs.toFixed(1)} ms | ` +
    `Collide ${lastCollideMs.toFixed(1)} ms | ` +
    `Integr ${lastIntegrateMs.toFixed(1)} ms | ` +
    `Render ${lastRenderMs.toFixed(1)} ms | ` +
    `Total ${lastTotalMs.toFixed(1)} ms`;

  requestRender();
})();

// Animation driver:
(function tick() {
  if (dragging || simActive() || selectedId || followSelected) requestRender();
  requestAnimationFrame(tick);
})();
