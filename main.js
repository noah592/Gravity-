(() => {
  // --- Config ---
  const WORLD_SIZE = 10000;          // 10k x 10k world
  let   DENSITY = 0.0008;            // fraction of filled cells (0..1), mutable via HUD
  const D_MIN = 1e-7, D_MAX = 0.1;   // density clamps
  const D_STEP = 1.1;                // small step (×/÷) for density buttons

  const ZOOM_STEP = 1.1;             // wheel zoom factor per notch
  const MAX_SCALE = 200;             // max pixels per world unit (max zoom-in)
  const MIN_SCALE = 3;               // min pixels per world unit (max zoom-out = 3 px/unit)

  // Gravity / physics knobs (world units: 1 unit = 1 grid cell)
  const GRAVITY_ZOOM_GATE = 1.5;     // simulate when scale >= this (px/unit)
  let   G = 230;                     // default gravitational constant (mutable)
  const G_MIN = 1e-6, G_MAX = 1e6;   // HUD bounds
  const G_STEP = 1.1;                // small step (×/÷) for G buttons
  const SOFTENING2 = 1e-4;           // epsilon^2 to avoid singularities
  const MAX_DT = 1 / 60;             // maximum physics dt (seconds)

  // Barnes–Hut (quadtree) parameters
  const THETA = 0.7;                 // opening angle (smaller = more accurate)
  const LEAF_CAPACITY = 1;           // (kept simple)

  // Visual base radius; final rWorld = BASE_R_WORLD * sizeMul
  const BASE_R_WORLD = 0.45;

  // Size variation (deterministic per cell), controlled via:
  // - SIZE_VAR_MIN / SIZE_VAR_MAX: multipliers on BASE_R_WORLD
  // - SIZE_BIAS: exponent k for u' = u^k (k>1 biases smaller)
  let   SIZE_VAR_MIN = 0.5 / BASE_R_WORLD;      // default min radius = 0.5 units
  let   SIZE_VAR_MAX = 3.0 / BASE_R_WORLD;      // default max radius = 3.0 units
  const S_MIN_MIN = 0.05 / BASE_R_WORLD;        // floor (0.05 u)
  const S_MIN_MAX = 15.0 / BASE_R_WORLD;        // ceiling for min (must stay <= max)
  const S_MAX_MIN = 0.1  / BASE_R_WORLD;        // floor for max (0.1 u)
  const S_MAX_MAX = 20.0 / BASE_R_WORLD;        // ceiling (20 u)
  const S_STEP    = 1.1;                         // small step (×/÷) for size buttons

  let   SIZE_BIAS = 2.0;             // default bias k (>1 = smaller)
  const BIAS_MIN = 0.2, BIAS_MAX = 10.0;
  const BIAS_STEP = 1.1;             // small step (×/÷)

  // Initial velocity upper bound (for first activation only)
  let   V0_MAX = 100.0;              // default V0 cap (units/s), mutable
  const V0_MIN = 0.1, V0_MAX_CAP = 5000.0;
  const V0_STEP = 1.1;               // small step (×/÷)

  // Manual control (thrusters) — acceleration magnitude in units/s^2
  const CONTROL_ACCEL = 10.0;

  // --- Canvas / HUD ---
  const canvas = document.getElementById('view');
  const ctx = canvas.getContext('2d', { alpha: false, desynchronized: true });
  const hudZ = document.getElementById('zv');
  const hudV = document.getElementById('vv');

  const gVal = document.getElementById('gval');
  const gMinusBtn = document.getElementById('gMinus');
  const gPlusBtn  = document.getElementById('gPlus');

  const vmaxVal = document.getElementById('vmaxVal');
  const vmaxMinusBtn = document.getElementById('vmaxMinus');
  const vmaxPlusBtn  = document.getElementById('vmaxPlus');

  const rmaxVal = document.getElementById('rmaxVal');
  const rmaxMinusBtn = document.getElementById('rmaxMinus');
  const rmaxPlusBtn  = document.getElementById('rmaxPlus');

  const rminVal = document.getElementById('rminVal');
  const rminMinusBtn = document.getElementById('rminMinus');
  const rminPlusBtn  = document.getElementById('rminPlus');

  const dVal = document.getElementById('dVal');
  const dMinusBtn = document.getElementById('dMinus');
  const dPlusBtn  = document.getElementById('dPlus');

  const biasVal = document.getElementById('biasVal');
  const biasMinusBtn = document.getElementById('biasMinus');
  const biasPlusBtn  = document.getElementById('biasPlus');

  const followVal = document.getElementById('followVal');

  // --- View state ---
  let scale = 5;      // starting zoom: 5 px/unit
  let viewX = 0;      // world x at screen left
  let viewY = 0;      // world y at screen top

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

  // --- Deterministic hash for randomness (stable per cell) ---
  function hash2D(x, y) {
    let n = Math.imul(x | 0, 374761393) ^ Math.imul(y | 0, 668265263);
    n = Math.imul(n ^ (n >>> 13), 1274126177);
    n = (n ^ (n >>> 16)) >>> 0;
    return n / 4294967295; // [0,1)
  }

  function sizeMulAt(x, y) {
    const u = hash2D(x + 9173, y + 12345);
    const ub = Math.pow(u, SIZE_BIAS); // bias toward small when k>1
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
  /** @type {Map<string, {id:string,x:number,y:number,vx:number,vy:number,m:number,rWorld:number,sources:string[]}>} */
  const bodies = new Map();
  const activatedCells = new Set(); // "c:x,y" cells that spawned bodies
  const consumedCells  = new Set(); // "c:x,y" cells merged away (never respawn)

  let lastSimTime = null;   // for physics dt
  let lastFrameTime = null; // for UI/control dt
  let mergeIdCounter = 1;   // for merged-body IDs

  // Selection & control
  let selectedId = null;
  let keyW = false, keyA = false, keyS = false, keyD = false;
  let followSelected = false;

  function simActive() {
    return scale >= GRAVITY_ZOOM_GATE;
  }

  function fmtG(g) {
    if (g >= 0.001 && g < 1000) return g.toFixed(3);
    return g.toExponential(2);
  }
  function fmtV0(x) {
    if (x < 10) return x.toFixed(2);
    if (x < 1000) return x.toFixed(1);
    return x.toExponential(1);
  }
  function fmtR(valUnits) {
    return valUnits.toFixed(valUnits < 10 ? 2 : 1);
  }
  function fmtDensity(d) {
    return d < 0.001 ? d.toExponential(2) : d.toFixed(5);
  }
  function fmtBias(k) { return k.toFixed(2); }

  function cellKey(x, y) { return `c:${x},${y}`; }

  function addCellBodyIfNeeded(x, y) {
    const key = cellKey(x, y);
    if (consumedCells.has(key)) return;
    if (activatedCells.has(key)) return;
    if (hash2D(x, y) >= DENSITY) return;

    // Deterministic per-cell size & mass (uses current controls)
    const sMul = sizeMulAt(x, y);
    const rWorld = BASE_R_WORLD * sMul;
    const mass = Math.pow(sMul, 3);

    // Initial velocity (deterministic): speed in [1, V0_MAX]
    const ang = 2 * Math.PI * hash2D(x + 4242, y + 7777);
    const speed = 1 + (V0_MAX - 1) * hash2D(x + 31415, y + 2718);
    const vx0 = Math.cos(ang) * speed;
    const vy0 = Math.sin(ang) * speed;

    bodies.set(key, {
      id: key,
      x: x + 0.5,
      y: y + 0.5,
      vx: vx0,
      vy: vy0,
      m: mass,
      rWorld,
      sources: [key]
    });
    activatedCells.add(key);
  }

  function mergeBodies(bi, bj) {
    const mSum = bi.m + bj.m;
    const xNew = (bi.x * bi.m + bj.x * bj.m) / mSum;
    const yNew = (bi.y * bi.m + bj.y * bj.m) / mSum;
    const vxNew = (bi.vx * bi.m + bj.vx * bj.m) / mSum;
    const vyNew = (bi.vy * bi.m + bj.vy * bj.m) / mSum;

    const sources = bi.sources.concat(bj.sources);
    for (const s of sources) consumedCells.add(s);

    const mNew = mSum;
    const sizeFactorNew = Math.cbrt(mNew); // m ∝ s^3 => s = m^(1/3)
    const rWorldNew = BASE_R_WORLD * sizeFactorNew;

    const id = `m:${mergeIdCounter++}`;
    const merged = { id, x: xNew, y: yNew, vx: vxNew, vy: vyNew, m: mNew, rWorld: rWorldNew, sources };

    if (selectedId === bi.id || selectedId === bj.id) selectedId = id;

    bodies.delete(bi.id);
    bodies.delete(bj.id);
    bodies.set(id, merged);

    return merged;
  }

  // --- Barnes–Hut quadtree ---
  function buildQuadTree(activeBodies) {
    if (activeBodies.length === 0) return null;

    // Compute bounding square
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const b of activeBodies) {
      if (b.x < minX) minX = b.x;
      if (b.y < minY) minY = b.y;
      if (b.x > maxX) maxX = b.x;
      if (b.y > maxY) maxY = b.y;
    }
    const w = Math.max(1e-6, maxX - minX);
    const h = Math.max(1e-6, maxY - minY);
    const size = Math.max(w, h) * 1.0001;
    const cx = (minX + maxX) * 0.5;
    const cy = (minY + maxY) * 0.5;
    const half = size * 0.5;

    function makeNode(cx, cy, half) {
      return {
        cx, cy, half,
        m: 0, mx: 0, my: 0,
        body: null,
        children: null,
        count: 0
      };
    }

    const root = makeNode(cx, cy, half);

    function insert(node, b) {
      node.count++;
      node.m  += b.m;
      node.mx += b.m * b.x;
      node.my += b.m * b.y;

      if (node.children === null) {
        if (node.body === null) {
          node.body = b;
        } else {
          const old = node.body;
          node.body = null;
          node.children = split(node);
          insert(childFor(node, old), old);
          insert(childFor(node, b),   b);
        }
      } else {
        insert(childFor(node, b), b);
      }
    }

    function split(node) {
      const h2 = node.half * 0.5;
      const cx = node.cx, cy = node.cy;
      return [
        makeNode(cx - h2, cy - h2, h2), // NW
        makeNode(cx + h2, cy - h2, h2), // NE
        makeNode(cx - h2, cy + h2, h2), // SW
        makeNode(cx + h2, cy + h2, h2)  // SE
      ];
    }

    function childFor(node, b) {
      const east = b.x >= node.cx;
      const south = b.y >= node.cy;
      if (!east && !south) return node.children[0];
      if ( east && !south) return node.children[1];
      if (!east &&  south) return node.children[2];
      return node.children[3];
    }

    for (const b of activeBodies) insert(root, b);

    function finalize(node) {
      if (node.count > 0) {
        node.comX = node.mx / node.m;
        node.comY = node.my / node.m;
      } else {
        node.comX = node.cx;
        node.comY = node.cy;
      }
      if (node.children) for (const c of node.children) finalize(c);
    }
    finalize(root);
    return root;
  }

  function accumulateForceFromTree(node, b, axay) {
    if (!node || node.count === 0) return;
    if (node.children === null && node.body === b) return;

    const dx = node.comX - b.x;
    const dy = node.comY - b.y;
    const dist2 = dx*dx + dy*dy + SOFTENING2;
    const dist = Math.sqrt(dist2);

    if (node.children === null || (node.half * 2) / dist < THETA) {
      const invR3 = 1 / (dist2 * dist);
      const f = G * node.m * invR3;
      axay.ax += f * dx;
      axay.ay += f * dy;
    } else {
      for (const c of node.children) accumulateForceFromTree(c, b, axay);
    }
  }

  function simulate(dt, viewBounds) {
    const { x0, y0, x1, y1 } = viewBounds;
    const active = [];
    for (const b of bodies.values()) {
      if (b.x + b.rWorld < x0 || b.y + b.rWorld < y0 ||
          b.x - b.rWorld > x1 || b.y - b.rWorld > y1) continue;
      active.push(b);
    }

    const n = active.length;
    if (n === 0) return;

    const root = buildQuadTree(active);

    const ax = new Float32Array(n);
    const ay = new Float32Array(n);

    for (let i = 0; i < n; i++) {
      const b = active[i];
      const acc = { ax: 0, ay: 0 };
      accumulateForceFromTree(root, b, acc);
      ax[i] = acc.ax;
      ay[i] = acc.ay;
    }

    // Add manual control acceleration (thrusters) for the selected body
    let ctrlDX = 0, ctrlDY = 0;
    if (keyA) ctrlDX -= 1;
    if (keyD) ctrlDX += 1;
    if (keyW) ctrlDY -= 1;
    if (keyS) ctrlDY += 1;
    let ctrlAX = 0, ctrlAY = 0;
    if (ctrlDX !== 0 || ctrlDY !== 0) {
      const invLen = 1 / Math.hypot(ctrlDX, ctrlDY);
      ctrlDX *= invLen; ctrlDY *= invLen;
      ctrlAX = CONTROL_ACCEL * ctrlDX;
      ctrlAY = CONTROL_ACCEL * ctrlDY;
    }

    // Integrate (semi-implicit Euler)
    for (let i = 0; i < n; i++) {
      const b = active[i];
      // Apply control accel only to the selected body
      const addAX = (selectedId && b.id === selectedId) ? ctrlAX : 0;
      const addAY = (selectedId && b.id === selectedId) ? ctrlAY : 0;

      b.vx += (ax[i] + addAX) * dt;
      b.vy += (ay[i] + addAY) * dt;
      b.x  += b.vx * dt;
      b.y  += b.vy * dt;
    }

    // Collision & merge
    for (let i = 0; i < n; i++) {
      const bi = active[i];
      if (!bodies.has(bi.id)) continue;
      for (let j = i + 1; j < n; j++) {
        const bj = active[j];
        if (!bodies.has(bj.id)) continue;
        if (bi.id === bj.id) continue;
        const dx = bj.x - bi.x;
        const dy = bj.y - bi.y;
        const sumR = bi.rWorld + bj.rWorld;
        if (dx*dx + dy*dy <= sumR*sumR) {
          const merged = mergeBodies(bi, bj);
          active[i] = merged;
        }
      }
    }
  }

  function render() {
    if (!needsRender) return;
    needsRender = false;

    const w = canvas.width, h = canvas.height;

    // Clear to black
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, w, h);

    // Visible world-cell bounds
    const x0 = Math.max(0, Math.floor(viewX));
    const y0 = Math.max(0, Math.floor(viewY));
    const x1 = Math.min(WORLD_SIZE, Math.ceil(viewX + w / scale));
    const y1 = Math.min(WORLD_SIZE, Math.ceil(viewY + h / scale));
    const viewBounds = { x0, y0, x1, y1 };

    // Timing
    const now = performance.now() * 0.001; // seconds
    let dtFrame = 0;
    if (lastFrameTime == null) {
      lastFrameTime = now;
    } else {
      dtFrame = Math.min(0.1, Math.max(0, now - lastFrameTime));
      lastFrameTime = now;
    }

    const active = simActive();

    // Spawn visible (when active)
    if (active) {
      for (let y = y0; y < y1; y++) {
        for (let x = x0; x < x1; x++) addCellBodyIfNeeded(x, y);
      }
    } else {
      lastSimTime = null;
    }

    // Physics step
    let dt = 0;
    if (active) {
      if (lastSimTime == null) lastSimTime = now;
      else {
        dt = Math.min(MAX_DT, Math.max(0, now - lastSimTime));
        lastSimTime = now;
      }
      if (dt > 0) simulate(dt, viewBounds);
    } else {
      // Even if gravity is off, allow manual thrusters to work on selected body
      if (selectedId && (keyW || keyA || keyS || keyD)) {
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
    }

    // Camera follow (after motion)
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

    // Draw bodies (filled white)
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

    // Highlight selected with blue stroke
    if (selectedId) {
      const b = bodies.get(selectedId);
      if (b) {
        const cx = (b.x - viewX) * scale;
        const cy = (b.y - viewY) * scale;
        const rpx = b.rWorld * scale;
        if (!(cx + rpx < 0 || cy + rpx < 0 || cx - rpx > w || cy - rpx > h)) {
          ctx.beginPath();
          ctx.arc(cx, cy, rpx, 0, Math.PI * 2);
          ctx.lineWidth = Math.max(2, 1.5 * (window.devicePixelRatio || 1));
          ctx.strokeStyle = '#33aaff';
          ctx.stroke();
        }
      } else {
        selectedId = null;
      }
    }

    // HUD
    hudZ.textContent = `${scale.toFixed(3)} px/unit`;
    hudV.textContent = `x:[${x0}..${x1}] y:[${y0}..${y1}]`;
    gVal.textContent = fmtG(G);
    vmaxVal.textContent = fmtV0(V0_MAX);
    rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
    rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
    dVal.textContent = fmtDensity(DENSITY);
    biasVal.textContent = fmtBias(SIZE_BIAS);
    followVal.textContent = followSelected ? 'On' : 'Off';
  }

  // --- Sizing ---
  function resizeCanvas() {
    const dpr = Math.max(1, Math.min(3, window.devicePixelRatio || 1));
    canvas.width  = Math.floor(canvas.clientWidth  * dpr);
    canvas.height = Math.floor(canvas.clientHeight * dpr);
    ctx.imageSmoothingEnabled = false;
    clampView();
    requestRender();
  }

  // --- Picking ---
  function pickBodyAt(worldX, worldY) {
    let best = null;
    let bestD2 = Infinity;
    for (const b of bodies.values()) {
      const dx = worldX - b.x;
      const dy = worldY - b.y;
      const d2 = dx*dx + dy*dy;
      if (d2 <= b.rWorld * b.rWorld && d2 < bestD2) {
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
      const CLICK_TOL2 = 25; // ~5px^2
      if (moved2 <= CLICK_TOL2 && e.button === 0) {
        const rect = canvas.getBoundingClientRect();
        const mx = (e.clientX - rect.left) * (canvas.width  / canvas.clientWidth);
        const my = (e.clientY - rect.top ) * (canvas.height / canvas.clientHeight);
        const worldX = viewX + (mx / scale);
        const worldY = viewY + (my / scale);
        const id = pickBodyAt(worldX, worldY);
        selectedId = id; // null if none
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

    if (simActive()) lastSimTime = null;
  }, { passive: false });

  // --- Keyboard control for selected body & follow toggle ---
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
    vmaxVal.textContent = fmtV0(V0_MAX);
    rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
    rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
    dVal.textContent = fmtDensity(DENSITY);
    biasVal.textContent = fmtBias(SIZE_BIAS);
    followVal.textContent = followSelected ? 'On' : 'Off';
    requestRender();
  })();

  // Animation driver:
  (function tick() {
    if (dragging || simActive() || selectedId || followSelected) requestRender();
    requestAnimationFrame(tick);
  })();
})();
