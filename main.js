(() => {
  // ─────────────────────────────────────────────────────────────────────────────
  // 1) Config & constants
  // ─────────────────────────────────────────────────────────────────────────────
  const WORLD_SIZE = 1_000_000;
  let   DENSITY = 0.0045;
  const D_MIN = 1e-7, D_MAX = 0.03;
  const D_STEP = 1.1;

  const ZOOM_STEP = 1.1;
  const MAX_SCALE = 200;
  const MIN_SCALE = 1;

  // Gravity / physics
  const GRAVITY_ZOOM_GATE = 1;
  let   G = 35;
  const G_MIN = 1e-6, G_MAX = 1e6;
  const G_STEP = 1.1;
  const SOFTENING2 = 0.15;
  const MAX_DT = 1 / 60;

  // Barnes–Hut
  const THETA = 1.0;
  const LEAF_CAPACITY = 1; // (kept for reference; current insert splits immediately)

  // Visual base radius; baseline r = BASE_R_WORLD * sqrt(m)
  const BASE_R_WORLD = 0.45;

  // Size variation (deterministic per cell)
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

  // Spawn-time horizontal velocity band bias (kept from prior version)
  let   VBIAS_BAND_H = 250;    // world units per band (defaults you requested earlier)
  let   VBIAS_ADD    = 2.0;    // added vx amplitude (units/s)

  // Linear “crush” density rule (kept; safe defaults)
  let   CRUSH_K  = 0.0001;     // slope after threshold
  let   CRUSH_MC = 1000;        // mass where crush starts
  const CRUSH_K_MIN=0, CRUSH_K_MAX=1;
  const CRUSH_MC_MIN=0, CRUSH_MC_MAX=1e6;

  // Merge & collision response
  let   MERGE_ON    = true;    // HUD toggle should control this if present
  let   STICKY_THR  = 800;     // EMA collisions/sec threshold
  let   REST_E      = 0.2;     // coefficient of restitution
  let   FRICTION_MU = 0.02;    // tangential (Coulomb) friction at contact
  const STICKY_STEP = 1.2;     // generic steps for +/- buttons if present
  const REST_STEP   = 1.1;
  const FRICT_STEP  = 1.1;

  // EMA smoothing for collisions/sec
  const EMA_TAU = 0.35; // seconds (time-constant)

  // Picking
  const MIN_CLICK_AREA = 5;
  const MIN_CLICK_RADIUS = Math.sqrt(MIN_CLICK_AREA / Math.PI);

  // Spatial hashing for collisions (multi-bucket with canonical bucket)
  const BUCKET_SIZE = 2.0;
  const MAX_CX = Math.floor(WORLD_SIZE / BUCKET_SIZE);
  const MAX_CY = Math.floor(WORLD_SIZE / BUCKET_SIZE);
  const BUCKET_STRIDE = 2_000_000;
  const bkKey = (cx, cy) => cx * BUCKET_STRIDE + cy;
  const clampCx = (cx) => Math.min(MAX_CX, Math.max(0, cx));
  const clampCy = (cy) => Math.min(MAX_CY, Math.max(0, cy));

  // ─────────────────────────────────────────────────────────────────────────────
  // 2) Canvas / HUD lookups (all guarded)
  // ─────────────────────────────────────────────────────────────────────────────
  const canvas = document.getElementById('view');
  const ctx = canvas.getContext('2d', { alpha: false, desynchronized: true });

  const hudZ = document.getElementById('zv');
  const hudV = document.getElementById('vv');
  const perfVal = document.getElementById('perfVal');

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

  // Optional HUD items (kept if present; harmless if absent)
  const mergeVal      = document.getElementById('mergeVal');
  const mergeToggle   = document.getElementById('mergeToggle');
  const stickyVal     = document.getElementById('stickyVal');
  const stickyMinus   = document.getElementById('stickyMinus');
  const stickyPlus    = document.getElementById('stickyPlus');
  const restVal       = document.getElementById('restVal');
  const restMinus     = document.getElementById('restMinus');
  const restPlus      = document.getElementById('restPlus');
  const muVal         = document.getElementById('muVal');
  const muMinus       = document.getElementById('muMinus');
  const muPlus        = document.getElementById('muPlus');

  const mcVal         = document.getElementById('mcVal');
  const mcMinus       = document.getElementById('mcMinus');
  const mcPlus        = document.getElementById('mcPlus');
  const kVal          = document.getElementById('kVal');
  const kMinus        = document.getElementById('kMinus');
  const kPlus         = document.getElementById('kPlus');

  const bandVal       = document.getElementById('bandVal');
  const bandMinus     = document.getElementById('bandMinus');
  const bandPlus      = document.getElementById('bandPlus');
  const vbiasVal      = document.getElementById('vbiasVal');
  const vbiasMinus    = document.getElementById('vbiasMinus');
  const vbiasPlus     = document.getElementById('vbiasPlus');

  const emaVal        = document.getElementById('emaVal');     // selected body's EMA CPS (optional)

  // ─────────────────────────────────────────────────────────────────────────────
  // 3) Viewport state & helpers
  // ─────────────────────────────────────────────────────────────────────────────
  let scale = 2;      // starting zoom
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

  // ─────────────────────────────────────────────────────────────────────────────
  // 4) Hashing / randomness helpers
  // ─────────────────────────────────────────────────────────────────────────────
  function hash2D_u32(x, y) {
    let n = Math.imul(x | 0, 374761393) ^ Math.imul(y | 0, 668265263);
    n = Math.imul(n ^ (n >>> 13), 1274126177);
    return (n ^ (n >>> 16)) >>> 0;
  }
  function hash01(x, y) { return hash2D_u32(x, y) / 4294967296; }

  function nkey(x, y) { return x * 1_000_000 + y; }

  let DENSITY_THR = Math.floor(DENSITY * 4294967296);

  function sizeMulAt(x, y) {
    const u = hash01(x + 9173, y + 12345);
    const ub = Math.pow(u, SIZE_BIAS);
    return SIZE_VAR_MIN + (SIZE_VAR_MAX - SIZE_VAR_MIN) * ub;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 5) Render/sim scheduling
  // ─────────────────────────────────────────────────────────────────────────────
  let needsRender = false;
  function requestRender() {
    if (!needsRender) {
      needsRender = true;
      requestAnimationFrame(render);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 6) Persistent state (bodies & cells)
  // ─────────────────────────────────────────────────────────────────────────────
  /**
   * Body fields:
   *  id, x, y, vx, vy, m, rWorld, sources[], emaCps, collCount
   */
  /** @type {Map<string, {id:string,x:number,y:number,vx:number,vy:number,m:number,rWorld:number,sources:number[], emaCps:number, collCount:number}>} */
  const bodies = new Map();
  const activatedCells = new Set();
  const consumedCells  = new Set();

  let lastSimTime = null;
  let lastFrameTime = null;
  let mergeIdCounter = 1;

  // Perf metrics (ms)
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
  function fmtBool(b) { return b ? 'On' : 'Off'; }
  function fmtFloat(x) { return (Math.abs(x) < 0.01 || Math.abs(x) >= 1000) ? x.toExponential(2) : x.toFixed(2); }

  function cellId(x, y) { return `c:${x},${y}`; }

  // Mass→radius with linear crush (safe default)
  function radiusForMass(m) {
    if (m <= CRUSH_MC) return BASE_R_WORLD * Math.sqrt(m);
    const q = 1 + CRUSH_K * (m - CRUSH_MC);        // density multiplier grows linearly
    const r = BASE_R_WORLD * Math.sqrt(m / q);     // denser → smaller radius than sqrt(m)
    return Math.max(1e-6, r);
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 7) Spawning & merging
  // ─────────────────────────────────────────────────────────────────────────────
  function addCellBodyIfNeeded(x, y) {
    if (hash2D_u32(x, y) >= DENSITY_THR) return;
    const nk = nkey(x, y);
    if (consumedCells.has(nk)) return;
    if (activatedCells.has(nk)) return;

    const sMul = sizeMulAt(x, y);
    const mass = Math.pow(sMul, 2);

    // Deterministic orientation & speed
    const ang = 2 * Math.PI * hash01(x + 4242, y + 7777);
    const speed = 1 + (V0_MAX - 1) * hash01(x + 31415, y + 2718);
    let vx0 = Math.cos(ang) * speed;
    let vy0 = Math.sin(ang) * speed;

    // Add banded horizontal velocity bias
    if (VBIAS_BAND_H > 0 && isFinite(VBIAS_BAND_H)) {
      const phase = (2 * Math.PI * (y)) / VBIAS_BAND_H;
      vx0 += Math.sin(phase) * VBIAS_ADD;
    }

    const id = cellId(x, y);
    const rWorld = radiusForMass(mass);
    bodies.set(id, {
      id,
      x: x + 0.5,
      y: y + 0.5,
      vx: vx0,
      vy: vy0,
      m: mass,
      rWorld,
      sources: [nk],
      emaCps: 0,
      collCount: 0
    });
    activatedCells.add(nk);
  }

  function mergeBodies(bi, bj) {
    const mSum = bi.m + bj.m;
    const xNew = (bi.x * bi.m + bj.x * bj.m) / mSum;
    const yNew = (bi.y * bi.m + bj.y * bj.m) / mSum;
    const vxNew = (bi.vx * bi.m + bj.vx * bj.m) / mSum;
    const vyNew = (bi.vy * bi.m + bj.vy * bj.m) / mSum;

    const sources = bi.sources.concat(bj.sources);
    for (const s of sources) consumedCells.add(s);

    const rWorldNew = radiusForMass(mSum);
    const id = `m:${mergeIdCounter++}`;
    const merged = {
      id, x: xNew, y: yNew, vx: vxNew, vy: vyNew, m: mSum, rWorld: rWorldNew, sources,
      emaCps: (bi.emaCps * bi.m + bj.emaCps * bj.m) / (mSum || 1),
      collCount: 0
    };

    if (selectedId === bi.id || selectedId === bj.id) selectedId = id;

    bodies.delete(bi.id);
    bodies.delete(bj.id);
    bodies.set(id, merged);

    return merged;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 8) Barnes–Hut quadtree
  // ─────────────────────────────────────────────────────────────────────────────
  function buildQuadTree(activeBodies) {
    if (activeBodies.length === 0) return null;

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
      return { cx, cy, half, m: 0, mx: 0, my: 0, body: null, children: null, count: 0 };
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
          const old = node.body; node.body = null;
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
        node.comX = node.mx / (node.m || 1);
        node.comY = node.my / (node.m || 1);
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

  // ─────────────────────────────────────────────────────────────────────────────
  // 9) Collision response (bounce-only path)
  // ─────────────────────────────────────────────────────────────────────────────
  function resolveCollisionNoMerge(a, b) {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const dist = Math.hypot(dx, dy) || 1e-9;
    const nx = dx / dist;
    const ny = dy / dist;

    // Positional correction to avoid sinking
    const sumR = a.rWorld + b.rWorld;
    const overlap = sumR - dist;
    if (overlap > 0) {
      const invA = 1 / a.m;
      const invB = 1 / b.m;
      const invSum = invA + invB;
      const corr = overlap / (invSum || 1e-9);
      a.x -= nx * corr * invA;
      a.y -= ny * corr * invA;
      b.x += nx * corr * invB;
      b.y += ny * corr * invB;
    }

    // Relative velocity
    const rvx = b.vx - a.vx;
    const rvy = b.vy - a.vy;

    const vn = rvx * nx + rvy * ny;   // approach speed
    if (vn > 0) return;               // already separating

    const tx = -ny, ty = nx;
    const vt = rvx * tx + rvy * ty;

    const invA = 1 / a.m;
    const invB = 1 / b.m;
    const invSum = invA + invB;

    // Normal impulse with restitution
    const e = REST_E;
    const jn = -(1 + e) * vn / (invSum || 1e-9);
    const jnx = jn * nx, jny = jn * ny;
    a.vx -= jnx * invA; a.vy -= jny * invA;
    b.vx += jnx * invB; b.vy += jny * invB;

    // Tangential (Coulomb) friction impulse
    let jt = -vt / (invSum || 1e-9);
    const jtMax = FRICTION_MU * Math.abs(jn);
    if (jt >  jtMax) jt =  jtMax;
    if (jt < -jtMax) jt = -jtMax;
    const jtx = jt * tx, jty = jt * ty;
    a.vx -= jtx * invA; a.vy -= jty * invA;
    b.vx += jtx * invB; b.vy += jty * invB;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 10) Simulation (forces, integration, collisions)
  // ─────────────────────────────────────────────────────────────────────────────
  function simulate(dt, viewBounds) {
    const { x0, y0, x1, y1 } = viewBounds;

    // Active list
    const active = [];
    for (const b of bodies.values()) {
      if (b.x + b.rWorld < x0 || b.y + b.rWorld < y0 ||
          b.x - b.rWorld > x1 || b.y - b.rWorld > y1) continue;
      active.push(b);
    }
    const n = active.length;
    if (n === 0) { lastGravMs = 0; lastIntegrateMs = 0; lastCollideMs = 0; return; }

    // Gravity timings
    const tTree0 = performance.now();
    const root = buildQuadTree(active);
    const tTree1 = performance.now();
    const treeMs = tTree1 - tTree0;

    const ax = new Float32Array(n);
    const ay = new Float32Array(n);

    const tForce0 = performance.now();
    for (let i = 0; i < n; i++) {
      const b = active[i];
      const acc = { ax: 0, ay: 0 };
      accumulateForceFromTree(root, b, acc);
      ax[i] = acc.ax;
      ay[i] = acc.ay;
    }
    const tForce1 = performance.now();
    lastGravMs = treeMs + (tForce1 - tForce0);

    // Integration
    const tInt0 = performance.now();
    for (let i = 0; i < n; i++) {
      const b = active[i];

      // Thrusters on selected body
      let addAX = 0, addAY = 0;
      if (selectedId && b.id === selectedId) {
        let dx = 0, dy = 0;
        if (keyA) dx -= 1;
        if (keyD) dx += 1;
        if (keyW) dy -= 1;
        if (keyS) dy += 1;
        if (dx !== 0 || dy !== 0) {
          const invLen = 1 / Math.hypot(dx, dy);
          dx *= invLen; dy *= invLen;
          const CONTROL_ACCEL = 25.0;
          addAX = CONTROL_ACCEL * dx;
          addAY = CONTROL_ACCEL * dy;
        }
      }

      b.vx += (ax[i] + addAX) * dt;
      b.vy += (ay[i] + addAY) * dt;
      b.x  += b.vx * dt;
      b.y  += b.vy * dt;
    }
    const tInt1 = performance.now();
    lastIntegrateMs = tInt1 - tInt0;

    // Broad phase with buckets
    const tColl0 = performance.now();

    /** Map<number, string[]> */
    const buckets = new Map();
    /** Map<string, {cx0:number,cx1:number,cy0:number,cy1:number}> */
    const ranges = new Map();

    function rangeForBody(b) {
      const cx0 = clampCx(Math.floor((b.x - b.rWorld) / BUCKET_SIZE));
      const cx1 = clampCx(Math.floor((b.x + b.rWorld) / BUCKET_SIZE));
      const cy0 = clampCy(Math.floor((b.y - b.rWorld) / BUCKET_SIZE));
      const cy1 = clampCy(Math.floor((b.y + b.rWorld) / BUCKET_SIZE));
      return { cx0, cx1, cy0, cy1 };
    }

    for (const b of active) {
      const r = rangeForBody(b);
      ranges.set(b.id, r);
      for (let cy = r.cy0; cy <= r.cy1; cy++) {
        for (let cx = r.cx0; cx <= r.cx1; cx++) {
          const key = bkKey(cx, cy);
          let arr = buckets.get(key);
          if (!arr) { arr = []; buckets.set(key, arr); }
          arr.push(b.id);
        }
      }
    }

    function insertBodyIntoBuckets(body) {
      const r = rangeForBody(body);
      ranges.set(body.id, r);
      for (let cy = r.cy0; cy <= r.cy1; cy++) {
        for (let cx = r.cx0; cx <= r.cx1; cx++) {
          const key = bkKey(cx, cy);
          let arr = buckets.get(key);
          if (!arr) { arr = []; buckets.set(key, arr); }
          arr.push(body.id);
        }
      }
    }

    function isCanonicalBucket(cx, cy, rA, rB) {
      const cxCanon = Math.max(rA.cx0, rB.cx0);
      const cyCanon = Math.max(rA.cy0, rB.cy0);
      return cx === cxCanon && cy === cyCanon;
    }

    // Narrow phase
    for (const [key, arr] of buckets) {
      const cx = Math.floor(key / BUCKET_STRIDE);
      const cy = key - cx * BUCKET_STRIDE;

      for (let i = 0; i < arr.length; i++) {
        let idA = arr[i];
        let a = bodies.get(idA);
        if (!a) continue;

        for (let j = i + 1; j < arr.length; j++) {
          const idB = arr[j];
          const b = bodies.get(idB);
          if (!b || idA === idB) continue;

          const rA = ranges.get(idA);
          const rB = ranges.get(idB);
          if (!rA || !rB) continue;

          if (!isCanonicalBucket(cx, cy, rA, rB)) continue;

          const dx = b.x - a.x;
          const dy = b.y - a.y;
          const sumR = a.rWorld + b.rWorld;
          if (dx*dx + dy*dy <= sumR*sumR) {
            // Count this collision for EMA (both bodies)
            a.collCount = (a.collCount || 0) + 1;
            b.collCount = (b.collCount || 0) + 1;

            // Sticky logic: BOTH bodies must exceed STICKY_THR and merging must be enabled
            const stickyA = (a.emaCps || 0) >= STICKY_THR;
            const stickyB = (b.emaCps || 0) >= STICKY_THR;
            const wantMerge = MERGE_ON && stickyA && stickyB;

            if (wantMerge) {
              const merged = mergeBodies(a, b);
              ranges.delete(idA);
              ranges.delete(idB);
              insertBodyIntoBuckets(merged);
              idA = merged.id;
              a = merged;
              arr[i] = idA;
              j = i; // restart comparisons for this i with the merged body
            } else {
              // Bounce (no unconditional merging anymore)
              resolveCollisionNoMerge(a, b);
              // keep both bodies; next frame broad-phase rebuild handles updates
            }
          }
        }
      }
    }

    const tColl1 = performance.now();
    lastCollideMs = tColl1 - tColl0;

    // Update EMA CPS for all active bodies
    if (dt > 0) {
      const alpha = 1 - Math.exp(-dt / EMA_TAU);
      for (const b of active) {
        const inst = (b.collCount || 0) / dt;
        b.emaCps = (b.emaCps || 0) + alpha * (inst - (b.emaCps || 0));
        b.collCount = 0;
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 11) Spawn delta logic (newly revealed cells only)
  // ─────────────────────────────────────────────────────────────────────────────
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

  // ─────────────────────────────────────────────────────────────────────────────
  // 12) Render
  // ─────────────────────────────────────────────────────────────────────────────
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

    // Frame dt for controls
    const now = performance.now() * 0.001;
    let dtFrame = 0;
    if (lastFrameTime == null) lastFrameTime = now;
    else { dtFrame = Math.min(0.1, Math.max(0, now - lastFrameTime)); lastFrameTime = now; }

    const active = simActive();

    // Spawn
    if (active) {
      const tSpawn0 = performance.now();
      spawnNewlyRevealed(viewBounds);
      const tSpawn1 = performance.now();
      lastSpawnMs = tSpawn1 - tSpawn0;
    } else {
      lastSimTime = null;
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
      else { dt = Math.min(MAX_DT, Math.max(0, nowS - lastSimTime)); lastSimTime = nowS; }
      if (dt > 0) simulate(dt, viewBounds);
    } else if (selectedId && (keyW || keyA || keyS || keyD)) {
      const b = bodies.get(selectedId);
      if (b) {
        let dx = 0, dy = 0;
        if (keyA) dx -= 1; if (keyD) dx += 1;
        if (keyW) dy -= 1; if (keyS) dy += 1;
        if (dx !== 0 || dy !== 0) {
          const inv = 1 / Math.hypot(dx, dy);
          dx *= inv; dy *= inv;
          const CONTROL_ACCEL = 25.0;
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
        // Optional: show selected emaCps if a HUD span exists
        if (emaVal) emaVal.textContent = (b.emaCps || 0).toFixed(1) + ' cps';
      } else {
        selectedId = null;
      }
    }

    const tDraw1 = performance.now();
    lastRenderMs = tDraw1 - tDraw0;

    // HUD
    const t1 = performance.now();
    lastTotalMs = t1 - t0;
    if ((hudCounter = (hudCounter + 1) % 6) === 0) {
      if (hudZ) hudZ.textContent = `${scale.toFixed(3)} px/unit`;
      if (hudV) hudV.textContent = `x:[${x0}..${x1}] y:[${y0}..${y1}]`;
      if (gVal) gVal.textContent = fmtG(G);
      if (vmaxVal) vmaxVal.textContent = fmtV0(V0_MAX);
      if (rmaxVal) rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
      if (rminVal) rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
      if (dVal) dVal.textContent = fmtDensity(DENSITY);
      if (biasVal) biasVal.textContent = fmtBias(SIZE_BIAS);
      if (followVal) followVal.textContent = followSelected ? 'On' : 'Off';

      if (mergeVal)  mergeVal.textContent  = fmtBool(MERGE_ON);
      if (stickyVal) stickyVal.textContent = STICKY_THR.toFixed(0) + ' cps';
      if (restVal)   restVal.textContent   = fmtFloat(REST_E);
      if (muVal)     muVal.textContent     = fmtFloat(FRICTION_MU);

      if (mcVal)     mcVal.textContent     = CRUSH_MC.toFixed(0);
      if (kVal)      kVal.textContent      = fmtFloat(CRUSH_K);

      if (bandVal)   bandVal.textContent   = VBIAS_BAND_H.toFixed(0);
      if (vbiasVal)  vbiasVal.textContent  = fmtFloat(VBIAS_ADD);

      if (perfVal) {
        perfVal.textContent =
          `Grav ${lastGravMs.toFixed(1)} ms | ` +
          `Spawn ${lastSpawnMs.toFixed(1)} ms | ` +
          `Collide ${lastCollideMs.toFixed(1)} ms | ` +
          `Integr ${lastIntegrateMs.toFixed(1)} ms | ` +
          `Render ${lastRenderMs.toFixed(1)} ms | ` +
          `Total ${lastTotalMs.toFixed(1)} ms`;
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 13) Sizing
  // ─────────────────────────────────────────────────────────────────────────────
  function resizeCanvas() {
    const dpr = Math.max(1, Math.min(3, window.devicePixelRatio || 1));
    canvas.width  = Math.floor(canvas.clientWidth  * dpr);
    canvas.height = Math.floor(canvas.clientHeight * dpr);
    ctx.imageSmoothingEnabled = false;
    clampView();
    requestRender();
    prevX0 = prevY0 = prevX1 = prevY1 = null;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 14) Picking & input
  // ─────────────────────────────────────────────────────────────────────────────
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
        selectedId = pickBodyAt(worldX, worldY);
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

  function handleKey(e, down) {
    const k = e.key.toLowerCase();
    if (k === 'w') { keyW = down; if (selectedId) e.preventDefault(); }
    else if (k === 'a') { keyA = down; if (selectedId) e.preventDefault(); }
    else if (k === 's') { keyS = down; if (selectedId) e.preventDefault(); }
    else if (k === 'd') { keyD = down; if (selectedId) e.preventDefault(); }
    else if (k === 'escape' && down) { selectedId = null; requestRender(); }
    else if (k === 'f' && down) { followSelected = !followSelected; if (followVal) followVal.textContent = followSelected ? 'On' : 'Off'; requestRender(); }
  }
  window.addEventListener('keydown', (e) => handleKey(e, true), { passive: false });
  window.addEventListener('keyup',   (e) => handleKey(e, false), { passive: false });

  // ─────────────────────────────────────────────────────────────────────────────
  // 15) HUD controls (only attach if present)
  // ─────────────────────────────────────────────────────────────────────────────
  function updateG(newG) {
    G = Math.min(G_MAX, Math.max(G_MIN, newG));
    if (gVal) gVal.textContent = fmtG(G);
    requestRender();
  }
  if (gMinusBtn) gMinusBtn.addEventListener('click', () => updateG(G / G_STEP));
  if (gPlusBtn)  gPlusBtn .addEventListener('click', () => updateG(G * G_STEP));

  function updateV0Max(newMax) {
    V0_MAX = Math.min(V0_MAX_CAP, Math.max(V0_MIN, newMax));
    if (vmaxVal) vmaxVal.textContent = fmtV0(V0_MAX);
  }
  if (vmaxMinusBtn) vmaxMinusBtn.addEventListener('click', () => updateV0Max(V0_MAX / V0_STEP));
  if (vmaxPlusBtn)  vmaxPlusBtn .addEventListener('click', () => updateV0Max(V0_MAX * V0_STEP));

  function updateRmax(multiplierNew) {
    SIZE_VAR_MAX = Math.min(S_MAX_MAX, Math.max(S_MAX_MIN, multiplierNew));
    if (SIZE_VAR_MAX < SIZE_VAR_MIN) SIZE_VAR_MIN = SIZE_VAR_MAX;
    if (rmaxVal) rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
    if (rminVal) rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
    requestRender();
  }
  if (rmaxMinusBtn) rmaxMinusBtn.addEventListener('click', () => updateRmax(SIZE_VAR_MAX / S_STEP));
  if (rmaxPlusBtn)  rmaxPlusBtn .addEventListener('click', () => updateRmax(SIZE_VAR_MAX * S_STEP));

  function updateRmin(multiplierNew) {
    SIZE_VAR_MIN = Math.min(S_MIN_MAX, Math.max(S_MIN_MIN, multiplierNew));
    if (SIZE_VAR_MIN > SIZE_VAR_MAX) SIZE_VAR_MAX = SIZE_VAR_MIN;
    if (rminVal) rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
    if (rmaxVal) rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
    requestRender();
  }
  if (rminMinusBtn) rminMinusBtn.addEventListener('click', () => updateRmin(SIZE_VAR_MIN / S_STEP));
  if (rminPlusBtn)  rminPlusBtn .addEventListener('click', () => updateRmin(SIZE_VAR_MIN * S_STEP));

  function updateDensity(newD) {
    DENSITY = Math.min(D_MAX, Math.max(D_MIN, newD));
    DENSITY_THR = Math.floor(DENSITY * 4294967296);
    if (dVal) dVal.textContent = fmtDensity(DENSITY);
    requestRender();
  }
  if (dMinusBtn) dMinusBtn.addEventListener('click', () => updateDensity(DENSITY / D_STEP));
  if (dPlusBtn)  dPlusBtn .addEventListener('click', () => updateDensity(DENSITY * D_STEP));

  function updateBias(newK) {
    SIZE_BIAS = Math.min(BIAS_MAX, Math.max(BIAS_MIN, newK));
    if (biasVal) biasVal.textContent = fmtBias(SIZE_BIAS);
    requestRender();
  }
  if (biasMinusBtn) biasMinusBtn.addEventListener('click', () => updateBias(SIZE_BIAS / BIAS_STEP));
  if (biasPlusBtn)  biasPlusBtn .addEventListener('click', () => updateBias(SIZE_BIAS * BIAS_STEP));

  // Merge HUD
  function setMergeOn(v) { MERGE_ON = !!v; if (mergeVal) mergeVal.textContent = fmtBool(MERGE_ON); }
  if (mergeToggle) mergeToggle.addEventListener('click', () => setMergeOn(!MERGE_ON));

  function setStickyThr(newThr) { STICKY_THR = Math.max(0, newThr); if (stickyVal) stickyVal.textContent = STICKY_THR.toFixed(0) + ' cps'; }
  if (stickyMinus) stickyMinus.addEventListener('click', () => setStickyThr(STICKY_THR / STICKY_STEP));
  if (stickyPlus)  stickyPlus .addEventListener('click', () => setStickyThr(STICKY_THR * STICKY_STEP));

  function setRest(newE) { REST_E = Math.max(0, Math.min(1.5, newE)); if (restVal) restVal.textContent = fmtFloat(REST_E); }
  if (restMinus) restMinus.addEventListener('click', () => setRest(REST_E / REST_STEP));
  if (restPlus)  restPlus .addEventListener('click', () => setRest(REST_E * REST_STEP));

  function setMu(newMu) { FRICTION_MU = Math.max(0, Math.min(5, newMu)); if (muVal) muVal.textContent = fmtFloat(FRICTION_MU); }
  if (muMinus) muMinus.addEventListener('click', () => setMu(FRICTION_MU / FRICT_STEP));
  if (muPlus)  muPlus .addEventListener('click', () => setMu(FRICTION_MU * FRICT_STEP));

  // Crush HUD
  function setMC(newMC) { CRUSH_MC = Math.min(CRUSH_MC_MAX, Math.max(CRUSH_MC_MIN, newMC)); if (mcVal) mcVal.textContent = CRUSH_MC.toFixed(0); requestRender(); }
  function setK(newK)   { CRUSH_K  = Math.min(CRUSH_K_MAX,  Math.max(CRUSH_K_MIN,  newK));   if (kVal)  kVal.textContent  = fmtFloat(CRUSH_K);   requestRender(); }
  if (mcMinus) mcMinus.addEventListener('click', () => setMC(CRUSH_MC - 10));
  if (mcPlus)  mcPlus .addEventListener('click', () => setMC(CRUSH_MC + 10));
  if (kMinus)  kMinus .addEventListener('click', () => setK(CRUSH_K / 1.2));
  if (kPlus)   kPlus  .addEventListener('click', () => setK(CRUSH_K * 1.2));

  // Band bias HUD
  function setBandH(v) { VBIAS_BAND_H = Math.max(1, Math.min(1000000, v)); if (bandVal) bandVal.textContent = VBIAS_BAND_H.toFixed(0); }
  function setVbias(v) { VBIAS_ADD = v; if (vbiasVal) vbiasVal.textContent = fmtFloat(VBIAS_ADD); }
  if (bandMinus) bandMinus.addEventListener('click', () => setBandH(VBIAS_BAND_H / 1.1));
  if (bandPlus)  bandPlus .addEventListener('click', () => setBandH(VBIAS_BAND_H * 1.1));
  if (vbiasMinus) vbiasMinus.addEventListener('click', () => setVbias(VBIAS_ADD / 1.1));
  if (vbiasPlus)  vbiasPlus .addEventListener('click', () => setVbias(VBIAS_ADD * 1.1));

  // ─────────────────────────────────────────────────────────────────────────────
  // 16) Bootstrap & tick
  // ─────────────────────────────────────────────────────────────────────────────
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  (function initView() {
    const vw = viewportWorldWidth();
    const vh = viewportWorldHeight();
    viewX = (WORLD_SIZE - vw) / 2;
    viewY = (WORLD_SIZE - vh) / 2;
    clampView();

    if (gVal) gVal.textContent = fmtG(G);
    if (vmaxVal) vmaxVal.textContent = fmtV0(V0_MAX);
    if (rmaxVal) rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
    if (rminVal) rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
    if (dVal) dVal.textContent = fmtDensity(DENSITY);
    if (biasVal) biasVal.textContent = fmtBias(SIZE_BIAS);
    if (followVal) followVal.textContent = followSelected ? 'On' : 'Off';

    if (mergeVal)  mergeVal.textContent  = fmtBool(MERGE_ON);
    if (stickyVal) stickyVal.textContent = STICKY_THR.toFixed(0) + ' cps';
    if (restVal)   restVal.textContent   = fmtFloat(REST_E);
    if (muVal)     muVal.textContent     = fmtFloat(FRICTION_MU);

    if (mcVal)     mcVal.textContent     = CRUSH_MC.toFixed(0);
    if (kVal)      kVal.textContent      = fmtFloat(CRUSH_K);

    if (bandVal)   bandVal.textContent   = VBIAS_BAND_H.toFixed(0);
    if (vbiasVal)  vbiasVal.textContent  = fmtFloat(VBIAS_ADD);

    if (perfVal) {
      perfVal.textContent =
        `Grav ${lastGravMs.toFixed(1)} ms | ` +
        `Spawn ${lastSpawnMs.toFixed(1)} ms | ` +
        `Collide ${lastCollideMs.toFixed(1)} ms | ` +
        `Integr ${lastIntegrateMs.toFixed(1)} ms | ` +
        `Render ${lastRenderMs.toFixed(1)} ms | ` +
        `Total ${lastTotalMs.toFixed(1)} ms`;
    }

    requestRender();
  })();

  (function tick() {
    if (dragging || simActive() || selectedId || followSelected) requestRender();
    requestAnimationFrame(tick);
  })();
})();
