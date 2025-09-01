(() => {
/* =============================[ Section 1: Config & Globals ]============================= */
  // --- Config ---
  const WORLD_SIZE = 1_000_000;       // 1,000,000 x 1,000,000 world
  let   DENSITY = 0.0045;             // fraction of filled cells (0..1), mutable via HUD
  const D_MIN = 1e-7, D_MAX = 0.03;   // density clamps
  const D_STEP = 1.1;                 // small step (×/÷) for density buttons

  const ZOOM_STEP = 1.1;              // wheel zoom factor per notch
  const MAX_SCALE = 200;              // max pixels per world unit (max zoom-in)
  const MIN_SCALE = 1;                // min pixels per world unit (max zoom-out)

  // Gravity / physics knobs (world units: 1 unit = 1 grid cell)
  const GRAVITY_ZOOM_GATE = 1;        // simulate when scale >= this (px/unit)
  let   G = 15;                       // gravitational constant (mutable)
  const G_MIN = 1e-6, G_MAX = 1e6;    // HUD bounds
  const G_STEP = 1.1;                 // small step (×/÷) for G buttons
  const SOFTENING2 = 0.15;            // epsilon^2 to avoid singularities
  const MAX_DT = 1 / 60;              // maximum physics dt (seconds)

  // Barnes–Hut (quadtree) parameters
  const THETA = 1.0;                  // opening angle (smaller = more accurate)
  const LEAF_CAPACITY = 1;

  // Visual base radius; final rWorld = BASE_R_WORLD * sizeFactor
  const BASE_R_WORLD = 0.45;

  // Size variation (deterministic per cell)
  let   SIZE_VAR_MIN = 0.31 / BASE_R_WORLD;     // min radius = 0.31 units
  let   SIZE_VAR_MAX = 1.50 / BASE_R_WORLD;     // max radius = 1.50 units
  const S_MIN_MIN = 0.05 / BASE_R_WORLD;        // floor (0.05 u)
  const S_MIN_MAX = 15.0 / BASE_R_WORLD;        // ceiling for min
  const S_MAX_MIN = 0.1  / BASE_R_WORLD;        // floor for max
  const S_MAX_MAX = 20.0 / BASE_R_WORLD;        // ceiling for max
  const S_STEP    = 1.1;                        // small step (×/÷) for size buttons

  let   SIZE_BIAS = 7.0;                        // default bias k (>1 = smaller)
  const BIAS_MIN = 0.2, BIAS_MAX = 10.0;
  const BIAS_STEP = 1.1;                        // small step (×/÷)

  // Initial velocity upper bound (for first activation only)
  let   V0_MAX = 7.0;                           // V0 cap (units/s), mutable
  const V0_MIN = 0.1, V0_MAX_CAP = 5000.0;
  const V0_STEP = 1.1;                          // small step (×/÷)

  // Manual control (thrusters) — acceleration magnitude in units/s^2
  const CONTROL_ACCEL = 25.0;

  // Picking: minimum clickable area and derived radius (world units)
  const MIN_CLICK_AREA = 5;                               // units^2
  const MIN_CLICK_RADIUS = Math.sqrt(MIN_CLICK_AREA / Math.PI);

  // Collision system knobs (merge toggle, restitution, friction μ)
  let MERGE_ENABLED = true;                   // HUD toggle
  let RESTITUTION = 0.85;                     // e in [0, 1.5] reasonable
  let FRICTION_MU = 0.30;                     // μ in [0, 2]

  const E_MIN = 0.0, E_MAX = 1.5, E_STEP = 1.1;
  const MU_MIN = 0.0, MU_MAX = 2.0, MU_STEP = 1.1;

  // --- Spatial hashing for collisions (Option A) ---
  const BUCKET_SIZE = 2.0;                  // S (world units per grid cell)
  const MAX_CX = Math.floor(WORLD_SIZE / BUCKET_SIZE);
  const MAX_CY = Math.floor(WORLD_SIZE / BUCKET_SIZE);
  const BUCKET_STRIDE = 2_000_000;          // for numeric composite keys
  const bkKey = (cx, cy) => cx * BUCKET_STRIDE + cy;
  const clampCx = (cx) => Math.min(MAX_CX, Math.max(0, cx));
  const clampCy = (cy) => Math.min(MAX_CY, Math.max(0, cy));

  // --- Canvas / HUD ---
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

  // New HUD handles for merge/e/μ
  const mergeVal = document.getElementById('mergeVal');
  const mergeToggleBtn = document.getElementById('mergeToggle');

  const eVal = document.getElementById('eVal');
  const eMinusBtn = document.getElementById('eMinus');
  const ePlusBtn  = document.getElementById('ePlus');

  const muVal = document.getElementById('muVal');
  const muMinusBtn = document.getElementById('muMinus');
  const muPlusBtn  = document.getElementById('muPlus');

  // --- View state ---
  let scale = 2;      // starting zoom
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

/* ==========================[ Section 2: Hashing & Utilities ]========================== */
  // --- Hashing ---
  function hash2D_u32(x, y) {
    let n = Math.imul(x | 0, 374761393) ^ Math.imul(y | 0, 668265263);
    n = Math.imul(n ^ (n >>> 13), 1274126177);
    return (n ^ (n >>> 16)) >>> 0; // uint32
  }
  function hash01(x, y) { return hash2D_u32(x, y) / 4294967296; } // [0,1)

  // Numeric cell key (no strings) for spawned cells
  function nkey(x, y) { return x * 1_000_000 + y; } // safe (<=1e12)

  // Density threshold for integer compare
  let DENSITY_THR = Math.floor(DENSITY * 4294967296);

  function sizeMulAt(x, y) {
    const u = hash01(x + 9173, y + 12345);
    const ub = Math.pow(u, SIZE_BIAS); // bias toward small when k>1
    return SIZE_VAR_MIN + (SIZE_VAR_MAX - SIZE_VAR_MIN) * ub;
  }

  // Formatting helpers
  function fmtG(g) { return (g >= 0.001 && g < 1000) ? g.toFixed(3) : g.toExponential(2); }
  function fmtV0(x) { return (x < 10) ? x.toFixed(2) : (x < 1000 ? x.toFixed(1) : x.toExponential(1)); }
  function fmtR(valUnits) { return valUnits.toFixed(valUnits < 10 ? 2 : 1); }
  function fmtDensity(d) { return d < 0.001 ? d.toExponential(2) : d.toFixed(5); }
  function fmtBias(k) { return k.toFixed(2); }
  function fmt01(x) { return x.toFixed(2); } // generic formatter for [0, ~] params

/* =====================[ Section 3: Render / Simulation Scheduling ]==================== */
  let needsRender = false;
  function requestRender() {
    if (!needsRender) {
      needsRender = true;
      requestAnimationFrame(render);
    }
  }

  // Persistent universe state
  /**
   * bodies: Map from body-id string -> body
   * body.sources: array of numeric cell keys that contributed to this body (for consumedCells)
   */
  /** @type {Map<string, {id:string,x:number,y:number,vx:number,vy:number,m:number,rWorld:number,sources:number[]}>} */
  const bodies = new Map();

  // Activated/consumed cells tracked with numeric keys (no strings)
  const activatedCells = new Set(); // Set<number>
  const consumedCells  = new Set(); // Set<number>

  let lastSimTime = null;   // for physics dt
  let lastFrameTime = null; // for UI/control dt
  let mergeIdCounter = 1;   // for merged-body IDs

  // Perf metrics (ms)
  let lastGravMs = 0.0;      // tree build + force accumulation
  let lastSpawnMs = 0.0;     // visible-cell spawning pass (delta stripes/rings)
  let lastCollideMs = 0.0;   // bucket build + collision checks/merges/impulses
  let lastIntegrateMs = 0.0; // integration loop only
  let lastRenderMs = 0.0;    // canvas drawing (fill + stroke)
  let lastTotalMs = 0.0;     // full render() duration
  let hudCounter = 0;        // throttle HUD updates

  // Selection & control
  let selectedId = null;
  let keyW = false, keyA = false, keyS = false, keyD = false;
  let followSelected = false;

  function simActive() { return scale >= GRAVITY_ZOOM_GATE; }

  function cellId(x, y) { return `c:${x},${y}`; }

  // Spawn helper with early-out by density
  function addCellBodyIfNeeded(x, y) {
    if (hash2D_u32(x, y) >= DENSITY_THR) return; // cheap reject
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
      sources: [nk]
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

    const mNew = mSum;
    const sizeFactorNew = Math.sqrt(mNew); // m ∝ s^2 => s = m^(1/2)
    const rWorldNew = BASE_R_WORLD * sizeFactorNew;

    const id = `m:${mergeIdCounter++}`;
    const merged = { id, x: xNew, y: yNew, vx: vxNew, vy: vyNew, m: mNew, rWorld: rWorldNew, sources };

    if (selectedId === bi.id || selectedId === bj.id) selectedId = id;

    bodies.delete(bi.id);
    bodies.delete(bj.id);
    bodies.set(id, merged);

    return merged;
  }

/* ======================[ Section 4: Barnes–Hut Quadtree & Forces ]===================== */
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

/* ==========================[ Section 5: Simulation Step ]========================== */
  function simulate(dt, viewBounds) {
    const { x0, y0, x1, y1 } = viewBounds;

    // Build active list (viewport filter)
    const active = [];
    for (const b of bodies.values()) {
      if (b.x + b.rWorld < x0 || b.y + b.rWorld < y0 ||
          b.x - b.rWorld > x1 || b.y - b.rWorld > y1) continue;
      active.push(b);
    }
    const n = active.length;
    if (n === 0) { lastGravMs = 0; lastIntegrateMs = 0; lastCollideMs = 0; return; }

    // --- Gravity timings ---
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

    // --- Integration timing ---
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

    // --- Collision broad phase with buckets (and resolve/merge) ---
    const tColl0 = performance.now();

    /** Map<number, string[]>: bucket key -> array of body ids overlapping that bucket */
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

    // Fill buckets with multi-bucket occupancy
    for (const b of active) {
      const r = rangeForBody(b);
      ranges.set(b.id, r);
      for (let cy = r.cy0; cy <= r.cy1; cy++) {
        const rowBase = cy;
        for (let cx = r.cx0; cx <= r.cx1; cx++) {
          const key = bkKey(cx, rowBase);
          let arr = buckets.get(key);
          if (!arr) { arr = []; buckets.set(key, arr); }
          arr.push(b.id);
        }
      }
    }

    // Helper: (re)insert a body into all buckets it overlaps (after merge)
    function insertBodyIntoBuckets(body) {
      const r = rangeForBody(body);
      ranges.set(body.id, r);
      for (let cy = r.cy0; cy <= r.cy1; cy++) {
        const rowBase = cy;
        for (let cx = r.cx0; cx <= r.cx1; cx++) {
          const key = bkKey(cx, rowBase);
          let arr = buckets.get(key);
          if (!arr) { arr = []; buckets.set(key, arr); }
          arr.push(body.id);
        }
      }
    }

    // Canonical bucket computation for pair (by ranges)
    function isCanonicalBucket(cx, cy, rA, rB) {
      const cxCanon = Math.max(rA.cx0, rB.cx0);
      const cyCanon = Math.max(rA.cy0, rB.cy0);
      return cx === cxCanon && cy === cyCanon;
    }

    // --- Collision response helpers (bounce + friction, no spin) ---
    function resolveBounceWithFriction(a, b, overlap, nx, ny) {
      // Inverse masses
      const invA = 1 / a.m;
      const invB = 1 / b.m;
      const invSum = invA + invB;
      if (invSum === 0) return; // both infinite mass? (not possible here)

      // Positional correction (to kill overlap). Push out along normal.
      const k_slop = 0.01;      // small allowance
      const percent = 0.8;      // 80% of penetration correction
      const corr = Math.max(overlap - k_slop, 0) * percent / invSum;
      a.x -= nx * corr * invA;
      a.y -= ny * corr * invA;
      b.x += nx * corr * invB;
      b.y += ny * corr * invB;

      // Relative velocity
      const rvx0 = b.vx - a.vx;
      const rvy0 = b.vy - a.vy;

      // Normal impulse (restitution)
      const vRelN = rvx0 * nx + rvy0 * ny; // rv·n
      if (vRelN > 0) return; // already separating

      const jn = -(1 + RESTITUTION) * vRelN / invSum;

      // Apply normal impulse
      const impNx = nx * jn;
      const impNy = ny * jn;
      a.vx -= impNx * invA;
      a.vy -= impNy * invA;
      b.vx += impNx * invB;
      b.vy += impNy * invB;

      // Recompute relative velocity after normal impulse
      const rvx = b.vx - a.vx;
      const rvy = b.vy - a.vy;

      // Tangent vector (perp to normal)
      let tx = -ny, ty = nx;
      // Project rv onto tangent
      const vRelT = rvx * tx + rvy * ty;

      // Friction impulse magnitude uncapped
      let jt_unc = - vRelT / invSum;

      // Coulomb clamp
      const jt_max = FRICTION_MU * jn;
      let jt;
      if (Math.abs(jt_unc) <= jt_max) {
        // static friction: remove all tangential slip
        jt = jt_unc;
      } else {
        // kinetic friction: clamp to μ * jn
        jt = -Math.sign(vRelT) * jt_max;
      }

      // Apply tangential impulse
      const impTx = tx * jt;
      const impTy = ty * jt;
      a.vx -= impTx * invA;
      a.vy -= impTy * invA;
      b.vx += impTx * invB;
      b.vy += impTy * invB;
    }

    // Narrow phase with canonical-bucket dedup
    for (const [key, arr] of buckets) {
      const cx = Math.floor(key / BUCKET_STRIDE);
      const cy = key - cx * BUCKET_STRIDE;

      for (let i = 0; i < arr.length; i++) {
        let idA = arr[i];
        let a = bodies.get(idA);
        if (!a) continue; // merged away earlier

        for (let j = i + 1; j < arr.length; j++) {
          const idB = arr[j];
          const b = bodies.get(idB);
          if (!b || idA === idB) continue;

          const rA = ranges.get(idA);
          const rB = ranges.get(idB);
          if (!rA || !rB) continue;

          // Only test in the canonical bucket shared by A & B
          if (!isCanonicalBucket(cx, cy, rA, rB)) continue;

          const dx = b.x - a.x;
          const dy = b.y - a.y;
          const sumR = a.rWorld + b.rWorld;
          const d2 = dx*dx + dy*dy;
          if (d2 <= sumR*sumR) {
            const dist = Math.sqrt(Math.max(1e-12, d2));
            const overlap = sumR - dist;

            if (MERGE_ENABLED) {
              const merged = mergeBodies(a, b);
              ranges.delete(idA);
              ranges.delete(idB);
              insertBodyIntoBuckets(merged);
              idA = merged.id;
              a = merged;
              arr[i] = idA;
              j = i;
            } else {
              // Resolve bounce with restitution & friction (no merge)
              let nx = dx / dist;
              let ny = dy / dist;
              if (!isFinite(nx) || !isFinite(ny)) { // degenerate
                // pick an arbitrary normal
                nx = 1; ny = 0;
              }
              resolveBounceWithFriction(a, b, overlap, nx, ny);
            }
          }
        }
      }
    }

    const tColl1 = performance.now();
    lastCollideMs = tColl1 - tColl0;
  }

/* =============================[ Section 6: Render Loop ]============================= */
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

    // Clear to black
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, w, h);

    // Visible world-cell bounds (integer rect)
    const x0 = Math.max(0, Math.floor(viewX));
    const y0 = Math.max(0, Math.floor(viewY));
    const x1 = Math.min(WORLD_SIZE, Math.ceil(viewX + w / scale));
    const y1 = Math.min(WORLD_SIZE, Math.ceil(viewY + h / scale));
    const viewBounds = { x0, y0, x1, y1 };

    // Timing (frame dt for controls)
    const now = performance.now() * 0.001; // seconds
    let dtFrame = 0;
    if (lastFrameTime == null) {
      lastFrameTime = now;
    } else {
      dtFrame = Math.min(0.1, Math.max(0, now - lastFrameTime));
      lastFrameTime = now;
    }

    const active = simActive();

    // Spawn only when active & only for newly revealed cells
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
      prevX0 = prevY0 = prevX1 = prevY1 = null; // force spawn next time (optional)
    }

    // Physics step
    let dt = 0;
    if (active) {
      const nowS = performance.now() * 0.001;
      if (lastSimTime == null) lastSimTime = nowS;
      else {
        dt = Math.min(MAX_DT, Math.max(0, nowS - lastSimTime));
        lastSimTime = nowS;
      }
      if (dt > 0) simulate(dt, viewBounds);
    } else {
      // Thrusters still operate when gravity is off (not counted in integration ms)
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

    // --- Render timing begins ---
    const tDraw0 = performance.now();

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
      vmaxVal.textContent = fmtV0(V0_MAX);
      rmaxVal.textContent = fmtR(SIZE_VAR_MAX * BASE_R_WORLD);
      rminVal.textContent = fmtR(SIZE_VAR_MIN * BASE_R_WORLD);
      dVal.textContent = fmtDensity(DENSITY);
      biasVal.textContent = fmtBias(SIZE_BIAS);
      followVal.textContent = followSelected ? 'On' : 'Off';

      mergeVal.textContent = MERGE_ENABLED ? 'On' : 'Off';
      eVal.textContent = fmt01(RESTITUTION);
      muVal.textContent = fmt01(FRICTION_MU);

      perfVal.textContent =
        `Grav ${lastGravMs.toFixed(1)} ms | ` +
        `Spawn ${lastSpawnMs.toFixed(1)} ms | ` +
        `Collide ${lastCollideMs.toFixed(1)} ms | ` +
        `Integr ${lastIntegrateMs.toFixed(1)} ms | ` +
        `Render ${lastRenderMs.toFixed(1)} ms | ` +
        `Total ${lastTotalMs.toFixed(1)} ms`;
    }
  }

/* ===============================[ Section 7: Sizing ]=============================== */
  function resizeCanvas() {
    const dpr = Math.max(1, Math.min(3, window.devicePixelRatio || 1));
    canvas.width  = Math.floor(canvas.clientWidth  * dpr);
    canvas.height = Math.floor(canvas.clientHeight * dpr);
    ctx.imageSmoothingEnabled = false;
    clampView();
    requestRender();
    // Force spawn on next active frame since viewport changed
    prevX0 = prevY0 = prevX1 = prevY1 = null;
  }

/* ===============================[ Section 8: Picking ]============================== */
  function pickBodyAt(worldX, worldY) {
    let best = null;
    let bestD2 = Infinity;
    for (const b of bodies.values()) {
      const dx = worldX - b.x;
      const dy = worldY - b.y;
      const rPick = Math.max(b.rWorld, MIN_CLICK_RADIUS); // ensure min clickable area
      const d2 = dx*dx + dy*dy;
      if (d2 <= rPick * rPick && d2 < bestD2) {
        bestD2 = d2;
        best = b;
      }
    }
    return best ? best.id : null;
  }

/* ======================[ Section 9: Input (pan/zoom/click) ]====================== */
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

    prevX0 = prevY0 = prevX1 = prevY1 = null; // force spawn recompute
    if (simActive()) lastSimTime = null;
  }, { passive: false });

/* ===================[ Section 10: Keys (thrusters & follow) ]==================== */
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

/* =============================[ Section 11: HUD controls ]============================= */
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
    DENSITY_THR = Math.floor(DENSITY * 4294967296); // update threshold
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

  // Merge toggle, Restitution, and Friction μ
  function updateMergeToggle() {
    MERGE_ENABLED = !MERGE_ENABLED;
    mergeVal.textContent = MERGE_ENABLED ? 'On' : 'Off';
    requestRender();
  }
  mergeToggleBtn.addEventListener('click', updateMergeToggle);

  function updateE(newE) {
    RESTITUTION = Math.min(E_MAX, Math.max(E_MIN, newE));
    eVal.textContent = fmt01(RESTITUTION);
    requestRender();
  }
  eMinusBtn.addEventListener('click', () => updateE(RESTITUTION / E_STEP));
  ePlusBtn .addEventListener('click', () => updateE(RESTITUTION * E_STEP));

  function updateMu(newMu) {
    FRICTION_MU = Math.min(MU_MAX, Math.max(MU_MIN, newMu));
    muVal.textContent = fmt01(FRICTION_MU);
    requestRender();
  }
  muMinusBtn.addEventListener('click', () => updateMu(FRICTION_MU / MU_STEP));
  muPlusBtn  .addEventListener('click', () => updateMu(FRICTION_MU * MU_STEP));

/* ===========================[ Section 12: Bootstrap & Tick ]=========================== */
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

    mergeVal.textContent = MERGE_ENABLED ? 'On' : 'Off';
    eVal.textContent = fmt01(RESTITUTION);
    muVal.textContent = fmt01(FRICTION_MU);

    perfVal.textContent =
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
})();
