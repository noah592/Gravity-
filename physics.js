// physics.js — gravity, integration, collisions (no DOM)

// Core physics constants (export SOFT2_SQRT so spawner can compute rRef consistently)
export const SOFTENING2 = 0.15;
export const SOFT2_SQRT = Math.sqrt(SOFTENING2);

// Step physics for one frame.
// bodies: Map<string, Body>
// dt: number (seconds)
// viewBounds: {x0,y0,x1,y1} — used to form active set
// controls: { selectedId, keyW, keyA, keyS, keyD, controlAccel }
// params: {
//   G, alpha, theta, worldSize, bucketSize, baseRWorld
// }
export function stepPhysics(bodies, dt, viewBounds, controls, params) {
  const {
    G = 10,
    alpha = 0.01,
    theta = 1.0,
    worldSize,
    bucketSize = 2.0,
    baseRWorld = 0.45
  } = params;

  const alphaInvSqrt = 1 / Math.sqrt(alpha);
  const BUCKET_STRIDE = 2_000_000;
  const MAX_CX = Math.floor(worldSize / bucketSize);
  const MAX_CY = Math.floor(worldSize / bucketSize);
  const clampCx = (cx) => Math.min(MAX_CX, Math.max(0, cx));
  const clampCy = (cy) => Math.min(MAX_CY, Math.max(0, cy));
  const bkKey = (cx, cy) => cx * BUCKET_STRIDE + cy;

  const active = [];
  const { x0, y0, x1, y1 } = viewBounds;
  for (const b of bodies.values()) {
    if (b.x + b.rWorld < x0 || b.y + b.rWorld < y0 ||
        b.x - b.rWorld > x1 || b.y - b.rWorld > y1) continue;
    active.push(b);
  }
  const n = active.length;
  if (n === 0) {
    return {
      activeCount: 0,
      treeBuildMs: 0, gravMs: 0, integrateMs: 0, collideMs: 0,
      selectedId: controls.selectedId
    };
  }

  // ---- Quadtree build
  const tTree0 = performance.now();
  const root = buildQuadTree(active);
  const tTree1 = performance.now();
  const treeBuildMs = tTree1 - tTree0;

  // ---- Force accumulation
  const ax = new Float32Array(n);
  const ay = new Float32Array(n);

  const tForce0 = performance.now();
  for (let i = 0; i < n; i++) {
    const b = active[i];
    const acc = { ax: 0, ay: 0 };
    accumulateForceFromTree(root, b, acc, theta, G, alphaInvSqrt);
    ax[i] = acc.ax;
    ay[i] = acc.ay;
  }
  const tForce1 = performance.now();
  const gravMs = treeBuildMs + (tForce1 - tForce0);

  // ---- Integrate (semi-implicit Euler) with optional thrust on selected
  const tInt0 = performance.now();
  for (let i = 0; i < n; i++) {
    const b = active[i];

    let addAX = 0, addAY = 0;
    if (controls.selectedId && b.id === controls.selectedId) {
      let dx = 0, dy = 0;
      if (controls.keyA) dx -= 1;
      if (controls.keyD) dx += 1;
      if (controls.keyW) dy -= 1;
      if (controls.keyS) dy += 1;
      if (dx !== 0 || dy !== 0) {
        const invLen = 1 / Math.hypot(dx, dy);
        dx *= invLen; dy *= invLen;
        addAX = controls.controlAccel * dx;
        addAY = controls.controlAccel * dy;
      }
    }

    b.vx += (ax[i] + addAX) * dt;
    b.vy += (ay[i] + addAY) * dt;
    b.x  += b.vx * dt;
    b.y  += b.vy * dt;
  }
  const tInt1 = performance.now();
  const integrateMs = tInt1 - tInt0;

  // ---- Collisions (spatial hashing with canonical bucket)
  const tColl0 = performance.now();

  /** Map<number, string[]> */
  const buckets = new Map();
  /** Map<string, {cx0:number,cx1:number,cy0:number,cy1:number}> */
  const ranges = new Map();

  function rangeForBody(b) {
    const cx0 = clampCx(Math.floor((b.x - b.rWorld) / bucketSize));
    const cx1 = clampCx(Math.floor((b.x + b.rWorld) / bucketSize));
    const cy0 = clampCy(Math.floor((b.y - b.rWorld) / bucketSize));
    const cy1 = clampCy(Math.floor((b.y + b.rWorld) / bucketSize));
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

  let newSelectedId = controls.selectedId;

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
          const merged = mergeBodies(a, b, baseRWorld);
          // Update selection if needed
          if (newSelectedId === a.id || newSelectedId === b.id) {
            newSelectedId = merged.id;
          }
          ranges.delete(idA);
          ranges.delete(idB);
          insertBodyIntoBuckets(merged);

          idA = merged.id;
          a = merged;
          arr[i] = idA;
          j = i;
        }
      }
    }
  }

  const tColl1 = performance.now();
  const collideMs = tColl1 - tColl0;

  return {
    activeCount: n,
    treeBuildMs,
    gravMs,
    integrateMs,
    collideMs,
    selectedId: newSelectedId
  };

  // ---- Helpers ----

  function buildQuadTree(activeBodies) {
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
      return { cx, cy, half, m: 0, mx: 0, my: 0, body: null, children: null, count: 0, comX: cx, comY: cy };
    }

    const root = makeNode(cx, cy, half);

    function childFor(node, b) {
      const east = b.x >= node.cx;
      const south = b.y >= node.cy;
      if (!east && !south) return node.children[0];
      if ( east && !south) return node.children[1];
      if (!east &&  south) return node.children[2];
      return node.children[3];
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

    for (const b of activeBodies) insert(root, b);

    function finalize(node) {
      if (node.count > 0) {
        node.comX = node.mx / node.m;
        node.comY = node.my / node.m;
      }
      if (node.children) for (const c of node.children) finalize(c);
    }
    finalize(root);
    return root;
  }

  function accumulateForceFromTree(node, b, out, theta, G, alphaInvSqrt) {
    if (!node || node.count === 0) return;
    if (node.children === null && node.body === b) return;

    // Influence radius for this body
    const rInfl = Math.max(b.rWorld, SOFT2_SQRT) * alphaInvSqrt;

    // Cull node entirely if outside influence sphere (using node center + radius)
    const dxC = node.cx - b.x;
    const dyC = node.cy - b.y;
    const distC = Math.hypot(dxC, dyC);
    const nodeRad = node.half * Math.SQRT2;
    if (distC - nodeRad > rInfl) return;

    // BH opening test using COM
    const dx = node.comX - b.x;
    const dy = node.comY - b.y;
    const dist2 = dx*dx + dy*dy + SOFTENING2;
    const dist = Math.sqrt(dist2);

    if (node.children === null || (node.half * 2) / dist < theta) {
      if (dist > rInfl) return;
      const invR3 = 1 / (dist2 * dist);
      const f = G * node.m * invR3;
      out.ax += f * dx;
      out.ay += f * dy;
    } else {
      for (const c of node.children) accumulateForceFromTree(c, b, out, theta, G, alphaInvSqrt);
    }
  }

  function mergeBodies(bi, bj, baseRWorld) {
    const mSum = bi.m + bj.m;
    const xNew = (bi.x * bi.m + bj.x * bj.m) / mSum;
    const yNew = (bi.y * bi.m + bj.y * bj.m) / mSum;
    const vxNew = (bi.vx * bi.m + bj.vx * bj.m) / mSum;
    const vyNew = (bi.vy * bi.m + bj.vy * bj.m) / mSum;

    const sources = bi.sources.concat(bj.sources);
    for (const s of sources) /* consumed by caller code as 'consumedCells'; not here */ void 0;

    const mNew = mSum;
    const sizeFactorNew = Math.sqrt(mNew); // m ∝ s^2 => s = m^(1/2)
    const rWorldNew = baseRWorld * sizeFactorNew;

    const id = `m:${Math.random().toString(36).slice(2)}`; // id uniqueness; caller doesn't rely on sequence
    const merged = { id, x: xNew, y: yNew, vx: vxNew, vy: vyNew, m: mNew,
                     rWorld: rWorldNew, rRef: Math.max(rWorldNew, SOFT2_SQRT), sources };

    bodies.delete(bi.id);
    bodies.delete(bj.id);
    bodies.set(id, merged);
    return merged;
  }
}
