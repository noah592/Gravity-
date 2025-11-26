// collideWorker.js
// Collision detection with constant-acceleration prediction.
// Given x, y, r, vx, vy, ax, ay, dt, we predict future positions and detect overlaps.

self.onmessage = function (e) {
  const { jobId, xs, ys, rs, vxs, vys, axs, ays, dt, count } = e.data;

  // Predict forward with constant acceleration:
  // x_pred = x + vx*dt + 0.5*ax*dt^2
  // y_pred = y + vy*dt + 0.5*ay*dt^2
  // We reuse xs/ys in-place because their buffers are transferred.
  const halfDt2 = 0.5 * dt * dt;
  for (let i = 0; i < count; i++) {
    xs[i] += vxs[i] * dt + axs[i] * halfDt2;
    ys[i] += vys[i] * dt + ays[i] * halfDt2;
  }

  const pairs = detectCollisions(xs, ys, rs, count);
  self.postMessage({ jobId, pairs }, [pairs.buffer]);
};

function detectCollisions(xs, ys, rs, count) {
  const BUCKET_SIZE   = 2.0;
  const WORLD_SIZE    = 1_000_000;
  const MAX_CX        = Math.floor(WORLD_SIZE / BUCKET_SIZE);
  const MAX_CY        = Math.floor(WORLD_SIZE / BUCKET_SIZE);
  const BUCKET_STRIDE = 2_000_000;

  // ranges: [cx0, cx1, cy0, cy1] per body
  const ranges = new Int32Array(count * 4);
  const buckets = new Map();

  function clamp(v, lo, hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  function bkKey(cx, cy) {
    return cx * BUCKET_STRIDE + cy;
  }

  // Build bucket ranges + fill buckets
  for (let i = 0; i < count; i++) {
    const x = xs[i];
    const y = ys[i];
    const r = rs[i];

    let cx0 = Math.floor((x - r) / BUCKET_SIZE);
    let cx1 = Math.floor((x + r) / BUCKET_SIZE);
    let cy0 = Math.floor((y - r) / BUCKET_SIZE);
    let cy1 = Math.floor((y + r) / BUCKET_SIZE);

    cx0 = clamp(cx0, 0, MAX_CX);
    cx1 = clamp(cx1, 0, MAX_CX);
    cy0 = clamp(cy0, 0, MAX_CY);
    cy1 = clamp(cy1, 0, MAX_CY);

    const off = i * 4;
    ranges[off + 0] = cx0;
    ranges[off + 1] = cx1;
    ranges[off + 2] = cy0;
    ranges[off + 3] = cy1;

    for (let cy = cy0; cy <= cy1; cy++) {
      for (let cx = cx0; cx <= cx1; cx++) {
        const key = bkKey(cx, cy);
        let arr = buckets.get(key);
        if (!arr) {
          arr = [];
          buckets.set(key, arr);
        }
        arr.push(i);
      }
    }
  }

  const resultPairs = [];

  // Canonical bucket logic to avoid duplicate pairs
  for (const [key, arr] of buckets) {
    const cx = Math.floor(key / BUCKET_STRIDE);
    const cy = key - cx * BUCKET_STRIDE;

    const len = arr.length;
    for (let ai = 0; ai < len; ai++) {
      const i = arr[ai];
      const offA = i * 4;
      const aCx0 = ranges[offA + 0];
      const aCy0 = ranges[offA + 2];

      for (let bi = ai + 1; bi < len; bi++) {
        const j = arr[bi];
        const offB = j * 4;
        const bCx0 = ranges[offB + 0];
        const bCy0 = ranges[offB + 2];

        // canonical bucket: max of cx0/cy0
        const cxCanon = Math.max(aCx0, bCx0);
        const cyCanon = Math.max(aCy0, bCy0);
        if (cx !== cxCanon || cy !== cyCanon) continue;

        const dx = xs[j] - xs[i];
        const dy = ys[j] - ys[i];
        const sumR = rs[i] + rs[j];
        if (dx * dx + dy * dy <= sumR * sumR) {
          resultPairs.push(i, j);
        }
      }
    }
  }

  return new Uint32Array(resultPairs);
}
