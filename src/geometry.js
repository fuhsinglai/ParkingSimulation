/** 凸多邊形幾何：SAT 碰撞 + 最近距離。所有座標單位為公尺。 */

export function rectPoly(x, y, w, h) {
  return [[x, y], [x + w, y], [x + w, y + h], [x, y + h]];
}

/** 把車體座標系的點集轉到世界座標。 */
export function transform(pts, pose) {
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
  return pts.map(([a, b]) => [pose.x + a * c - b * s, pose.y + a * s + b * c]);
}

function axesOf(poly) {
  const out = [];
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i], q = poly[(i + 1) % poly.length];
    const ax = -(q[1] - p[1]), ay = q[0] - p[0];
    const d = Math.hypot(ax, ay) || 1;
    out.push([ax / d, ay / d]);
  }
  return out;
}

function project(poly, [ux, uy]) {
  let lo = Infinity, hi = -Infinity;
  for (const [px, py] of poly) {
    const v = px * ux + py * uy;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return [lo, hi];
}

/** 分離軸定理。兩個凸多邊形是否相交。 */
export function polysOverlap(a, b) {
  for (const u of axesOf(a)) { const [al, ah] = project(a, u), [bl, bh] = project(b, u); if (ah < bl || bh < al) return false; }
  for (const u of axesOf(b)) { const [al, ah] = project(a, u), [bl, bh] = project(b, u); if (ah < bl || bh < al) return false; }
  return true;
}

/**
 * 加了裕度的相交測試：把 b 沿各分離軸外擴 margin。
 * 對凸多邊形而言等價於 Minkowski 膨脹的保守近似，夠用且很快。
 */
export function polysOverlapWithMargin(a, b, margin) {
  if (margin <= 0) return polysOverlap(a, b);
  for (const u of [...axesOf(a), ...axesOf(b)]) {
    const [al, ah] = project(a, u), [bl, bh] = project(b, u);
    if (ah < bl - margin || bh < al - margin) return false;
  }
  return true;
}

function pointSegDistance(p, a, b) {
  const vx = b[0] - a[0], vy = b[1] - a[1];
  const wx = p[0] - a[0], wy = p[1] - a[1];
  const len2 = vx * vx + vy * vy;
  const t = len2 ? Math.max(0, Math.min(1, (wx * vx + wy * vy) / len2)) : 0;
  return Math.hypot(wx - t * vx, wy - t * vy);
}

function segSegDistance(p1, p2, q1, q2) {
  return Math.min(
    pointSegDistance(p1, q1, q2), pointSegDistance(p2, q1, q2),
    pointSegDistance(q1, p1, p2), pointSegDistance(q2, p1, p2),
  );
}

/** 兩凸多邊形的最短距離；相交時回傳 0。 */
export function polyDistance(a, b) {
  if (polysOverlap(a, b)) return 0;
  let best = Infinity;
  for (let i = 0; i < a.length; i++) {
    const p1 = a[i], p2 = a[(i + 1) % a.length];
    for (let j = 0; j < b.length; j++) {
      const q1 = b[j], q2 = b[(j + 1) % b.length];
      const d = segSegDistance(p1, p2, q1, q2);
      if (d < best) best = d;
    }
  }
  return best;
}

/** 把角度正規化到 (-π, π]。 */
export function normAngle(a) {
  let r = a % (Math.PI * 2);
  if (r > Math.PI) r -= Math.PI * 2;
  if (r <= -Math.PI) r += Math.PI * 2;
  return r;
}
