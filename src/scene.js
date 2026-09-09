/**
 * 由設定值產生場景（障礙物、起點、目標），並提供碰撞與間隙查詢。
 *
 * 座標：x 沿巷道向右，y = 與住家牆面的距離（0 = 牆面，往下為對面建物）。
 * 車頭朝 +x 行進時，住家牆面在畫面上方，也就是駕駛的左側。
 *
 * 「可用車位長度」是算出來的，不是輸入的：它就是住家側兩個障礙物之間的空隙。
 * 把後方那台車往前挪，可用長度就跟著變 —— 這才符合實際量測的直覺。
 *
 * 碰撞效能備註：所有障礙都是軸對齊矩形，所以不用通用 SAT，
 * 改用 OBB(車) vs AABB(障礙) 的四軸特化版本，每個障礙約 20 次浮點運算。
 */
import { VEHICLE, SAFETY_MARGIN, makeVehicle } from './config.js';
import { rectPoly, polyDistance, normAngle } from './geometry.js';
import { bodyPolygon, chargePort } from './vehicle.js';

function box(def, x, y, w, h) {
  return {
    id: def.id, label: def.label, kind: def.kind, side: def.side,
    poly: rectPoly(x, y, w, h),
    cx: x + w / 2, cy: y + h / 2, hx: w / 2, hy: h / 2,
  };
}

/** 住家側障礙物之間的空隙，取離 targetX 最近的那一段當作要停的車位。 */
function findSlot(items, targetX, xMin, xMax) {
  const spans = items
    .filter(o => o.on && o.side === 'house' && o.depth > 0.35)
    .map(o => [o.x - o.length / 2, o.x + o.length / 2])
    .sort((a, b) => a[0] - b[0]);

  const merged = [];
  for (const s of spans) {
    const last = merged[merged.length - 1];
    if (last && s[0] <= last[1]) last[1] = Math.max(last[1], s[1]);
    else merged.push([s[0], s[1]]);
  }

  const gaps = [];
  let cursor = xMin;
  for (const [a, b] of merged) {
    if (a > cursor) gaps.push([cursor, a]);
    cursor = Math.max(cursor, b);
  }
  if (cursor < xMax) gaps.push([cursor, xMax]);
  if (!gaps.length) return { start: targetX - 2, end: targetX + 2, length: 4 };

  const dist = (g) => (targetX >= g[0] && targetX <= g[1])
    ? 0
    : Math.min(Math.abs(targetX - g[0]), Math.abs(targetX - g[1]));
  let best = gaps[0];
  for (const g of gaps) if (dist(g) < dist(best)) best = g;
  return { start: best[0], end: best[1], length: best[1] - best[0] };
}

/**
 * 某段 x 範圍內車體可以走的橫向通道 [top, bottom]。
 * 起點要放哪、巷子哪裡被夾住，都靠這個算。
 */
export function corridorBetween(obstacles, x0, x1, alleyWidth) {
  let top = 0, bottom = alleyWidth;
  for (const o of obstacles) {
    if (o.kind === 'wall') continue;
    if (o.cx + o.hx < x0 || o.cx - o.hx > x1) continue;
    const y0 = o.cy - o.hy, y1 = o.cy + o.hy;
    if (y0 < 0.01) top = Math.max(top, y1);      // 貼住家側
    else bottom = Math.min(bottom, y0);          // 貼對面側
  }
  return [top, bottom];
}

export function buildScene(cfg, vIn) {
  // 車輛規格可調，最大轉角每次由迴轉半徑推導
  const v = vIn || (cfg.vehicle ? makeVehicle(cfg.vehicle) : VEHICLE);
  const items = cfg.obstacles || [];
  const active = items.filter(o => o.on);
  const W = cfg.alleyWidth;

  const rightMost = active.reduce((m, o) => Math.max(m, o.x + o.length / 2), 12);
  const leftMost = active.reduce((m, o) => Math.min(m, o.x - o.length / 2), 0);
  const xMin = Math.min(0, leftMost) - 7;
  const xMax = rightMost + 7;

  const obstacles = [
    box({ id: 'house', label: '住家牆面', kind: 'wall', side: 'house' }, xMin - 3, -2.4, xMax - xMin + 6, 2.4),
    box({ id: 'far', label: '對面建物', kind: 'wall', side: 'far' }, xMin - 3, W, xMax - xMin + 6, 2.4),
  ];
  for (const o of active) {
    const y = o.side === 'house' ? 0 : W - o.depth;
    const b = box(o, o.x - o.length / 2, y, o.length, o.depth);
    b.def = o;
    obstacles.push(b);
  }

  const slot = findSlot(items, cfg.targetX, xMin, xMax);
  const bodyOffset = (v.wheelbase + v.frontOverhang - v.rearOverhang) / 2;
  const parkLeft = cfg.parkDirection === 'left';
  const goalTheta = parkLeft ? Math.PI : 0;
  const slotCentre = (slot.start + slot.end) / 2;

  // 停妥位置：貼緊車位旁邊那台「汽車」，不是停在車位正中間，也不是貼機車。
  //
  // 一開始是依車頭方向挑「前面那一端」，但實際上不管從哪個方向停都要貼汽車 ——
  // 汽車是固定又平整的參考物，機車歪歪斜斜又會被挪動，貼過去沒有意義。
  const gap = Math.max(0.02, cfg.frontGap ?? 0.15);
  const half = v.length / 2;
  const edgeAt = (x) => items.find(o => o.on && o.side === 'house' && o.depth > 0.35
    && (Math.abs(o.x + o.length / 2 - x) < 0.06 || Math.abs(o.x - o.length / 2 - x) < 0.06));
  const carAtStart = edgeAt(slot.start)?.kind === 'car';
  const carAtEnd = edgeAt(slot.end)?.kind === 'car';
  const tightToStart = carAtStart !== carAtEnd ? carAtStart : parkLeft;
  const wantCx = tightToStart ? slot.start + gap + half : slot.end - gap - half;
  const bodyCx = Math.min(Math.max(wantCx, slot.start + half), slot.end - half);
  const goal = {
    x: bodyCx - bodyOffset * Math.cos(goalTheta),
    y: cfg.wallGap + v.width / 2,
    theta: goalTheta,
  };

  // 起點的車頭朝向由「從哪一頭開進來」決定，跟停妥朝向無關。
  const fromRight = cfg.approachFrom === 'right';
  const startTheta = fromRight ? Math.PI : 0;

  // 起點有兩個候選，因為真實駕駛的起手式本來就有兩種：
  //   before  還沒開到車位（前進入庫從這裡開始）
  //   past    已經開過車位（倒車入庫必須先到這裡）
  // 兩個都丟給規劃器，哪個到得了就用哪個。只給一個等於替駕駛決定了手法。
  const front = v.wheelbase + v.frontOverhang;
  const dir = fromRight ? -1 : 1;                    // 行進方向（+1 = 往 x 大的方向）

  /** 給定車頭或車尾的位置，算出完整的車道姿態（橫向置於當地可走通道的中央）。 */
  const laneAt = (anchorX, anchor) => {
    const sx = anchor === 'nose' ? anchorX - dir * front : anchorX + dir * v.rearOverhang;
    const noseX = sx + dir * front;
    const rearX = sx - dir * v.rearOverhang;
    const [top, bottom] = corridorBetween(
      obstacles, Math.min(rearX, noseX), Math.max(rearX, noseX), W);
    const lo = top + v.width / 2 + 0.02;
    const hi = Math.max(bottom - v.width / 2 - 0.02, lo);
    return { x: sx, y: Math.min(Math.max((top + bottom) / 2, lo), hi), theta: startTheta };
  };

  // 前進入庫：還沒開到車位，車頭停在車位邊緣前 0.5m。
  const before = laneAt(fromRight ? slot.end + 0.50 : slot.start - 0.50, 'nose');
  // 倒車入庫：已經開過車位、與前車並排 —— 車尾大約與車位前緣切齊，
  // 這才是實際的起手式；把車跨在空車位上倒不進去。
  const past = laneAt(fromRight ? slot.start - 0.30 : slot.end + 0.30, 'rear');
  // 倒車入庫的起手式本來就是「先開過車位」，畫面上的起點要跟著換，
  // 不然選了倒車入庫卻什麼都沒變，看起來像沒生效。
  const start = cfg.entryStyle === 'reverse' ? past : before;

  return {
    cfg, v, obstacles, goal, slot, bodyOffset,
    charger: cfg.charger ? { x: cfg.chargerX, y: 0 } : null,
    gate: cfg.gate ? { start: cfg.gateX - cfg.gateWidth / 2, end: cfg.gateX + cfg.gateWidth / 2 } : null,
    farGate: cfg.farGate ? { start: cfg.farGateX - cfg.farGateWidth / 2, end: cfg.farGateX + cfg.farGateWidth / 2 } : null,
    parkBodyCx: bodyCx,
    tightTo: tightToStart ? 'start' : 'end',
    tightLabel: (tightToStart ? edgeAt(slot.start) : edgeAt(slot.end))?.label || '鄰車',
    start,
    lanePoses: [before, past],
    xMin, xMax, worldLength: xMax - xMin,
    spare: slot.length - v.length,
  };
}

/**
 * 車體在此姿態是否無碰撞（含安全裕度）。
 * OBB vs AABB 的分離軸測試，只需四個軸。
 */
export function isFree(scene, pose, margin = SAFETY_MARGIN) {
  const v = scene.v;
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
  const cx = pose.x + scene.bodyOffset * c;
  const cy = pose.y + scene.bodyOffset * s;
  const hx = v.length / 2, hy = v.width / 2;
  const ac = Math.abs(c), as = Math.abs(s);
  const rx = hx * ac + hy * as;
  const ry = hx * as + hy * ac;
  const obs = scene.obstacles;
  for (let i = 0; i < obs.length; i++) {
    const o = obs[i];
    const dx = o.cx - cx, dy = o.cy - cy;
    const ohx = o.hx + margin, ohy = o.hy + margin;
    if (Math.abs(dx) > ohx + rx) continue;
    if (Math.abs(dy) > ohy + ry) continue;
    if (Math.abs(dx * c + dy * s) > hx + ohx * ac + ohy * as) continue;
    if (Math.abs(-dx * s + dy * c) > hy + ohx * as + ohy * ac) continue;
    return false;
  }
  return true;
}

/** 撞到誰？供 UI 顯示原因用。 */
export function collide(scene, pose, margin = SAFETY_MARGIN) {
  const v = scene.v;
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
  const cx = pose.x + scene.bodyOffset * c, cy = pose.y + scene.bodyOffset * s;
  const hx = v.length / 2, hy = v.width / 2;
  const ac = Math.abs(c), as = Math.abs(s);
  const rx = hx * ac + hy * as, ry = hx * as + hy * ac;
  for (const o of scene.obstacles) {
    const dx = o.cx - cx, dy = o.cy - cy;
    const ohx = o.hx + margin, ohy = o.hy + margin;
    if (Math.abs(dx) > ohx + rx) continue;
    if (Math.abs(dy) > ohy + ry) continue;
    if (Math.abs(dx * c + dy * s) > hx + ohx * ac + ohy * as) continue;
    if (Math.abs(-dx * s + dy * c) > hy + ohx * as + ohy * ac) continue;
    return o;
  }
  return null;
}

/** 各障礙與車體的最短距離，由近到遠。 */
export function clearances(scene, pose) {
  const body = bodyPolygon(pose, scene.v);
  return scene.obstacles
    .map(o => ({ id: o.id, label: o.label, distance: polyDistance(body, o.poly) }))
    .sort((a, b) => a.distance - b.distance);
}

/**
 * 四個車角各自離最近障礙多遠 —— 實際開車時真正在意的數字。
 *
 * 刻意不用「左前／右後」：車頭朝左或朝右時，同一個角的左右會翻轉，
 * 看圖的人和坐在車上的人講的又是不同的左右。改用「牆側／外側」+「前／後」，
 * 不管車身轉到哪個方向都指同一個角。
 */
export function cornerClearances(scene, pose) {
  const body = bodyPolygon(pose, scene.v);
  const isFront = [false, true, true, false];   // bodyPolygon 的角順序
  const centreY = pose.y + scene.bodyOffset * Math.sin(pose.theta);
  const names = body.map((pt, i) =>
    (pt[1] < centreY ? '牆側' : '外側') + (isFront[i] ? '前角' : '後角'));
  return body.map((pt, i) => {
    let best = Infinity, who = null;
    for (const o of scene.obstacles) {
      const d = polyDistance([pt, pt, pt], o.poly);
      if (d < best) { best = d; who = o.label; }
    }
    return { corner: names[i], point: pt, distance: best, label: who };
  });
}

/** 是否算停妥：位置、角度都在容許範圍內，且完全沒碰到。 */
export function isParked(scene, pose) {
  const g = scene.goal;
  const dTheta = Math.abs(normAngle(pose.theta - g.theta));
  const bodyCx = pose.x + scene.bodyOffset * Math.cos(pose.theta);
  const half = scene.v.length / 2;
  return Math.abs(pose.y - g.y) < 0.25
    && bodyCx - half > scene.slot.start - 0.05
    && bodyCx + half < scene.slot.end + 0.05
    && dTheta < 0.09
    && collide(scene, pose, 0) === null;
}

/** 一條路徑全程的最小間隙，以及發生在哪個障礙。 */
export function pathMinClearance(scene, poses) {
  let best = Infinity, at = null, idx = 0;
  for (let i = 0; i < poses.length; i += 3) {
    const c = clearances(scene, poses[i])[0];
    if (c && c.distance < best) { best = c.distance; at = c.label; idx = i; }
  }
  return { distance: best, label: at, index: idx };
}

/**
 * 巷道沿線最窄的地方。
 * 現場配置圖上車位前方那段兩側都停了機車，淨寬只剩 2.3m ——
 * 這種瓶頸直接決定「能不能先開過車位再倒車」，值得單獨算出來講。
 */
export function narrowestPoint(scene) {
  let best = { width: Infinity, x: scene.xMin };
  for (let x = scene.xMin; x <= scene.xMax; x += 0.15) {
    const [top, bottom] = corridorBetween(scene.obstacles, x, x, scene.cfg.alleyWidth);
    const w = bottom - top;
    if (w < best.width) best = { width: w, x };
  }
  return best;
}

/** 充電口與充電器的距離（公尺）。停哪個方向比較好充，看這個數字就知道。 */
export function chargeReach(scene, pose) {
  if (!scene.charger) return null;
  const p = chargePort(pose, scene.v, scene.cfg.portFromNose);
  return { distance: Math.hypot(p[0] - scene.charger.x, p[1] - scene.charger.y), port: p };
}

/** 人要側身通過所需的最小寬度。低於這個數字就等於出不去。 */
export const PASS_WIDTH = 0.55;

/**
 * 車停好之後大門還剩多寬能走人。
 *
 * 大門是牆上的開口、不是障礙物，所以不進碰撞檢查。而且以現場尺寸來說
 * 它必然會被擋掉一部分（車位起點就在大門旁邊），所以重點不是「有沒有擋到」，
 * 而是「剩下的連續開口還走不走得過去」—— 這正是必須貼緊鄰車的原因。
 */
export function gateBlock(scene, pose) {
  if (!scene.gate) return null;
  const { start, end } = scene.gate;
  const half = scene.v.length / 2;
  const cx = pose.x + scene.bodyOffset * Math.cos(pose.theta);
  const carA = cx - half, carB = cx + half;
  const blocked = Math.max(0, Math.min(carB, end) - Math.max(carA, start));
  const leftFree = Math.max(0, Math.min(carA, end) - start);
  const rightFree = Math.max(0, end - Math.max(carB, start));
  const free = Math.max(leftFree, rightFree);
  return { blocked, free, width: end - start, ok: free >= PASS_WIDTH, leftFree, rightFree };
}

/**
 * 對面大門還剩多寬能走人。它擋不到你停車，但擋住別人家的門一樣不行 ——
 * 而且門前只停得下一台機車，這件事直接影響巷道最窄處落在哪裡。
 * 這裡看的是「對面側的障礙物」有沒有蓋住它，不是你的車。
 */
export function farGateBlock(scene) {
  if (!scene.farGate) return null;
  const { start, end } = scene.farGate;
  const spans = scene.obstacles
    .filter(o => o.side === 'far' && o.kind !== 'wall')
    .map(o => [Math.max(start, o.cx - o.hx), Math.min(end, o.cx + o.hx)])
    .filter(([a, b]) => b > a)
    .sort((a, b) => a[0] - b[0]);
  let free = 0, cursor = start;
  for (const [a, b] of spans) {
    free = Math.max(free, a - cursor);
    cursor = Math.max(cursor, b);
  }
  free = Math.max(free, end - cursor);
  const blocked = spans.reduce((t, [a, b]) => t + (b - a), 0);
  return { free, blocked, width: end - start, ok: free >= PASS_WIDTH };
}
