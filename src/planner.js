/**
 * 路徑規劃。
 *
 * 這是這個專案與參考版最大的差別：參考版的「示範」是人工挑好的關鍵點做線性
 * 內插，車子會橫向滑移、不做碰撞檢查、改了尺寸也不會變。這裡的路徑是「搜出來的」
 * —— 每一段都由同一個自行車模型積分產生，每一小步都做 SAT 碰撞檢查，
 * 所以巷子變窄時，答案真的會從「折 2 次」變成「折 5 次」或「無解」。
 *
 * 演算法：Hybrid A*（在連續姿態上做 A*，用離散格子去重）。
 */
import { VEHICLE, SAFETY_MARGIN } from './config.js';
import { normAngle, rectPoly, polysOverlapWithMargin } from './geometry.js';
import { integrate, bodyPolygon, minTurnRadius } from './vehicle.js';
import { isFree } from './scene.js';

// ---------- 最小二元堆 ----------
class Heap {
  constructor() { this.a = []; }
  get size() { return this.a.length; }
  push(node) {
    const a = this.a;
    a.push(node);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].f <= a[i].f) break;
      const t = a[p]; a[p] = a[i]; a[i] = t;
      i = p;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < a.length && a[l].f < a[m].f) m = l;
        if (r < a.length && a[r].f < a[m].f) m = r;
        if (m === i) break;
        const t = a[m]; a[m] = a[i]; a[i] = t;
        i = m;
      }
    }
    return top;
  }
}

// ---------- 搜尋參數 ----------
const SUB_MAX = 0.08;           // 碰撞檢查取樣間距上限（公尺）

/**
 * 取樣間距要跟安全裕度掛鉤。
 *
 * 固定 8cm 取樣時，滿舵每步車身會轉 0.02 rad，最遠的車角因此位移 5cm ——
 * 比 2cm 的裕度還大，車角會從兩個取樣點「之間」掃過障礙，
 * 於是規劃器說沒撞、重新積分卻撞得到。
 * 讓車角的單步位移不超過裕度，就不會發生。
 */
function subStep(v, margin) {
  const cornerR = Math.hypot(v.wheelbase + v.frontOverhang, v.width / 2);
  const tanMax = Math.tan(v.maxSteerDeg * Math.PI / 180);
  return Math.min(SUB_MAX, Math.max(0.015, Math.max(margin, 0.01) * v.wheelbase / (cornerR * tanMax)));
}
const REVERSE_COST = 1.10;      // 倒車略貴
const DEFAULT_SWITCH_COST = 1.30;  // 每折一次的代價，讓解偏向少折幾次
const STEER_COST = 0.35;
const DEFAULT_WEIGHT = 1.5;     // 啟發式權重，大於 1：犧牲最優性換速度

/** 停車位置偏離「貼緊前車」每公尺要付的代價，約等於兩次折返。 */
const PARK_BIAS = 8.0;

const POS_TOL = 0.16;
const ANG_TOL = 0.05;

function makeKey(grid, ang) {
  return (p) => Math.round(p.x / grid) + ',' + Math.round(p.y / grid) + ',' + Math.round(normAngle(p.theta) / ang);
}

function heuristic(p, goals) {
  let best = Infinity;
  for (const g of goals) {
    const h = Math.hypot(g.x - p.x, g.y - p.y) + Math.abs(normAngle(g.theta - p.theta)) * 1.1;
    if (h < best) best = h;
  }
  return best;
}

function atGoal(p, goals, accept) {
  // 有 accept 就代表終點是一個「區域」，goals 只是啟發式的方向參考。
  // 這時候不能再認引導點 —— 引導點本來就可能落在區域外（停入時會拿另一頭的
  // 車道姿態當方向參考），碰到它就收工會讓搜尋停在錯的那一頭。
  if (accept) return accept(p);
  return goals.some(g => Math.hypot(g.x - p.x, g.y - p.y) < POS_TOL
    && Math.abs(normAngle(g.theta - p.theta)) < ANG_TOL);
}

/**
 * 停妥的判定是一個「區域」，不是一個點。
 *
 * 原本把目標寫死成「車位正中間、離牆剛好 wallGap」，結果：停入允許 16cm 誤差沒問題，
 * 但反過來要從那個寫死的點駛出時，每個動作都撞 —— 於是出現「停得進去卻出不來」
 * 這種不可能的結論。實際開車本來就是「能停哪就停哪」，所以改成範圍判定。
 */
export function parkedRegion(scene) {
  const g = scene.goal;
  const half = scene.v.length / 2;
  // 縱向只接受「貼緊前車」附近的一段 —— 停在車位中間雖然也停得進去，
  // 但門打不開，對使用者來說等於沒停好。
  const TOL = 0.12;
  return (p) => {
    if (Math.abs(normAngle(p.theta - g.theta)) > ANG_TOL) return false;
    if (p.y < g.y - 0.06 || p.y > g.y + 0.22) return false;
    const bodyCx = p.x + scene.bodyOffset * Math.cos(p.theta);
    if (Math.abs(bodyCx - scene.parkBodyCx) > TOL) return false;
    return bodyCx - half > scene.slot.start - 0.02 && bodyCx + half < scene.slot.end + 0.02;
  };
}

/**
 * 車道區域：車已經離開車位、回到巷道上、車頭朝行進方向。
 *
 * 原本只給兩個固定的車道姿態（還沒過車位／已經過車位），但實際起手位置是浮動的 ——
 * 用折車法時根本不會乖乖停在前車旁邊。寫死成兩點會排除掉真正好用的路徑。
 *
 * side='approach' 只認「進來的那一頭」：停入的起點必須是駕駛真正開進巷子的位置，
 * 從左邊進就得在車位左邊。開過車位再倒回來是路徑的一部分，不能當成起點。
 * side='any' 兩頭都認，駛出用 —— 離開時要往哪一頭走是駕駛的自由。
 */
export function laneRegion(scene, side = 'any') {
  const half = scene.v.length / 2;
  const theta = scene.start.theta;
  const minY = scene.cfg.wallGap + scene.v.width / 2 + 0.30;
  const fromRight = scene.cfg.approachFrom === 'right';
  return (p) => {
    if (Math.abs(normAngle(p.theta - theta)) > ANG_TOL) return false;
    if (p.y < minY) return false;
    const bodyCx = p.x + scene.bodyOffset * Math.cos(p.theta);
    const pastEnd = bodyCx - half > scene.slot.end;
    const beforeStart = bodyCx + half < scene.slot.start;
    if (side === 'approach') return fromRight ? pastEnd : beforeStart;
    return pastEnd || beforeStart;
  };
}

/** 停在車位裡的幾個候選姿態 —— 駛出時當作多重起點，代表「你可能停在哪」。 */
export function parkedPoses(scene) {
  const g = scene.goal;
  const half = scene.v.length / 2;
  const lo = scene.slot.start + half;
  const hi = scene.slot.end - half;
  // 掃過整個車位，但每個位置帶一個「離理想停車位置多遠」的起始代價。
  // 搜尋因此會偏好貼緊前車的解，同時保留其他位置當退路。
  const out = [];
  for (let bodyCx = lo; bodyCx <= hi + 1e-6; bodyCx += 0.06) {
    const bias = Math.abs(bodyCx - scene.parkBodyCx) * PARK_BIAS;
    for (const dy of [0, 0.08, 0.18]) {
      out.push({ x: bodyCx - scene.bodyOffset * Math.cos(g.theta), y: g.y + dy, theta: g.theta, bias });
    }
  }
  return out.length ? out : [{ ...g, bias: 0 }];
}

/** 沿固定轉角行進 dist，逐格檢查碰撞。可行則回傳終點姿態，否則 null。 */
function driveSegment(scene, pose, steer, dist, margin, sub = SUB_MAX) {
  const n = Math.max(1, Math.ceil(Math.abs(dist) / sub));
  const d = dist / n;
  let p = pose;
  for (let i = 0; i < n; i++) {
    p = integrate(p, d, steer, scene.v.wheelbase);
    if (!isFree(scene, p, margin)) return null;
  }
  return p;
}

/**
 * 解析式收尾：若已經幾乎與目標同向、同橫向位置，直接直線開到定位。
 * 沒有這一步，Hybrid A* 很難剛好落進終點容許範圍內。
 */
function analyticFinish(scene, pose, goal, margin, sub) {
  const goals = [goal];
  // 門檻收緊到 1.4 度：直線行進不會改變車身角度，這裡放多鬆，停好的車就有多歪。
  if (Math.abs(normAngle(goal.theta - pose.theta)) > 0.025) return null;
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
  const dx = goal.x - pose.x, dy = goal.y - pose.y;
  const along = dx * c + dy * s;
  const lateral = -dx * s + dy * c;
  if (Math.abs(lateral) > 0.09 || Math.abs(along) > 6 || Math.abs(along) < 1e-4) return null;
  const end = driveSegment(scene, pose, 0, along, margin, sub);
  if (!end || !atGoal(end, goals)) return null;
  return { dist: along, end };
}

/**
 * 動作原語。轉角刻意取「打到底／打一半／回正」這種粗檔位 ——
 * 一來人本來就是這樣開車，指令才唸得出來；二來搜出來的段數自然變少。
 * 只有最緊的那一層才放開更細的檔位。
 */
function buildPrimitives(v, lengths, factors) {
  const max = v.maxSteerDeg * Math.PI / 180;
  const steers = factors.map(f => f * max);
  const prims = [];
  for (const dir of [1, -1]) {
    for (const steer of steers) {
      for (const len of lengths) prims.push({ steer, dist: dir * len, dir });
    }
  }
  return prims;
}

/**
 * 規劃 start -> goal 的可行路徑。
 * 成功：{ ok:true, segments, cost, reversals, expansions }
 * 失敗：{ ok:false, reason, expansions }
 */
export function planPath(scene, opts) {
  const o = opts || {};
  const starts = o.starts || [o.start || scene.start];
  // 終點可以有多個候選（例如車道上「還沒過車位」與「已經過了車位」兩種起手位置）
  const goals = o.goals || [o.goal || scene.goal];
  const goal = goals[0];
  const accept = o.accept || null;
  // 使用者刻意把靠牆距離設得比安全裕度還小時，不該因此判成無解 —— 把裕度夾住。
  const rawMargin = o.margin !== undefined ? o.margin : SAFETY_MARGIN;
  const margin = Math.min(rawMargin, Math.max(0.005, scene.cfg.wallGap - 0.015));
  const maxExpansions = o.maxExpansions || 70000;
  const timeBudgetMs = o.timeBudgetMs || 4000;
  const grid = o.grid || 0.20;
  const ang = o.ang || Math.PI / 18;
  const lengths = o.lengths || [0.32, 1.10];
  const factors = o.factors || [-1, -0.5, 0, 0.5, 1];
  const W = o.weight || DEFAULT_WEIGHT;
  const switchCost = o.switchCost !== undefined ? o.switchCost : DEFAULT_SWITCH_COST;
  // 從車位往外搜時的第一步方向。因為停入是把這條路徑倒著走，
  // 「離開車位的第一步」反過來就是「進入車位的最後一步」：
  //   往外第一步前進  ->  停入最後一步倒車  ->  倒車入庫
  //   往外第一步倒車  ->  停入最後一步前進  ->  前進入庫
  const firstDir = o.firstDir || 0;
  const key = makeKey(grid, ang);
  const sub = subStep(scene.v, margin);

  const roots = starts.filter(p => isFree(scene, p, 0));
  if (!roots.length) {
    return { ok: false, reason: '起點就卡住了：巷道太窄或對面車突出太多，車開不到那個位置。', expansions: 0 };
  }
  // 目標姿態用零裕度檢查：那是使用者指定的位置，不是搜尋出來的
  if (!accept && !goals.some(g => isFree(scene, g, 0))) {
    return { ok: false, reason: '目標停車位置本身放不下車：車位長度或靠牆距離不足。', expansions: 0 };
  }
  if (accept && !parkedPoses(scene).some(p => isFree(scene, p, 0))) {
    return { ok: false, reason: '車位本身放不下車：長度或靠牆距離不足。', expansions: 0 };
  }

  const prims = buildPrimitives(scene.v, lengths, factors);
  const open = new Heap();
  const best = new Map();
  const t0 = Date.now();

  // 起點全部推進去，不在這裡做格子去重。
  // 去重是有損的（同一格裡的姿態不等價），在起點就剪掉會讓「哪個起點活下來」
  // 隨偏好值變動，進而產生時有時無的假無解。
  for (const r of roots) {
    const g0 = r.bias || 0;
    const k = key(r);
    open.push({ pose: r, g: g0, f: g0 + W * heuristic(r, goals), parent: null, seg: null });
    if (!best.has(k) || best.get(k) > g0) best.set(k, g0);
  }

  let expansions = 0;
  while (open.size) {
    if ((expansions & 1023) === 0 && Date.now() - t0 > timeBudgetMs) {
      return { ok: false, reason: '搜尋逾時：這個尺寸組合太緊，找不到可行路徑。', expansions };
    }
    if (expansions > maxExpansions) {
      return { ok: false, reason: '搜尋達到節點上限，找不到不碰撞的路徑。', expansions };
    }
    const node = open.pop();
    expansions++;

    if (atGoal(node.pose, goals, accept)) return finish(node, expansions);

    // 目標是區域時不需要解析式收尾：區域夠寬，節點本來就會落進去。
    let shortcut = null;
    if (!accept) {
      for (const g of goals) {
        shortcut = analyticFinish(scene, node.pose, g, margin, sub);
        if (shortcut) break;
      }
    }
    if (shortcut) {
      const child = {
        pose: shortcut.end,
        g: node.g + Math.abs(shortcut.dist),
        f: 0,
        parent: node,
        seg: { steer: 0, dist: shortcut.dist, dir: Math.sign(shortcut.dist) || 1 },
      };
      return finish(child, expansions);
    }

    for (const pr of prims) {
      if (firstDir && node.seg === null && pr.dir !== firstDir) continue;
      const end = driveSegment(scene, node.pose, pr.steer, pr.dist, margin, sub);
      if (!end) continue;
      let g = node.g + Math.abs(pr.dist) * (pr.dir < 0 ? REVERSE_COST : 1);
      if (node.seg) {
        if (Math.sign(node.seg.dist) !== pr.dir) g += switchCost;
        g += Math.abs(node.seg.steer - pr.steer) * STEER_COST;
      }
      const k = key(end);
      const prev = best.get(k);
      if (prev !== undefined && prev <= g + 1e-6) continue;
      best.set(k, g);
      open.push({
        pose: end, g,
        f: g + W * heuristic(end, goals),
        parent: node, seg: { steer: pr.steer, dist: pr.dist, dir: pr.dir },
      });
    }
  }
  return { ok: false, reason: '搜尋空間走完了：這個車位在目前設定下停不進去。', expansions };
}

function finish(node, expansions) {
  const raw = [];
  let root = node;
  for (let n = node; n && n.seg; n = n.parent) {
    raw.unshift({ steer: n.seg.steer, dist: n.seg.dist, from: n.parent.pose });
    root = n.parent;
  }
  const segments = mergeSegments(raw);
  let reversals = 0;
  for (let i = 1; i < segments.length; i++) {
    if (Math.sign(segments[i].dist) !== Math.sign(segments[i - 1].dist)) reversals++;
  }
  const cost = segments.reduce((s, x) => s + Math.abs(x.dist), 0);
  return { ok: true, segments, cost, reversals, expansions, startPose: root.pose };
}

/** 把連續同轉角同方向的小段併成一段指令，方便播放與敘述。 */
function mergeSegments(raw) {
  const out = [];
  for (const s of raw) {
    const last = out[out.length - 1];
    if (last && Math.abs(last.steer - s.steer) < 1e-9 && Math.sign(last.dist) === Math.sign(s.dist)) {
      last.dist += s.dist;
    } else {
      out.push({ steer: s.steer, dist: s.dist, from: s.from });
    }
  }
  return out;
}

/** 把指令序列展開成密集姿態序列（動畫、掃掠面積、逐步檢視都用它）。 */
export function expandPoses(start, segments, wheelbase, stepLen) {
  const step = stepLen || 0.05;
  const poses = [{ x: start.x, y: start.y, theta: start.theta, steer: 0, dir: 0, seg: -1 }];
  let p = start;
  segments.forEach((s, si) => {
    const n = Math.max(1, Math.ceil(Math.abs(s.dist) / step));
    const d = s.dist / n;
    for (let i = 0; i < n; i++) {
      p = integrate(p, d, s.steer, wheelbase);
      poses.push({ x: p.x, y: p.y, theta: p.theta, steer: s.steer, dir: Math.sign(s.dist), seg: si });
    }
  });
  return poses;
}

// ---------- 理想最小車位長度 ----------

/**
 * 用「對稱雙圓弧倒車入位」求單次進入所需的最短車位長度。
 *
 * 不套文獻公式，而是直接用本專案的車輛模型模擬、用同一套 SAT 檢查，
 * 對車位長度做二分搜尋 —— 這個數字與畫面上跑的物理完全同源。
 *
 * 場景理想化：無限寬巷道、車位前後為直牆，靠牆側留 wallGap。
 * 這就是教科書上那個數字，只是誠實算出來的。
 */
export function idealMinSlotLength(v, wallGap, margin) {
  const veh = v || VEHICLE;
  const gap = wallGap !== undefined ? wallGap : 0.20;
  const m = margin !== undefined ? margin : 0.02;
  const R = minTurnRadius(veh);
  const maxSteer = veh.maxSteerDeg * Math.PI / 180;

  const feasible = (S) => {
    // 最小化停車時，車尾是貼著後車的 —— 置中會白白浪費一半的餘裕。
    const goal = { x: veh.rearOverhang + 0.02, y: gap + veh.width / 2, theta: 0 };
    // 前後各停一台車（深度 1.9m，一般車寬），不是無限高的牆 ——
    // 否則車頭永遠沒辦法擺出角度，會算出無解。
    const obs = [
      rectPoly(-8, 0, 8, 1.9),      // 後車
      rectPoly(S, 0, 8, 1.9),       // 前車
      rectPoly(-16, -4, 40, 4),     // 住家牆面 / 路緣（y < 0）
    ];
    const hits = (p) => {
      const body = bodyPolygon(p, veh);
      return obs.some(o => polysOverlapWithMargin(body, o, m));
    };
    // 掃描橫向偏移 k（起始車道線與停妥線的距離），取任一可行者
    for (let k = 2.2; k >= 0.35; k -= 0.05) {
      const cosT = 1 - k / (2 * R);
      if (cosT < -1 || cosT > 1) continue;
      const theta = Math.acos(cosT);
      const arc = R * theta;
      // 從停妥姿態反推：先 +δmax 前進 arc，再 -δmax 前進 arc，
      // 終點即為起始姿態；沿途姿態集合與正著倒車進來時完全相同。
      let p = goal;
      let ok = true;
      for (const pair of [[maxSteer, arc], [-maxSteer, arc]]) {
        const n = Math.max(2, Math.ceil(pair[1] / 0.05));
        for (let i = 0; i < n; i++) {
          p = integrate(p, pair[1] / n, pair[0], veh.wheelbase);
          if (hits(p)) { ok = false; break; }
        }
        if (!ok) break;
      }
      if (ok) return true;
    }
    return false;
  };

  let lo = veh.length, hi = veh.length + 4;
  if (!feasible(hi)) return { length: NaN, radius: R };
  for (let i = 0; i < 22; i++) {
    const mid = (lo + hi) / 2;
    if (feasible(mid)) hi = mid; else lo = mid;
  }
  return { length: hi, radius: R };
}

/**
 * 對外的主要進入點：分層嘗試，先求「好開的解」，找不到才逐步放寬。
 *
 * 分層的用意是讓 App 能講出有意義的話：
 *   第 1 層成功 -> 「正常餘裕下可停」
 *   第 2 層成功 -> 「要貼到只剩 2cm 才進得去」
 *   全部失敗   -> 「這個尺寸真的停不進去」
 */
/** 「好開程度」：總行程加上每次折返的代價。數字越小越好開。 */
export function maneuverScore(plan) {
  return plan.cost + plan.reversals * 0.8;
}

export function planParking(scene, opts) {
  const o = { ...(opts || {}) };

  // 「自動」不是隨便挑一個，而是把倒車與前進各搜一次再比。
  //
  // 單靠不設限的搜尋不可靠：實測現場配置時它挑了前進入庫（35.2m），
  // 但倒車入庫其實只要 31.2m —— 它只是沒搜到。
  // 而且平行停車本來就該倒車入庫比較有利（轉向輪在前，倒車時後軸當支點），
  // 對面淨空的對照實驗裡總行程 8m vs 14m，差距很明顯。
  if (o.entryStyle === 'any' || o.entryStyle === undefined) {
    const tries = ['reverse', 'forward']
      .map(st => ({ st, r: planParking(scene, { ...opts, entryStyle: st }) }))
      .filter(x => x.r.ok);
    if (!tries.length) return planParking(scene, { ...opts, entryStyle: 'reverse' });
    tries.sort((a, b) => maneuverScore(a.r) - maneuverScore(b.r));
    const best = tries[0];
    return {
      ...best.r,
      alternatives: tries.map(x => ({
        entry: x.st, cost: x.r.cost, reversals: x.r.reversals, score: maneuverScore(x.r),
      })),
    };
  }

  const wantPark = o.mode !== 'exit';
  delete o.mode;

  // 一律從「車位裡」往外搜，停入就是把結果倒著走。
  //
  // 自行車模型是時間可逆的，所以反轉出來的路徑一樣合法。而方向差很多：
  // 從車道往死角搜，啟發式一路指向口袋深處，得在極窄的空間裡碰運氣；
  // 從車位往外搜則是往越來越開闊的地方展開，同樣的預算下好找得多。
  // 之前「出得來卻進不去」這種不可能的結論，就是搜錯方向造成的。
  o.starts = parkedPoses(scene);
  const style = o.entryStyle;
  if (style === 'reverse') o.firstDir = 1;        // 倒車入庫
  else if (style === 'forward') o.firstDir = -1;  // 前進入庫
  delete o.entryStyle;

  // 終點（駛出方向）是「回到巷道」這個區域，不是某一個座標 ——
  // 起手位置本來就是浮動的，寫死會排除掉真正好用的路徑。
  // 兩個代表性車道姿態仍然保留，但只當啟發式的方向參考：同一個區域用不同方向
  // 引導，搜到的路徑不一樣（格子去重是有損的），兩個都試才不會漏。
  // 停入時只接受「進來的那一頭」當起點，駛出時兩頭都行。
  o.accept = laneRegion(scene, wantPark ? 'approach' : 'any');
  const lanes = scene.lanePoses || [scene.start];
  // 兩個車道姿態都只是啟發式的方向參考。停入時終點固定在進來的那一頭，
  // 所以先用該側引導；引導到另一頭仍然有用 —— 倒車入庫的路徑本來就會先繞過去。
  const laneOrder = wantPark
    ? [lanes[0], lanes[1]]
    : (style === 'reverse' ? [lanes[1], lanes[0]] : [lanes[0], lanes[1]]);

  const tiers = [
    // 第 0 層：低貪婪度，先試著找「開起來漂亮」的路徑（少折、少無謂動作）。
    // 空間充裕時幾乎都能在預算內找到；找不到就交給下面的貪婪層。
    { label: 'clean', note: '餘裕充足', margin: 0.10, grid: 0.18, ang: Math.PI / 20, lengths: [0.32, 1.10], factors: [-1, -0.5, 0, 0.5, 1], weight: 1.04, maxExpansions: 40000, timeBudgetMs: 900 },
    { label: 'comfortable', note: '餘裕充足', margin: 0.10, grid: 0.20, ang: Math.PI / 18, lengths: [0.32, 1.10], factors: [-1, -0.5, 0, 0.5, 1], maxExpansions: 60000, timeBudgetMs: 1500 },
    { label: 'tight', note: '偏緊', margin: 0.04, grid: 0.15, ang: Math.PI / 24, lengths: [0.22, 0.75], factors: [-1, -0.5, 0, 0.5, 1], maxExpansions: 130000, timeBudgetMs: 3800 },
    { label: 'extreme', note: '極限操作', margin: 0.01, grid: 0.11, ang: Math.PI / 30, lengths: [0.16, 0.55], factors: [-1, -0.72, -0.45, -0.2, 0, 0.2, 0.45, 0.72, 1], maxExpansions: 220000, timeBudgetMs: 5200 },
    // 搓車層：車位前後被深障礙夾住時，唯一的出路是反覆小幅前後搓、每次多轉幾度。
    // 這種解需要「很細的角度格子」，否則搓兩下還落在同一格，會被去重直接剪掉；
    // 同時要調低換檔代價，不然搜尋會一直嫌折返太貴而繞遠路。
    { label: 'shuffle', note: '大量折返搓車', margin: 0.02, grid: 0.09, ang: Math.PI / 90, lengths: [0.16, 0.55], factors: [-1, -0.5, 0, 0.5, 1], switchCost: 0.35, maxExpansions: 160000, timeBudgetMs: 6000 },
    // 換一組格距再試一次。Hybrid A* 的格子去重是有損的，同一個場景會因為
    // 格線剛好切在哪裡而時有時無 —— 只差 5cm 的設定一個有解一個無解，
    // 那是離散化的運氣，不是幾何。換格距重試能把這種假無解濾掉。
    { label: 'shuffle2', note: '大量折返搓車', margin: 0.02, grid: 0.065, ang: Math.PI / 72, lengths: [0.13, 0.42], factors: [-1, -0.6, -0.3, 0, 0.3, 0.6, 1], switchCost: 0.30, maxExpansions: 200000, timeBudgetMs: 7000 },
  ];
  let last = null;
  for (const t of tiers) {
    for (const laneGoal of laneOrder) {
    const r = planPath(scene, { ...t, ...o, goal: laneGoal, goals: [laneGoal] });
    if (r.ok) {
      const out = { ...r, tier: t.label, tierNote: t.note, margin: t.margin };
      const final = wantPark ? reversePlan(out, scene.v) : out;
      const lastPark = (wantPark ? final.segments : [...final.segments].reverse().map(x => ({ dist: -x.dist })))
        .slice(-1)[0];
      final.entry = lastPark && lastPark.dist < 0 ? 'reverse' : 'forward';
      return final;
    }
    last = r;
    }
    // 目標位置本身放不下 -> 再細也沒用
    if (last && last.expansions === 0) break;
  }
  // 搜尋在幾個節點內就走完，代表車子從起點根本動不了，跟「搜不到」是兩回事
  if (last && last.expansions > 0 && last.expansions < 60) {
    return { ok: false, reason: '車子在這個位置幾乎動彈不得：前後與側向都不夠讓車身轉出角度。', expansions: last.expansions };
  }
  return { ok: false, reason: last ? last.reason : '無法規劃', expansions: last ? last.expansions : 0 };
}

/** 把一條路徑倒過來走：段序顛倒、距離變號、轉角不變，起點換成原本的終點。 */
function reversePlan(plan, v) {
  let end = plan.startPose;
  for (const seg of plan.segments) {
    const n = Math.max(1, Math.ceil(Math.abs(seg.dist) / 0.02));
    for (let i = 0; i < n; i++) end = integrate(end, seg.dist / n, seg.steer, v.wheelbase);
  }
  const segments = [...plan.segments].reverse().map(s => ({ steer: s.steer, dist: -s.dist }));
  let reversals = 0;
  for (let i = 1; i < segments.length; i++) {
    if (Math.sign(segments[i].dist) !== Math.sign(segments[i - 1].dist)) reversals++;
  }
  return { ...plan, segments, reversals, startPose: end };
}
