/**
 * 規劃器驗證：對每個情境獨立重跑一次積分與碰撞檢查。
 *
 * 重點是「不信任規劃器自己的回報」—— 拿它輸出的指令序列，用更細的步長
 * (0.02m) 重新積分整條路徑，逐格檢查是否真的無碰撞、終點是否真的停妥。
 *
 * 執行： node test/verify.mjs
 */
import { VEHICLE, DEFAULT_SCENE, DEFAULT_OBSTACLES } from '../src/config.js';
import { buildScene, isFree, isParked, narrowestPoint } from '../src/scene.js';
import { planParking, idealMinSlotLength, expandPoses, laneRegion, parkedRegion } from '../src/planner.js';

const base = () => ({ ...DEFAULT_SCENE, obstacles: DEFAULT_OBSTACLES.map(o => ({ ...o })) });
const without = (cfg, ...ids) => {
  cfg.obstacles = cfg.obstacles.filter(o => !ids.includes(o.id));
  return cfg;
};

const CASES = [
  ['左進（車頭朝右）', () => base()],
  ['右進（車頭朝左）', () => ({ ...base(), approachFrom: 'right', parkDirection: 'left' })],
  ['指定倒車入庫', () => ({ ...base(), entryStyle: 'reverse' })],
  ['指定前進入庫', () => ({ ...base(), entryStyle: 'forward' })],
  ['對面車開走了', () => without(base(), 'oppCar')],
  ['前方機車清空', () => without(base(), 'frontScooters')],
  ['對面機車全清', () => without(base(), 'oppScooters', 'oppScooters2', 'oppBox')],
  ['巷子寬一點 6.6m', () => ({ ...base(), alleyWidth: 6.6 })],
  ['巷子窄一點 5.3m', () => ({ ...base(), alleyWidth: 5.3 })],
  ['後車往左挪 1m', () => { const c = base(); c.obstacles.find(o => o.id === 'rearCar').x -= 1; return c; }],
  ['貼牆 0.08m', () => ({ ...base(), wallGap: 0.08 })],
];

const ideal = idealMinSlotLength(VEHICLE, DEFAULT_SCENE.wallGap);
console.log(`後軸最小迴轉半徑 ${ideal.radius.toFixed(2)}m ／ 理想最小車位長度 ${ideal.length.toFixed(2)}m（車長 ${VEHICLE.length}m）\n`);

let failures = 0;
for (const [label, make] of CASES) {
  const scene = buildScene(make());
  const n = narrowestPoint(scene);
  const head = `${label}  [車位 ${scene.slot.length.toFixed(2)}m, 最窄 ${n.width.toFixed(2)}m]`;
  console.log(head);
  for (const dir of ['park', 'exit']) {
    const t = Date.now();
    const r = planParking(scene, { mode: dir, entryStyle: scene.cfg.entryStyle });
    const from = r.startPose || (dir === 'exit' ? scene.goal : scene.start);

    const ms = Date.now() - t;
    const tag = dir === 'exit' ? '駛出' : '停入';
    if (!r.ok) { console.log(`    ??  ${tag}  無解：${r.reason}（${ms}ms）`); continue; }

    const poses = expandPoses(from, r.segments, VEHICLE.wheelbase, 0.02);
    const collisions = poses.filter(p => !isFree(scene, p, 0)).length;
    const end = poses[poses.length - 1];
    // 駛出的終點是「回到車道」這個區域，不是某個座標
    const reached = dir === 'exit' ? laneRegion(scene)(end) : isParked(scene, end);
    const ok = collisions === 0 && reached;
    if (!ok) failures++;
    const style = r.entry === 'reverse' ? '倒入' : '前入';
    console.log(`    ${ok ? 'OK' : '!!'}  ${tag}  ${style} 折${r.reversals} ${String(r.segments.length).padStart(2)}段 ${r.cost.toFixed(2)}m ${r.tier.padEnd(11)} ${String(ms).padStart(5)}ms | 碰撞${collisions} 到位${reached}`);
  }
}
console.log(failures ? `\n${failures} 個情境驗證失敗` : '\n全部通過');
process.exit(failures ? 1 : 0);
