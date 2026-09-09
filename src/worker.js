/**
 * 規劃器 worker。
 *
 * 最緊的組合可能要搜到 9 秒，放在主執行緒會整個畫面凍住。
 * 丟到 worker 之後，UI 照常轉、可以顯示進度、也能取消。
 */
import { buildScene, pathMinClearance } from './scene.js';
import { planParking, expandPoses } from './planner.js';
import { suggestFixes } from './advice.js';

/** 折返超過這個次數，實務上就算「停得進去也不會想停」。 */
const HARD_REVERSALS = 8;

self.onmessage = (e) => {
  const { id, cfg, mode } = e.data;
  const t0 = Date.now();
  const scene = buildScene(cfg);
  // 駛出與停入共用同一套規劃器，差別只在起點／終點怎麼給。
  const result = planParking(scene, { mode, entryStyle: cfg.entryStyle });
  const from = result.startPose || (mode === 'exit' ? scene.goal : scene.start);
  const poses = result.ok ? expandPoses(from, result.segments, scene.v.wheelbase, 0.035) : null;
  const payload = {
    id,
    mode,
    elapsed: Date.now() - t0,
    ok: result.ok,
    reason: result.reason || null,
    tier: result.tier || null,
    tierNote: result.tierNote || null,
    margin: result.margin ?? null,
    expansions: result.expansions,
    segments: result.ok ? result.segments : null,
    reversals: result.ok ? result.reversals : null,
    cost: result.ok ? result.cost : null,
    startPose: result.startPose || null,
    entry: result.entry || null,
    alternatives: result.alternatives || null,
    poses,
    minClearance: poses ? pathMinClearance(scene, poses) : null,
  };

  // 路徑先送出去（約 0.1 秒），建議另外算完再補送。
  // 綁在一起的話，使用者要為了那幾條 what-if 多等好幾秒才看得到路徑。
  const wantAdvice = !result.ok || result.reversals >= HARD_REVERSALS;
  payload.advicePending = wantAdvice;
  self.postMessage(payload);

  if (wantAdvice) {
    const advice = suggestFixes(cfg, {
      baseline: result.ok ? result.reversals : Infinity,
      entryStyle: result.entry || cfg.entryStyle,
      budgetMs: 5000,
    }).suggestions;
    self.postMessage({ id, mode, adviceOnly: true, advice });
  }
};
