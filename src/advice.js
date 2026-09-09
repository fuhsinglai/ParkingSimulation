/**
 * 停不進去的時候，回答「那要改什麼才停得進去」。
 *
 * 做法很直接：把每一個可以在現實中動的東西各動一點，重跑一次規劃，
 * 看哪些改動能讓問題變成有解，並回報「最小的改動量」。
 *
 * 因為規劃器現在是從車位往外搜（見 planner.js），失敗的情況幾毫秒就會走完，
 * 所以跑上百次 what-if 也還在互動可接受的時間內。
 */
import { buildScene } from './scene.js';
import { planParking } from './planner.js';

const clone = (cfg) => ({ ...cfg, obstacles: cfg.obstacles.map(o => ({ ...o })) });
const find = (cfg, id) => cfg.obstacles.find(o => o.id === id);

/** 沿巷道挪動時的嘗試量，由小到大，找到就停 —— 回報的是「最少要挪多少」。 */
const SHIFTS = [0.3, 0.6, 1.0, 1.5, 2.0, 3.0];
/** 機車改停法後可能的突出深度（斜停、靠牆停、改停別處）。 */
const DEPTHS = [1.5, 1.2, 0.9, 0.6];

/**
 * @param opts.baseline 目前這個設定要折幾次（無解時給 Infinity）。
 *   只有能明顯減少折返次數的改動才值得講 —— 「挪了 0.3m 還是要折 34 次」不是建議。
 */
export function suggestFixes(cfg, opts = {}) {
  const budgetMs = opts.budgetMs || 9000;
  const baseline = opts.baseline ?? Infinity;
  // what-if 一律用固定的入庫方式。若沿用「自動」，每次嘗試都會變成兩次搜尋，
  // 上百次 what-if 就把時間翻倍 —— 主要規劃已經比較過了，這裡不需要再比一次。
  const style = opts.entryStyle && opts.entryStyle !== 'any' ? opts.entryStyle : 'reverse';
  const worthIt = (r) => r.reversals <= (Number.isFinite(baseline) ? baseline - 4 : Infinity);
  const t0 = Date.now();
  const found = [];
  const active = cfg.obstacles.filter(o => o.on);

  const attempt = (mutate) => {
    if (Date.now() - t0 > budgetMs) return null;
    const c = clone(cfg);
    mutate(c);
    const scene = buildScene(c);
    const r = planParking(scene, { mode: 'exit', entryStyle: style, maxExpansions: 30000, timeBudgetMs: 400 });
    return r.ok && worthIt(r) ? r : null;
  };

  // 1) 整個移走
  for (const o of active) {
    const r = attempt(c => { find(c, o.id).on = false; });
    if (r) found.push({
      text: `把「${o.label}」移走`,
      detail: `折 ${r.reversals} 次`,
      effort: o.kind === 'car' ? 40 : 12,
    });
  }

  // 2) 沿巷道挪開 —— 對機車來說這是最容易做到的事
  for (const o of active) {
    for (const dir of [1, -1]) {
      for (const d of SHIFTS) {
        const r = attempt(c => { find(c, o.id).x += dir * d; });
        if (r) {
          found.push({
            text: `把「${o.label}」沿巷道往${dir > 0 ? '右' : '左'}挪 ${d.toFixed(1)}m`,
            detail: `折 ${r.reversals} 次`,
            effort: (o.kind === 'car' ? 14 : 4) * d,
          });
          break;
        }
      }
    }
  }

  // 3) 機車改停法，少突出一點
  for (const o of active.filter(x => x.kind === 'scooter')) {
    for (const nd of DEPTHS) {
      if (nd >= o.depth - 0.05) continue;
      const r = attempt(c => { find(c, o.id).depth = nd; });
      if (r) {
        found.push({
          text: `「${o.label}」改停法（斜停或靠牆），突出從 ${o.depth.toFixed(2)}m 縮到 ${nd.toFixed(2)}m`,
          detail: `折 ${r.reversals} 次`,
          effort: 6 + (o.depth - nd) * 4,
        });
        break;
      }
    }
  }

  // 4) 自己少貼一點牆 —— 完全不用麻煩別人，優先推薦
  for (const g of [0.3, 0.4, 0.5, 0.6]) {
    if (g <= cfg.wallGap + 0.02) continue;
    const r = attempt(c => { c.wallGap = g; });
    if (r) {
      found.push({
        text: `不要停那麼貼牆：靠牆距離放到 ${g.toFixed(2)}m`,
        detail: `折 ${r.reversals} 次`,
        effort: 2,
      });
      break;
    }
  }

  // 5) 換從巷子另一頭開進來（車頭朝向會跟著反過來，不需要調頭）
  const flip = (v) => (v === 'left' ? 'right' : 'left');
  const other = flip(cfg.approachFrom);
  const r5 = attempt(c => { c.approachFrom = other; c.parkDirection = flip(other); });
  if (r5) found.push({
    text: `改從巷子${other === 'left' ? '左' : '右'}邊開進來（車頭會變成朝${flip(other) === 'left' ? '左' : '右'}）`,
    detail: `折 ${r5.reversals} 次`,
    effort: 4,
  });

  found.sort((a, b) => a.effort - b.effort);
  // 同一個障礙只留最省力的那個建議，免得清單都在講同一件事
  const seen = new Set();
  const out = [];
  for (const f of found) {
    const key = f.text.match(/「(.+?)」/)?.[1] || f.text;
    if (seen.has(key)) continue;
    seen.add(key);
    out.push(f);
    if (out.length >= 5) break;
  }
  return { suggestions: out, elapsed: Date.now() - t0 };
}
