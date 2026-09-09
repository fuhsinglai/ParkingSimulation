/**
 * UI 組裝層：把設定、規劃器、模擬與繪圖接起來。
 *
 * 只有一個 requestAnimationFrame 迴圈，而且是「需要時才重畫」——
 * 參考版有兩個永不停止的 rAF 迴圈，分頁在背景時照樣燒 CPU。
 */
import { VEHICLE, DEFAULT_SCENE, DEFAULT_OBSTACLES, SCOOTER_PITCH, CHALLENGES, makeVehicle, steerFromTurningRadius } from './config.js';
import { buildScene, collide, cornerClearances, clearances, isParked, narrowestPoint, chargeReach, gateBlock, farGateBlock, PASS_WIDTH } from './scene.js';
import { idealMinSlotLength, expandPoses } from './planner.js';
import { integrate, describeSteer } from './vehicle.js';
import { makeView, drawScene, drawCar, drawTrace, drawSweep, drawClearances } from './render.js';

const $ = (s) => document.querySelector(s);
const canvas = $('#scene');
const ctx = canvas.getContext('2d');

// ---------------------------------------------------------------- 設定保存
//
// 場景是使用者一格一格量出來、拖出來的，重新整理就沒了很難用。
// 存在 localStorage；讀回來時與預設值合併，之後新增欄位才不會讓舊資料壞掉。

const STORE_KEY = 'rav4-parking-sim/v1';

/** 保存狀態，開發時用來確認到底是「沒存到」還是「沒讀到」。 */
let storeState = '未儲存';

function loadSaved() {
  let raw;
  try {
    raw = localStorage.getItem(STORE_KEY);
  } catch (err) {
    storeState = '瀏覽器不允許儲存（無痕視窗或封鎖網站資料）';
    return null;
  }
  if (!raw) { storeState = '這台裝置還沒存過設定'; return null; }
  try {
    const saved = JSON.parse(raw);
    if (!saved || typeof saved !== 'object') throw new Error('格式不符');
    if (!Array.isArray(saved.cfg?.obstacles) || !saved.cfg.obstacles.length) throw new Error('缺少障礙物');
    storeState = `已還原 ${saved.at ? new Date(saved.at).toLocaleString('zh-TW') : '上次'} 的設定`;
    return saved;
  } catch (err) {
    storeState = '存檔讀不出來（格式不符），已改用預設值';
    return null;
  }
}

const saved = loadSaved();
const cfg = {
  ...DEFAULT_SCENE,
  ...(saved?.cfg || {}),
  vehicle: { ...DEFAULT_SCENE.vehicle, ...(saved?.cfg?.vehicle || {}) },
  obstacles: (saved?.cfg?.obstacles || DEFAULT_OBSTACLES).map(o => ({ ...o })),
};
const opts = {
  sweep: true, trace: true, clear: true, ghost: true, compare: true,
  step: 0.10,   // 每按一下前進／倒車走多遠（公尺）
  ...(saved?.opts || {}),
};

// 挑戰會把場景換成關卡的配置。使用者自己量出來的那一份先收在這裡，
// 存檔一律存這一份 —— 玩個挑戰就把辛苦量的現場設定洗掉，那太糟了。
let userCfg = null;

let saveTimer = null;
function saveNow() {
  clearTimeout(saveTimer);
  saveTimer = null;
  try {
    localStorage.setItem(STORE_KEY, JSON.stringify({ cfg: userCfg || cfg, opts, at: Date.now() }));
    storeState = '設定已存到這台裝置';
  } catch (err) {
    storeState = '存不進去：' + (err && err.name ? err.name : '未知錯誤');
  }
  const el = document.querySelector('#storeState');
  if (el) el.textContent = storeState;
}

function saveSoon() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(saveNow, 300);
}

// 改完設定馬上重新整理時，300ms 的防抖還沒觸發就會漏存 —— 離開前補寫一次。
addEventListener('pagehide', () => { if (saveTimer) saveNow(); });
addEventListener('visibilitychange', () => { if (document.hidden && saveTimer) saveNow(); });

let scene = buildScene(cfg);
let narrow = narrowestPoint(scene);
let view = null;

// 手動模式狀態
let pose = { ...scene.start };
let steerDeg = 0;
let manualPoses = [{ ...pose, dir: 0 }];
// 每按一下前進／倒車就記一筆指令。manualPoses 是畫出來的軌跡，這個是「怎麼開的」，
// 回放時餵給 expandPoses 就變成跟規劃結果同形狀的路徑。兩者一起 push、一起 pop。
let manualSegs = [];
let history = [];
let moves = 0;

// 規劃結果狀態
let plan = null;          // { poses, segments, reversals, cost, tier, ... }
// 挑戰狀態。宣告在這裡是因為畫面更新會讀它，而那比下面的挑戰區塊更早跑到。
let challenge = null;     // { def, startPose, par, parState, done }
let playhead = 0;
let playing = false;
let lastFrame = 0;
let rafId = null;
let worker = null;
let planSeq = 0;
let planMode = 'park';

const PLAY_SPEED = 0.85;   // 播放速度（公尺／秒）
const POSE_STRIDE = 0.035; // expandPoses 的取樣間距，要與 worker 一致

// ---------------------------------------------------------------- 尺寸

function resize() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const r = canvas.getBoundingClientRect();
  if (!r.width) return;
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  view = makeView(canvas, scene);
  invalidate();
}
new ResizeObserver(resize).observe(canvas);

// ---------------------------------------------------------------- 繪圖

function activePoses() {
  return plan ? plan.poses : manualPoses;
}

/** playhead 是浮點數（動畫用），取樣時一律先取整，否則會拿到 undefined。 */
function headIndex() {
  return Math.max(0, Math.min(Math.floor(playhead), plan.poses.length - 1));
}

function currentPose() {
  return plan ? plan.poses[headIndex()] : pose;
}

function currentSteer() {
  return plan ? (plan.poses[headIndex()].steer || 0) : steerDeg * Math.PI / 180;
}

function draw() {
  if (!view) return;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawScene(ctx, scene, view, { goalGhost: opts.ghost, narrowest: narrow, gate: gateBlock(scene, scene.goal), farGate: farGateBlock(scene) });

  const poses = activePoses();
  const upto = plan ? headIndex() + 1 : poses.length;

  // 對照播：規劃器的路徑與車按同樣的「完成比例」跟著跑。兩條路徑長度不一樣，
  // 用索引對齊會有一邊先跑完；用比例才看得出誰在哪一段多繞了。
  const par = opts.compare && challenge && challenge.par ? challenge.par : null;
  if (par && par.poses.length) {
    const frac = poses.length > 1 ? (upto - 1) / (poses.length - 1) : 0;
    const j = Math.max(0, Math.min(par.poses.length - 1, Math.round(frac * (par.poses.length - 1))));
    drawTrace(ctx, par.poses, j + 1, { ghost: true });
    drawCar(ctx, par.poses[j], scene.v, { ghost: true, steer: par.poses[j].steer || 0 });
  }

  if (opts.sweep && poses.length > 1) drawSweep(ctx, poses, scene.v, upto);
  if (opts.trace && poses.length > 1) drawTrace(ctx, poses, upto);

  const p = currentPose();
  const hit = collide(scene, p, 0) !== null;
  drawCar(ctx, p, scene.v, { hit, steer: currentSteer(), portFromNose: cfg.charger ? cfg.portFromNose : 0 });
  if (opts.clear) drawClearances(ctx, cornerClearances(scene, p), view);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
}

/**
 * 重畫入口。互動時同步重畫（沒有一格延遲），rAF 迴圈只在播放時才跑，
 * 所以閒置時完全不吃 CPU —— 參考版是兩個永不停止的 rAF 迴圈。
 */
function invalidate() {
  try { draw(); updateStatus(); } catch (err) { console.error('render error:', err); }
}

function loop(t) {
  const dt = lastFrame ? Math.min((t - lastFrame) / 1000, 0.05) : 0;
  lastFrame = t;
  if (playing && plan) {
    playhead += (PLAY_SPEED * dt) / POSE_STRIDE;
    if (playhead >= plan.poses.length - 1) {
      playhead = plan.poses.length - 1;
      setPlaying(false);
    }
    syncScrub();
  }
  invalidate();
  if (playing) rafId = requestAnimationFrame(loop);
  else rafId = null;
}

// ---------------------------------------------------------------- 讀數

function updateStatus() {
  const p = currentPose();
  const hitObs = collide(scene, p, 0);
  const parked = isParked(scene, p);
  const near = clearances(scene, p)[0];

  const pill = $('#status');
  const text = hitObs ? `碰到${hitObs.label}` : parked ? '已停妥' : plan ? '路徑播放中' : '調整中';
  $('#statusText').textContent = text;
  const tone = hitObs ? 'bad' : parked ? 'ok' : 'warn';
  pill.style.background = { bad: '#fdeaea', ok: '#e9f7f1', warn: '#fff4df' }[tone];
  pill.style.color = { bad: '#a42b2b', ok: '#08744d', warn: '#885306' }[tone];
  pill.querySelector('.dot').style.background = { bad: '#dc5151', ok: '#17a673', warn: '#ed9e32' }[tone];

  $('#mHeading').textContent = (p.theta * 180 / Math.PI).toFixed(1) + '°';
  $('#mClear').textContent = near ? near.distance.toFixed(2) + 'm' : '—';
  $('#mMoves').textContent = moves;
  $('#mShifts').textContent = plan ? plan.reversals : manualReversals();
  if (challenge) $('#chNow').textContent = fmtScore(scoreOf());
}

/**
 * 自己開的時候折了幾次：相鄰兩步的行進方向相反就算一次，跟規劃器報的「折N」同一個算法。
 *
 * 從 manualPoses 現算而不另外計數 —— 那個陣列在「上一步」時會一起被 pop，
 * 少維護一個會跟 undo 走散的計數器。連按同方向不算折返，換方向那一下才算。
 */
function manualReversals() {
  let n = 0;
  let prev = 0;
  for (const p of manualPoses) {
    if (!p.dir) continue;
    if (prev && p.dir !== prev) n++;
    prev = p.dir;
  }
  return n;
}

function setTip(msg) { $('#tip').textContent = msg; }

/**
 * 從巷子哪一頭進來，決定了行進方向 —— 而且兩者是相反的：
 * 從「左邊」進來就是往右開，車頭朝右。平行停車不調頭的話，停妥車頭就是這個方向。
 */
function naturalHeading(from) { return from === 'left' ? 'right' : 'left'; }

// ---------------------------------------------------------------- 可行性

function refreshFacts() {
  const ideal = idealMinSlotLength(scene.v, cfg.wallGap);
  const slotLen = scene.slot.length;
  $('#fSlot').textContent = slotLen.toFixed(2) + ' m';
  $('#hSlot').textContent = slotLen.toFixed(2) + ' m';
  // 屋前空間長度與可用車位是同一件事，滑桿要跟著算出來的值走
  $('#houseFront').value = slotLen;
  $('#houseFrontO').textContent = slotLen.toFixed(2) + 'm';
  $('#hIdeal').textContent = Number.isFinite(ideal.length) ? ideal.length.toFixed(2) + ' m' : '不可能';
  $('#hNarrow').textContent = Number.isFinite(narrow.width) ? narrow.width.toFixed(2) + ' m' : '—';
  $('#parkDirOut').textContent = cfg.parkDirection === 'left' ? '左' : '右';
  const car = scene.v;
  $('#vSteer').textContent = car.maxSteerDeg.toFixed(1) + '°';
  $('#vRear').textContent = (car.wheelbase / Math.tan(car.maxSteerDeg * Math.PI / 180)).toFixed(2) + ' m';
  $('#vOver').textContent = `${car.frontOverhang.toFixed(2)} / ${car.rearOverhang.toFixed(2)} m`;
  const far = farGateBlock(scene);
  $('#hFarGate').textContent = far ? far.free.toFixed(2) + ' m' : '—';
  $('#hFarGate').parentElement.classList.toggle('bad', !!far && !far.ok);
  const gate = gateBlock(scene, scene.goal);
  $('#hGate').textContent = gate ? gate.free.toFixed(2) + ' m' : '—';
  $('#hGate').parentElement.classList.toggle('bad', !!gate && !gate.ok);
  const reach = chargeReach(scene, scene.goal);
  $('#hCharge').textContent = reach ? reach.distance.toFixed(2) + ' m' : '—';
  $('#hCharge').parentElement.classList.toggle('bad', !!reach && reach.distance > 5);
  $('#hSlot').parentElement.classList.toggle('bad', slotLen < ideal.length);
  $('#hNarrow').parentElement.classList.toggle('bad', narrow.width - scene.v.width < 0.9);
  $('#fWidth').textContent = Number.isFinite(narrow.width)
    ? `${narrow.width.toFixed(2)} m（x=${narrow.x.toFixed(1)}）` : '—';
  $('#fRadius').textContent = ideal.radius.toFixed(2) + ' m';

  const v = $('#verdict');
  if (!Number.isFinite(ideal.length)) {
    $('#fIdeal').textContent = '不可能';
    v.className = 'verdict bad';
    v.textContent = `靠牆只留 ${cfg.wallGap.toFixed(2)}m 時，車尾內角在轉動中一定會刮到牆 —— 單次進入無解，只能靠折車。`;
    return;
  }
  $('#fIdeal').textContent = ideal.length.toFixed(2) + ' m';
  const diff = slotLen - ideal.length;
  if (diff >= 0.35) {
    v.className = 'verdict ok';
    v.textContent = `車位比單次進入所需多出 ${diff.toFixed(2)}m，一般平行停車手法就夠了。`;
  } else if (diff >= 0) {
    v.className = 'verdict warn';
    v.textContent = `只多出 ${diff.toFixed(2)}m，理論上能一次進去，但幾乎沒有修正空間。`;
  } else {
    v.className = 'verdict bad';
    v.textContent = `比單次進入所需短了 ${Math.abs(diff).toFixed(2)}m —— 這就是為什麼一般平行停車停不進去，必須折車。`;
  }
  // 巷道瓶頸常常比車位長度更致命：過不去就沒辦法先開過車位再倒車。
  const squeeze = narrow.width - scene.v.width;
  if (Number.isFinite(narrow.width) && squeeze < 0.9) {
    v.textContent += squeeze < 0
      ? ` 而且巷道最窄處只有 ${narrow.width.toFixed(2)}m，比車還窄，根本開不過去。`
      : ` 另外巷道最窄處 ${narrow.width.toFixed(2)}m，左右合計只剩 ${(squeeze * 100).toFixed(0)}cm。`;
  }
}

// ---------------------------------------------------------------- 規劃

function ensureWorker() {
  if (worker) return worker;
  worker = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' });
  worker.onmessage = (e) => {
    const d = e.data;
    if (d.id !== planSeq) return;   // 過期結果，使用者已經改了設定
    if (d.adviceOnly) { renderAdvice(d.advice); return; }
    $('#planBtn').disabled = false;
    $('#exitBtn').disabled = false;
    if (!d.ok) {
      clearPlan();
      $('#planBtn').textContent = '⌕ 規劃停入路徑';
      const cmp = (d.alternatives || []).length > 1
      ? '<div class="cmp">' + d.alternatives.map((a, i) =>
          `<span class="${i === 0 ? 'win' : ''}">${a.entry === 'reverse' ? '倒車入庫' : '前進入庫'} ` +
          `${a.cost.toFixed(1)}m／折${a.reversals}</span>`).join('') + '</div>'
      : '';
    const fixes = (d.advice || []).map(a =>
        `<li>${a.text}<span class="fix-d">${a.detail}</span></li>`).join('');
      $('#planNote').innerHTML =
        `<b>停不進去。</b>搜了 ${d.expansions.toLocaleString()} 個節點 / ${(d.elapsed / 1000).toFixed(1)}s。<br>${diagnose()}` +
        (fixes ? `<div class="fixes"><b>怎樣才有解（依省力程度排序）</b><ol>${fixes}</ol></div>`
               : '<div class="fixes">試過移走／挪動每一個障礙、放寬靠牆距離、換方向停，都還是停不進去。</div>');
      setTip('停不進去：' + d.reason);
      return;
    }
    plan = { ...d };
    playhead = 0;
    $('#planBtn').textContent = '⌕ 重新規劃停入';
    const mc = d.minClearance;
    const cmp = (d.alternatives || []).length > 1
      ? '<div class="cmp">' + d.alternatives.map((a, i) =>
          `<span class="${i === 0 ? 'win' : ''}">${a.entry === 'reverse' ? '倒車入庫' : '前進入庫'} ` +
          `${a.cost.toFixed(1)}m／折${a.reversals}</span>`).join('') + '</div>'
      : '';
    $('#planNote').innerHTML =
      `全程最近間隙 <b>${(mc.distance * 100).toFixed(0)}cm</b>（貼到${mc.label}）· ${d.tierNote}<br>` +
      `總行程 <b>${d.cost.toFixed(2)}m</b>，搜尋 ${d.expansions.toLocaleString()} 節點 / ${(d.elapsed / 1000).toFixed(1)}s。` + cmp +
      (d.advicePending ? '<div class="fixes" id="fixes"><b>折 ' + d.reversals + ' 次太多了，正在找怎樣會輕鬆很多…</b></div>' : '');
    renderSegments();
    $('#timeline').hidden = false;
    $('#scrub').max = d.poses.length - 1;
    const what = d.mode === 'exit' ? '駛出' : '停入';
    const how = d.entry === 'reverse' ? '倒車入庫' : '前進入庫';
    setTip(d.reversals >= 8
      ? `${how}：要折 ${d.reversals} 次、共 ${d.segments.length} 段 —— 幾何上可行，但實務上這已經是不合理的次數。`
      : `${how}：折 ${d.reversals} 次、共 ${d.segments.length} 段。按播放看車怎麼走。`);
    setPlaying(true);
    invalidate();
  };
  return worker;
}

/** 無解時指出是哪個尺寸卡住的 —— 光說「找不到」沒有幫助。 */
function diagnose() {
  const ideal = idealMinSlotLength(scene.v, cfg.wallGap);
  const bits = [];
  const shortfall = ideal.length - scene.slot.length;
  if (Number.isFinite(ideal.length) && shortfall > 0) {
    bits.push(`車位 ${scene.slot.length.toFixed(2)}m，單次進入需要 ${ideal.length.toFixed(2)}m（差 ${shortfall.toFixed(2)}m）`);
  }
  const squeeze = narrow.width - scene.v.width;
  if (Number.isFinite(narrow.width) && squeeze < 1.0) {
    bits.push(`巷道最窄 ${narrow.width.toFixed(2)}m，左右合計只剩 ${(squeeze * 100).toFixed(0)}cm，車在那裡轉不動`);
  }
  const deep = cfg.obstacles.filter(o => o.on && o.side === 'house' && o.depth > 1.5);
  if (deep.length >= 2) {
    bits.push(`車位前後的障礙深達 ${Math.max(...deep.map(o => o.depth)).toFixed(2)}m，車頭甩不出去`);
  }
  return bits.length ? '卡住的原因：' + bits.join('；') + '。' : '';
}

/** 建議是第二段訊息送來的，路徑不必等它。 */
function renderAdvice(list) {
  const box = $('#fixes');
  if (!box) return;
  if (!list || !list.length) {
    box.innerHTML = '試過移走／挪動每一個障礙、放寬靠牆距離、換方向開進來，都沒有更好的走法。';
    return;
  }
  const items = list.map(a => `<li>${a.text}<span class="fix-d">${a.detail}</span></li>`).join('');
  box.innerHTML = `<b>怎樣會輕鬆很多</b><ol>${items}</ol>`;
}

function requestPlan(mode) {
  planMode = mode;
  setPlaying(false);
  $('#planBtn').disabled = true;
  $('#exitBtn').disabled = true;
  $('#planNote').textContent = mode === 'exit'
    ? '正在搜尋駛出路徑…'
    : '正在搜尋不碰撞的停入路徑，車位越緊越久。';
  ensureWorker().postMessage({ id: ++planSeq, cfg: { ...cfg }, mode });
}

function clearPlan() {
  plan = null;
  playhead = 0;
  setPlaying(false);
  $('#timeline').hidden = true;
  $('#seglist').innerHTML = '';
  invalidate();
}

function describeSegment(s) {
  const dir = s.dist > 0 ? '前進' : '倒車';
  const deg = s.steer * 180 / Math.PI;
  const lock = Math.abs(deg) > scene.v.maxSteerDeg - 0.5 ? '（打到底）' : '';
  return `${dir} ${Math.abs(s.dist).toFixed(2)}m，${describeSteer(deg, scene.v)}${lock}`;
}

function renderSegments() {
  const list = $('#seglist');
  list.innerHTML = '';
  // 每一段在 poses 陣列中的起始索引，點清單就能跳到那一段
  const starts = [];
  let acc = 0;
  plan.segments.forEach((s) => {
    starts.push(acc);
    acc += Math.max(1, Math.ceil(Math.abs(s.dist) / 0.035));
  });
  plan.segments.forEach((s, i) => {
    const li = document.createElement('li');
    if (s.dist < 0) li.classList.add('rev');
    li.innerHTML = `<span class="n">${i + 1}</span><span>${describeSegment(s)}</span><span class="d">${Math.abs(s.dist).toFixed(2)}m</span>`;
    li.onclick = () => { setPlaying(false); playhead = starts[i]; syncScrub(); invalidate(); };
    list.appendChild(li);
  });
  plan.starts = starts;
}

function highlightSegment() {
  if (!plan || !plan.starts) return;
  let idx = 0;
  for (let i = 0; i < plan.starts.length; i++) if (playhead >= plan.starts[i]) idx = i;
  [...$('#seglist').children].forEach((li, i) => li.classList.toggle('active', i === idx));
}

function setPlaying(on) {
  playing = on && !!plan;
  $('#playBtn').textContent = playing ? '❚❚ 暫停' : '▶ 播放';
  if (playing && rafId === null) { lastFrame = 0; rafId = requestAnimationFrame(loop); }
}

function syncScrub() {
  const i = Math.round(playhead);
  $('#scrub').value = i;
  if (plan) {
    let dist = 0;
    for (let k = 1; k <= i && k < plan.poses.length; k++) {
      dist += Math.hypot(plan.poses[k].x - plan.poses[k - 1].x, plan.poses[k].y - plan.poses[k - 1].y);
    }
    $('#scrubPos').textContent = dist.toFixed(2) + ' m';
  }
  highlightSegment();
}

// ---------------------------------------------------------------- 手動駕駛

function move(dir) {
  if (plan) clearPlan();          // 手動接管，離開規劃檢視
  history.push({ pose: { ...pose }, moves });
  pose = integrate(pose, dir * opts.step, steerDeg * Math.PI / 180, scene.v.wheelbase);
  manualPoses.push({ ...pose, dir });
  manualSegs.push({ dist: dir * opts.step, steer: steerDeg * Math.PI / 180 });
  moves++;

  const hitObs = collide(scene, pose, 0);
  if (hitObs) {
    setTip(`碰到${hitObs.label} —— 退回上一步，先把角度或距離調開。`);
  } else if (isParked(scene, pose)) {
    setTip('停妥了。這個位置左前充電口靠住家側。');
  } else {
    const c = cornerClearances(scene, pose).sort((a, b) => a.distance - b.distance)[0];
    setTip(`${dir < 0 ? '倒車' : '前進'} ${Math.round(opts.step * 100)}cm：最近的是${c.corner}，離${c.label} ${c.distance.toFixed(2)}m。`);
  }
  if (challenge && !challenge.done && !hitObs && isParked(scene, pose)) finishChallenge();
  invalidate();
}

/** 這一趟總共開了多遠。折返次數同分時用它分高下。 */
function manualCost() {
  return manualSegs.reduce((sum, s) => sum + Math.abs(s.dist), 0);
}

function resetTo(target) {
  clearPlan();
  pose = { ...target };
  manualPoses = [{ ...pose, dir: 0 }];
  manualSegs = [];
  history = [];
  moves = 0;
  steerDeg = 0;
  $('#steer').value = 0;
  $('#steerOut').textContent = '回正';
  invalidate();
}

// ---------------------------------------------------------------- 場景重建

function rebuild() {
  saveSoon();
  scene = buildScene(cfg);
  syncSteerRange();
  narrow = narrowestPoint(scene);
  view = makeView(canvas, scene);
  clearPlan();
  pose = { ...scene.start };
  manualPoses = [{ ...pose, dir: 0 }];
  manualSegs = [];
  history = [];
  moves = 0;
  $('#planBtn').textContent = '⌕ 規劃停入路徑';
  $('#planNote').textContent = '會實際搜尋不碰撞的路徑；車位越緊，算越久（最多約 10 秒）。';
  refreshFacts();
  invalidate();
}

// ---------------------------------------------------------------- 事件

const sliders = [
  ['alley', 'alleyWidth', 'alleyO'],
  ['wallGap', 'wallGap', 'wallGapO'],
  ['targetX', 'targetX', 'targetXO'],
  ['frontGap', 'frontGap', 'frontGapO'],
  ['gateX', 'gateX', 'gateXO'],
  ['gateWidth', 'gateWidth', 'gateWidthO'],
  ['farGateX', 'farGateX', 'farGateXO'],
  ['farGateWidth', 'farGateWidth', 'farGateWidthO'],
  ['chargerX', 'chargerX', 'chargerXO'],
  ['portFromNose', 'portFromNose', 'portFromNoseO'],
];
for (const [id, key, out] of sliders) {
  $('#' + id).addEventListener('input', (e) => {
    cfg[key] = +e.target.value;
    $('#' + out).textContent = (+e.target.value).toFixed(2) + 'm';
    rebuild();
  });
}
/**
 * 屋前空間長度 —— 自家門面那一段的總長，也就是住家側兩個障礙物之間的空隙。
 *
 * 它跟「可用車位」是同一個數字，本來只是算出來的結果；但現場拿捲尺量得到的
 * 就是這一段，所以這裡讓它可以直接輸入：拉滑桿＝搬動車位前端那個障礙物，
 * 後端釘住不動（往後端搬會連帶改到別的東西，往前端搬只影響這個空隙）。
 *
 * 大門不參與這個長度 —— 它是牆上的參照物，不佔車位也不進碰撞檢查。
 */
function setHouseFront(len) {
  const front = cfg.obstacles.find(o => o.on && o.side === 'house' && o.depth > 0.35
    && Math.abs(o.x - o.length / 2 - scene.slot.end) < 0.06);
  if (!front) return false;
  front.x += scene.slot.start + len - scene.slot.end;
  syncRow(front);
  rebuild();
  return true;
}

$('#houseFront').addEventListener('input', (e) => {
  if (!setHouseFront(+e.target.value)) {
    setTip('屋前空間的前端沒有障礙物擋著（車位一路開到巷底），沒有東西可以搬 —— 先在住家側加一個障礙物。');
  }
});

/**
 * 車長或軸距一改，前後懸就要跟著重算 —— 三者是綁死的（長 = 前懸 + 軸距 + 後懸）。
 * 前後比例沿用原廠的 0.94 : 0.97。
 */
function normaliseVehicle(v) {
  const over = Math.max(0.20, v.length - v.wheelbase);
  const ratio = 0.94 / (0.94 + 0.97);
  v.frontOverhang = +(over * ratio).toFixed(3);
  v.rearOverhang = +(over * (1 - ratio)).toFixed(3);
}

const vehicleSliders = [
  ['vLength', 'length', 'vLengthO'],
  ['vWidth', 'width', 'vWidthO'],
  ['vWheelbase', 'wheelbase', 'vWheelbaseO'],
  ['vTurn', 'turningRadius', 'vTurnO'],
];
for (const [id, key, out] of vehicleSliders) {
  $('#' + id).addEventListener('input', (e) => {
    cfg.vehicle[key] = +e.target.value;
    if (key === 'length' || key === 'wheelbase') normaliseVehicle(cfg.vehicle);
    $('#' + out).textContent = (+e.target.value).toFixed(2) + 'm';
    rebuild();
  });
}

$('#approach').onchange = (e) => {
  cfg.approachFrom = e.target.value;
  cfg.parkDirection = naturalHeading(cfg.approachFrom);   // 不可能調頭，直接推導
  rebuild();
};
$('#entry').onchange = (e) => {
  cfg.entryStyle = e.target.value;
  rebuild();
  setTip(e.target.value === 'reverse'
    ? '倒車入庫：起點不變（還是你開進來的那一頭），規劃器會自己先開過車位再倒回來。車頭朝向也不變，那是「停妥車頭朝」在管的。'
    : e.target.value === 'forward'
      ? '前進入庫：起點放在還沒開到車位的位置，直接切進去。'
      : '自動：兩種起手式都會試，取比較好開的那個。');
};

// ---------------------------------------------------------------- 障礙物編輯器

const KINDS = [['car', '汽車'], ['scooter', '機車群'], ['fixed', '固定設施']];
const SIDES = [['house', '住家側'], ['far', '對面側']];
const SWATCH = { car: '#aeb7c0', scooter: '#df8737', fixed: '#3a3f45' };
/** 機車群用「台數」而不是「長度」—— 現場數車比量長度直覺得多。 */
const scooterCount = (o) => Math.max(1, Math.round(o.length / SCOOTER_PITCH));

function fieldsFor(o) {
  const pos = { key: 'x', label: '位置', min: -4, max: 24, step: 0.1,
    get: () => o.x, set: (v) => { o.x = v; }, fmt: (v) => v.toFixed(2) };
  const depth = { key: 'depth', label: '突出', min: 0.1, max: 3, step: 0.05,
    get: () => o.depth, set: (v) => { o.depth = v; }, fmt: (v) => v.toFixed(2) };
  if (o.kind === 'scooter') {
    return [pos, { key: 'count', label: '台數', min: 1, max: 12, step: 1,
      get: () => scooterCount(o), set: (v) => { o.length = v * SCOOTER_PITCH; }, fmt: (v) => v + ' 台' }, depth];
  }
  return [pos, { key: 'length', label: '長度', min: 0.3, max: 10, step: 0.05,
    get: () => o.length, set: (v) => { o.length = v; }, fmt: (v) => v.toFixed(2) }, depth];
}

const rowRefs = new Map();   // obstacle.id -> { x, length, depth } 的 {range, output}

function obstacleRow(o) {
  const row = document.createElement('div');
  row.className = 'obs' + (o.on ? '' : ' off');
  const refs = {};
  rowRefs.set(o.id, refs);

  const head = document.createElement('div');
  head.className = 'obs-head';

  const on = document.createElement('input');
  on.type = 'checkbox';
  on.checked = o.on;
  on.title = '是否存在';
  on.onchange = () => { o.on = on.checked; row.classList.toggle('off', !o.on); rebuild(); };

  const sw = document.createElement('span');
  sw.className = 'swatch';
  sw.style.background = SWATCH[o.kind];

  const name = document.createElement('input');
  name.type = 'text';
  name.value = o.label;
  name.oninput = () => { o.label = name.value || '障礙'; rebuild(); };

  const kind = document.createElement('select');
  for (const [v, t] of KINDS) kind.add(new Option(t, v));
  kind.value = o.kind;
  kind.onchange = () => {
    o.kind = kind.value;
    if (o.kind === 'scooter') o.length = scooterCount(o) * SCOOTER_PITCH;  // 對齊台數
    renderObstacles();   // 欄位要從「長度」換成「台數」
    rebuild();
  };

  const side = document.createElement('select');
  for (const [v, t] of SIDES) side.add(new Option(t, v));
  side.value = o.side;
  side.onchange = () => { o.side = side.value; rebuild(); };

  const del = document.createElement('button');
  del.className = 'del';
  del.textContent = '✕';
  del.title = '刪除';
  del.onclick = () => {
    cfg.obstacles = cfg.obstacles.filter(x => x !== o);
    renderObstacles();
    rebuild();
  };

  head.append(on, sw, name, kind, side, del);
  row.appendChild(head);

  for (const f of fieldsFor(o)) {
    const wrap = document.createElement('div');
    wrap.className = 'setting';
    const lab = document.createElement('label');
    lab.textContent = f.label;
    const range = document.createElement('input');
    range.type = 'range';
    range.min = f.min; range.max = f.max; range.step = f.step; range.value = f.get();
    const out = document.createElement('output');
    out.textContent = f.fmt(f.get());
    range.oninput = () => {
      f.set(+range.value);
      out.textContent = f.fmt(f.get());
      rebuild();
    };
    refs[f.key] = { range, out, f };
    wrap.append(lab, range, out);
    row.appendChild(wrap);
  }
  return row;
}

/** 拖曳改了數值之後，把編輯器裡的滑桿也跟著更新。 */
function syncRow(o) {
  const refs = rowRefs.get(o.id);
  if (!refs) return;
  for (const r of Object.values(refs)) {
    r.range.value = r.f.get();
    r.out.textContent = r.f.fmt(r.f.get());
  }
}

function renderObstacles() {
  const list = $('#obstacleList');
  list.innerHTML = '';
  rowRefs.clear();
  for (const o of cfg.obstacles) list.appendChild(obstacleRow(o));
}

let addSeq = 0;
function addObstacle(side) {
  cfg.obstacles.push({
    id: 'custom' + (++addSeq),
    label: side === 'house' ? '新障礙（住家側）' : '新障礙（對面側）',
    side, kind: 'scooter', x: Math.round(cfg.targetX * 10) / 10, length: 3 * SCOOTER_PITCH, depth: 1.6, on: true,
  });
  renderObstacles();
  rebuild();
}
$('#addHouse').onclick = () => addObstacle('house');
$('#addFar').onclick = () => addObstacle('far');
$('#resetObs').onclick = () => {
  // 不用 removeItem：rebuild() 會馬上把預設值再存回去，結果一樣，
  // 但寫成「清除儲存」會是假的敘述。這裡就是「存成預設值」。
  Object.assign(cfg, DEFAULT_SCENE, {
    vehicle: { ...DEFAULT_SCENE.vehicle },
    obstacles: DEFAULT_OBSTACLES.map(o => ({ ...o })),
  });
  renderObstacles();
  syncControls();
  rebuild();
};

$('#steer').oninput = (e) => {
  steerDeg = +e.target.value;
  $('#steerOut').textContent = describeSteer(steerDeg, scene.v);
  invalidate();
};
/** 設定方向盤角度：夾在這台車的最大轉向角內，並同步滑桿與讀數。 */
function setSteer(deg) {
  const max = scene.v.maxSteerDeg;
  steerDeg = Math.round(Math.max(-max, Math.min(max, deg)) * 10) / 10;
  $('#steer').value = steerDeg;
  $('#steerOut').textContent = describeSteer(steerDeg, scene.v);
  invalidate();
}
/** 方向盤按鈕：一次打 1/4 個行程，連按就到底。滑桿仍在，要微調用滑桿。 */
function nudgeSteer(dir) {
  setSteer(dir === 0 ? 0 : steerDeg + dir * scene.v.maxSteerDeg / 4);
}

/**
 * 滑鼠滾輪轉方向盤：往上滾＝打向外側，跟滑桿往右同一個方向，一格 2 度。
 *
 * 只在圖上與駕駛列接管滾輪，其他地方照常捲頁面 —— 眼睛盯著車看的時候手不用移開，
 * 但整頁還是捲得動。要一次打到底用 A／D 按鈕，那是一次 1/4 個行程。
 *
 * 沒有做 Shift 加速：Windows 上 Shift＋滾輪會被瀏覽器轉成水平捲動，
 * deltaY 直接變成 0，加速鍵反而會讓方向盤不動。
 */
const WHEEL_STEER_DEG = 2;
function wheelSteer(e) {
  if (!e.deltaY) return;
  setSteer(steerDeg - Math.sign(e.deltaY) * WHEEL_STEER_DEG);
  e.preventDefault();
}
canvas.addEventListener('wheel', wheelSteer, { passive: false });
$('.drivebar').addEventListener('wheel', wheelSteer, { passive: false });
/**
 * 每次移動的距離。預設 10cm —— 車位只剩十幾公分餘裕時，25cm 一步就過頭了。
 * 想快速移動再切到 25/50cm。
 */
function setStep(v) {
  opts.step = v;
  const cm = Math.round(v * 100);
  $('#stepFwd').textContent = cm;
  $('#stepBack').textContent = cm;
  for (const b of $('#stepPick').querySelectorAll('button')) {
    b.classList.toggle('on', Math.abs(+b.dataset.step - v) < 1e-9);
  }
  saveSoon();
}
for (const b of $('#stepPick').querySelectorAll('button')) {
  b.onclick = () => setStep(+b.dataset.step);
}

$('#steerWall').onclick = () => nudgeSteer(-1);
$('#steerZero').onclick = () => nudgeSteer(0);
$('#steerOut2').onclick = () => nudgeSteer(1);

$('#forward').onclick = () => move(1);
$('#reverse').onclick = () => move(-1);
$('#undo').onclick = () => {
  if (plan || !history.length) return;
  const h = history.pop();
  pose = h.pose; moves = h.moves; manualPoses.pop(); manualSegs.pop();
  invalidate();
};
$('#toStart').onclick = () => resetTo(scene.start);
$('#toGoal').onclick = () => resetTo(scene.goal);

// ---------------------------------------------------------------- 挑戰
//
// 關卡是固定的題目：同一條巷子的幾種現場狀況。開始挑戰時場景鎖住，因為改場景
// 就是改題目，成績也就沒得比了。評分先看折返次數，同分再比總行程 —— 跟規劃器
// 判斷「哪種手法好開」用的是同一個順序。

const CH_KEY = STORE_KEY + '/challenges';

let bestRuns = loadBests();
let parWorker = null;
let parSeq = 0;

function loadBests() {
  try { return JSON.parse(localStorage.getItem(CH_KEY)) || {}; } catch (err) { return {}; }
}
function saveBests() {
  // 存不進去（無痕視窗、封鎖網站資料）就算了，挑戰照樣能玩，只是紀錄留不住。
  try { localStorage.setItem(CH_KEY, JSON.stringify(bestRuns)); } catch (err) { /* 忽略 */ }
}

/** 關卡的場景設定：一律從預設值長出來，不受使用者調過的設定影響。 */
function challengeCfg(def) {
  const out = {
    ...DEFAULT_SCENE,
    ...(def.patch || {}),
    vehicle: { ...DEFAULT_SCENE.vehicle },
    approachFrom: 'left',
    parkDirection: 'right',
    entryStyle: 'any',
    obstacles: DEFAULT_OBSTACLES
      .filter(o => !(def.without || []).includes(o.id))
      .map(o => ({ ...o })),
  };
  for (const [id, dx] of Object.entries(def.nudge || {})) {
    const o = out.obstacles.find(x => x.id === id);
    if (o) o.x += dx;
  }
  return out;
}

function scoreOf() {
  return { reversals: manualReversals(), cost: manualCost() };
}
/** a 比 b 好嗎：折返次數優先，同分才比行程。 */
function beats(a, b) {
  if (!b) return true;
  if (a.reversals !== b.reversals) return a.reversals < b.reversals;
  return a.cost < b.cost - 1e-9;
}
function fmtScore(s) {
  return s ? `折 ${s.reversals} 次 · ${s.cost.toFixed(2)}m` : '—';
}

/** 連按同方向、同角度的那幾下併成一段，回放的步驟表才讀得下去。 */
function mergeSegs(segs) {
  const out = [];
  for (const s of segs) {
    const last = out[out.length - 1];
    if (last && last.steer === s.steer && Math.sign(last.dist) === Math.sign(s.dist)) last.dist += s.dist;
    else out.push({ ...s });
  }
  return out;
}

/** 把一串手動指令變成跟規劃結果同形狀的物件，就能直接餵給既有的播放器。 */
function runAsPlan(segs, from) {
  const merged = mergeSegs(segs);
  let reversals = 0;
  for (let i = 1; i < merged.length; i++) {
    if (Math.sign(merged[i].dist) !== Math.sign(merged[i - 1].dist)) reversals++;
  }
  return {
    poses: expandPoses(from, merged, scene.v.wheelbase, POSE_STRIDE),
    segments: merged,
    reversals,
    cost: merged.reduce((sum, x) => sum + Math.abs(x.dist), 0),
  };
}

/** 把一條路徑掛上既有的時間軸播放器。 */
function showPath(p) {
  plan = p;
  playhead = 0;
  renderSegments();
  $('#timeline').hidden = false;
  $('#scrub').max = p.poses.length - 1;
  setPlaying(true);
  invalidate();
}

function setSceneLocked(on) {
  for (const id of ['#panelVehicle', '#panelLane', '#panelObstacles']) $(id).classList.toggle('locked', on);
  $('#approach').disabled = on;
  $('#entry').disabled = on;
  // 「放到定位」是把車瞬移進車位，挑戰時等於一鍵過關，要一起鎖掉。
  // 「回到起點」留著 —— 它跟「重來」是同一件事。
  $('#toGoal').disabled = on;
}

function startChallenge(def) {
  // 只在「從非挑戰狀態進來」時備份，換關卡不能把備份蓋成上一關的配置。
  if (!challenge) userCfg = JSON.parse(JSON.stringify(cfg));
  Object.assign(cfg, challengeCfg(def));
  renderObstacles();
  syncControls();
  rebuild();                    // 順便把車放回起點、計數歸零
  challenge = { def, startPose: { ...scene.start }, par: null, parState: '計算中…', done: false };
  setSceneLocked(true);
  $('#chResult').hidden = true;
  requestPar();
  renderChallenge();
  setTip(`挑戰「${def.name}」：${def.hint} 用 A／D（或左右鍵）把車停進車位。`);
}

function quitChallenge() {
  challenge = null;
  setSceneLocked(false);
  if (userCfg) {
    Object.assign(cfg, userCfg);
    userCfg = null;
    renderObstacles();
    syncControls();
    rebuild();
  }
  renderChallenge();
  setTip('已離開挑戰，場景換回你自己的設定了。');
}

function finishChallenge() {
  challenge.done = true;
  const score = scoreOf();
  const isBest = beats(score, bestRuns[challenge.def.id]);
  if (isBest) {
    bestRuns[challenge.def.id] = { ...score, segs: mergeSegs(manualSegs) };
    saveBests();
  }
  const par = challenge.par;
  const vs = !par ? ''
    : score.reversals < par.reversals ? ' 比規劃器還少折，漂亮。'
    : score.reversals === par.reversals ? ' 跟規劃器打平。'
    : ` 規劃器只折 ${par.reversals} 次。`;
  const box = $('#chResult');
  box.hidden = false;
  box.className = 'ch-result ' + (isBest ? 'best' : 'done');
  box.innerHTML = `<b>停進去了 —— ${fmtScore(score)}。</b>`
    + (isBest ? '這是你在這一關的最佳成績。' : `最佳仍是 ${fmtScore(bestRuns[challenge.def.id])}。`)
    + vs;
  renderChallenge();
}

/**
 * 電腦成績跑在自己的 worker 上。跟使用者手動按的「規劃停入路徑」共用一個的話，
 * 兩邊的序號會互相把對方的結果作廢，先按的那個就永遠等不到答案。
 */
function requestPar() {
  if (!parWorker) {
    parWorker = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' });
    parWorker.onmessage = (e) => {
      const d = e.data;
      if (d.id !== parSeq || !challenge || d.adviceOnly) return;
      if (d.ok) {
        challenge.par = { poses: d.poses, segments: d.segments, reversals: d.reversals, cost: d.cost };
        challenge.parState = null;
      } else {
        challenge.parState = '這一關規劃器也停不進去';
      }
      renderChallenge();
      invalidate();
    };
  }
  parWorker.postMessage({ id: ++parSeq, cfg: { ...cfg }, mode: 'park', purpose: 'par', noAdvice: true });
}

function renderChallenge() {
  const list = $('#chList');
  list.innerHTML = '';
  for (const def of CHALLENGES) {
    const b = document.createElement('button');
    const best = bestRuns[def.id];
    b.innerHTML = `${def.name}<span class="ch-medal">${best ? '最佳 ' + fmtScore(best) : '尚未通關'}</span>`;
    if (challenge && challenge.def.id === def.id) b.classList.add('on');
    b.onclick = () => startChallenge(def);
    list.appendChild(b);
  }
  $('#chLive').hidden = !challenge;
  $('#cmpWrap').hidden = !(challenge && challenge.par);
  if (!challenge) return;
  $('#chName').textContent = challenge.def.name;
  $('#chHint').textContent = challenge.def.hint;
  $('#chNow').textContent = fmtScore(scoreOf());
  $('#chBest').textContent = fmtScore(bestRuns[challenge.def.id]);
  $('#chPar').textContent = challenge.par ? fmtScore(challenge.par) : (challenge.parState || '—');
}

$('#chQuit').onclick = quitChallenge;
$('#chRetry').onclick = () => {
  if (!challenge) return;
  resetTo(challenge.startPose);
  challenge.done = false;
  $('#chResult').hidden = true;
  renderChallenge();
  setTip('重來一次。');
};
$('#chReplay').onclick = () => {
  if (!challenge) return;
  if (!manualSegs.length) { setTip('這一趟還沒開過，沒有東西可以回放。'); return; }
  showPath(runAsPlan(manualSegs, challenge.startPose));
  setTip('回放你剛才那一趟。' + (challenge.par ? '勾下面那個框可以同時看規劃器怎麼走。' : ''));
};
renderChallenge();

$('#planBtn').onclick = () => requestPlan('park');
$('#exitBtn').onclick = () => requestPlan('exit');
$('#playBtn').onclick = () => {
  if (!plan) return;
  if (!playing && playhead >= plan.poses.length - 1) playhead = 0;
  setPlaying(!playing);
};
$('#scrub').oninput = (e) => { setPlaying(false); playhead = +e.target.value; syncScrub(); invalidate(); };

for (const [id, key] of [['optSweep', 'sweep'], ['optTrace', 'trace'], ['optClear', 'clear'], ['optGhost', 'ghost'], ['optCompare', 'compare']]) {
  $('#' + id).onchange = (e) => { opts[key] = e.target.checked; saveSoon(); invalidate(); };
}

// ---------------------------------------------------------------- 畫布拖曳

/** 邊緣判定範圍（公尺）：離邊界這麼近就當成要拉邊界而不是整塊搬。 */
const EDGE = 0.20;
const LIMITS = { x: [-4, 24], length: [0.3, 10], depth: [0.1, 3], width: [0.5, 5] };
const clamp = (v, [lo, hi]) => Math.min(Math.max(v, lo), hi);

let drag = null;

function pointerWorld(e) {
  const r = canvas.getBoundingClientRect();
  const sx = canvas.width / r.width, sy = canvas.height / r.height;
  return view.toWorld((e.clientX - r.left) * sx, (e.clientY - r.top) * sy);
}

/** 大門與充電器畫在牆上（巷道範圍外），所以先於障礙物判定，彼此不會搶。 */
function pickWallItem(wx, wy) {
  const W = cfg.alleyWidth;
  // 判定範圍比畫出來的圖示大一圈：圖示只有 0.38m 寬，螢幕上大約 12px，太難點。
  if (cfg.charger && wy < 0.05 && wy > -0.8 && Math.abs(wx - cfg.chargerX) < 0.42) {
    return { mode: 'charger' };
  }
  const band = (a, b, key, kx, kw) => {
    if (wy < a || wy > b) return null;
    const lo = cfg[kx] - cfg[kw] / 2, hi = cfg[kx] + cfg[kw] / 2;
    if (wx < lo - EDGE || wx > hi + EDGE) return null;
    if (Math.abs(wx - lo) < EDGE) return { mode: key + 'End', end: -1, kx, kw };
    if (Math.abs(wx - hi) < EDGE) return { mode: key + 'End', end: 1, kx, kw };
    return { mode: key, kx, kw };
  };
  if (cfg.gate) { const h = band(-0.34, 0.02, 'gate', 'gateX', 'gateWidth'); if (h) return h; }
  if (cfg.farGate) { const f = band(W - 0.02, W + 0.34, 'farGate', 'farGateX', 'farGateWidth'); if (f) return f; }
  return null;
}

/** 游標下是什麼？後畫的（機車、固定設施）優先，跟視覺疊放順序一致。 */
function pickAt(wx, wy) {
  const wall = pickWallItem(wx, wy);
  if (wall) return wall;
  const list = cfg.obstacles.filter(o => o.on);
  const ordered = [...list.filter(o => o.kind !== 'car'), ...list.filter(o => o.kind === 'car')];
  for (const o of ordered) {
    const y0 = o.side === 'house' ? 0 : cfg.alleyWidth - o.depth;
    const y1 = y0 + o.depth;
    const x0 = o.x - o.length / 2, x1 = o.x + o.length / 2;
    if (wx < x0 - EDGE || wx > x1 + EDGE || wy < y0 - EDGE || wy > y1 + EDGE) continue;
    const innerY = o.side === 'house' ? y1 : y0;      // 面向巷道的那一邊
    if (Math.abs(wy - innerY) < EDGE) return { o, mode: 'depth' };
    if (Math.abs(wx - x0) < EDGE) return { o, mode: 'end', end: -1 };
    if (Math.abs(wx - x1) < EDGE) return { o, mode: 'end', end: 1 };
    if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) return { o, mode: 'move' };
  }
  const slotDepth = scene.v.width + cfg.wallGap + 0.1;
  if (wy >= 0 && wy <= slotDepth && wx >= scene.slot.start && wx <= scene.slot.end) {
    return { mode: 'slot' };
  }
  return null;
}

const CURSORS = {
  move: 'move', end: 'ew-resize', depth: 'ns-resize', slot: 'grab',
  charger: 'move', gate: 'move', farGate: 'move', gateEnd: 'ew-resize', farGateEnd: 'ew-resize',
};

/** 拖曳改了設定值之後，把對應的滑桿也更新。 */
function syncSlider(key) {
  const row = sliders.find(([, k]) => k === key);
  if (!row) return;
  const [id, , out] = row;
  $('#' + id).value = cfg[key];
  $('#' + out).textContent = (+cfg[key]).toFixed(2) + 'm';
}

function applyDrag(wx, wy) {
  const d = drag;
  if (d.mode === 'charger') {
    cfg.chargerX = clamp(d.chargerX0 + (wx - d.wx0), LIMITS.x);
    syncSlider('chargerX');
    rebuild();
    return;
  }
  if (d.mode === 'gate' || d.mode === 'farGate') {
    cfg[d.kx] = clamp(d.kx0 + (wx - d.wx0), LIMITS.x);
    syncSlider(d.kx);
    rebuild();
    return;
  }
  if (d.mode === 'gateEnd' || d.mode === 'farGateEnd') {
    const fixed = d.kx0 - d.end * d.kw0 / 2;      // 另一端釘住
    const w = clamp(Math.abs(wx - fixed), LIMITS.width);
    cfg[d.kw] = w;
    cfg[d.kx] = fixed + d.end * w / 2;
    syncSlider(d.kx);
    syncSlider(d.kw);
    rebuild();
    return;
  }
  if (d.mode === 'slot') {
    cfg.targetX = clamp(d.target0 + (wx - d.wx0), [0, 20]);
    $('#targetX').value = cfg.targetX;
    $('#targetXO').textContent = cfg.targetX.toFixed(2) + 'm';
  } else if (d.mode === 'move') {
    d.o.x = clamp(d.x0 + (wx - d.wx0), LIMITS.x);
  } else if (d.mode === 'end') {
    // 拉一端，另一端釘住；機車群吸附到整數台
    const fixed = d.x0 - d.end * d.len0 / 2;
    let len = clamp(Math.abs(wx - fixed), LIMITS.length);
    if (d.o.kind === 'scooter') {
      len = Math.min(12, Math.max(1, Math.round(len / SCOOTER_PITCH))) * SCOOTER_PITCH;
    }
    d.o.length = len;
    d.o.x = fixed + d.end * len / 2;
  } else if (d.mode === 'depth') {
    const raw = d.o.side === 'house' ? wy : cfg.alleyWidth - wy;
    d.o.depth = clamp(raw, LIMITS.depth);
  }
  if (d.o) syncRow(d.o);
  rebuild();
}

canvas.addEventListener('pointerdown', (e) => {
  if (!view) return;
  const [wx, wy] = pointerWorld(e);
  const hit = pickAt(wx, wy);
  if (!hit) return;
  drag = {
    ...hit, wx0: wx, wy0: wy,
    x0: hit.o ? hit.o.x : 0,
    len0: hit.o ? hit.o.length : 0,
    target0: cfg.targetX,
    chargerX0: cfg.chargerX,
    kx0: hit.kx ? cfg[hit.kx] : 0,
    kw0: hit.kw ? cfg[hit.kw] : 0,
  };
  canvas.setPointerCapture(e.pointerId);
  e.preventDefault();
});

canvas.addEventListener('pointermove', (e) => {
  if (!view) return;
  const [wx, wy] = pointerWorld(e);
  if (!drag) {
    const hit = pickAt(wx, wy);
    canvas.style.cursor = hit ? CURSORS[hit.mode] : 'default';
    return;
  }
  applyDrag(wx, wy);
});

for (const ev of ['pointerup', 'pointercancel']) {
  canvas.addEventListener(ev, (e) => {
    if (!drag) return;
    drag = null;
    canvas.releasePointerCapture?.(e.pointerId);
  });
}

/**
 * 鍵盤操作：
 *   A、←    往畫面的左邊開
 *   D、→    往畫面的右邊開
 *   Q       方向盤回正
 *   空白鍵   播放／暫停已規劃的路徑
 *
 * 全部以「車子在圖上往哪邊走」為準。俯視圖上車只沿巷道左右移動，先把前進倒車
 * 在腦中換算成左右再按鍵是多餘的一步，所以 W/S 與 ↑/↓ 都收掉了。
 * 方向盤交給滑鼠滾輪、滑桿與畫面上的按鈕，鍵盤只留回正。
 */
addEventListener('keydown', (e) => {
  const tag = e.target.tagName;
  if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();

  if (k === 'q') { nudgeSteer(0); e.preventDefault(); return; }

  // A/← 與 D/→ 是「往畫面的哪一邊開」，不是前進倒車。車頭朝右時按 D 是前進，
  // 車頭朝左時按 D 就是倒車 —— 按鍵對應的是車子在圖上移動的方向。
  const toRight = k === 'd' || e.key === 'ArrowRight';
  if (toRight || k === 'a' || e.key === 'ArrowLeft') {
    const noseRight = Math.cos(pose.theta) >= 0;
    move(toRight === noseRight ? 1 : -1);
    e.preventDefault();
    return;
  }
  if (e.key === ' ' && plan) { setPlaying(!playing); e.preventDefault(); }
});

// 分頁在背景時不必重畫
document.addEventListener('visibilitychange', () => { if (!document.hidden) invalidate(); });

/** 把每個控制項的值對回 cfg／opts —— 從 localStorage 讀回來之後一定要做。 */
function syncControls() {
  for (const [id, key, out] of sliders) {
    const el = $('#' + id);
    if (!el || cfg[key] === undefined) continue;
    el.value = cfg[key];
    $('#' + out).textContent = (+cfg[key]).toFixed(2) + 'm';
  }
  for (const [id, key, out] of vehicleSliders) {
    $('#' + id).value = cfg.vehicle[key];
    $('#' + out).textContent = (+cfg.vehicle[key]).toFixed(2) + 'm';
  }
  $('#approach').value = cfg.approachFrom;
  $('#entry').value = cfg.entryStyle;
  for (const [id, key] of [['optSweep', 'sweep'], ['optTrace', 'trace'], ['optClear', 'clear'], ['optGhost', 'ghost'], ['optCompare', 'compare']]) {
    $('#' + id).checked = opts[key];
  }
  setStep(opts.step);
}

function syncSteerRange() {
  const max = Math.round(scene.v.maxSteerDeg);
  const el = $('#steer');
  el.min = -max; el.max = max;
  if (Math.abs(+el.value) > max) { el.value = 0; steerDeg = 0; $('#steerOut').textContent = '回正'; }
}

renderObstacles();
syncControls();
{
  const el = document.querySelector('#storeState');
  if (el) el.textContent = storeState;
}
resize();
refreshFacts();
invalidate();
