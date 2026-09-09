/**
 * 繪圖層。所有繪圖都在「公尺」座標系做，只在最外層乘一次縮放。
 *
 * 相對參考版的重點差異：
 *  - 掃掠面積（swept area）：實際會刮到東西的是車體外緣掃過的整片區域，
 *    只畫中心軌跡會嚴重低估風險。
 *  - 前輪會依照當下轉角轉動，看得出方向盤打了多少。
 *  - 最近的兩個車角會標出實際間隙數字。
 */
import { bodyPolygon, chargePort } from './vehicle.js';
import { SCOOTER_PITCH } from './config.js';

/** 計算把整個場景塞進畫布的縮放與位移。 */
/**
 * 場景到畫布的轉換。
 *
 * insetTop／insetBottom 是畫布上下要讓出來的畫素 —— 參數列與提示條浮在畫布上，
 * 場景畫進剩下的那一段才不會被蓋住。單位是畫布畫素（已含 devicePixelRatio）。
 */
export function makeView(canvas, scene, pad = 0.4, insetTop = 0, insetBottom = 0) {
  const shoulder = 1.05;
  const worldW = scene.xMax - scene.xMin;
  const worldH = scene.cfg.alleyWidth + shoulder * 2;
  const availH = Math.max(1, canvas.height - insetTop - insetBottom);
  const s = Math.min(canvas.width / (worldW + pad * 2), availH / (worldH + pad * 2));
  return {
    scale: s,
    offsetX: (canvas.width - worldW * s) / 2 - scene.xMin * s,
    offsetY: insetTop + (availH - worldH * s) / 2 + shoulder * s,
    apply(ctx) { ctx.setTransform(s, 0, 0, s, this.offsetX, this.offsetY); },
    toWorld(px, py) { return [(px - this.offsetX) / s, (py - this.offsetY) / s]; },
  };
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.roundRect(x, y, w, h, r);
}

export function drawScene(ctx, scene, view, opts = {}) {
  const { cfg, slot, xMin, xMax } = scene;
  const W = cfg.alleyWidth;
  view.apply(ctx);

  // 巷道路面
  ctx.fillStyle = '#2c343a';
  ctx.fillRect(xMin, 0, xMax - xMin, W);

  // 兩側建物
  ctx.fillStyle = '#aeb7ba';
  ctx.fillRect(xMin, -1.6, xMax - xMin, 1.6);
  ctx.fillRect(xMin, W, xMax - xMin, 1.6);

  // 巷道中線
  ctx.strokeStyle = 'rgba(150,162,168,.4)';
  ctx.lineWidth = 0.04;
  ctx.setLineDash([0.45, 0.35]);
  ctx.beginPath();
  ctx.moveTo(xMin, W / 2);
  ctx.lineTo(xMax, W / 2);
  ctx.stroke();
  ctx.setLineDash([]);

  // 目標車位（住家側兩個障礙之間的空隙）
  const slotDepth = scene.v.width + cfg.wallGap + 0.1;
  ctx.fillStyle = 'rgba(80,190,150,.15)';
  ctx.fillRect(slot.start, 0, slot.length, slotDepth);
  ctx.strokeStyle = 'rgba(90,220,170,.6)';
  ctx.lineWidth = 0.045;
  ctx.setLineDash([0.25, 0.2]);
  ctx.strokeRect(slot.start, 0.02, slot.length, slotDepth - 0.04);
  ctx.setLineDash([]);

  // 住家牆面／鐵捲門
  ctx.fillStyle = '#b87842';
  ctx.fillRect(slot.start, -0.13, slot.length, 0.13);

  // 障礙物。先畫汽車再畫機車與固定設施 —— 否則體積小的會被大的整個蓋掉。
  const drawable = scene.obstacles.filter(o => o.kind !== 'wall');
  const ordered = [
    ...drawable.filter(o => o.kind === 'car'),
    ...drawable.filter(o => o.kind !== 'car'),
  ];
  for (const o of ordered) {
    const x = o.cx - o.hx, y = o.cy - o.hy, w = o.hx * 2, h = o.hy * 2;
    if (o.kind === 'car') drawParkedCar(ctx, x, y, w, h);
    else if (o.kind === 'scooter') drawScooters(ctx, x, y, w, h);
    else drawFixed(ctx, x, y, w, h);
    // 機車群一律不寫字：一台機車才 0.7m 寬，字壓在分隔線上根本讀不出來。
    // 其他障礙物也要字放得下才寫 —— 像 0.6m 寬的電箱，字會整個溢出方塊變成一團糊。
    // 名稱在側欄的清單裡都看得到，圖上少幾行字反而看得清楚。
    if (o.kind !== 'scooter') {
      ctx.font = 'bold 0.23px system-ui, sans-serif';
      if (ctx.measureText(o.label).width < o.hx * 2 - 0.18) {
        label(ctx, o.label, o.cx, o.cy, o.kind === 'car');
      }
    }
  }

  // 尺寸標註
  ctx.fillStyle = 'rgba(255,255,255,.6)';
  ctx.font = '0.26px system-ui, sans-serif';
  ctx.textAlign = 'center';
  // 標在車位裡面而不是牆上那條帶子 —— 帶子上還有充電器與大門，擠在一起會互相蓋掉。
  ctx.fillText(`可用車位 ${slot.length.toFixed(2)}m`, slot.start + slot.length / 2, 0.36);
  ctx.textAlign = 'end';
  ctx.fillText(`巷寬 ${W.toFixed(2)}m`, xMax - 0.25, W / 2 - 0.16);
  ctx.textAlign = 'start';

  if (scene.gate) drawGate(ctx, scene, opts.gate);
  if (scene.farGate) drawFarGate(ctx, scene, opts.farGate);
  if (scene.charger) drawCharger(ctx, scene);
  drawRuler(ctx, scene);
  if (opts.narrowest && Number.isFinite(opts.narrowest.width)) drawNarrowest(ctx, scene, opts.narrowest);
  if (opts.goalGhost) drawGhost(ctx, scene.goal, scene.v);
}

/**
 * 巷道方向與公尺刻度。
 *
 * 少了這個，介面上所有「位置 7.10m」都不知道是從哪裡量起的；
 * 而且「往右挪」的右到底是誰的右，也得講清楚 —— 這裡的左右一律是「看圖的左右」。
 */
function drawRuler(ctx, scene) {
  const { xMin, xMax, cfg } = scene;
  const y = cfg.alleyWidth + 0.30;

  ctx.strokeStyle = 'rgba(52,60,68,.5)';
  ctx.fillStyle = 'rgba(52,60,68,.85)';
  ctx.lineWidth = 0.028;
  ctx.beginPath();
  ctx.moveTo(xMin, y);
  ctx.lineTo(xMax, y);
  ctx.stroke();

  ctx.font = '0.26px system-ui, sans-serif';
  ctx.textAlign = 'center';
  for (let x = Math.ceil(xMin); x <= xMax; x++) {
    const major = x % 2 === 0;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x, y + (major ? 0.20 : 0.10));
    ctx.stroke();
    if (major) ctx.fillText(String(x), x, y + 0.52);
  }

  ctx.font = 'bold 0.34px system-ui, sans-serif';
  ctx.fillStyle = 'rgba(40,48,56,.9)';
  ctx.textAlign = 'start';
  ctx.fillText('◀ 巷子左', xMin + 0.35, cfg.alleyWidth + 1.00);
  ctx.textAlign = 'end';
  ctx.fillText('巷子右 ▶', xMax - 0.35, cfg.alleyWidth + 1.00);
  ctx.textAlign = 'start';
}

function label(ctx, text, cx, cy, light) {
  ctx.fillStyle = light ? 'rgba(238,243,245,.9)' : 'rgba(20,28,34,.8)';
  ctx.font = 'bold 0.23px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(text, cx, cy + 0.08);
  ctx.textAlign = 'start';
}

function drawParkedCar(ctx, x, y, w, h) {
  ctx.fillStyle = '#aeb7c0';
  ctx.strokeStyle = '#1c2730';
  ctx.lineWidth = 0.05;
  roundRect(ctx, x, y, w, h, 0.18);
  ctx.fill(); ctx.stroke();
  ctx.fillStyle = '#33434e';
  roundRect(ctx, x + w / 2 - 0.6, y + 0.16, 1.2, Math.max(0.2, h - 0.34), 0.1);
  ctx.fill();
}

/** 機車群畫成一台一台的，跟現場一樣是垂直於牆面排開。 */
function drawScooters(ctx, x, y, w, h) {
  const n = Math.max(1, Math.round(w / SCOOTER_PITCH));
  const pitch = w / n;
  ctx.strokeStyle = 'rgba(20,28,34,.5)';
  ctx.lineWidth = 0.03;
  for (let i = 0; i < n; i++) {
    ctx.fillStyle = '#df8737';
    roundRect(ctx, x + i * pitch + 0.05, y, pitch - 0.1, h, 0.07);
    ctx.fill(); ctx.stroke();
  }
}

function drawFixed(ctx, x, y, w, h) {
  ctx.fillStyle = '#3a3f45';
  ctx.strokeStyle = 'rgba(20,28,34,.6)';
  ctx.lineWidth = 0.035;
  roundRect(ctx, x, y, w, h, 0.05);
  ctx.fill(); ctx.stroke();
}

/** 巷道最窄處：這常常才是「開不開得過去」的關鍵。 */
function drawNarrowest(ctx, scene, n) {
  const W = scene.cfg.alleyWidth;
  let top = 0, bottom = W;
  for (const o of scene.obstacles) {
    if (o.kind === 'wall') continue;
    if (o.cx + o.hx < n.x || o.cx - o.hx > n.x) continue;
    const y0 = o.cy - o.hy, y1 = o.cy + o.hy;
    if (y0 < 0.01) top = Math.max(top, y1); else bottom = Math.min(bottom, y0);
  }
  const tight = n.width < scene.v.width + 0.5;
  ctx.strokeStyle = tight ? '#ff6b6b' : '#ffd166';
  ctx.lineWidth = 0.06;
  ctx.setLineDash([0.16, 0.12]);
  ctx.beginPath();
  ctx.moveTo(n.x, top); ctx.lineTo(n.x, bottom);
  ctx.stroke();
  ctx.setLineDash([]);
  const txt = `最窄 ${n.width.toFixed(2)}m`;
  ctx.font = 'bold 0.26px system-ui, sans-serif';
  const tw = ctx.measureText(txt).width + 0.2;
  // 標在缺口靠對面那一端，不放正中間 —— 正中間正好是車要開過去的地方，會被車蓋住。
  // 缺口太窄就沒得挑，還是放中間。
  const ly = bottom - top > 1.0 ? bottom - 0.30 : (top + bottom) / 2;
  ctx.fillStyle = tight ? 'rgba(220,81,81,.95)' : 'rgba(23,33,42,.9)';
  roundRect(ctx, n.x - tw / 2, ly - 0.2, tw, 0.42, 0.1);
  ctx.fill();
  ctx.fillStyle = '#fff';
  ctx.textAlign = 'center';
  ctx.fillText(txt, n.x, ly + 0.1);
  ctx.textAlign = 'start';
}

/**
 * 住家大門：牆上的開口。被車擋住的那一段畫紅色，剩下的畫淡色並標出寬度。
 *
 * 可通行的那一段刻意不用實心綠 —— 那跟充電器同色，會被當成第二個充電器。
 * 開口是「沒有東西」的地方，用淡色底＋虛線框比較不會誤讀成物件。
 */
function drawGate(ctx, scene, info) {
  const { start, end } = scene.gate;
  const y = -0.30, h = 0.30;
  ctx.fillStyle = '#e6ebee';
  ctx.fillRect(start, y, end - start, h);

  if (info) {
    const half = scene.v.length / 2;
    const cx = scene.goal.x + scene.bodyOffset * Math.cos(scene.goal.theta);
    const a = Math.max(start, cx - half), b = Math.min(end, cx + half);
    if (b > a) { ctx.fillStyle = 'rgba(220,81,81,.8)'; ctx.fillRect(a, y, b - a, h); }
    const fs = info.rightFree >= info.leftFree ? Math.max(start, cx + half) : start;
    if (info.free > 0.01) {
      ctx.fillStyle = info.ok ? 'rgba(23,166,115,.22)' : 'rgba(237,158,50,.35)';
      ctx.fillRect(fs, y, info.free, h);
    }
  }

  ctx.strokeStyle = '#5b6870';
  ctx.lineWidth = 0.04;
  ctx.setLineDash([0.16, 0.12]);
  ctx.strokeRect(start, y, end - start, h);
  ctx.setLineDash([]);
  ctx.fillStyle = 'rgba(40,48,56,.9)';
  ctx.font = 'bold 0.28px system-ui, sans-serif';
  ctx.textAlign = 'center';
  const txt = info ? `大門 可通行 ${info.free.toFixed(2)}m` : '大門';
  ctx.fillText(txt, (start + end) / 2, -0.46);
  ctx.textAlign = 'start';
}

/** 對面建物的大門。擋不到你停車，但門前只停得下一台機車，會改變巷道最窄處在哪。 */
function drawFarGate(ctx, scene, info) {
  const { start, end } = scene.farGate;
  const W = scene.cfg.alleyWidth;
  ctx.fillStyle = info && !info.ok ? 'rgba(220,81,81,.7)' : '#e6ebee';
  ctx.fillRect(start, W, end - start, 0.30);
  ctx.strokeStyle = '#5b6870';
  ctx.lineWidth = 0.04;
  ctx.setLineDash([0.16, 0.12]);
  ctx.strokeRect(start, W, end - start, 0.30);
  ctx.setLineDash([]);
  ctx.fillStyle = 'rgba(40,48,56,.9)';
  ctx.font = 'bold 0.26px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(info ? `對面大門 可通行 ${info.free.toFixed(2)}m` : '對面大門', (start + end) / 2, W + 0.58);
  ctx.textAlign = 'start';
}

/** 住家牆上的充電器，以及「停到目標位置後」充電線要拉多遠。 */
function drawCharger(ctx, scene) {
  const { charger, cfg, v } = scene;
  ctx.fillStyle = '#17a673';
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 0.07;
  roundRect(ctx, charger.x - 0.26, -0.56, 0.52, 0.56, 0.08);
  ctx.fill(); ctx.stroke();
  ctx.strokeStyle = '#0b5f45';
  ctx.lineWidth = 0.03;
  roundRect(ctx, charger.x - 0.26, -0.56, 0.52, 0.56, 0.08);
  ctx.stroke();
  ctx.fillStyle = '#fff';
  ctx.font = 'bold 0.38px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('⚡', charger.x, -0.16);
  ctx.fillStyle = 'rgba(23,166,115,.95)';
  ctx.font = 'bold 0.26px system-ui, sans-serif';
  ctx.fillText('充電器', charger.x, -0.72);

  const port = chargePort(scene.goal, v, cfg.portFromNose);
  const d = Math.hypot(port[0] - charger.x, port[1] - charger.y);
  ctx.strokeStyle = d > 5 ? 'rgba(220,81,81,.85)' : 'rgba(23,166,115,.8)';
  ctx.lineWidth = 0.05;
  ctx.setLineDash([0.18, 0.14]);
  ctx.beginPath();
  ctx.moveTo(charger.x, 0);
  ctx.lineTo(port[0], port[1]);
  ctx.stroke();
  ctx.setLineDash([]);

  ctx.beginPath();
  ctx.arc(port[0], port[1], 0.13, 0, Math.PI * 2);
  ctx.fillStyle = '#17a673';
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 0.04;
  ctx.stroke();

  // 長度標在充電器正上方的牆面帶上，不放在線的中點 —— 中點會落在車位裡，
  // 蓋住正在看的停車空間，還會跟車角間隙那些標註擠在一起。
  const label = `充電線 ${d.toFixed(2)}m`;
  ctx.font = 'bold 0.26px system-ui, sans-serif';
  const w = ctx.measureText(label).width + 0.2;
  const mx = charger.x, my = -1.22;   // 再高一點，不然會壓到下面「充電器」那三個字
  ctx.fillStyle = d > 5 ? 'rgba(220,81,81,.95)' : 'rgba(11,95,69,.92)';
  roundRect(ctx, mx - w / 2, my - 0.2, w, 0.4, 0.1);
  ctx.fill();
  ctx.fillStyle = '#fff';
  ctx.fillText(label, mx, my + 0.1);
  ctx.textAlign = 'start';
}

function drawGhost(ctx, pose, v) {
  const pts = bodyPolygon(pose, v);
  ctx.strokeStyle = 'rgba(120,235,185,.75)';
  ctx.lineWidth = 0.05;
  ctx.setLineDash([0.2, 0.16]);
  ctx.beginPath();
  pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
  ctx.closePath();
  ctx.stroke();
  ctx.setLineDash([]);

  // 停妥後車頭朝哪，直接畫在目標框上
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.theta);
  const nose = v.wheelbase + v.frontOverhang;
  ctx.fillStyle = 'rgba(120,235,185,.9)';
  ctx.beginPath();
  ctx.moveTo(nose - 0.15, 0);
  ctx.lineTo(nose - 0.55, -0.28);
  ctx.lineTo(nose - 0.55, 0.28);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

/** 車體外緣掃過的區域。真正會刮到東西的就是這一片。 */
export function drawSweep(ctx, poses, v, upto = poses.length) {
  // 疊太多層會糊成一片實心色塊，反而看不到車。控制在 ~26 個輪廓。
  ctx.fillStyle = 'rgba(96,166,255,.055)';
  ctx.strokeStyle = 'rgba(96,166,255,.28)';
  ctx.lineWidth = 0.022;
  const stride = Math.max(1, Math.round(poses.length / 26));
  for (let i = 0; i < upto; i += stride) {
    const pts = bodyPolygon(poses[i], v);
    ctx.beginPath();
    pts.forEach((p, k) => (k ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
    ctx.closePath();
    ctx.fill(); ctx.stroke();
  }
}

/** 後軸中心軌跡；倒車段用不同顏色，一眼看出折了幾次。 */
export function drawTrace(ctx, poses, upto = poses.length, opts = {}) {
  if (poses.length < 2) return;
  ctx.save();
  // 對照用的軌跡畫細一點、灰一點，免得跟自己開的那條搶視線。
  ctx.lineWidth = opts.ghost ? 0.05 : 0.075;
  if (opts.ghost) ctx.globalAlpha = 0.55;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  let i = 1;
  while (i < upto) {
    const dir = poses[i].dir;
    ctx.strokeStyle = opts.ghost ? '#9aa3ad' : (dir < 0 ? '#ffb648' : '#4b9cff');
    ctx.beginPath();
    ctx.moveTo(poses[i - 1].x, poses[i - 1].y);
    while (i < upto && poses[i].dir === dir) { ctx.lineTo(poses[i].x, poses[i].y); i++; }
    ctx.stroke();
  }
  ctx.restore();
}

export function drawCar(ctx, pose, v, opts = {}) {
  const { hit = false, steer = 0 } = opts;
  ctx.save();
  if (opts.ghost) ctx.globalAlpha = 0.3;
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.theta);

  const back = -v.rearOverhang;
  const nose = v.wheelbase + v.frontOverhang;
  const hw = v.width / 2;

  ctx.fillStyle = hit ? '#ffd4d4' : '#ffffff';
  ctx.strokeStyle = hit ? '#d43b3b' : '#17212a';
  ctx.lineWidth = 0.055;
  roundRect(ctx, back, -hw, v.length, v.width, 0.2);
  ctx.fill(); ctx.stroke();

  // 車艙偏後，車頭留白 —— 光靠外形就看得出哪一頭是前面
  ctx.fillStyle = '#273944';
  roundRect(ctx, back + 0.55, -hw * 0.74, 1.85, v.width * 0.74, 0.14);
  ctx.fill();

  // 擋風玻璃
  ctx.fillStyle = '#7ea8c9';
  roundRect(ctx, back + 2.45, -hw * 0.66, 0.34, v.width * 0.66, 0.06);
  ctx.fill();

  // 車頭：整片塗色 + 箭頭，方向不會看錯
  ctx.fillStyle = hit ? '#d43b3b' : '#1f6fd0';
  ctx.beginPath();
  ctx.moveTo(nose - 0.62, -hw + 0.06);
  ctx.lineTo(nose - 0.12, -hw + 0.06);
  ctx.quadraticCurveTo(nose - 0.02, 0, nose - 0.12, hw - 0.06);
  ctx.lineTo(nose - 0.62, hw - 0.06);
  ctx.closePath();
  ctx.fill();

  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.moveTo(nose - 0.20, 0);
  ctx.lineTo(nose - 0.50, -0.30);
  ctx.lineTo(nose - 0.50, 0.30);
  ctx.closePath();
  ctx.fill();

  // 頭燈
  ctx.fillStyle = '#ffeec2';
  ctx.fillRect(nose - 0.70, -hw + 0.08, 0.09, 0.34);
  ctx.fillRect(nose - 0.70, hw - 0.42, 0.09, 0.34);

  // 輪胎：前輪跟著轉角轉，看得出方向盤打了多少
  ctx.fillStyle = '#12181d';
  const tw = 0.63, th = 0.19;
  for (const side of [-1, 1]) {
    ctx.fillRect(-0.32, side * hw - (side > 0 ? th : 0), tw, th);
    ctx.save();
    ctx.translate(v.wheelbase, side * hw - (side > 0 ? th / 2 : -th / 2));
    ctx.rotate(steer);
    ctx.fillRect(-tw / 2, -th / 2, tw, th);
    ctx.restore();
  }

  // 充電口（駕駛座前面一點，車身左側）。畫成貼在車身上的一塊綠標，像油箱蓋 ——
  // 掛在車外的話會跟車角間隙那些標註疊在一起被蓋掉，而它們是預設開著的。
  // 不寫字：圖上東西已經夠多，名稱在圖例與側欄裡都有。
  if (opts.portFromNose) {
    const px = nose - opts.portFromNose;
    const py = -hw + 0.02;
    roundRect(ctx, px - 0.26, py - 0.12, 0.52, 0.24, 0.08);
    ctx.fillStyle = '#17a673';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 0.05;
    ctx.stroke();
  }

  // 「車頭」二字永遠正著寫，車身轉過來也讀得到
  ctx.save();
  ctx.rotate(-pose.theta);
  ctx.fillStyle = '#1f6fd0';
  ctx.font = 'bold 0.3px system-ui, sans-serif';
  ctx.textAlign = 'center';
  const c = Math.cos(pose.theta), sn = Math.sin(pose.theta);
  const lx = (nose + 0.42) * c, ly = (nose + 0.42) * sn;
  ctx.fillText('車頭', lx, ly + 0.1);
  ctx.restore();

  ctx.restore();
}

/** 標出最近的兩個車角與實際間隙 —— 開車時真正在看的數字。 */
export function drawClearances(ctx, corners, view) {
  const near = corners.filter(c => Number.isFinite(c.distance)).slice(0, 2);
  for (const c of near) {
    const warn = c.distance < 0.15;
    ctx.beginPath();
    ctx.arc(c.point[0], c.point[1], 0.11, 0, Math.PI * 2);
    ctx.fillStyle = warn ? '#dc5151' : '#17a673';
    ctx.fill();
    const label = `${c.corner} ${c.distance.toFixed(2)}m`;
    ctx.font = 'bold 0.26px system-ui, sans-serif';
    const w = ctx.measureText(label).width + 0.18;
    const bx = c.point[0] + 0.18, by = c.point[1] - 0.36;
    ctx.fillStyle = warn ? 'rgba(220,81,81,.94)' : 'rgba(23,33,42,.88)';
    roundRect(ctx, bx, by, w, 0.4, 0.1);
    ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.fillText(label, bx + 0.09, by + 0.29);
  }
}
