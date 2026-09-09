/**
 * 自行車模型（bicycle model）。
 *
 * 與參考版的關鍵差異：狀態點取「後軸中心」而非車體中心。
 * 後軸才是實車的瞬時迴轉基準，用車體中心積分會讓迴轉半徑與軌跡都失真。
 */
import { transform } from './geometry.js';

/** 一步位移的精確圓弧積分（不是 Euler，長步長也不會漂）。 */
export function integrate(pose, dist, steerRad, wheelbase) {
  const { x, y, theta } = pose;
  if (Math.abs(steerRad) < 1e-7) {
    return { x: x + dist * Math.cos(theta), y: y + dist * Math.sin(theta), theta };
  }
  const R = wheelbase / Math.tan(steerRad);
  const t2 = theta + dist / R;
  return {
    x: x + R * (Math.sin(t2) - Math.sin(theta)),
    y: y - R * (Math.cos(t2) - Math.cos(theta)),
    theta: t2,
  };
}

/** 車體四角（世界座標）。pose 為後軸中心。 */
export function bodyPolygon(pose, v) {
  const front = v.wheelbase + v.frontOverhang, back = -v.rearOverhang, hw = v.width / 2;
  return transform([[back, -hw], [front, -hw], [front, hw], [back, hw]], pose);
}

/** 後軸迴轉半徑（公尺）。 */
export function turnRadius(steerRad, v) {
  return Math.abs(steerRad) < 1e-7 ? Infinity : Math.abs(v.wheelbase / Math.tan(steerRad));
}

export function minTurnRadius(v) {
  return turnRadius(v.maxSteerDeg * Math.PI / 180, v);
}

/** 車體最外側（前外角）掃過的半徑，判斷「會不會刮到」用。 */
export function outerBodyRadius(v) {
  const R = minTurnRadius(v);
  return Math.hypot(R + v.width / 2, v.wheelbase + v.frontOverhang);
}

/** 把前輪轉角換算成方向盤圈數 —— 一般人不用「度」思考。 */
export function steerToTurns(steerDeg, v) {
  return steerDeg / v.maxSteerDeg * v.lockTurns;
}

export function describeSteer(steerDeg, v) {
  if (Math.abs(steerDeg) < 0.5) return '回正';
  const turns = Math.abs(steerToTurns(steerDeg, v));
  const side = steerDeg < 0 ? '向牆側' : '向外側';
  return `${side} ${turns.toFixed(2)} 圈`;
}

/**
 * 充電口在世界座標的位置。
 * 車身座標裡 -width/2 那一側是駕駛的左邊（車頭朝 +x 時等於畫面上方＝住家牆側）。
 */
export function chargePort(pose, v, fromNose) {
  const lx = v.wheelbase + v.frontOverhang - fromNose;
  return transform([[lx, -v.width / 2]], pose)[0];
}
