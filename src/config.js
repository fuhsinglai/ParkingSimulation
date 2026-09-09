/**
 * 車輛與場景參數。
 *
 * 車輛尺寸取自 Toyota RAV4 (XA50, 2019+)。
 * maxSteerDeg 不是隨便填的：由原廠迴轉直徑 11.0m 反推 ——
 *   外前輪迴轉半徑 5.5m = hypot(R + track/2, wheelbase)
 *   → R(後軸迴轉半徑) ≈ 4.0m → δmax = atan(wheelbase / R) ≈ 34°
 * 參考版寫 38°，會讓車比實車更靈活，模擬出來的結果偏樂觀。
 */
export const VEHICLE_BASE = {
  name: 'Toyota RAV4 PHEV (XA50)',
  length: 4.60,
  width: 1.855,
  wheelbase: 2.69,
  frontOverhang: 0.94,
  rearOverhang: 0.97,
  track: 1.60,
  /**
   * 規格表上的「最小迴轉半徑」（量的是外前輪）。
   * PHEV 標配 19 吋輪，是 5.7m；一般版 17 吋才是 5.5m —— 差 0.2m 看似不多，
   * 但換算成後軸迴轉半徑是 4.23m vs 3.99m，在只差十幾公分的車位裡差很多。
   */
  turningRadius: 5.70,
  /** 方向盤單邊圈數（lock-to-lock 約 2.7 圈） */
  lockTurns: 1.35,
};

/**
 * 由規格表的最小迴轉半徑反推前輪最大轉角。
 *
 *   R_外前輪 = hypot(R_後軸 + 輪距/2, 軸距)
 *   → R_後軸 = sqrt(R_外前輪² - 軸距²) - 輪距/2
 *   → δmax  = atan(軸距 / R_後軸)
 *
 * 直接讓使用者填規格表上查得到的數字，比要他猜「前輪最大轉幾度」實際得多。
 */
export function steerFromTurningRadius(v) {
  const rear = Math.sqrt(Math.max(0.04, v.turningRadius ** 2 - v.wheelbase ** 2)) - v.track / 2;
  return Math.atan(v.wheelbase / Math.max(0.6, rear)) * 180 / Math.PI;
}

/** 把可調的車輛欄位補上推導出來的最大轉角。 */
export function makeVehicle(over = {}) {
  const v = { ...VEHICLE_BASE, ...over };
  v.maxSteerDeg = steerFromTurningRadius(v);
  return v;
}

export const VEHICLE = makeVehicle();

/** 一台機車（含後照鏡）垂直停放時佔的巷道寬度，用來把「長度」換算成台數。 */
export const SCOOTER_PITCH = 0.55;

/**
 * 巷道本身的參數。
 *
 * 座標：x 沿巷道，y = 與住家牆面的距離（0 = 住家牆面，alleyWidth = 對面建物）。
 */
export const DEFAULT_SCENE = {
  /** 車輛規格。只存可調的欄位，maxSteerDeg 每次由迴轉半徑推導。 */
  vehicle: {
    length: VEHICLE_BASE.length,
    width: VEHICLE_BASE.width,
    wheelbase: VEHICLE_BASE.wheelbase,
    frontOverhang: VEHICLE_BASE.frontOverhang,
    rearOverhang: VEHICLE_BASE.rearOverhang,
    turningRadius: VEHICLE_BASE.turningRadius,
  },

  alleyWidth: 5.90,
  wallGap: 0.20,
  /** 想停的大概位置；實際車位長度由左右障礙物之間的空隙算出來。 */
  targetX: 7.10,
  /**
   * approachFrom  車從巷子的哪一頭開進來
   * parkDirection 停妥後車頭朝哪 —— 這是「推導」出來的，不開放選擇。
   *
   * 平行停車不管倒車或前進入庫，車最後都與牆平行、車頭沿著行進方向，
   * 而「從左邊進來」＝往右開＝車頭朝右（兩者相反）。要停成反方向只能在巷內調頭，
   * 窄巷做不到，所以介面上不提供，免得選到必然無解的組合。
   */
  approachFrom: 'left',
  parkDirection: 'right',

  /**
   * 停妥後車頭與前車的間距。
   * 不是「停在車位正中間」—— 實際上要貼緊前車，後面才留得出開門的空間。
   * 「前車」是哪一台由車頭朝向決定，不是由障礙物的名稱決定。
   */
  frontGap: 0.15,

  /**
   * 住家大門。這不是障礙物，是「不能擋住」的區域 ——
   * 車貼緊鄰車停，就是為了把大門這一段讓出來。
   *
   * 實測：家門前的立面總長 6.10m = 可停段 4.60m ＋ 大門 1.50m。
   * 可停段剛好等於車長，所以車貼緊鄰車停好之後大門幾乎全部露出來。
   *   鄰車 0.38~4.43 → 可停段 4.43~9.03 → 大門 9.03~10.53 → 機車 10.53~
   */
  gate: true,
  gateX: 9.775,
  gateWidth: 1.50,

  /** 對面建物的大門。同樣是不能擋的開口，門前只停得下一台機車。 */
  farGate: true,
  farGateX: 9.15,
  farGateWidth: 1.65,

  /** 住家牆上充電器的位置（沿巷道）。 */
  charger: true,
  chargerX: 8.10,
  /**
   * 車上充電口的位置：在車身「左側」、距車頭這麼遠。
   * 使用者描述是「駕駛座前面一點」——台灣右駕道路、駕駛座在左，所以是左前側。
   * 車頭朝右時左側才會朝住家牆，這也決定了充電要停哪一個方向。
   */
  portFromNose: 2.00,
  /**
   * 入庫方式：'any' 讓規劃器自己選、'reverse' 倒車入庫、'forward' 前進入庫。
   * 因為停入與駛出是同一條路徑倒著走，倒車入庫 ⇔ 前進駛出。
   */
  entryStyle: 'any',
};

/**
 * 障礙物清單 —— 依現場配置圖換算，每一項的位置與尺寸都可以在介面上調。
 *
 *   side  : 'house' 住家側（車位這一側）／'far' 對面側
 *   kind  : 'car' 汽車／'scooter' 機車群／'fixed' 固定設施（電箱、水錶、垃圾桶）
 *   x     : 中心的縱向位置（公尺）
 *   length: 沿巷道方向的長度
 *   depth : 從所屬牆面突出到巷道裡的深度
 */
export const DEFAULT_OBSTACLES = [
  { id: 'rearCar', label: '車位後方停車', side: 'house', kind: 'car', x: 2.40, length: 4.05, depth: 1.90, on: true },
  { id: 'frontScooters', label: '車位前方機車', side: 'house', kind: 'scooter', x: 11.35, length: 1.65, depth: 1.90, on: true },
  { id: 'leftScooters', label: '對面左側斜停機車', side: 'far', kind: 'scooter', x: 2.50, length: 2.60, depth: 1.60, on: true },
  { id: 'oppCar', label: '對面停車', side: 'far', kind: 'car', x: 5.95, length: 3.85, depth: 1.78, on: true },
  // 對面大門前只停得下一台機車 —— 門要留出入口，排不了一整排。
  { id: 'oppScooters', label: '對面大門前機車', side: 'far', kind: 'scooter', x: 10.28, length: 0.55, depth: 1.70, on: true },
  { id: 'oppBox', label: '電箱／水錶', side: 'far', kind: 'fixed', x: 11.15, length: 0.60, depth: 1.35, on: true },
  { id: 'oppScooters2', label: '對面機車群（右）', side: 'far', kind: 'scooter', x: 11.95, length: 1.10, depth: 1.70, on: true },
];

/** 安全裕度：規劃器要求車體與障礙至少留這麼多（公尺）。 */
export const SAFETY_MARGIN = 0.06;
