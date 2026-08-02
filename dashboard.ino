// dashboard.ino
// M5Stack M5Paper —— PC 系统监视器
//
// 串口协议:  "MEM:<0-100>,MEMG:<GB>,TEMP:<°C>,PWR:<watts>\n"
//
// 单个区块布局(竖屏 540 × 960,共 3 个区块):
//
//   ─────────────────────────────────────────────
//   MEMORY
//   ┌──────────────────────┐
//   └──────────────────────┘          83 G
//   ─────────────────────────────────────────────
//
// 进度条在左,大号数值在条右侧、按条高垂直居中。
// 每个区块用自己的小画布,推送到对应的屏幕 Y 坐标。

#include <M5EPD.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include "binaryttf.h"

// ── 时间参数 ─────────────────────────────────────────────────────────────────

#define SERIAL_BAUD         115200
#define LOOP_DELAY_MS       50
#define UPDATE_INTERVAL_MS    10000UL
#define DISCONNECT_TIMEOUT_MS 30000UL

// ── 刷新策略(抗反白)────────────────────────────────────────────────────────
// 反白(黑态褪成灰白)的根因经热像实测已确认是**面板下方芯片区的局部自发热**,不是早先
// 以为的局部刷新串扰。2026-07-10 实测(室温 26.3 °C):反白最严重处 33.7 °C(+7.4 °C),
// 屏幕其余部分 27 °C(+0.7 °C ≈ 室温)。判据:
//   · 其余部分几乎等于室温 → 排除环境热、排除整机蓄热;
//   · 只有一处集中热点     → 是某个持续耗电的元件,而非大面积发热(如电池充电);
//   · 三块目前刷新方式完全相同,却只有下半部发白 → 串扰假说被证伪。
// 机理:墨水层夹在外壳与主板之间,比外表面读数更热(约 35–40 °C),已进入这类面板图像
// 保持能力陡降的区间。温度升高使胶囊内液体粘度下降、布朗运动增强、TFT 漏电增大(约每
// 8–10 °C 翻倍),黑态保持不住而漂向白。
//
// 边界为何如此干脆(下半部明显白、上半部完全没事):热区冷区衰减速率只差约 2 倍,但旧逻辑
// 把刷新完全挂在“整数数值变化”上、没有任何时钟兜底 —— 主机空闲时一次都不刷,漂移可以连续
// 累积几小时无上限,足以让热区越过肉眼可见阈值而冷区仍在阈值之下。
//
// 于是策略分两条正交的轴:
//   A 降热(治本):刷完让 IT8951 进 StandBy,别再 24 小时常燃(见 EPD_STANDBY_IDLE);
//                  每次只重刷真正变化的那一块,不再无谓地三块全刷(串扰假说已证伪)。
//   B 封顶(兜底):按时钟强制刷新给累积封顶,且 GC16 深刷同时受“次数”和“时间”触发。
// 注意:全刷家族(GC16 / GL16)无视新旧、驱动区域内每个像素,才能把淡掉的像素拉回黑;
//       差分模式(DU/A2/DU4)只驱动“新内容 ≠ 帧缓存旧内容”的像素,对物理漂移无效。
#define REFRESH_MODE_FULL  UPDATE_MODE_GC16   // 周期深刷:反色全刷、零残影(闪最明显)
#define REFRESH_MODE_KEEP  UPDATE_MODE_GL16   // 平时刷新:驱动像素消反白,闪较轻、带些残影
#define GC16_EVERY         30                 // 每累计这么多次刷新,做一次 GC16 全刷清残影

// ── 降温 / 兜底 ──────────────────────────────────────────────────────────────
// EPD_STANDBY_IDLE:M5EPD 的 begin() 之后 EPD 电源与 IT8951 一直停在 SYS_RUN,即使整晚
// 不刷屏也在满功率空转发热 —— 热点就是这么来的。改成刷之前 Active()、刷完 StandBy(),
// 把常燃变成按需。StandBy 比 Sleep 保守(保留 DRAM 自刷新,波形表不丢),先用它;若实测
// 降温不够再考虑 Sleep() 或直接 disableEPDPower()。置 0 可一键退回原行为。
// 空闲时 IT8951 进哪种低功耗态:2=Sleep(更深、更省) 1=StandBy(保守) 0=不休眠(原行为)。
// 若出现刷新异常(花屏/不刷,即波形表丢失),把它改成 1 即可退回保守档。
#define EPD_IDLE_MODE       2
#define EPD_WAKE_DELAY_MS   50          // Active() 之后给 IT8951 的稳定时间(Sleep 唤醒比 StandBy 慢)

// 时钟兜底:这两条只会**增加**刷新,不会降低刷新频率、不影响数据实时性。
// 代价:三块 GL16 约 1.35 s,10 分钟一次 ≈ 0.2% 占空比,发热可忽略。
#define REFRESH_MAX_AGE_MS  600000UL    // 超过 10 分钟没刷过,数值没变也强制刷一遍(给漂移封顶)
#define GC16_MAX_AGE_MS    1800000UL    // 距上次 GC16 超过 30 分钟就深刷一次(空闲时也能清干净)

// ── 屏幕 ─────────────────────────────────────────────────────────────────────

#define SCR_W   M5EPD_PANEL_W   // 540
#define SCR_H   M5EPD_PANEL_H   // 960

// ── 区块布局 ─────────────────────────────────────────────────────────────────
//
// 三个区块无缝铺满整块屏幕(960 / 3 = 320 px 每块,占满宽度)。由于每个像素都在
// 被刷新的区域内,长时间运行也不会在边缘形成永久的“未刷新边框”而积累残影。
// 内容四周的留白是在每个区块内部用 padding 制造的,而不是靠留出屏幕边缘不刷新。

#define NUM_METRICS 3
#define SECTION_H   (SCR_H / NUM_METRICS)     // 320 px —— 铺满整屏

// 内部留白(内容与区块/屏幕边缘之间的空白)
#define PAD_TOP     20                        // 区块内顶部留白
#define PAD_BOT     20                        // 区块内底部留白
#define PAD_LEFT    30                        // 左侧留白(稍宽一点)

// 区块内可用的内容带
#define CONTENT_TOP PAD_TOP
#define CONTENT_H   (SECTION_H - PAD_TOP - PAD_BOT)   // 280
#define HALF_H      (CONTENT_H / 2)                    // 140

// 标签行 —— 内容带上半部分,垂直居中
#define LABEL_X     PAD_LEFT
#define LABEL_CY    (CONTENT_TOP + HALF_H / 2)         // ≈90

// 进度条 —— 内容带下半部分,占左侧 2/3 宽度
#define BAR_X       PAD_LEFT
#define BAR_W       (SCR_W * 2 / 3)                    // 360 px —— 三分之二
#define BAR_TOP     (CONTENT_TOP + HALF_H)             // 160
#define BAR_H       HALF_H                              // 140
#define BAR_BORDER  8                                   // 边框粗细(原为 2)

// 数值文字 —— 进度条右侧空白区的底部居中
#define VAL_CX      ((BAR_X + BAR_W + SCR_W) / 2)      // 条右侧空白区的中心
#define VAL_CY      (BAR_TOP + BAR_H)                   // 基线 = 条底部

// 字号 —— 每个字号只 createRender 一次,避免 PSRAM 碎片化。
// 两种字号载入同一个画布,用 setTextSize() 切换。
#define FONT_LABEL  60                        // "MEMORY" / "TEMPERATURE" / "POWER"
#define FONT_VALUE  CONTENT_H                  // 280 —— 填满内容带高度

// 伪粗体字重 —— TTF 只有一种字重,通过在微小偏移处叠印来加粗笔画。N 约增加 N px
// 笔画宽度(以及 N px 高度,这里可忽略)。想离远了更醒目就调大;设为 0 即原始细笔画。
#define BOLD_VALUE  2
#define BOLD_LABEL  1

// 向上箭头图标 —— 当某指标高于其风扇关闭阈值(即它正是风扇运转的原因)时,画在该行。
// 位置/高度与标签一致:右边缘对齐进度条右边缘,高度 ≈ 标签高度。
#define ARROW_R   (FONT_LABEL / 2)                 // 半高 ≈ 标签高度的一半
#define ARROW_CX  (BAR_X + BAR_W - ARROW_R)        // 靠近进度条右边缘
#define ARROW_CY  LABEL_CY                         // 在标签行垂直居中

// 温度条刻度:20 °C = 0 %,100 °C = 100 %
#define TEMP_MIN_C  20
#define TEMP_MAX_C  100

// ── 风扇 / 继电器控制(G26) ─────────────────────────────────────────────────
// 滞回:温度/功率达到或超过开启阈值时风扇 ON,低到或低于关闭阈值时 OFF;两阈值
// 之间保持当前状态。断连(无温度读数)时强制 OFF。
// 关闭延时(抗抖动):负载回落到关阈值以下后,还要持续 FAN_OFF_DELAY_MS 才真正关;
// 期间只要负载任一项再升到关阈值以上,计时立即清零重来。于是风扇一旦开启至少运转
// 这段时间,负载在阈值附近反复波动也不会频繁开关。
#define FAN_PIN         26
#define FAN_ON_C        60         // °C —— 达到/超过此温度则开风扇
#define FAN_OFF_C       50         // °C —— 仅当低于此温度才允许关
#define FAN_ON_W        30         // W  —— 达到/超过此功率则开风扇
#define FAN_OFF_W       20         // W  —— 仅当低于此功率才允许关
#define FAN_OFF_DELAY_MS 60000UL   // ms —— 负载持续低于关阈值多久才真正关(1 分钟)
#define FAN_ACTIVE_LOW  0          // 若继电器为低电平触发(LOW 导通),设为 1

// 继电器(M5Stack 三线式模块)的 5V 取自 PORT.B,由板上 EXT 5V 升压电路提供。
// M5.begin() 之后这条升压一直开着,但风扇不转时它没有任何负载,纯属空耗。
// 置 1 则:开风扇前才给 EXT 上电,关风扇后立刻断掉。
// 注意上下电顺序 —— 上电时先供 5V 再给信号,断电时先撤信号再断 5V,避免经信号脚倒灌。
#define FAN_GATE_EXT_POWER  1
#define FAN_EXT_SETTLE_MS   50     // 给 EXT 5V 的建立时间

// 0 = 白,15 = 黑(M5Paper IT8951 约定)
#define C_WHITE  0
#define C_BLACK  15

// ── 数据 ─────────────────────────────────────────────────────────────────────

struct Stats { int mem, memGB, temp, pwr; };

Stats cur  = {0, 0, 0, 0};
Stats prev = {-1, -1, -1, -1};

bool     connected     = false;   // 收到一个有效数据包后置 true
bool     prevConnected = false;
uint32_t lastRxMs      = 0;
bool     fanOn         = false;
uint32_t fanOffEligibleSinceMs = 0;  // 负载连续低于关阈值的起始时刻(0 = 未在低负载/已清零)
bool     forceRefresh  = false;   // 风扇开/关时置位,不等更新间隔立即重绘

uint32_t lastUpdateMs  = 0;
uint32_t lastPushMs[NUM_METRICS] = {0, 0, 0};  // 各块上次被驱动的时刻(REFRESH_MAX_AGE_MS 按块兜底)
uint32_t lastGc16Ms    = 0;  // 上次 GC16 深刷的时刻(用于 GC16_MAX_AGE_MS 兜底)
uint32_t lastAfsrWaitMs = 0; // 上次待机前等面板刷完花了多久(诊断用,经 [T] 回报)
uint8_t  refreshCount = 0;   // GL16 刷新累计数;到 GC16_EVERY 就触发一次 GC16 全刷
bool     prevArrow[NUM_METRICS]  = {false, false, false};  // 上次绘制的箭头状态
String   rxBuf        = "";

Adafruit_SHT31 sht30;
M5EPD_Canvas   sCanvas(&M5.EPD);

static const char* LABELS[NUM_METRICS] = { "MEMORY", "TEMPERATURE", "POWER" };

// ── 前向声明 ─────────────────────────────────────────────────────────────────

void drainSerial();
void handleLine(const String& line);
void reportTempHum();
void setFan(bool on);
void updateFan();
void drawUpArrow(int cx, int cy, int r);
void parseStats(const String& line);
int  extractInt(const String& s, const char* key);
int  barPct(int idx);
void valStr(int idx, char* buf, size_t len);
bool arrowOn(int idx);
bool sectionChanged(int idx);
void saveSectionPrev(int idx);
void pushSection(int idx, m5epd_update_mode_t mode);
void pushAll();
void epdWake();
void epdRest();

// ── 初始化与主循环 ───────────────────────────────────────────────────────────

void setup() {
  M5.begin(false, false, false, false, false);
  setCpuFrequencyMhz(80);
  Serial.begin(SERIAL_BAUD);
  Serial.flush();

  Wire.begin(21, 22);
  sht30.begin(0x44);

  pinMode(FAN_PIN, OUTPUT);
  setFan(false);             // 在温度读数说明之前,风扇先保持关闭

  M5.EPD.SetRotation(0);
  M5.EPD.Clear(true);

  sCanvas.loadFont(binaryttf, sizeof(binaryttf));
  sCanvas.createCanvas(SCR_W, SECTION_H);

  // 为两种字号各创建一次 render。
  // 字形缓存数量保持适度,避免 PSRAM 碎片化。
  sCanvas.createRender(FONT_LABEL, 48);
  sCanvas.createRender(FONT_VALUE, 24);

  pushAll();
  epdRest();                 // 底图画完就让 IT8951 待机,后续按需唤醒
  prev = cur;
  for (int i = 0; i < NUM_METRICS; i++) prevArrow[i] = arrowOn(i);
  uint32_t t0   = millis();
  lastUpdateMs  = t0;
  for (int i = 0; i < NUM_METRICS; i++) lastPushMs[i] = t0;   // 兜底计时从开机起算
  lastGc16Ms    = t0;        // pushAll 本身就是一次 GC16 深刷
}

void loop() {
  delay(LOOP_DELAY_MS);
  drainSerial();

  uint32_t now = millis();

  if (connected && (now - lastRxMs >= DISCONNECT_TIMEOUT_MS))
    connected = false;

  updateFan();

  if (forceRefresh || now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
    forceRefresh = false;
    lastUpdateMs = now;

    // 逐块判断是否需要重绘:①数值变了,或②这一块自己太久没被驱动过。
    // 兜底必须**按块**算:反白发生在长期不重绘的那一块上,而不是"整屏多久没刷过"——
    // 内存频繁跳动时整屏一直在刷,但温度/功率那两块可能很久没被驱动,正是它们会褪色。
    bool needPush[NUM_METRICS];
    bool any = false;
    for (int i = 0; i < NUM_METRICS; i++) {
      needPush[i] = sectionChanged(i) ||
                    (now - lastPushMs[i] >= REFRESH_MAX_AGE_MS);
      if (needPush[i]) any = true;
    }

    if (any) {
      // 深刷时机:累计次数到了,或距上次 GC16 太久(空闲时段靠后者兜底)
      bool deepClean = (++refreshCount >= GC16_EVERY) ||
                       (now - lastGc16Ms >= GC16_MAX_AGE_MS);
      // 深刷时三块一起清残影;平时只刷需要刷的块 —— 面板驱动工作量约降到 1/3
      m5epd_update_mode_t mode = deepClean ? REFRESH_MODE_FULL : REFRESH_MODE_KEEP;

      epdWake();
      for (int i = 0; i < NUM_METRICS; i++) {
        if (!deepClean && !needPush[i]) continue;
        pushSection(i, mode);
        lastPushMs[i] = millis();
        saveSectionPrev(i);   // 只把已重绘块的基准前移,没刷的块保持旧基准
      }
      epdRest();

      if (deepClean) { refreshCount = 0; lastGc16Ms = millis(); }
      // 连接状态一变会让三块同时判定为“已变化”,故此处无条件前移是安全的
      prevConnected = connected;
    }
  }
}

// ── 串口 ─────────────────────────────────────────────────────────────────────

void drainSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) { handleLine(rxBuf); rxBuf = ""; }
    } else if (rxBuf.length() < 128) {
      rxBuf += c;
    }
  }
}

void handleLine(const String& line) {
  if (line == "[T]") { reportTempHum(); return; }
  parseStats(line);
}

void reportTempHum() {
  float t = sht30.readTemperature();
  float h = sht30.readHumidity();
  if (isnan(t) || isnan(h)) {
    Serial.println("{\"error\": \"sht30 read failed\"}");
  } else {
    // 附带 afsr_ms:上次待机前等面板刷完的毫秒数。设备没有调试口,靠它从主机侧确认
    // 刷新确实跑完了才 StandBy(0 表示还没刷过或没等到)。JSON 多一个键不影响解析。
    char buf[80];
    snprintf(buf, sizeof(buf), "{\"Temp\": %.1f, \"Hum\": %.1f, \"afsr_ms\": %lu}",
             t, h, (unsigned long)lastAfsrWaitMs);
    Serial.println(buf);
  }
}

void parseStats(const String& line) {
  bool got = false;
  int v;
  if ((v = extractInt(line, "MEM:"))  >= 0) { cur.mem   = constrain(v, 0, 100); got = true; }
  if ((v = extractInt(line, "MEMG:")) >= 0) { cur.memGB = v;                    got = true; }
  if ((v = extractInt(line, "TEMP:")) >= 0) { cur.temp  = v;                    got = true; }
  if ((v = extractInt(line, "PWR:"))  >= 0) { cur.pwr   = v;                    got = true; }
  if (got) { connected = true; lastRxMs = millis(); }
}

int extractInt(const String& s, const char* key) {
  int idx = s.indexOf(key);
  if (idx < 0) return -1;
  int st = idx + strlen(key), en = st;
  while (en < (int)s.length() && isDigit(s.charAt(en))) en++;
  return (en > st) ? s.substring(st, en).toInt() : -1;
}

// ── 风扇 / 继电器控制 ────────────────────────────────────────────────────────

void setFan(bool on) {
  bool changed = (on != fanOn);
  fanOn = on;
  int level = on ? HIGH : LOW;
  if (FAN_ACTIVE_LOW) level = !level;

#if FAN_GATE_EXT_POWER
  if (on) {
    M5.enableEXTPower();            // 先供 5V
    delay(FAN_EXT_SETTLE_MS);       // 等它建立
    digitalWrite(FAN_PIN, level);   // 再给信号
  } else {
    digitalWrite(FAN_PIN, level);   // 先撤信号
    M5.disableEXTPower();           // 再断 5V,消除风扇不转时升压电路的空耗
  }
#else
  digitalWrite(FAN_PIN, level);
#endif

  if (changed) forceRefresh = true;   // 立即把箭头变化反映到屏幕上
}

void updateFan() {
  if (!connected) {                     // 无数据 → 立即关(安全优先,不受延时约束)
    if (fanOn) setFan(false);
    fanOffEligibleSinceMs = 0;
    return;
  }
  if (!fanOn) {
    // 温度 或 功率 任一越过高阈值则开
    if (cur.temp >= FAN_ON_C || cur.pwr >= FAN_ON_W) {
      setFan(true);
      fanOffEligibleSinceMs = 0;        // 刚开:关闭计时清零,重新起表
    }
  } else {
    // 关闭延时:温度 且 功率都回落到低阈值以下才算"可关",并须持续 FAN_OFF_DELAY_MS。
    // 中途负载任一项升回阈值以上就清零重计,于是每次开启后至少运转这段时间。
    bool offEligible = (cur.temp <= FAN_OFF_C && cur.pwr <= FAN_OFF_W);
    if (!offEligible) {
      fanOffEligibleSinceMs = 0;        // 负载仍高:清零,持续时间重新计
    } else {
      uint32_t now = millis();
      if (fanOffEligibleSinceMs == 0) fanOffEligibleSinceMs = now;   // 刚进入低负载,起表
      if (now - fanOffEligibleSinceMs >= FAN_OFF_DELAY_MS) setFan(false);
    }
  }
}

// ── 各区块辅助函数 ───────────────────────────────────────────────────────────

int barPct(int idx) {
  if (!connected) return 0;
  switch (idx) {
    case 0: return cur.mem;
    case 1: return constrain((cur.temp - TEMP_MIN_C) * 100
                              / (TEMP_MAX_C - TEMP_MIN_C), 0, 100);
    case 2: return constrain(cur.pwr, 0, 100);
  }
  return 0;
}

void valStr(int idx, char* buf, size_t len) {
  if (!connected) { snprintf(buf, len, "-"); return; }
  switch (idx) {
    case 0: snprintf(buf, len, "%d", cur.memGB); break;
    case 1: snprintf(buf, len, "%d", cur.temp);  break;
    case 2: snprintf(buf, len, "%d", cur.pwr);   break;
  }
}

// 箭头与风扇同步:仅在风扇运转时显示,且画在仍维持其运转的那一行(指标高于其
// 关闭阈值)。因此最后一个箭头会在风扇关闭的瞬间消失。
bool arrowOn(int idx) {
  if (!fanOn) return false;
  if (idx == 1) return cur.temp > FAN_OFF_C;
  if (idx == 2) return cur.pwr  > FAN_OFF_W;
  return false;
}

bool sectionChanged(int idx) {
  if (connected != prevConnected) return true;
  if (!connected) return false;
  if (arrowOn(idx) != prevArrow[idx]) return true;
  switch (idx) {
    case 0: return cur.mem  != prev.mem  || cur.memGB != prev.memGB;
    case 1: return cur.temp != prev.temp;
    case 2: return cur.pwr  != prev.pwr;
  }
  return false;
}

// 把某一块的对比基准前移到当前值。必须逐块做:现在只重刷变化的块,若整体 prev = cur,
// 未重绘块的新数值会被当成“已画过”而永远刷不出来。
void saveSectionPrev(int idx) {
  prevArrow[idx] = arrowOn(idx);
  switch (idx) {
    case 0: prev.mem  = cur.mem; prev.memGB = cur.memGB; break;
    case 1: prev.temp = cur.temp; break;
    case 2: prev.pwr  = cur.pwr;  break;
  }
}

// ── 绘制 ─────────────────────────────────────────────────────────────────────

// 伪粗体:在微小偏移处叠印字符串来加粗笔画(载入的 TTF 只有一种字重)。
// `wt` = 额外的笔画像素数。沿用调用方已设置的字号/颜色/对齐基准。
void drawBold(const char* s, int x, int y, int wt) {
  for (int dx = 0; dx <= wt; dx++)
    for (int dy = 0; dy <= wt; dy++)
      sCanvas.drawString(s, x + dx, y + dy);
}

// 向上箭头图标(箭头 + 杆),总高 2*r,以 (cx, cy) 为中心。
void drawUpArrow(int cx, int cy, int r) {
  int headB = cy - r / 4;          // 箭头底边
  int hw    = (r * 4) / 5;         // 箭头半宽
  int sw    = r / 4;               // 杆半宽
  sCanvas.fillTriangle(cx, cy - r, cx - hw, headB, cx + hw, headB, C_BLACK);
  sCanvas.fillRect(cx - sw, headB, 2 * sw + 1, (cy + r) - headB, C_BLACK);
}

void pushSection(int idx, m5epd_update_mode_t mode) {
  int screenY = idx * SECTION_H;

  // ── 背景 ─────────────────────────────────────────────────────────────────────
  sCanvas.fillRect(0, 0, SCR_W, SECTION_H, C_WHITE);

  // ── 数值(超大字号,先画,使进度条和标签盖在其上)──────────────────────────
  char buf[14];
  valStr(idx, buf, sizeof(buf));
  sCanvas.setTextSize(FONT_VALUE);
  sCanvas.setTextColor(C_BLACK);
  sCanvas.setTextDatum(7);   // BC_DATUM:底部居中
  drawBold(buf, VAL_CX, VAL_CY, BOLD_VALUE);

  // ── 进度条:纯 fillRect —— 不用 drawRect —— 实心 BAR_BORDER 边框 ──────────────
  sCanvas.fillRect(BAR_X, BAR_TOP, BAR_W, BAR_H, C_BLACK);
  sCanvas.fillRect(BAR_X + BAR_BORDER, BAR_TOP + BAR_BORDER,
                   BAR_W - 2 * BAR_BORDER, BAR_H - 2 * BAR_BORDER, C_WHITE);
  int fw = (int)((long)(BAR_W - 2 * BAR_BORDER) * barPct(idx) / 100);
  if (fw > 0)
    sCanvas.fillRect(BAR_X + BAR_BORDER, BAR_TOP + BAR_BORDER,
                     fw, BAR_H - 2 * BAR_BORDER, C_BLACK);

  // ── 标签(最后画,保证始终压在数值文字之上、清晰可读)────────────────────────
  sCanvas.setTextSize(FONT_LABEL);
  sCanvas.setTextColor(C_BLACK);
  sCanvas.setTextDatum(3);   // ML_DATUM:左对齐 + 垂直居中
  drawBold(LABELS[idx], LABEL_X, LABEL_CY, BOLD_LABEL);

  // ── 向上箭头 —— 与风扇同步绘制(见 arrowOn)──────────────────────────────────
  if (arrowOn(idx))
    drawUpArrow(ARROW_CX, ARROW_CY, ARROW_R);

  // ── 把区块画布推送到正确的屏幕位置(刷新模式由调用方决定)────────────────────
  sCanvas.pushCanvas(0, screenY, mode);
}

void pushAll() {
  // 首次全屏绘制:全部用 REFRESH_MODE_FULL 反色全刷,得到最干净的底图
  for (int i = 0; i < NUM_METRICS; i++) pushSection(i, REFRESH_MODE_FULL);
}

// ── IT8951 电源状态 ──────────────────────────────────────────────────────────
// 库的 begin() 之后 IT8951 一直停在 SYS_RUN,不刷屏也在空转发热,而它正好位于反白区
// 正下方。改为刷之前唤醒、刷完待机,把 24 小时常燃变成按需通电。

void epdWake() {
#if EPD_IDLE_MODE
  M5.EPD.Active();
  delay(EPD_WAKE_DELAY_MS);   // 给 IT8951 一点稳定时间再灌数据
#endif
}

void epdRest() {
#if EPD_IDLE_MODE
  // 必须先等面板把最后一次刷新走完再待机。pushCanvas() 是**异步**的:
  // UpdateArea() 只在开头 CheckAFSR() 等「上一次」刷完,发出 DPY_BUF_AREA 命令后
  // 立刻返回,此时面板还要再驱动约 450 ms。中间几块靠下一次 pushSection 开头的
  // CheckAFSR() 替它们等完,而最后一块没有"下一次"——不等就 StandBy() 会把它的
  // 刷新拦腰切断,表现为该块永远刷不出来。
  uint32_t t0 = millis();
  M5.EPD.CheckAFSR();         // 轮询 LUTAFSR 直到面板空闲(3 秒超时)
  lastAfsrWaitMs = millis() - t0;
  #if EPD_IDLE_MODE == 2
    M5.EPD.Sleep();           // 更深:多关一些时钟域
  #else
    M5.EPD.StandBy();         // 保守:保留 DRAM 自刷新,波形表和帧缓存不丢
  #endif
#endif
}
