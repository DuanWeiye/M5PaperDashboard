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
// 反白的根因不是墨水自然衰减,而是局部刷新的串扰:只推送某一块时,另外两块的像素
// 完全不被驱动,相邻块反复刷新会通过共享源极/VCOM 线对静止像素产生寄生扰动,日积
// 月累把静止黑像素推向白。解法:每次有区块变化触发刷新时,三块一起重绘——变化的块
// 用 REFRESH_MODE_FULL(反色全刷、零残影,但会闪);未变化的块用 REFRESH_MODE_KEEP
// 补刷一次,把静止像素重新驱动回目标态以抵消反白。补刷内容与屏上完全相同,所以不会
// 产生新残影。
//
// 补刷模式按“越不闪”排序可选(库注释里的特性):
//   UPDATE_MODE_DU  —— 纯黑白、Low 残影、~260ms,几乎不闪,对黑色重驱有力(默认,
//                       最适合本画面的黑底白字);
//   UPDATE_MODE_A2  —— 2 级、最快、最不闪,但残影最重(靠下面的深度全刷兜底清理);
//   UPDATE_MODE_DU4 —— 4 级、~120ms,对比度偏低,Medium 残影;
//   UPDATE_MODE_GL16—— 16 灰、保留抗锯齿,但带“白过渡”会比较明显地闪(之前那版)。
#define REFRESH_MODE_FULL  UPDATE_MODE_GC16   // 变化块 / 深度全刷:反色全刷,零残影(会闪)
#define REFRESH_MODE_KEEP  UPDATE_MODE_DU     // 静止块补刷:几乎不闪,重新驱动抵消反白

// 深度全刷:每隔这么久,把三块全部用 REFRESH_MODE_FULL 反色刷一次,清理补刷模式长期
// 累积的底灰/残影/直流偏置(清洁保险,与抗反白无关)。设为 0 可完全关闭。
#define DEEP_CLEAN_MS  1800000UL   // 30 分钟

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
// 之间保持当前状态,以免继电器抖动。断连(无温度读数)时强制 OFF。
#define FAN_PIN         26
#define FAN_ON_C        60     // °C —— 达到/超过此温度则开风扇
#define FAN_OFF_C       50     // °C —— 仅当低于此温度才允许关
#define FAN_ON_W        30     // W  —— 达到/超过此功率则开风扇
#define FAN_OFF_W       20     // W  —— 仅当低于此功率才允许关
#define FAN_ACTIVE_LOW  0      // 若继电器为低电平触发(LOW 导通),设为 1

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
bool     forceRefresh  = false;   // 风扇开/关时置位,不等更新间隔立即重绘

uint32_t lastUpdateMs    = 0;
uint32_t lastDeepCleanMs = 0;   // 上次“深度全刷”时间
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
void pushSection(int idx, m5epd_update_mode_t mode);
void pushAll();

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
  prev = cur;
  for (int i = 0; i < NUM_METRICS; i++) prevArrow[i] = arrowOn(i);
  lastUpdateMs = millis();
  lastDeepCleanMs = millis();
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

    // 是否到了“深度全刷”时间(三块全用 GC16 反色刷,清底灰/直流偏置)
    bool deepClean = (DEEP_CLEAN_MS > 0) && (now - lastDeepCleanMs >= DEEP_CLEAN_MS);

    // 先判断每块是否变化
    bool changed[NUM_METRICS];
    bool any = false;
    for (int i = 0; i < NUM_METRICS; i++) {
      changed[i] = sectionChanged(i);
      if (changed[i]) any = true;
    }

    // 只要有任意一块变化(或到了深度全刷),就重绘全部三块:变化的块用 GC16
    // 反色全刷;未变化的块用补刷模式重新驱动一次以抵消反白。无任何变化时则什么
    // 都不刷——没有相邻刷新活动,也就不会反白。
    if (any || deepClean) {
      for (int i = 0; i < NUM_METRICS; i++) {
        m5epd_update_mode_t mode =
            (changed[i] || deepClean) ? REFRESH_MODE_FULL : REFRESH_MODE_KEEP;
        pushSection(i, mode);
      }
      prev = cur;
      prevConnected = connected;
      for (int i = 0; i < NUM_METRICS; i++) prevArrow[i] = arrowOn(i);
      if (deepClean) lastDeepCleanMs = now;
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
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"Temp\": %.1f, \"Hum\": %.1f}", t, h);
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
  digitalWrite(FAN_PIN, level);
  if (changed) forceRefresh = true;   // 立即把箭头变化反映到屏幕上
}

void updateFan() {
  if (!connected) { if (fanOn) setFan(false); return; }   // 无数据 → 关风扇
  if (!fanOn) {
    // 温度 或 功率 任一越过高阈值则开
    if (cur.temp >= FAN_ON_C || cur.pwr >= FAN_ON_W) setFan(true);
  } else {
    // 仅当温度 且 功率都回落到各自低阈值以下才关
    if (cur.temp <= FAN_OFF_C && cur.pwr <= FAN_OFF_W) setFan(false);
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
