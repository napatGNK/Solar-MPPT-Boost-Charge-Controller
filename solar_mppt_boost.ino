/*
 * Solar MPPT BOOST Charge Controller — ESP32   (Rev.3.3)
 * ------------------------------------------------------
 * แผง 20W (Vmp ~12V, Imp 1.11A)  ->  แบต SLA 12V 7Ah
 * Boost converter · MOSFET ฝั่งต่ำ ขับด้วยขา 5 (LO) ของ IR2104
 *
 * ============ แก้จาก Rev.3.2 ============
 * 1. ไม่ยอมเปิดไดรเวอร์เลยถ้า INA219 ตัวใดตัวหนึ่งหาไม่เจอ
 *    ระบบป้องกันทั้งหมดอาศัยค่าจากเซนเซอร์ เซนเซอร์ตาย = ไม่มีตัวป้องกัน
 * 2. ข้าม OLED ทั้งหมดถ้าตอนบูตหาไม่เจอ
 *    เดิม: ยังยิง I2C ไปหาจอที่ไม่มีทุก 500ms กินเวลาบัสและอาจค้าง
 * 3. เพิ่มการตรวจว่า boost ทำงานจริง — duty สูงแต่แรงดันไม่ขึ้น = เข้า FAULT
 * 4. เข้าโหมด MPPT แบบไต่ขึ้นจากศูนย์ ไม่กระโดดไป 0.05 ทันที
 * 5. FAULT จากเซนเซอร์หายจะไม่กลับมาเอง ต้องรีเซ็ตบอร์ด
 *
 * ============ อ่านก่อนแก้โค้ด ============
 * IR2104 มีอินพุตเดียว (IN) และดาต้าชีตระบุว่า HO อยู่ในเฟสเดียวกับ IN
 * แปลว่า LO กลับเฟสกับ IN เสมอ
 *   IN = LOW  ->  LO = HIGH  ->  Q1 นำกระแส
 * ดังนั้นค่าที่เขียนออก GPIO25 = (1 - duty ของ Q1)
 * ให้ใช้ setDutyQ1() เท่านั้น ห้ามเรียก ledcWrite() ตรง ๆ
 *
 * ขาของ IR2104 (ยืนยันจากดาต้าชีต PD60046-P)
 *   1 VCC   2 IN   3 SD   4 COM   5 LO   6 VS   7 HO   8 VB
 *   ขา 5 และ ขา 7 เป็นเอาต์พุต · ขา 7 ปล่อยลอย · ขา 8 ต่อไปขา 1
 *
 * ============ ขา ESP32 ============
 * ตัวอักษร "D" บนบอร์ดเป็นแค่คำนำหน้า เลขหลัง D คือเลข GPIO ตรง ๆ
 *   D25 -> IR2104 ขา 2 (IN)   ·  D26 -> IR2104 ขา 3 (SD)
 *   D21 / D22 -> I2C SDA / SCL ·  D27 -> รีเลย์ (ทางเลือก)
 *
 * Library ที่ต้องลง: Adafruit INA219 · Adafruit SSD1306 · Adafruit GFX
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SSD1306.h>

// เอาคอมเมนต์ออกถ้าติดตั้งรีเลย์ตัดสายแผงตามส่วนที่ 7.2 ของเอกสารหลัก
// #define USE_RELAY

// ---------- ขา ----------
#define PIN_PWM     25
#define PIN_SD      26
#define PIN_RELAY   27

// ---------- PWM ----------
const int PWM_FREQ = 50000;
const int PWM_RES  = 10;
const int PWM_MAX  = (1 << PWM_RES) - 1;

// ---------- ขอบเขต duty ของ Q1 ----------
// boost: Vout/Vin = 1/(1-D)   D=0.167 -> 1.20 เท่า   D=0.40 -> 1.67 เท่า
const float DUTY_START = 0.05;
const float DUTY_MAX   = 0.40;

// ---------- แบต SLA 12V ----------
// ไม่มีเซนเซอร์อุณหภูมิ จึงชดเชยตามอุณหภูมิไม่ได้
// ค่ามาตรฐาน 25°C คือ absorb 14.40 / float 13.70 · สัมประสิทธิ์ -0.024 V/°C
// ตั้งชดเชยไว้ที่ ~33°C ตามอุณหภูมิห้องบ้านเรา (ปลอดภัยไว้ก่อน)
// ถ้าทดสอบในห้องแอร์ต่ำกว่า 25°C ปรับขึ้นเป็น 14.40 / 13.70 ได้
const float V_ABSORB    = 14.20;
const float V_FLOAT     = 13.50;
const float V_HARD_CUT  = 14.90;
const float V_RECOVER   = 13.20;
const float V_BAT_MIN   =  8.00;
const float I_CHG_MAX   =  1.50;

// ---------- แผง ----------
const float V_PV_MIN    = 10.00;   // ต่ำกว่านี้ IR2104 เข้า UVLO อยู่แล้ว

// ---------- ตรวจว่า boost ทำงานจริง ----------
const float    DUTY_CHECK  = 0.15;    // เกินนี้แรงดันต้องขึ้นแล้ว
const uint32_t T_NOBOOST   = 10000;   // ทนได้ 10 วินาทีก่อนเข้า FAULT

// ---------- คาบเวลา (ms) ----------
const uint32_t T_CTRL = 100;
const uint32_t T_LOG  = 1000;
const uint32_t T_OLED = 500;

Adafruit_INA219 ina1(0x40);
Adafruit_INA219 ina2(0x41);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

bool ina1Ok = false, ina2Ok = false, oledOk = false;
bool sensorFault = false;          // FAULT ชนิดที่กลับมาเองไม่ได้

enum Mode { M_IDLE, M_MPPT, M_ABSORB, M_FLOAT, M_FAULT };
Mode mode = M_IDLE;
const char* modeName[] = {"IDLE", "MPPT", "ABSORB", "FLOAT", "FAULT"};

float dutyQ1 = 0;
float vPv = 0, iPv = 0, pPv = 0;
float vBat = 0, iBat = 0;
float pPrev = 0;
int   poDir = +1;
const float STEP = 0.004;

uint32_t tCtrl = 0, tLog = 0, tOled = 0, tBadStart = 0;

// ============================================================
//  PWM — ที่เดียวที่แตะ ledcWrite (กลับเฟสให้แล้ว)
// ============================================================
void setDutyQ1(float d) {
  if (d < 0)        d = 0;
  if (d > DUTY_MAX) d = DUTY_MAX;
  dutyQ1 = d;
  uint32_t val = (uint32_t)((1.0f - dutyQ1) * PWM_MAX + 0.5f);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWM, val);
#else
  ledcWrite(0, val);
#endif
}

void driverEnable(bool on) { digitalWrite(PIN_SD, on ? HIGH : LOW); }

void relaySet(bool on) {
#ifdef USE_RELAY
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

void enterFault(const char* why, bool permanent) {
  setDutyQ1(0);
  driverEnable(false);
  relaySet(false);
  if (permanent) sensorFault = true;
  if (mode != M_FAULT) {
    Serial.print("[FAULT] ");
    Serial.println(why);
    if (permanent) Serial.println("        ต้องแก้แล้วรีเซ็ตบอร์ด ระบบจะไม่กลับมาเอง");
  }
  mode = M_FAULT;
}

void goIdle(const char* why) {
  setDutyQ1(0);
  driverEnable(false);
  tBadStart = 0;
  if (mode != M_IDLE) {
    Serial.print("[IDLE] ");
    Serial.println(why);
  }
  mode = M_IDLE;
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // ปิดทุกอย่างก่อนเป็นอันดับแรกเสมอ
  pinMode(PIN_SD, OUTPUT);
  driverEnable(false);
#ifdef USE_RELAY
  pinMode(PIN_RELAY, OUTPUT);
  relaySet(false);
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RES);
#else
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM, 0);
#endif
  setDutyQ1(0);            // Q1 ปิด = เขียนค่าสูงสุดออกไปที่ IN

  Wire.begin(21, 22);
  Wire.setClock(100000);

  Serial.println("=== Solar MPPT Boost Rev.3.3 ===");

  ina1Ok = ina1.begin();
  ina2Ok = ina2.begin();
  if (!ina1Ok) Serial.println("[ERR] ไม่พบ INA219 ตัวที่ 1 (0x40)");
  if (!ina2Ok) Serial.println("[ERR] ไม่พบ INA219 ตัวที่ 2 (0x41)");
  if (ina1Ok) ina1.setCalibration_32V_2A();
  if (ina2Ok) ina2.setCalibration_32V_2A();

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println("Solar MPPT Boost");
    oled.println("Rev.3.3");
    oled.display();
  } else {
    Serial.println("[ERR] ไม่พบ OLED — ข้ามการแสดงผล ดูค่าทาง Serial แทน");
  }

  if (!ina1Ok || !ina2Ok) {
    enterFault("เซนเซอร์กระแสไม่ครบ — ไม่มีระบบป้องกัน จึงไม่เปิดไดรเวอร์", true);
  }

  Serial.print("[INFO] เป้าหมาย absorb ");
  Serial.print(V_ABSORB, 2);
  Serial.print("V · float ");
  Serial.print(V_FLOAT, 2);
  Serial.println("V (คงที่ ไม่ชดเชยอุณหภูมิ)");

  Serial.println("ms,mode,Vpv,Ipv,Ppv,Vbat,Ibat,dutyQ1");
  delay(1000);
}

// ============================================================
void readSensors() {
  vPv = ina1.getBusVoltage_V();
  iPv = ina1.getCurrent_mA() / 1000.0f;
  if (iPv < 0) iPv = 0;
  pPv = vPv * iPv;

  vBat = ina2.getBusVoltage_V();
  iBat = ina2.getCurrent_mA() / 1000.0f;
}

// ============================================================
//  P&O — เดินก้าวเดียวทุกรอบ ถ้ากำลังลดลงก็กลับทิศ
// ============================================================
void mpptStep() {
  if (pPv < pPrev - 0.03f) poDir = -poDir;
  pPrev = pPv;
  setDutyQ1(dutyQ1 + poDir * STEP);
}

// ============================================================
//  ควบคุมแรงดันคงที่ — ใช้ตอน absorb และ float
// ============================================================
void cvStep(float target) {
  float adj = (target - vBat) * 0.010f;
  if (adj >  0.010f) adj =  0.010f;
  if (adj < -0.020f) adj = -0.020f;      // ลดเร็วกว่าเพิ่ม เพื่อความปลอดภัย
  setDutyQ1(dutyQ1 + adj);
}

// ============================================================
//  boost ต้องดันแรงดันขึ้นจริง ถ้า duty สูงแล้วยังไม่ขึ้น
//  แปลว่า Q1 ไม่สวิตช์ หรือ D1 กลับด้าน — ไม่มีประโยชน์ที่จะเดินต่อ
// ============================================================
void checkBoostWorking() {
  if (dutyQ1 < DUTY_CHECK || vPv < V_PV_MIN) { tBadStart = 0; return; }

  if (vBat < vPv * 1.02f) {
    if (tBadStart == 0) tBadStart = millis();
    else if (millis() - tBadStart > T_NOBOOST) {
      enterFault("duty สูงแต่แรงดันไม่ขึ้น — Q1 ไม่สวิตช์ ตรวจ SD และ Gate", true);
    }
  } else {
    tBadStart = 0;
  }
}

// ============================================================
void control() {
  if (sensorFault) return;               // ล็อกถาวรจนกว่าจะรีเซ็ต

  // ---- ฉุกเฉิน ----
  if (vBat > V_HARD_CUT) { enterFault("แรงดันแบตเกินขีด", false); return; }
  if (vBat < V_BAT_MIN)  { enterFault("ไม่พบแบตเตอรี่",   false); return; }

  // ---- ออกจาก FAULT ----
  if (mode == M_FAULT) {
    if (vBat < V_RECOVER && vBat > V_BAT_MIN) {
      Serial.println("[INFO] สภาวะกลับสู่ปกติ");
      mode = M_IDLE;
      relaySet(true);
    }
    return;
  }

  // ---- แสงไม่พอ ----
  if (vPv < V_PV_MIN) { goIdle("แรงดันแผงต่ำเกินไป"); return; }

  // ---- เริ่มทำงาน ----
  if (mode == M_IDLE) {
    relaySet(true);
    setDutyQ1(0);                        // ไต่ขึ้นจากศูนย์ ไม่กระโดด
    driverEnable(true);
    pPrev = 0;
    poDir = +1;
    tBadStart = 0;
    mode = M_MPPT;
    Serial.println("[INFO] เริ่ม MPPT");
    return;
  }

  // ---- ไต่ขึ้นถึงจุดเริ่มต้นก่อน แล้วค่อยให้ P&O ทำงาน ----
  if (mode == M_MPPT && dutyQ1 < DUTY_START) {
    setDutyQ1(dutyQ1 + STEP);
    return;
  }

  // ---- จำกัดกระแสชาร์จ ----
  if (iBat > I_CHG_MAX) { setDutyQ1(dutyQ1 - 0.02f); return; }

  // ---- สลับสถานะ ----
  if (mode == M_MPPT   && vBat >= V_ABSORB)  { mode = M_ABSORB; Serial.println("[INFO] เข้าโหมด ABSORB"); }
  if (mode == M_ABSORB && iBat <  0.10f)     { mode = M_FLOAT;  Serial.println("[INFO] เข้าโหมด FLOAT"); }
  if (mode != M_MPPT   && vBat <  V_RECOVER) { mode = M_MPPT;   pPrev = 0; Serial.println("[INFO] กลับสู่ MPPT"); }

  switch (mode) {
    case M_MPPT:   mpptStep();        break;
    case M_ABSORB: cvStep(V_ABSORB);  break;
    case M_FLOAT:  cvStep(V_FLOAT);   break;
    default: break;
  }

  checkBoostWorking();
}

// ============================================================
void drawOled() {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.setTextSize(1);

  oled.print("MODE "); oled.println(modeName[mode]);
  oled.print("PV  "); oled.print(vPv, 2); oled.print("V ");
                      oled.print(iPv, 2); oled.println("A");
  oled.print("    "); oled.print(pPv, 2); oled.println("W");
  oled.print("BAT "); oled.print(vBat, 2); oled.print("V ");
                      oled.print(iBat, 2); oled.println("A");
  oled.print("D   "); oled.print(dutyQ1 * 100.0f, 1); oled.println("%");

  oled.display();
}

void logCsv() {
  Serial.print(millis());       Serial.print(',');
  Serial.print(modeName[mode]); Serial.print(',');
  Serial.print(vPv, 3);         Serial.print(',');
  Serial.print(iPv, 3);         Serial.print(',');
  Serial.print(pPv, 3);         Serial.print(',');
  Serial.print(vBat, 3);        Serial.print(',');
  Serial.print(iBat, 3);        Serial.print(',');
  Serial.println(dutyQ1, 4);
}

// ============================================================
void loop() {
  uint32_t now = millis();

  if (now - tCtrl >= T_CTRL) {
    tCtrl = now;
    if (!sensorFault) readSensors();
    control();
  }
  if (now - tOled >= T_OLED) { tOled = now; drawOled(); }
  if (now - tLog  >= T_LOG)  { tLog  = now; logCsv();  }
}
