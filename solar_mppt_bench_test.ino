/*
 * BENCH TEST — Solar MPPT Boost   (Rev.3.3)
 * -----------------------------------------
 * สเก็ตช์นี้ใช้เฉพาะตอนทดสอบด่าน T3 ถึง T6
 * สั่ง duty ด้วยมือ ไม่มี MPPT ไม่มีการชาร์จอัตโนมัติ
 *
 * *** ต้องมีโหลดต่อที่เอาต์พุตเสมอก่อนสั่งสวิตช์ ***
 * boost ที่ไม่มีโหลด แรงดันจะไต่ขึ้นจนพังตัวเอง
 *
 * ============ แก้จาก Rev.3.2 ============
 * 1. คำสั่ง e บังคับ duty = 0 ทุกครั้ง
 *    เดิม: ถ้าเคยตั้ง d 0.25 ไว้แล้วพิมพ์ e วงจรจะกระโดดไป 25% ทันที
 * 2. เพิ่ม soft-start — duty ไต่ถึงเป้าใน ~1 วินาที ไม่กระโดด
 * 3. ข้าม OLED ทั้งหมดถ้าตอนบูตหาไม่เจอ
 *    เดิม: ยังยิง I2C ไปหาจอที่ไม่มีทุก 500ms กินเวลาบัสและอาจค้าง
 * 4. ไม่ยอมให้สั่ง e ถ้า INA219 ตัวใดตัวหนึ่งหาไม่เจอ
 *    ระบบตัดอัตโนมัติอาศัยค่าจากเซนเซอร์ ถ้าเซนเซอร์ตายก็ไม่มีตัวป้องกัน
 * 5. เตือนอัตโนมัติเมื่อสั่ง duty แล้วแรงดันไม่ขึ้น (Q1 ไม่สวิตช์)
 * 6. ตัดเองถ้าเปิดค้างเกิน 10 นาที กันลืมปล่อยทิ้งไว้
 *
 * ============ ขา ESP32 ============
 * ตัวอักษร "D" บนบอร์ดเป็นแค่คำนำหน้า เลขหลัง D คือเลข GPIO ตรง ๆ
 *
 * คำสั่งใน Serial Monitor (115200, ตั้ง Line ending = Newline)
 *   e            เปิดไดรเวอร์ (duty เริ่มที่ 0 เสมอ)
 *   s            หยุดทันที duty = 0 และปิดไดรเวอร์
 *   d 0.15       ตั้ง duty ของ Q1 เป็น 15%
 *   +            เพิ่ม duty ทีละ 1%
 *   -            ลด duty ทีละ 1%
 *   g            บอกว่าแรงดันเฉลี่ยที่ Gate ควรวัดได้เท่าไหร่ (ใช้ตอน T4)
 *   ?            แสดงคำสั่ง
 *
 * ระบบตัดอัตโนมัติ: Vout > 17V · Iin > 1.8A · Vin < 8V · เปิดค้างเกิน 10 นาที
 *
 * Library ที่ต้องลง: Adafruit INA219 · Adafruit SSD1306 · Adafruit GFX
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SSD1306.h>

#define PIN_PWM   25      // D25 -> IR2104 ขา 2 (IN)  กลับเฟส
#define PIN_SD    26      // D26 -> IR2104 ขา 3 (SD)

const int PWM_FREQ = 50000;
const int PWM_RES  = 10;
const int PWM_MAX  = (1 << PWM_RES) - 1;

const float DUTY_CEIL  = 0.30;     // เพดานตอนทดสอบ ต่ำกว่าตอนใช้จริง
const float V_OUT_TRIP = 17.0;
const float I_IN_TRIP  = 1.80;
const float V_IN_MIN   = 8.0;

// soft-start: ขยับ 0.005 ทุก 20ms -> 0.30 ใช้เวลาราว 1.2 วินาที
const float    RAMP_STEP  = 0.005;
const uint32_t T_RAMP     = 20;
const uint32_t T_MAX_ON   = 10UL * 60UL * 1000UL;   // ตัดเองหลัง 10 นาที
const uint32_t T_NOBOOST  = 3000;                   // เตือนหลังผิดปกติ 3 วินาที

Adafruit_INA219 ina1(0x40);
Adafruit_INA219 ina2(0x41);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

bool ina1Ok = false, ina2Ok = false, oledOk = false;

float dutyTarget = 0;      // ค่าที่สั่ง
float dutyNow    = 0;      // ค่าที่ออกจริง ไต่เข้าหาเป้า
bool  enabled    = false;
float vIn = 0, iIn = 0, vOut = 0, iOut = 0;

uint32_t tRead = 0, tPrint = 0, tRamp = 0, tOn = 0, tBadStart = 0;
bool warnedNoBoost = false;
String cmd = "";

// ------------------------------------------------------------
void writeDuty(float d) {
  if (d < 0)         d = 0;
  if (d > DUTY_CEIL) d = DUTY_CEIL;
  dutyNow = d;
  uint32_t val = (uint32_t)((1.0f - dutyNow) * PWM_MAX + 0.5f);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWM, val);
#else
  ledcWrite(0, val);
#endif
}

void stopAll(const char* why) {
  dutyTarget = 0;
  writeDuty(0);                       // หยุดต้องทันที ไม่ไต่ลง
  digitalWrite(PIN_SD, LOW);
  enabled = false;
  tBadStart = 0;
  warnedNoBoost = false;
  Serial.print("\n*** หยุด: ");
  Serial.print(why);
  Serial.println(" ***\n");
}

void gateHint() {
  Serial.print("duty สั่ง "); Serial.print(dutyTarget * 100, 1);
  Serial.print("%  ออกจริง ");  Serial.print(dutyNow * 100, 1);
  Serial.println("%");
  Serial.print("  Gate ควรวัดได้ราว ");
  Serial.print(dutyTarget * vIn, 2); Serial.println(" V");
  Serial.print("  ถ้าวัดได้ราว ");
  Serial.print((1.0f - dutyTarget) * vIn, 2);
  Serial.println(" V = เฟสกลับผิด พิมพ์ s หยุดทันที");
  Serial.println("  ถ้าวัดได้ 0V = ไดรเวอร์ปิดอยู่ ตรวจ socket ขา 3 ต้องได้ 3.3V");
}

void help() {
  Serial.println();
  Serial.println("  e        เปิดไดรเวอร์ (duty เริ่มที่ 0)");
  Serial.println("  s        หยุดทันที");
  Serial.println("  d 0.15   ตั้ง duty ของ Q1");
  Serial.println("  + / -    ปรับทีละ 1%");
  Serial.println("  g        ค่า Gate ที่ควรวัดได้ตอนนี้");
  Serial.println("  ?        แสดงคำสั่งนี้");
  Serial.println();
  Serial.println("  เตือน: ต้องมีโหลดต่อที่เอาต์พุตเสมอ");
  Serial.println();
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  // ปิดไดรเวอร์ก่อนเป็นอันดับแรกเสมอ
  pinMode(PIN_SD, OUTPUT);
  digitalWrite(PIN_SD, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RES);
#else
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM, 0);
#endif
  writeDuty(0);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  Serial.println("=== BENCH TEST — Solar MPPT Boost Rev.3.3 ===");

  Serial.print("I2C ที่เจอ: ");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.print("0x"); Serial.print(a, HEX); Serial.print(" "); }
  }
  Serial.println();
  Serial.println("ควรเจอ 0x3C 0x40 0x41");

  ina1Ok = ina1.begin();
  ina2Ok = ina2.begin();
  if (!ina1Ok) Serial.println("[ERR] ไม่พบ INA219 0x40 — สั่ง e ไม่ได้");
  if (!ina2Ok) Serial.println("[ERR] ไม่พบ INA219 0x41 — สั่ง e ไม่ได้");
  if (ina1Ok) ina1.setCalibration_32V_2A();
  if (ina2Ok) ina2.setCalibration_32V_2A();

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) {
    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
    oled.setCursor(0, 0); oled.println("BENCH TEST"); oled.display();
  } else {
    Serial.println("[ERR] ไม่พบ OLED — ข้ามการแสดงผล ดูค่าทาง Serial แทน");
  }

  help();
}

// ------------------------------------------------------------
void readAll() {
  if (ina1Ok) {
    vIn = ina1.getBusVoltage_V();
    iIn = ina1.getCurrent_mA() / 1000.0f;
  }
  if (ina2Ok) {
    vOut = ina2.getBusVoltage_V();
    iOut = ina2.getCurrent_mA() / 1000.0f;
  }
}

void guard() {
  if (!enabled) return;

  if (vOut > V_OUT_TRIP) { stopAll("แรงดันเอาต์พุตเกิน 17V — ต่อโหลดหรือยัง"); return; }
  if (iIn  > I_IN_TRIP)  { stopAll("กระแสอินพุตเกิน 1.8A"); return; }
  if (vIn  < V_IN_MIN)   { stopAll("แรงดันอินพุตตกต่ำกว่า 8V"); return; }

  if (millis() - tOn > T_MAX_ON) { stopAll("เปิดค้างเกิน 10 นาที — ตัดเองกันลืม"); return; }

  // ---- ตรวจว่า boost ทำงานจริงไหม ----
  // duty เกิน 8% แล้ว Vout ต้องสูงกว่า Vin ถ้าไม่ใช่แปลว่า Q1 ไม่สวิตช์
  if (dutyNow > 0.08f && vIn > V_IN_MIN) {
    if (vOut < vIn * 1.02f) {
      if (tBadStart == 0) tBadStart = millis();
      else if (!warnedNoBoost && millis() - tBadStart > T_NOBOOST) {
        warnedNoBoost = true;
        Serial.println("\n[เตือน] สั่ง duty แล้วแรงดันไม่ขึ้น — Q1 น่าจะไม่ได้สวิตช์");
        Serial.println("  1. วัด socket ขา 3 (SD) ต้องได้ 3.3V");
        Serial.println("  2. วัดแรงดันเฉลี่ยที่ Gate ของ Q1");
        Serial.println("  3. ถ้า Gate ได้ 0V = ไดรเวอร์ไม่ออก · ถ้าสูงผิด = เฟสกลับ");
        Serial.println("  ไม่ตัดให้อัตโนมัติเพราะไม่อันตราย แต่อย่าไล่ duty ขึ้นต่อ\n");
      }
    } else {
      tBadStart = 0;
    }
  } else {
    tBadStart = 0;
  }
}

void rampStep() {
  uint32_t now = millis();
  if (now - tRamp < T_RAMP) return;
  tRamp = now;
  if (dutyNow == dutyTarget) return;

  if (dutyNow < dutyTarget) writeDuty(min(dutyNow + RAMP_STEP, dutyTarget));
  else                      writeDuty(max(dutyNow - RAMP_STEP, dutyTarget));
}

void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      cmd.trim();
      if (cmd.length()) {
        if (cmd == "s") stopAll("สั่งหยุดด้วยมือ");

        else if (cmd == "e") {
          if (!ina1Ok || !ina2Ok) {
            Serial.println("[ปฏิเสธ] เซนเซอร์ INA219 ไม่ครบ ระบบตัดอัตโนมัติใช้งานไม่ได้");
            Serial.println("         แก้ I2C ให้เจอ 0x40 และ 0x41 ก่อน");
          } else {
            dutyTarget = 0;
            writeDuty(0);                 // เริ่มที่ศูนย์เสมอ ไม่กระโดด
            digitalWrite(PIN_SD, HIGH);
            enabled = true;
            tOn = millis();
            tBadStart = 0;
            warnedNoBoost = false;
            Serial.println("ไดรเวอร์เปิดแล้ว · duty = 0 · สั่ง d ทีละขั้น");
          }
        }

        else if (cmd == "?") help();
        else if (cmd == "g") gateHint();

        else if (cmd == "+") { dutyTarget += 0.01; if (dutyTarget > DUTY_CEIL) dutyTarget = DUTY_CEIL; Serial.print("duty เป้า = "); Serial.println(dutyTarget, 3); }
        else if (cmd == "-") { dutyTarget -= 0.01; if (dutyTarget < 0) dutyTarget = 0; Serial.print("duty เป้า = "); Serial.println(dutyTarget, 3); }

        else if (cmd.startsWith("d ") || cmd.startsWith("d")) {
          String arg = cmd.substring(1);
          arg.trim();
          if (arg.length() == 0) {
            Serial.println("ใส่ค่าด้วย เช่น d 0.15");
          } else if (!enabled) {
            Serial.println("[ปฏิเสธ] ยังไม่ได้เปิดไดรเวอร์ พิมพ์ e ก่อน");
          } else {
            dutyTarget = arg.toFloat();
            if (dutyTarget < 0)         dutyTarget = 0;
            if (dutyTarget > DUTY_CEIL) { dutyTarget = DUTY_CEIL; Serial.println("(เกินเพดาน 30% ตัดให้เหลือ 30%)"); }
            tBadStart = 0;
            warnedNoBoost = false;
            gateHint();
          }
        }

        else Serial.println("ไม่รู้จักคำสั่งนี้ พิมพ์ ? เพื่อดูรายการ");
      }
      cmd = "";
    } else cmd += ch;
  }
}

void printStatus() {
  Serial.print(enabled ? "[ON ] " : "[OFF] ");
  Serial.print("duty "); Serial.print(dutyNow * 100, 1);
  if (dutyNow != dutyTarget) { Serial.print("->"); Serial.print(dutyTarget * 100, 1); }
  Serial.print("%  |  ");
  Serial.print("Vin ");  Serial.print(vIn, 2);  Serial.print("V ");
  Serial.print(iIn, 3);  Serial.print("A  |  ");
  Serial.print("Vout "); Serial.print(vOut, 2); Serial.print("V ");
  Serial.print(iOut, 3); Serial.print("A  |  ");
  Serial.print("อัตราส่วน ");
  Serial.println(vIn > 1 ? vOut / vIn : 0, 3);
}

void drawOled() {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(enabled ? "BENCH  [ON]" : "BENCH  [OFF]");
  oled.print("D   "); oled.print(dutyNow * 100, 1); oled.println("%");
  oled.print("IN  "); oled.print(vIn, 2); oled.print("V "); oled.print(iIn, 2); oled.println("A");
  oled.print("OUT "); oled.print(vOut, 2); oled.print("V "); oled.print(iOut, 2); oled.println("A");
  oled.print("R   "); oled.println(vIn > 1 ? vOut / vIn : 0, 3);
  oled.display();
}

// ------------------------------------------------------------
void loop() {
  handleSerial();
  rampStep();

  uint32_t now = millis();
  if (now - tRead  >= 100) { tRead  = now; readAll(); guard(); }
  if (now - tPrint >= 500) { tPrint = now; printStatus(); drawOled(); }
}
