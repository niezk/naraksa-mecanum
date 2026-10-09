/*
 * ESP32 Mecanum Robot
 * -------------------------------------------------------------
 *  - 2x DRV8833 dual H-bridge  -> 4 mecanum wheels
 *  - MPU6050 (I2C 0x68)        -> gyro yaw, heading hold, field-centric drive
 *  - SSD1306 0.96" OLED (0x3C) -> status, wheel bars, settings menu
 *  - DualShock 3 via Bluepad32 -> control
 *
 * Board package : "esp32_bluepad32" (Boards Manager) -> "ESP32 Dev Module"
 * Libraries     : Adafruit SSD1306, Adafruit GFX   (Library Manager)
 *
 * NOTE: DS3 needs classic Bluetooth -> original ESP32 only (NOT S2/S3/C3/C6).
 *
 * DRIVING
 *   Left stick         : move (forward/back + strafe)
 *   Right stick X      : rotate
 *   R1 (hold)          : boost        L1 (hold): slow / precision
 *   Triangle           : toggle FIELD-CENTRIC drive (needs MPU6050)
 *   Square             : toggle HEADING HOLD / gyro assist (needs MPU6050)
 *   Cross              : zero yaw (current heading becomes "forward")
 *   D-pad Down         : re-calibrate gyro (keep robot still!)
 *   D-pad Left / Right : CRAB WALK - pure sideways slide, heading hold is
 *                        forced ON while held (robot-relative)
 *   Circle             : KICK - fires the solenoid for a short pulse
 *                        (set by "Kick pulse ms" in settings), then locks
 *                        out further kicks until "Kick cooldown" elapses
 *
 * SETTINGS MENU  (motors are stopped while the menu is open)
 *   Start              : open / close menu
 *   D-pad Up / Down    : select setting
 *   D-pad Left / Right : decrease / increase (hold to repeat, R1 = x5 step)
 *   Cross              : reset selected setting to default
 *   Triangle           : reset ALL settings to default
 *   Select             : SAVE all settings to flash (survive power-off)
 *   A '*' on the OLED means there are unsaved changes.
 *   (If Start/Select do nothing on your pad, change MISC_*_MASK below.)
 *
 * SERIAL (115200, send with newline)
 *   help | list | set <key> <value> | save | defaults | test | cal | kick
 *   e.g.  set kp 0.04      set invfl on      set kickms 200
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <Preferences.h>
#include <Bluepad32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =============================================================
//              FIXED CONFIG (wiring - needs re-flash)
// =============================================================

// ---- DRV8833 #1  (front)  A = Front-Left , B = Front-Right ----
constexpr uint8_t D1_A1 = 25;  // 1_A1
constexpr uint8_t D1_A2 = 19;  // 1_A2
constexpr uint8_t D1_B1 = 33;  // 1_B1
constexpr uint8_t D1_B2 = 32;  // 1_B2

// ---- DRV8833 #2  (rear)   A = Rear-Left  , B = Rear-Right -----
constexpr uint8_t D2_A1 = 27;  // 2_A1
constexpr uint8_t D2_A2 = 26;  // 2_A2
constexpr uint8_t D2_B1 = 14;  // 2_B1
constexpr uint8_t D2_B2 = 13;  // 2_B2

// ---- I2C (MPU6050 + OLED share the bus) ----
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t OLED_ADDR = 0x3C;

// ---- Kicker solenoid (BJT Darlington driver, e.g. TIP122/TIP120) ----
constexpr uint8_t KICK_PIN = 23;

// ---- PWM ----
constexpr uint32_t PWM_FREQ = 20000;  // 20 kHz, silent
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1u << PWM_BITS) - 1;

// ---- Timing ----
constexpr uint32_t CONTROL_PERIOD_MS = 10;  // 100 Hz
constexpr uint32_t OLED_PERIOD_MS = 200;    // 5 Hz  (100 ms in menu)
constexpr uint32_t PAD_TIMEOUT_MS = 1000;   // stop if no data (0 = disable)
constexpr uint32_t ROT_SETTLE_MS = 300;

// ---- Controller bit masks ----
constexpr uint8_t DPAD_UP_MASK = 0x01;
constexpr uint8_t DPAD_DOWN_MASK = 0x02;
constexpr uint8_t DPAD_RIGHT_MASK = 0x04;
constexpr uint8_t DPAD_LEFT_MASK = 0x08;
constexpr uint8_t MISC_SELECT_MASK = 0x02;  // Select / Back
constexpr uint8_t MISC_START_MASK = 0x04;   // Start / Home

// Button press events (one per control cycle).
// MUST stay above the first function: the Arduino IDE auto-generates function
// prototypes at that point and they need to know this type.
struct Edges {
  bool tri, sqr, cross, up, down, start, sel, kick;
};

// =============================================================
//        TUNABLE SETTINGS (editable at runtime + saved)
// =============================================================
struct Settings {
  float speedNormal = 0.60f;                                        // normal top speed (0..1)
  float speedBoost = 1.00f;                                         // R1
  float speedSlow = 0.30f;                                          // L1
  float rotateGain = 0.80f;                                         // rotation strength
  float deadzone = 0.08f;                                           // stick deadzone
  float minDuty = 0.12f;                                            // PWM floor to overcome motor dead-zone
  float accelLimit = 8.0f;                                          // max change per second (1.0 = full range)
  float headingKp = 0.025f;                                         // heading hold strength (per degree)
  float headingMax = 0.50f;                                         // heading hold max correction
  bool gyroInvert = false;                                          // flip if yaw counts the wrong way
  bool invFL = false, invFR = false, invRL = false, invRR = false;  // wheel direction
  bool brakeIdle = true;                                            // true = brake, false = coast when idle
  float kickMs = 150.0f;                                            // solenoid pulse length (ms)
  float kickCoolMs = 600.0f;                                        // minimum time between kicks (ms)
};

Settings S = Settings();
const Settings DEF = Settings();

struct Item {
  const char *key;    // serial name
  const char *label;  // OLED name (max 12 chars)
  bool isBool;
  float *f;
  bool *b;
  const float *df;
  const bool *db;
  float minV, maxV, step;
};

#define FITEM(key, label, field, mn, mx, st) \
  { key, label, false, &S.field, nullptr, &DEF.field, nullptr, mn, mx, st }
#define BITEM(key, label, field) \
  { key, label, true, nullptr, &S.field, nullptr, &DEF.field, 0, 1, 1 }

Item items[] = {
  FITEM("spd", "Speed normal", speedNormal, 0.10f, 1.00f, 0.05f),
  FITEM("boost", "Speed boost", speedBoost, 0.10f, 1.00f, 0.05f),
  FITEM("slow", "Speed slow", speedSlow, 0.05f, 1.00f, 0.05f),
  FITEM("rot", "Rotate gain", rotateGain, 0.10f, 1.50f, 0.05f),
  FITEM("dead", "Deadzone", deadzone, 0.00f, 0.40f, 0.01f),
  FITEM("mind", "Min duty", minDuty, 0.00f, 0.50f, 0.01f),
  FITEM("acc", "Accel limit", accelLimit, 1.00f, 30.0f, 0.50f),
  FITEM("kp", "Heading Kp", headingKp, 0.00f, 0.20f, 0.005f),
  FITEM("hmax", "Heading max", headingMax, 0.10f, 1.00f, 0.05f),
  BITEM("gyroinv", "Gyro invert", gyroInvert),
  BITEM("invfl", "Inv FL", invFL),
  BITEM("invfr", "Inv FR", invFR),
  BITEM("invrl", "Inv RL", invRL),
  BITEM("invrr", "Inv RR", invRR),
  BITEM("brake", "Brake idle", brakeIdle),
  FITEM("kickms", "Kick pulse ms", kickMs, 20.0f, 1000.0f, 10.0f),
  FITEM("kickcd", "Kick cooldown", kickCoolMs, 100.0f, 5000.0f, 50.0f),
};
constexpr int N_ITEMS = sizeof(items) / sizeof(items[0]);

Preferences prefs;
bool dirty = false;  // unsaved changes

static void formatValue(const Item &it, char *out, size_t n) {
  if (it.isBool) {
    snprintf(out, n, "%s", *it.b ? "ON" : "off");
  } else {
    int dec = (it.step >= 0.5f) ? 1 : (it.step >= 0.01f ? 2 : 3);
    snprintf(out, n, "%.*f", dec, *it.f);
  }
}

static void loadSettings() {
  if (!prefs.begin("mecanum", false)) return;
  for (int i = 0; i < N_ITEMS; i++) {
    Item &it = items[i];
    if (it.isBool) *it.b = prefs.getBool(it.key, *it.b);
    else *it.f = constrain(prefs.getFloat(it.key, *it.f), it.minV, it.maxV);
  }
  prefs.end();
}

static void saveSettings() {
  if (!prefs.begin("mecanum", false)) {
    Serial.println("Save failed");
    return;
  }
  for (int i = 0; i < N_ITEMS; i++) {
    Item &it = items[i];
    if (it.isBool) prefs.putBool(it.key, *it.b);
    else prefs.putFloat(it.key, *it.f);
  }
  prefs.end();
  dirty = false;
  Serial.println("Settings saved to flash");
}

// =============================================================
//                     PWM helpers (core 2.x / 3.x)
// =============================================================
static void pwmAttach(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

static inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// =============================================================
//                          MOTOR
// =============================================================
// DRV8833: PWM on one input, other input LOW = one direction, swap = reverse.
// Both LOW = coast, both HIGH = brake.
struct Motor {
  uint8_t pin1, pin2, ch1, ch2;
  bool invert = false;
  float current = 0.0f;  // logical speed -1..1 (before invert)

  Motor(uint8_t p1, uint8_t p2, uint8_t c1, uint8_t c2)
    : pin1(p1), pin2(p2), ch1(c1), ch2(c2) {}

  void begin() {
    pwmAttach(pin1, ch1);
    pwmAttach(pin2, ch2);
    stop();
  }

  void set(float v) {  // immediate
    current = constrain(v, -1.0f, 1.0f);
    apply();
  }

  void drive(float target, float dt) {  // with acceleration ramp
    float maxStep = S.accelLimit * dt;
    float diff = constrain(target, -1.0f, 1.0f) - current;
    if (diff > maxStep) diff = maxStep;
    if (diff < -maxStep) diff = -maxStep;
    current += diff;
    apply();
  }

  void stop() {
    current = 0.0f;
    apply();
  }

private:
  void apply() {
    float v = invert ? -current : current;
    float mag = fabsf(v);
    if (mag < 0.02f) {
      uint32_t idle = S.brakeIdle ? PWM_MAX : 0;
      pwmWrite(pin1, ch1, idle);
      pwmWrite(pin2, ch2, idle);
      return;
    }
    float d = S.minDuty + mag * (1.0f - S.minDuty);
    uint32_t duty = (uint32_t)(d * PWM_MAX);
    if (v > 0) {
      pwmWrite(pin1, ch1, duty);
      pwmWrite(pin2, ch2, 0);
    } else {
      pwmWrite(pin1, ch1, 0);
      pwmWrite(pin2, ch2, duty);
    }
  }
};

Motor mFL(D1_A1, D1_A2, 0, 1);
Motor mFR(D1_B1, D1_B2, 2, 3);
Motor mRL(D2_A1, D2_A2, 4, 5);
Motor mRR(D2_B1, D2_B2, 6, 7);

void stopAll() {
  mFL.stop();
  mFR.stop();
  mRL.stop();
  mRR.stop();
}

// Copy settings that live inside other objects
static void applySettings() {
  mFL.invert = S.invFL;
  mFR.invert = S.invFR;
  mRL.invert = S.invRL;
  mRR.invert = S.invRR;
}

// =============================================================
//                     KICKER SOLENOID
// =============================================================
// GPIO23 -> resistor -> BJT Darlington base (e.g. TIP122/TIP120).
// Solenoid + flyback diode (1N4007 or similar) across the collector/emitter,
// solenoid supply on its own rail, driver emitter to that rail's ground,
// and that ground tied to the ESP32 ground. GPIO23 HIGH turns the
// solenoid ON; the pulse is short and non-blocking so it never stalls
// motor control or the OLED.
struct Kicker {
  bool firing = false;
  uint32_t offAtMs = 0;
  uint32_t readyAtMs = 0;  // cooldown expiry

  void begin() {
    pinMode(KICK_PIN, OUTPUT);
    digitalWrite(KICK_PIN, LOW);
  }

  bool ready(uint32_t now) const {
    return !firing && now >= readyAtMs;
  }

  void fire(uint32_t now) {
    if (!ready(now)) return;
    digitalWrite(KICK_PIN, HIGH);
    firing = true;
    offAtMs = now + (uint32_t)S.kickMs;
    readyAtMs = now + (uint32_t)(S.kickMs + S.kickCoolMs);
  }

  void update(uint32_t now) {
    if (firing && now >= offAtMs) {
      digitalWrite(KICK_PIN, LOW);
      firing = false;
    }
  }
};

Kicker kicker;

// =============================================================
//                          MPU6050
// =============================================================
static float wrapDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

struct Mpu6050 {
  bool ok = false;
  float yaw = 0, pitch = 0, roll = 0;  // degrees
  float gyroZ = 0;                     // deg/s (bias removed)
  float biasX = 0, biasY = 0, biasZ = 0;
  uint32_t lastUs = 0;

  bool begin() {
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() != 0) {
      ok = false;
      return false;
    }
    writeReg(0x6B, 0x80);
    delay(100);  // reset
    writeReg(0x6B, 0x01);
    delay(50);             // wake, PLL clock
    writeReg(0x1A, 0x03);  // DLPF ~44 Hz
    writeReg(0x1B, 0x08);  // gyro  +-500 dps  (65.5 LSB/dps)
    writeReg(0x1C, 0x00);  // accel +-2 g      (16384 LSB/g)
    ok = true;
    lastUs = micros();
    return true;
  }

  void calibrate() {
    if (!ok) return;
    const int N = 500;
    double sx = 0, sy = 0, sz = 0;
    float ax, ay, az, gx, gy, gz;
    int good = 0;
    for (int i = 0; i < N; i++) {
      if (readRaw(ax, ay, az, gx, gy, gz)) {
        sx += gx;
        sy += gy;
        sz += gz;
        good++;
      }
      delay(2);
    }
    if (good > 0) {
      biasX = sx / good;
      biasY = sy / good;
      biasZ = sz / good;
    }
    yaw = 0;
    lastUs = micros();
  }

  void update() {
    if (!ok) return;
    float ax, ay, az, gx, gy, gz;
    if (!readRaw(ax, ay, az, gx, gy, gz)) return;

    uint32_t now = micros();
    float dt = (now - lastUs) * 1e-6f;
    lastUs = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

    gx -= biasX;
    gy -= biasY;
    gz -= biasZ;
    if (S.gyroInvert) gz = -gz;
    if (fabsf(gz) < 0.08f) gz = 0;  // noise gate
    gyroZ = gz;
    yaw = wrapDeg(yaw + gz * dt);

    // complementary filter for tilt (display only)
    float accRoll = atan2f(ay, az) * RAD_TO_DEG;
    float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    roll = 0.98f * (roll + gx * dt) + 0.02f * accRoll;
    pitch = 0.98f * (pitch + gy * dt) + 0.02f * accPitch;
  }

private:
  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  bool readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
    int16_t v[7];
    for (int i = 0; i < 7; i++) {
      uint8_t hi = Wire.read();
      uint8_t lo = Wire.read();
      v[i] = (int16_t)((hi << 8) | lo);
    }
    ax = v[0] / 16384.0f;
    ay = v[1] / 16384.0f;
    az = v[2] / 16384.0f;
    // v[3] = temperature
    gx = v[4] / 65.5f;
    gy = v[5] / 65.5f;
    gz = v[6] / 65.5f;
    return true;
  }
};

Mpu6050 imu;

// =============================================================
//                          OLED
// =============================================================
Adafruit_SSD1306 display(128, 64, &Wire, -1, 400000UL, 400000UL);
bool oledOk = false;

// UI state
bool fieldCentric = false;
bool headingHold = false;
float speedScale = 0.6f;
bool padConnected = false;
int crabDir = 0;  // -1 = crab left, +1 = crab right, 0 = off
bool menuOpen = false;
int menuSel = 0;
char toastMsg[20] = "";
uint32_t toastUntil = 0;

static void showToast(const char *m) {
  strlcpy(toastMsg, m, sizeof(toastMsg));
  toastUntil = millis() + 1500;
}

static void drawBar(int x, int y, int w, int h, float val) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int mid = x + w / 2;
  int len = (int)(constrain(val, -1.0f, 1.0f) * (w / 2 - 1));
  if (len > 0) display.fillRect(mid, y + 1, len, h - 2, SSD1306_WHITE);
  else if (len < 0) display.fillRect(mid + len, y + 1, -len, h - 2, SSD1306_WHITE);
  display.drawFastVLine(mid, y, h, SSD1306_WHITE);
}

static void oledMessage(const char *l1, const char *l2 = "") {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.print(l1);
  display.setCursor(0, 32);
  display.print(l2);
  display.display();
}

static void oledMain() {
  char buf[32];
  display.setCursor(0, 0);
  if (crabDir > 0) display.print("CRAB >>");
  else if (crabDir < 0) display.print("CRAB <<");
  else {
    snprintf(buf, sizeof(buf), "MECANUM%s", dirty ? "*" : "");
    display.print(buf);
  }
  display.setCursor(74, 0);
  display.print(padConnected ? "DS3:OK" : "DS3:--");

  snprintf(buf, sizeof(buf), "FLD:%-3s HLD:%-3s %3d%%",
           fieldCentric ? "ON" : "off", headingHold ? "ON" : "off",
           (int)(speedScale * 100));
  display.setCursor(0, 9);
  display.print(buf);

  if (imu.ok) {
    snprintf(buf, sizeof(buf), "Yaw %+7.1f", imu.yaw);
    display.setCursor(0, 18);
    display.print(buf);
  } else {
    display.setCursor(0, 18);
    display.print("MPU6050 not found");
  }
  uint32_t now = millis();
  const char *kstate = kicker.firing ? "FIRE" : (kicker.ready(now) ? "rdy" : "cool");
  snprintf(buf, sizeof(buf), "Kick:%-4s", kstate);
  display.setCursor(74, 18);
  display.print(buf);

  if (imu.ok) {
    snprintf(buf, sizeof(buf), "P%+6.1f R%+6.1f", imu.pitch, imu.roll);
    display.setCursor(0, 27);
    display.print(buf);
  }

  // Wheel bars laid out like the robot (front row / rear row)
  display.setCursor(0, 41);
  display.print("FL");
  drawBar(14, 39, 46, 10, mFL.current);
  display.setCursor(66, 41);
  display.print("FR");
  drawBar(80, 39, 46, 10, mFR.current);
  display.setCursor(0, 54);
  display.print("RL");
  drawBar(14, 52, 46, 10, mRL.current);
  display.setCursor(66, 54);
  display.print("RR");
  drawBar(80, 52, 46, 10, mRR.current);
}

static void oledMenu() {
  char buf[32], val[24];

  if (millis() < toastUntil) {
    snprintf(buf, sizeof(buf), "%s", toastMsg);
  } else {
    snprintf(buf, sizeof(buf), "SETTINGS %2d/%d%s", menuSel + 1, N_ITEMS, dirty ? " *" : "");
  }
  display.setCursor(0, 0);
  display.print(buf);
  display.drawFastHLine(0, 8, 128, SSD1306_WHITE);

  const int rows = 5;
  int first = constrain(menuSel - 2, 0, max(0, N_ITEMS - rows));
  for (int i = 0; i < rows && first + i < N_ITEMS; i++) {
    int idx = first + i;
    formatValue(items[idx], val, sizeof(val));
    snprintf(buf, sizeof(buf), "%c%-12.12s %7.7s", idx == menuSel ? '>' : ' ', items[idx].label, val);
    int y = 11 + i * 9;
    if (idx == menuSel) {
      display.fillRect(0, y - 1, 128, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, y);
    display.print(buf);
  }
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 56);
  display.print("R1:x5 X:dflt SEL:save");
}

static void oledUpdate() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  if (menuOpen) oledMenu();
  else oledMain();
  display.display();
}

// =============================================================
//                        BLUEPAD32
// =============================================================
ControllerPtr gamepad = nullptr;
uint32_t lastPadDataMs = 0;

void onConnectedController(ControllerPtr ctl) {
  if (gamepad == nullptr) {
    Serial.printf("Controller connected: %s\n", ctl->getModelName().c_str());
    gamepad = ctl;
    lastPadDataMs = millis();
  } else {
    Serial.println("Extra controller ignored (only one supported)");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (gamepad == ctl) {
    Serial.println("Controller disconnected");
    gamepad = nullptr;
  }
}

// =============================================================
//                     CONTROL HELPERS
// =============================================================
static float applyDeadzone(float v) {
  v = constrain(v, -1.0f, 1.0f);
  if (fabsf(v) < S.deadzone) return 0.0f;
  return (v - copysignf(S.deadzone, v)) / (1.0f - S.deadzone);
}

// mild expo curve: fine control near centre, full range at the edge
static float shape(float v) {
  return 0.5f * v + 0.5f * v * v * v;
}

static bool risingEdge(bool now, bool &prev) {
  bool e = now && !prev;
  prev = now;
  return e;
}

// Standard X-configuration mecanum mix.
//   vx = strafe right (+), vy = forward (+), w = rotate CCW (+)
static void mecanumDrive(float vx, float vy, float w, float dt) {
  float fl = vy + vx - w;
  float fr = vy - vx + w;
  float rl = vy - vx - w;
  float rr = vy + vx + w;

  float m = max(max(fabsf(fl), fabsf(fr)), max(fabsf(rl), fabsf(rr)));
  if (m > 1.0f) {
    fl /= m;
    fr /= m;
    rl /= m;
    rr /= m;
  }

  mFL.drive(fl, dt);
  mFR.drive(fr, dt);
  mRL.drive(rl, dt);
  mRR.drive(rr, dt);
}

// ---------------- settings menu logic ----------------
static void menuAdjust(int dir, bool coarse, bool first) {
  Item &it = items[menuSel];
  if (it.isBool) {
    if (!first) return;  // toggle only once per press
    *it.b = !*it.b;
  } else {
    float v = *it.f + dir * it.step * (coarse ? 5.0f : 1.0f);
    v = roundf(v / it.step) * it.step;
    *it.f = constrain(v, it.minV, it.maxV);
  }
  applySettings();
  dirty = true;
}

static void menuHandle(ControllerPtr c, const Edges &e, uint32_t now) {
  if (e.up) menuSel = (menuSel + N_ITEMS - 1) % N_ITEMS;
  if (e.down) menuSel = (menuSel + 1) % N_ITEMS;

  uint8_t dp = c->dpad();
  int dir = (dp & DPAD_RIGHT_MASK) ? 1 : ((dp & DPAD_LEFT_MASK) ? -1 : 0);
  static int lastDir = 0;
  static uint32_t nextRepeat = 0;
  if (dir == 0) {
    lastDir = 0;
  } else if (dir != lastDir) {
    lastDir = dir;
    menuAdjust(dir, c->r1(), true);
    nextRepeat = now + 400;  // delay before auto-repeat
  } else if (now >= nextRepeat) {
    menuAdjust(dir, c->r1(), false);
    nextRepeat = now + 80;
  }

  if (e.cross) {
    Item &it = items[menuSel];
    if (it.isBool) *it.b = *it.db;
    else *it.f = *it.df;
    applySettings();
    dirty = true;
    showToast("Item reset");
  }
  if (e.tri) {
    S = DEF;
    applySettings();
    dirty = true;
    showToast("All defaults");
  }
  if (e.sel) {
    saveSettings();
    showToast("Saved!");
  }
}

// ---------------- main control step ----------------
float headingTarget = 0;
uint32_t lastRotMs = 0;

static void controlStep(float dt) {
  uint32_t now = millis();

  // ---- failsafe ----
  bool linkOk = gamepad && gamepad->isConnected() && (PAD_TIMEOUT_MS == 0 || now - lastPadDataMs < PAD_TIMEOUT_MS);
  padConnected = linkOk;
  if (!linkOk) {
    crabDir = 0;
    menuOpen = false;
    stopAll();
    return;
  }

  ControllerPtr c = gamepad;

  // ---- button edges (computed every cycle) ----
  static bool pTri = false, pSqr = false, pCross = false, pUp = false,
              pDown = false, pStart = false, pSel = false, pKick = false;
  uint8_t dp = c->dpad();
  uint8_t misc = c->miscButtons();
  Edges e;
  e.tri = risingEdge(c->y(), pTri);
  e.sqr = risingEdge(c->x(), pSqr);
  e.cross = risingEdge(c->a(), pCross);
  e.up = risingEdge(dp & DPAD_UP_MASK, pUp);
  e.down = risingEdge(dp & DPAD_DOWN_MASK, pDown);
  e.start = risingEdge(misc & MISC_START_MASK, pStart);
  e.sel = risingEdge(misc & MISC_SELECT_MASK, pSel);
  e.kick = risingEdge(c->b(), pKick);  // (still tracked, not needed for firing)
  bool kickHeld = c->b();              // Circle button held

  // ---- kicker solenoid (works even while the settings menu is open,
  //      so long as a wheel isn't also fighting the driver) ----
  if (kickHeld) kicker.fire(now);  // held = auto-repeat at the cooldown rate

  // ---- Start toggles the settings menu ----
  if (e.start) {
    menuOpen = !menuOpen;
    crabDir = 0;
    stopAll();
  }
  if (menuOpen) {
    stopAll();  // robot never moves while editing
    menuHandle(c, e, now);
    return;
  }

  // ---- mode buttons ----
  if (e.tri) fieldCentric = !fieldCentric && imu.ok;
  if (e.sqr) {
    headingHold = !headingHold && imu.ok;
    headingTarget = imu.yaw;
  }
  if (e.cross) {
    imu.yaw = 0;
    headingTarget = 0;
  }
  if (e.down) {
    stopAll();
    oledMessage("Calibrating gyro...", "Keep robot still");
    imu.calibrate();
    headingTarget = 0;
    return;
  }

  // ---- sticks ----
  float fx = shape(applyDeadzone(c->axisX() / 512.0f));    // right +
  float fy = shape(applyDeadzone(-c->axisY() / 512.0f));   // forward +
  float rot = shape(applyDeadzone(c->axisRX() / 512.0f));  // right = clockwise

  speedScale = c->r1() ? S.speedBoost : (c->l1() ? S.speedSlow : S.speedNormal);
  fx *= speedScale;
  fy *= speedScale;
  float w = -rot * speedScale * S.rotateGain;  // CCW positive

  // ---- field-centric: rotate the stick vector by -yaw ----
  float vx = fx, vy = fy;
  if (fieldCentric && imu.ok) {
    float th = imu.yaw * DEG_TO_RAD;
    float cs = cosf(th), sn = sinf(th);
    vx = fx * cs + fy * sn;
    vy = -fx * sn + fy * cs;
  }

  // ---- crab walk: D-pad Left/Right = pure robot-relative sideways slide ----
  bool dpR = dp & DPAD_RIGHT_MASK;
  bool dpL = dp & DPAD_LEFT_MASK;
  crabDir = (dpR == dpL) ? 0 : (dpR ? 1 : -1);  // both/none -> off
  if (crabDir != 0) {
    vx = crabDir * speedScale;
    vy = 0.0f;
  }

  // ---- heading hold (gyro assist while driving) ----
  // Forced on while crab walking so the robot slides straight, not in an arc.
  bool rotating = fabsf(rot) > 0.0f;
  bool translating = (crabDir != 0) || (fabsf(fx) + fabsf(fy)) > 0.05f;
  if (rotating) {
    headingTarget = imu.yaw;
    lastRotMs = now;
  } else if ((headingHold || crabDir != 0) && imu.ok && translating) {
    if (now - lastRotMs < ROT_SETTLE_MS) {
      headingTarget = imu.yaw;  // let rotation settle first
    } else {
      float err = wrapDeg(headingTarget - imu.yaw);
      if (fabsf(err) > 1.0f) w = constrain(S.headingKp * err, -S.headingMax, S.headingMax);
    }
  } else {
    headingTarget = imu.yaw;
  }

  mecanumDrive(vx, vy, w, dt);
}

// =============================================================
//                    SERIAL / TEST HELPERS
// =============================================================
static void motorTest() {
  Motor *m[4] = { &mFL, &mFR, &mRL, &mRR };
  const char *n[4] = { "FL", "FR", "RL", "RR" };
  for (int i = 0; i < 4; i++) {
    Serial.printf("Motor %s -> should spin FORWARD\n", n[i]);
    m[i]->set(0.6f);
    delay(800);
    m[i]->set(0);
    delay(400);
  }
  Serial.println("Test done");
}

static void printSettings() {
  char val[24], def[24];
  Serial.println("key       value     default   range");
  for (int i = 0; i < N_ITEMS; i++) {
    Item &it = items[i];
    formatValue(it, val, sizeof(val));
    if (it.isBool) snprintf(def, sizeof(def), "%s", *it.db ? "ON" : "off");
    else snprintf(def, sizeof(def), "%g", *it.df);
    if (it.isBool) Serial.printf("%-9s %-9s %-9s on/off\n", it.key, val, def);
    else Serial.printf("%-9s %-9s %-9s %g..%g\n", it.key, val, def, it.minV, it.maxV);
  }
  if (dirty) Serial.println("(unsaved changes - type 'save')");
}

static bool setByKey(const char *key, const char *val) {
  for (int i = 0; i < N_ITEMS; i++) {
    Item &it = items[i];
    if (strcmp(it.key, key) != 0) continue;
    if (it.isBool) {
      *it.b = (!strcasecmp(val, "on") || !strcasecmp(val, "true") || atoi(val) != 0);
    } else {
      float v = (float)atof(val);
      *it.f = constrain(v, it.minV, it.maxV);
    }
    applySettings();
    dirty = true;
    return true;
  }
  return false;
}

static void handleLine(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcmp(cmd, "help")) {
    Serial.println("help | list | set <key> <value> | save | defaults | test | cal");
  } else if (!strcmp(cmd, "list")) {
    printSettings();
  } else if (!strcmp(cmd, "set")) {
    char *k = strtok(NULL, " \t");
    char *v = strtok(NULL, " \t");
    if (!k || !v) {
      Serial.println("usage: set <key> <value>");
      return;
    }
    if (setByKey(k, v)) Serial.printf("%s updated (type 'save' to keep)\n", k);
    else Serial.println("unknown key - type 'list'");
  } else if (!strcmp(cmd, "save")) {
    saveSettings();
  } else if (!strcmp(cmd, "defaults")) {
    S = DEF;
    applySettings();
    dirty = true;
    Serial.println("Defaults restored (type 'save' to keep)");
  } else if (!strcmp(cmd, "test") || !strcmp(cmd, "t")) {
    stopAll();
    motorTest();
  } else if (!strcmp(cmd, "cal") || !strcmp(cmd, "c")) {
    stopAll();
    Serial.println("Calibrating gyro, keep robot still...");
    imu.calibrate();
    Serial.println("Done");
  } else if (!strcmp(cmd, "kick")) {
    uint32_t now = millis();
    if (kicker.ready(now)) {
      kicker.fire(now);
      Serial.println("Kick!");
    } else Serial.println("Kicker not ready (cooling down)");
  } else {
    Serial.println("unknown command - type 'help'");
  }
}

static void handleSerial() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (len) {
        buf[len] = 0;
        handleLine(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
}



// [BEGIN lopaka generated]
static const unsigned char image_paint_5_bits[] U8X8_PROGMEM = {0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x08,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00,0x02,0x00};
static const unsigned char image_paint_6_bits[] U8X8_PROGMEM = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xe0,0x7b,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x42,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const unsigned char image_paint_8_bits[] U8X8_PROGMEM = {0x01,0x40};
static const unsigned char image_Screenshot_2026_10_08_191837_bits[] U8X8_PROGMEM = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x00,0x00,0x00,0x80,0x04,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x24,0x0d,0x00,0x00,0x00,0xc0,0x0f,0x3c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xff,0x0d,0x00,0x00,0x00,0x00,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x07,0x08,0x00,0x00,0x00,0x00,0x1d,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x10,0x00,0x00,0x00,0x00,0x3d,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1e,0x00,0x00,0x00,0x00,0x00,0x39,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3c,0x00,0x00,0x00,0x00,0x00,0x79,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0xf1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x00,0x00,0x00,0x00,0x00,0xe1,0x10,0xf0,0x63,0xe0,0x07,0xfc,0x18,0x0e,0x0e,0xe0,0x01,0xf8,0x31,0x00,0x00,0xe1,0x19,0x38,0x23,0x30,0x0e,0xce,0x19,0x0e,0x0f,0xc0,0x03,0x1c,0x33,0x00,0x00,0xc1,0x01,0x1c,0x26,0x18,0x1c,0x87,0x09,0x8e,0x07,0x80,0x03,0x0c,0x37,0x00,0x00,0xc1,0x1b,0x0c,0x3e,0x18,0x1c,0x87,0x0b,0xce,0x00,0x80,0x01,0x0e,0x16,0x00,0x00,0x81,0x07,0x0e,0x3e,0x1c,0x1c,0x03,0x0f,0xce,0x00,0xc0,0x00,0x0e,0x1e,0x00,0x00,0x01,0x1f,0x0e,0x1c,0x1c,0x9c,0x03,0x07,0xfe,0x00,0x60,0x00,0x0e,0x1e,0x00,0x00,0x01,0x0f,0x0e,0x1c,0x1c,0x9c,0x03,0x07,0xfe,0x00,0x18,0x00,0x06,0x0e,0x00,0x00,0x01,0x0e,0x0e,0x0c,0x1c,0x1c,0x03,0x07,0xce,0x01,0x0c,0x20,0x0e,0x0c,0x00,0x00,0x01,0x1e,0x0c,0x1c,0x1c,0x1c,0x07,0x07,0x8e,0x03,0x06,0x30,0x0e,0x0e,0x00,0x00,0x01,0x0c,0x1c,0x1f,0x3c,0x0c,0x87,0x07,0x0e,0x07,0xff,0x1f,0x0e,0x1f,0x00,0x80,0x01,0x18,0xf8,0xf9,0x7c,0x07,0xfe,0x3e,0x0e,0x8f,0xff,0x1f,0xfc,0x79,0x00,0xc0,0x07,0x18,0xf0,0x30,0xdc,0x03,0x3c,0x1c,0x04,0xdc,0xff,0x1f,0x70,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void splashScreen(void) {

    // logo
    display.drawBitmap(
        1, 14,
        image_Screenshot_2026_10_08_191837_bits,
        128, 40,
        SSD1306_WHITE
    );

    // paint 5
    display.drawBitmap(
        9, 27,
        image_paint_5_bits,
        12, 16,
        SSD1306_WHITE
    );

    // paint 6
    display.drawBitmap(
        20, 24,
        image_paint_6_bits,
        112, 18,
        SSD1306_WHITE
    );

    // paint 8
    display.drawBitmap(
        108, 38,
        image_paint_8_bits,
        7, 2,
        SSD1306_WHITE
    );
}
// [END lopaka generated]


// =============================================================
//                           SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Mecanum robot ===");

  loadSettings();
  mFL.begin();
  mFR.begin();
  mRL.begin();
  mRR.begin();
  kicker.begin();
  applySettings();
  Serial.println("Settings loaded (type 'help' for serial commands)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Serial.println(oledOk ? "OLED ok" : "OLED not found");
  oledMessage("Mecanum robot", "Starting...");

  if (imu.begin()) {
    Serial.println("MPU6050 ok, calibrating (keep still)...");
    oledMessage("Calibrating gyro...", "Keep robot still");
    imu.calibrate();
    Serial.println("Gyro calibrated");
  } else {
    Serial.println("MPU6050 NOT found - gyro features disabled");
  }

  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();   // uncomment once if pairing gets stuck

  const uint8_t *a = BP32.localBdAddress();
  Serial.printf("ESP32 BT address (pair the DS3 to this): %02X:%02X:%02X:%02X:%02X:%02X\n",
                a[0], a[1], a[2], a[3], a[4], a[5]);

  char buf[24];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           a[0], a[1], a[2], a[3], a[4], a[5]);
  oledMessage("Waiting for DS3", buf);
  splashScreen();
  delay(1500);
}

// =============================================================
//                            LOOP
// =============================================================
void loop() {
  static uint32_t lastCtrl = 0, lastOled = 0;

  if (BP32.update()) lastPadDataMs = millis();

  uint32_t now = millis();
  kicker.update(now);

  if (now - lastCtrl >= CONTROL_PERIOD_MS) {
    float dt = (now - lastCtrl) * 0.001f;
    lastCtrl = now;
    imu.update();
    controlStep(dt);
  }

  uint32_t oledPeriod = menuOpen ? 100 : OLED_PERIOD_MS;
  if (now - lastOled >= oledPeriod) {
    lastOled = now;
    oledUpdate();
  }

  handleSerial();
  vTaskDelay(1);  // yield to Bluetooth / watchdog
}
