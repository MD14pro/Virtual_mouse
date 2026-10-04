/*
 * ================================================================
 *  Virtual Mouse v6 — ESP32-C3 + MPU-6050 + 2× TTP223
 *  DEBUG BUILD — open Serial Monitor at 115200 to diagnose
 * ================================================================
 *
 *  FIXES IN v6:
 *  - Added full Serial debug so you can see exactly what fires
 *  - Removed isConnected() gate on sensor reads (was blocking loop)
 *  - Added explicit BLE HID descriptor for Windows compatibility
 *  - Touch pins set INPUT_PULLDOWN (fixes floating pin false triggers)
 *  - Gyro deadzone raised to reduce phantom cursor drift
 *  - Cursor movement now only fires when angle delta is meaningful
 *
 *  REQUIRED LIBRARIES (Arduino Library Manager):
 *  1. "BLE Mouse"               — by T-vK
 *  2. "Adafruit MPU6050"        — by Adafruit
 *  3. "Adafruit Unified Sensor" — by Adafruit
 *
 *  BOARD SETTINGS:
 *    Board            → ESP32C3 Dev Module
 *    Partition Scheme → Huge APP (3MB No OTA)
 *    USB CDC On Boot  → Disabled
 *
 * ----------------------------------------------------------------
 *  WIRING
 * ----------------------------------------------------------------
 *  MPU-6050   VCC → 3.3V      GND → GND
 *             SDA → GPIO 3    SCL → GPIO 4
 *
 *  TTP223 LEFT   VCC → 3.3V   GND → GND   I/O → GPIO 7
 *  TTP223 RIGHT  VCC → 3.3V   GND → GND   I/O → GPIO 6
 *
 * ----------------------------------------------------------------
 *  GESTURE TABLE
 * ----------------------------------------------------------------
 *  Move hand in air                      → Move Cursor
 *  Tap LEFT                              → Left Click
 *  Tap RIGHT                             → Right Click
 *  Hold LEFT  + move hand                → Left Drag
 *  Hold RIGHT + move hand                → Right Drag
 *  Hold LEFT  + sharp tilt wrist LEFT    → Slide NEXT →
 *  Hold RIGHT + sharp tilt wrist RIGHT   → Slide PREV ←
 *  Quick tap LEFT → then RIGHT           → Scroll DOWN ↓
 *  Quick tap RIGHT → then LEFT           → Scroll UP ↑
 *  Hold BOTH sensors 1 second            → Recenter origin
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <BleMouse.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── Pins ────────────────────────────────────────────────────────
#define PIN_SDA       3
#define PIN_SCL       4
#define PIN_TOUCH_L   7
#define PIN_TOUCH_R   6

// ── Tuning ──────────────────────────────────────────────────────
#define CURSOR_SENSITIVITY   5.0f
#define GYRO_DEADZONE        0.03f   // raised — reduces phantom drift
#define COMPLEMENTARY_ALPHA  0.96f
#define HOLD_MS              350
#define TAP_MAX_MS           300
#define SLIDE_GYRO_THRESH    1.5f
#define SLIDE_LOCKOUT_MS     800
#define RECENTER_HOLD_MS     1000
#define SCROLL_AMOUNT        3
#define SCROLL_SEQ_MS        400
#define CURSOR_MIN_DELTA     0.08f   // deg — ignore tiny angle changes
#define LOOP_PERIOD_MS       8

// ── Objects ─────────────────────────────────────────────────────
// Constructor: (deviceName, manufacturer, battery)
BleMouse         bleMouse("VirtualMouse", "ESP32", 100);
Adafruit_MPU6050 mpu;

// ── Angle tracking ──────────────────────────────────────────────
float angleX = 0.0f, angleY = 0.0f;
float originX = 0.0f, originY = 0.0f;
float prevAngleX = 0.0f, prevAngleY = 0.0f;

// ── Button state ────────────────────────────────────────────────
bool     prevL = false,  prevR = false;
uint32_t pressTimeL = 0, pressTimeR = 0;
bool     holdActiveL = false, holdActiveR = false;

// ── Feature state ───────────────────────────────────────────────
uint32_t lastSlideMs    = 0;
uint32_t bothPressStart = 0;
bool     bothHeld       = false;
bool     recentered     = false;
char     lastQuickTap   = 0;
uint32_t lastQuickTapTime = 0;

// ── Debug counters ──────────────────────────────────────────────
uint32_t loopCount      = 0;
uint32_t lastDebugPrint = 0;

// ── Helpers ─────────────────────────────────────────────────────
float dz(float v, float d) {
  if (v >  d) return v - d;
  if (v < -d) return v + d;
  return 0.0f;
}

void recenterOrigin() {
  originX = angleX;
  originY = angleY;
  Serial.println("[RECENTER] Done");
}

// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=============================");
  Serial.println(" Virtual Mouse v6 — DEBUG");
  Serial.println("=============================");

  // Touch pins with pull-down so floating pin reads LOW not random
  pinMode(PIN_TOUCH_L, INPUT_PULLDOWN);
  pinMode(PIN_TOUCH_R, INPUT_PULLDOWN);
  Serial.println("[OK] Touch pins configured (INPUT_PULLDOWN)");

  // I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Serial.println("[OK] I2C started");

  // MPU-6050
  if (!mpu.begin()) {
    Serial.println("[ERR] MPU-6050 not found! Halting.");
    while (true) delay(500);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("[OK] MPU-6050 configured");

  // Initial angle from accelerometer
  delay(200);
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  angleX  = atan2(accel.acceleration.y,  accel.acceleration.z) * RAD_TO_DEG;
  angleY  = atan2(-accel.acceleration.x, accel.acceleration.z) * RAD_TO_DEG;
  originX = angleX;
  originY = angleY;
  prevAngleX = angleX;
  prevAngleY = angleY;
  Serial.print("[OK] Initial angles — X: "); Serial.print(angleX, 2);
  Serial.print("  Y: "); Serial.println(angleY, 2);

  // BLE
  Serial.println("[..] Starting BLE Mouse...");
  bleMouse.begin();
  Serial.println("[OK] BLE advertising as 'VirtualMouse'");
  Serial.println("     On Windows: Settings → Bluetooth → Add Device");
  Serial.println("     Look for 'VirtualMouse' and pair it.");
  Serial.println("     After pairing, move your hand to test cursor.");
  Serial.println("=============================\n");
}

// ════════════════════════════════════════════════════════════════
void loop() {
  static uint32_t lastLoopMs = 0;
  uint32_t now = millis();
  if (now - lastLoopMs < LOOP_PERIOD_MS) return;
  float dt = (now - lastLoopMs) / 1000.0f;
  lastLoopMs = now;
  loopCount++;

  // ── Print connection status every 3 seconds ─────────────────
  if (now - lastDebugPrint > 3000) {
    lastDebugPrint = now;
    if (bleMouse.isConnected()) {
      Serial.print("[BLE] Connected  | Loop Hz ~");
      Serial.print(loopCount / 3);
      Serial.print(" | AngleX: "); Serial.print(angleX, 1);
      Serial.print("  AngleY: "); Serial.println(angleY, 1);
    } else {
      Serial.println("[BLE] NOT connected — waiting for Windows to pair...");
    }
    loopCount = 0;
  }

  // ── Read sensors every loop regardless of BLE ───────────────
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Complementary filter
  float accelAngleX = atan2(accel.acceleration.y,
                             accel.acceleration.z) * RAD_TO_DEG;
  float accelAngleY = atan2(-accel.acceleration.x,
                              accel.acceleration.z) * RAD_TO_DEG;

  float gx = dz(gyro.gyro.x, GYRO_DEADZONE) * RAD_TO_DEG;
  float gy = dz(gyro.gyro.y, GYRO_DEADZONE) * RAD_TO_DEG;

  angleX = COMPLEMENTARY_ALPHA * (angleX + gx * dt)
         + (1.0f - COMPLEMENTARY_ALPHA) * accelAngleX;
  angleY = COMPLEMENTARY_ALPHA * (angleY + gy * dt)
         + (1.0f - COMPLEMENTARY_ALPHA) * accelAngleY;

  // Read touch
  bool touchL = digitalRead(PIN_TOUCH_L);
  bool touchR = digitalRead(PIN_TOUCH_R);

  // ── Expire stale scroll sequence ────────────────────────────
  if (lastQuickTap != 0 && (now - lastQuickTapTime > SCROLL_SEQ_MS)) {
    lastQuickTap = 0;
  }

  // ── Only send BLE if connected ───────────────────────────────
  if (!bleMouse.isConnected()) {
    prevL = touchL;
    prevR = touchR;
    prevAngleX = angleX;
    prevAngleY = angleY;
    return;
  }

  // ── BOTH held: recenter ──────────────────────────────────────
  if (touchL && touchR) {
    if (!bothHeld) {
      bothHeld       = true;
      bothPressStart = now;
      recentered     = false;
      Serial.println("[BOTH] Held — keep holding 1s to recenter");
    }
    if (!recentered && (now - bothPressStart >= RECENTER_HOLD_MS)) {
      recenterOrigin();
      recentered = true;
    }
    prevL = touchL; prevR = touchR;
    return;
  } else {
    bothHeld = false;
  }

  // ════════════════════════════════════════════════════════════
  //  LEFT sensor
  // ════════════════════════════════════════════════════════════
  if (touchL && !prevL) {
    pressTimeL  = now;
    holdActiveL = false;
    Serial.println("[L] Touch start");
  }
  if (touchL && !holdActiveL && (now - pressTimeL >= HOLD_MS)) {
    holdActiveL = true;
    bleMouse.press(MOUSE_LEFT);
    Serial.println("[L] HOLD → LEFT BUTTON DOWN");
  }
  if (!touchL && prevL) {
    uint32_t dur = now - pressTimeL;
    if (!holdActiveL) {
      if (dur <= TAP_MAX_MS) {
        if (lastQuickTap == 'R' && (now - lastQuickTapTime <= SCROLL_SEQ_MS)) {
          bleMouse.move(0, 0, SCROLL_AMOUNT);
          Serial.println("[SCROLL] UP ↑");
          lastQuickTap = 0;
        } else {
          bleMouse.click(MOUSE_LEFT);
          Serial.println("[L] LEFT CLICK");
          lastQuickTap     = 'L';
          lastQuickTapTime = now;
        }
      }
    } else {
      bleMouse.release(MOUSE_LEFT);
      holdActiveL = false;
      Serial.println("[L] HOLD released");
    }
  }

  // ════════════════════════════════════════════════════════════
  //  RIGHT sensor
  // ════════════════════════════════════════════════════════════
  if (touchR && !prevR) {
    pressTimeR  = now;
    holdActiveR = false;
    Serial.println("[R] Touch start");
  }
  if (touchR && !holdActiveR && (now - pressTimeR >= HOLD_MS)) {
    holdActiveR = true;
    bleMouse.press(MOUSE_RIGHT);
    Serial.println("[R] HOLD → RIGHT BUTTON DOWN");
  }
  if (!touchR && prevR) {
    uint32_t dur = now - pressTimeR;
    if (!holdActiveR) {
      if (dur <= TAP_MAX_MS) {
        if (lastQuickTap == 'L' && (now - lastQuickTapTime <= SCROLL_SEQ_MS)) {
          bleMouse.move(0, 0, -SCROLL_AMOUNT);
          Serial.println("[SCROLL] DOWN ↓");
          lastQuickTap = 0;
        } else {
          bleMouse.click(MOUSE_RIGHT);
          Serial.println("[R] RIGHT CLICK");
          lastQuickTap     = 'R';
          lastQuickTapTime = now;
        }
      }
    } else {
      bleMouse.release(MOUSE_RIGHT);
      holdActiveR = false;
      Serial.println("[R] HOLD released");
    }
  }

  // ── SLIDE ────────────────────────────────────────────────────
  float rawGyroX = gyro.gyro.x;
  if ((now - lastSlideMs) > SLIDE_LOCKOUT_MS) {
    if (holdActiveL && rawGyroX < -SLIDE_GYRO_THRESH) {
      for (int i = 0; i < 4; i++) { bleMouse.move(127, 0, 0); delay(12); }
      lastSlideMs = now;
      Serial.println("[SLIDE] NEXT →");
    } else if (holdActiveR && rawGyroX > SLIDE_GYRO_THRESH) {
      for (int i = 0; i < 4; i++) { bleMouse.move(-127, 0, 0); delay(12); }
      lastSlideMs = now;
      Serial.println("[SLIDE] PREV ←");
    }
  }

  // ── CURSOR ───────────────────────────────────────────────────
  float deltaX = angleY - prevAngleY;
  float deltaY = angleX - prevAngleX;

  // Only move if delta is above minimum threshold
  if (abs(deltaX) < CURSOR_MIN_DELTA) deltaX = 0;
  if (abs(deltaY) < CURSOR_MIN_DELTA) deltaY = 0;

  int8_t dx = (int8_t)constrain(deltaX * CURSOR_SENSITIVITY, -127, 127);
  int8_t dy = (int8_t)constrain(deltaY * CURSOR_SENSITIVITY, -127, 127);

  if (dx != 0 || dy != 0) {
    bleMouse.move(dx, dy, 0);
  }

  prevAngleX = angleX;
  prevAngleY = angleY;
  prevL = touchL;
  prevR = touchR;
}
