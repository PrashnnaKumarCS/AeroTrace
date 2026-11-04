// AeroTrace Level 1 prototype
// ESP32 + AMG8833 thermal sensor + pan servo + tilt servo + 4-pin PWM fan
// Libraries: Adafruit AMG88xx, ESP32Servo
// Not bench-tested: expect to adjust pins, SIGN_* values and HOT_DELTA on your hardware.

#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <ESP32Servo.h>

Adafruit_AMG88xx amg;
Servo panServo, tiltServo;

const int SDA_PIN = 21, SCL_PIN = 22;
const int PAN_PIN = 18, TILT_PIN = 19;
const int FAN_PWM_PIN = 25;
const uint8_t SENSOR_ADDR = 0x69;  // use 0x68 if your module is set that way
const int MODE_PIN = 4;            // pin to GND = normal fixed swing, open = AeroTrace

const int SIGN_PAN = 1;            // flip to -1 if the head turns the wrong way
const int SIGN_TILT = 1;           // flip to -1 if tilt is reversed
const int SIGN_COL = 1;            // flip to -1 if left/right detection is mirrored

const float DEG_PER_PX = 7.5;      // AMG8833: 60 degree field of view / 8 pixels
const float HOT_DELTA = 2.0;       // degrees C above background to count as a person
const int MIN_HOT_PIXELS = 2;      // ignore single-pixel noise
const float MARGIN = 10.0;         // degrees added each side of the detected people
const float PAN_LIMIT = 30.0;      // demo zone = sensor field of view
const float BEAM_HALF = 5.0;       // assumed half-width of the airflow on a person
const float SWEEP_DPS = 30.0;      // sweep speed, degrees per second
const unsigned long STANDBY_MS = 15000;
const unsigned long SCAN_MS = 120;

const int PWM_FREQ = 25000;        // 4-pin PC fans expect about 25 kHz
const int PWM_BITS = 8;
const int DUTY_FULL = 255;
const int DUTY_EMPTY = 180;        // about 70% speed over empty space

float pix[64];
float bg[64];
float theta = 0, aMin = 0, aMax = 0, tiltDeg = 0, tiltSm = 0;
int dir = 1;
bool everSeen = false, lastAdaptive = true;
bool sensorOk = true, needBg = false;
unsigned long lastLoop = 0, lastScan = 0, lastSeen = 0, lastPrint = 0;
float onTime = 0, totalTime = 0;

void learnBackground() {
  for (int i = 0; i < 64; i++) bg[i] = 0;
  const int N = 20;
  for (int k = 0; k < N; k++) {
    amg.readPixels(pix);
    for (int i = 0; i < 64; i++) bg[i] += pix[i] / N;
    delay(100);
  }
}

void scan() {
  Wire.beginTransmission(SENSOR_ADDR);
  if (Wire.endTransmission() != 0) {   // sensor not answering: fail-safe
    sensorOk = false;
    needBg = true;
    return;
  }
  sensorOk = true;
  if (needBg) {
    learnBackground();
    needBg = false;
  }
  amg.readPixels(pix);
  int cMin = 8, cMax = -1, n = 0;
  float rSum = 0;
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      int i = r * 8 + c;
      if (pix[i] - bg[i] > HOT_DELTA) {
        if (c < cMin) cMin = c;
        if (c > cMax) cMax = c;
        rSum += r;
        n++;
      }
    }
  }
  if (n >= MIN_HOT_PIXELS) {
    float a1 = SIGN_COL * (cMin - 4) * DEG_PER_PX;
    float a2 = SIGN_COL * (cMax + 1 - 4) * DEG_PER_PX;
    aMin = min(a1, a2);
    aMax = max(a1, a2);
    float elev = (3.5 - rSum / n) * DEG_PER_PX;
    tiltDeg = constrain(elev, -20, 20);
    lastSeen = millis();
    everSeen = true;
  } else {
    for (int i = 0; i < 64; i++) bg[i] = 0.98 * bg[i] + 0.02 * pix[i];
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MODE_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!amg.begin(SENSOR_ADDR)) {
    Serial.println("AMG8833 not found: running fail-safe normal sweep");
    sensorOk = false;
    needBg = true;
  }
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  ledcAttach(FAN_PWM_PIN, PWM_FREQ, PWM_BITS);   // ESP32 Arduino core 3.x
  panServo.write(90);
  tiltServo.write(90);
  ledcWrite(FAN_PWM_PIN, DUTY_FULL);
  Serial.println("Keep the area empty: learning background...");
  delay(2000);
  if (sensorOk) {
    learnBackground();
    needBg = false;
  }
  Serial.println("Ready");
  lastLoop = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  if (now - lastScan >= SCAN_MS) {
    lastScan = now;
    scan();
  }

  bool adaptive = digitalRead(MODE_PIN) == HIGH && sensorOk;
  if (adaptive != lastAdaptive) {
    lastAdaptive = adaptive;
    onTime = 0;
    totalTime = 0;
  }

  bool seen = everSeen && (now - lastSeen < STANDBY_MS);
  bool onPerson = seen && theta >= aMin - BEAM_HALF && theta <= aMax + BEAM_HALF;
  int duty;

  if (!adaptive) {
    theta += dir * SWEEP_DPS * dt;
    if (theta > PAN_LIMIT) { theta = PAN_LIMIT; dir = -1; }
    if (theta < -PAN_LIMIT) { theta = -PAN_LIMIT; dir = 1; }
    duty = DUTY_FULL;
  } else if (seen) {
    float lo = constrain(aMin - MARGIN, -PAN_LIMIT, PAN_LIMIT);
    float hi = constrain(aMax + MARGIN, -PAN_LIMIT, PAN_LIMIT);
    float rate = SWEEP_DPS * (onPerson ? 0.6 : 1.0);
    theta += dir * rate * dt;
    if (theta > hi) { theta = hi; dir = -1; }
    if (theta < lo) { theta = lo; dir = 1; }
    duty = onPerson ? DUTY_FULL : DUTY_EMPTY;
  } else {
    if (theta > 0.5) theta -= 20 * dt;
    else if (theta < -0.5) theta += 20 * dt;
    duty = 0;
  }

  float tiltTarget = (adaptive && seen) ? tiltDeg : 0;
  tiltSm += 0.1 * (tiltTarget - tiltSm);

  panServo.write(constrain(90 + SIGN_PAN * theta, 0, 180));
  tiltServo.write(constrain(90 + SIGN_TILT * tiltSm, 0, 180));
  ledcWrite(FAN_PWM_PIN, duty);

  if (seen) {
    totalTime += dt;
    if (onPerson) onTime += dt;
  }

  if (now - lastPrint > 2000) {
    lastPrint = now;
    Serial.printf("%s seen=%d arc=[%.0f,%.0f] theta=%.0f duty=%d onPerson=%.0f%%\n",
                  !sensorOk ? "FAILSAFE" : (adaptive ? "AERO" : "NORMAL"), seen, aMin, aMax, theta, duty,
                  totalTime > 0 ? 100.0 * onTime / totalTime : 0.0);
  }

  delay(10);
}
