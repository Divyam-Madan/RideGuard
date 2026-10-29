
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <math.h>

// =====================================================
//                    RIDEGUARD
//              MASTER CONTROLLER  v2
// =====================================================

// ---------------- PIN DEFINITIONS ----------------

#define MPU_ADDR       0x68

#define VIBRATION_PIN  27
#define IR_PIN         34

#define SERVO_PIN      18
#define BUZZER_PIN     25
#define STATUS_LED      2

#define ARM_BUTTON     32
#define DISARM_BUTTON  33

#define GPS_RX         16
#define GPS_TX         17

#define OLED_WIDTH     128
#define OLED_HEIGHT     64
#define OLED_ADDR      0x3C

// ---------------- WIFI ----------------

const char* ssid = "TestESP";
const char* password = "12345678";

IPAddress local_IP(10, 140, 37, 65);
IPAddress gateway(10, 140, 37, 210);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(10, 140, 37, 210);

WebServer server(80);

// ---------------- OBJECTS ----------------

Servo lockServo;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ---------------- SERVO ----------------

const int UNLOCK_ANGLE = 0;
const int LOCK_ANGLE   = 90;

// ---------------- TUNING (yahan se adjust kar) ----------------

const bool          SERIAL_DEBUG     = true;   // har 1 sec serial me sensor line
const unsigned long ARM_SETTLE_MS    = 4000;   // arm ke baad bike ko shaant hone ka time
const unsigned long HOLD_MS          = 400;    // detection kitni der "yaad" rahe (combos ke liye)
const int           VIB_MIN_EDGES    = 2;      // 100ms me itne pulses = vibration (kam = zyada sensitive)
const float         MOTION_THRESHOLD = 0.35;   // g
const float         STRONG_MOTION    = 0.70;   // g
const float         GYRO_THRESHOLD   = 150.0;  // deg/s
const float         STRONG_GYRO      = 300.0;  // deg/s
const unsigned long WARNING_SHOW_MS  = 2500;
const unsigned long OLED_SCREEN_TIME = 2500;
const int IR_ACTIVE_LEVEL = LOW;   // IR module: low = object detected

// ---------------- MPU VALUES ----------------

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float gxRaw = 0, gyRaw = 0, gzRaw = 0;
float gxOff = 0, gyOff = 0, gzOff = 0;
float tempC = 0;

float baseAx = 0, baseAy = 0, baseAz = 1.0;
float movementAmount = 0;

bool rawMotion = false;
bool rawGyro = false;

// ---------------- SENSOR STATES ----------------

bool vibrationDetected = false;
bool irDetected = false;
bool motionDetected = false;
bool gyroDetected = false;

bool oledOk = false;
bool mpuOk = false;

// vibration (interrupt based, polarity independent)
volatile uint32_t vibEdges = 0;
uint32_t vibTotal = 0;

// IR

uint8_t irCount = 0;

// hold timers
unsigned long vibHoldUntil = 0;
unsigned long motionHoldUntil = 0;
unsigned long gyroHoldUntil = 0;
unsigned long blankUntil = 0;   // beep/servo ke baad thodi der sensors ignore
unsigned long beepUntil = 0;

// ---------------- SECURITY STATES ----------------

bool armed = false;
bool arming = false;
bool theftAlert = false;
bool warningActive = false;
bool testAlarm = false;
bool selfTesting = false;

unsigned long armingStart = 0;
unsigned long warningStart = 0;
unsigned long lastButtonTime = 0;
unsigned long testAlarmStart = 0;

// ---------------- TIMERS ----------------

unsigned long lastSensorRead = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastScreenChange = 0;
unsigned long lastDebug = 0;
unsigned long lastWifiTry = 0;

int oledScreen = 0;

// ---------------- GPS ----------------

const long _a = 842743;
const long _c = 153900;
const float _e = 2.1f;

double refLat, refLng, refAlt;

const double LAT_DRIFT = 0.000075;
const double LNG_DRIFT = 0.000075;
const double ALT_DRIFT = 1.8;

double curLat, curLng, curAlt;
int curSats = 9;

bool gpsFix = false;
uint32_t gpsChars = 0;
unsigned long lastGPSUpdate = 0;

String lastEvent = "SYSTEM DISARMED";

// =====================================================
//                    HELPERS
// =====================================================

void IRAM_ATTR vibISR()
{
  vibEdges++;
}

String ipStr()
{
  if (WiFi.status() == WL_CONNECTED)
    return WiFi.localIP().toString();
  return String("NO WIFI");
}

void beep(unsigned long ms)
{
  beepUntil = millis() + ms;
  blankUntil = millis() + ms + 350;
}

int16_t read16()
{
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// =====================================================
//                    GPS
// =====================================================

void loadReferencePoint()
{
  refLat = 12.0 + ((double)_a / 1000000.0);
  refLng = 80.0 + ((double)_c / 1000000.0);
  refAlt = (double)_e * 10.0;

  curLat = refLat;
  curLng = refLng;
  curAlt = refAlt;
}

void feedGPS()
{
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());
}

void updateGPS()
{
  feedGPS();

  gpsChars = gps.charsProcessed();

  gpsFix = gps.location.isValid() && gps.location.age() < 5000;

  // Real GPS fix mila to wahi use karo
  if (gpsFix)
  {
    curLat = gps.location.lat();
    curLng = gps.location.lng();

    if (gps.altitude.isValid())
      curAlt = gps.altitude.meters();

    if (gps.satellites.isValid())
      curSats = gps.satellites.value();

    return;
  }

  // Fix nahi hai -> demo coordinates (UI pe DEMO DATA likha aayega)
  if (millis() - lastGPSUpdate < 2000)
    return;

  lastGPSUpdate = millis();

  if (curLat > refLat + LAT_DRIFT) curLat = refLat + LAT_DRIFT;
  if (curLat < refLat - LAT_DRIFT) curLat = refLat - LAT_DRIFT;

  if (curLng > refLng + LNG_DRIFT) curLng = refLng + LNG_DRIFT;
  if (curLng < refLng - LNG_DRIFT) curLng = refLng - LNG_DRIFT;

  if (curAlt > refAlt + ALT_DRIFT) curAlt = refAlt + ALT_DRIFT;
  if (curAlt < refAlt - ALT_DRIFT) curAlt = refAlt - ALT_DRIFT;

  curSats = 1;
}

// =====================================================
//                    MPU6050
// =====================================================

bool readMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0)
  {
    mpuOk = false;
    return false;
  }

  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() != 14)
  {
    mpuOk = false;
    return false;
  }

  int16_t AcX = read16();
  int16_t AcY = read16();
  int16_t AcZ = read16();
  int16_t Tmp = read16();
  int16_t GyX = read16();
  int16_t GyY = read16();
  int16_t GyZ = read16();

  mpuOk = true;

  ax = AcX / 16384.0;
  ay = AcY / 16384.0;
  az = AcZ / 16384.0;

  tempC = Tmp / 340.0 + 36.53;

  gxRaw = GyX / 131.0;
  gyRaw = GyY / 131.0;
  gzRaw = GyZ / 131.0;

  gx = gxRaw - gxOff;
  gy = gyRaw - gyOff;
  gz = gzRaw - gzOff;

  float dx = ax - baseAx;
  float dy = ay - baseAy;
  float dz = az - baseAz;

  movementAmount = sqrt(dx * dx + dy * dy + dz * dz);

  rawMotion = movementAmount >= MOTION_THRESHOLD;

  rawGyro = (fabs(gx) >= GYRO_THRESHOLD) ||
            (fabs(gy) >= GYRO_THRESHOLD) ||
            (fabs(gz) >= GYRO_THRESHOLD);

  return true;
}

void calibrateMPU(int samples, int delayMs)
{
  float sx = 0, sy = 0, sz = 0;
  float sgx = 0, sgy = 0, sgz = 0;
  int count = 0;

  Serial.println();
  Serial.println("Calibrating MPU6050... keep RideGuard STILL");

  for (int i = 0; i < samples; i++)
  {
    if (readMPU())
    {
      sx += ax;   sy += ay;   sz += az;
      sgx += gxRaw; sgy += gyRaw; sgz += gzRaw;
      count++;
    }

    delay(delayMs);
  }

  if (count == 0)
  {
    Serial.println("MPU calibration FAILED (no data) - check SDA/SCL wiring");
    return;
  }

  baseAx = sx / count;
  baseAy = sy / count;
  baseAz = sz / count;

  gxOff = sgx / count;
  gyOff = sgy / count;
  gzOff = sgz / count;

  Serial.print("Base  X/Y/Z: ");
  Serial.print(baseAx, 3); Serial.print(" / ");
  Serial.print(baseAy, 3); Serial.print(" / ");
  Serial.println(baseAz, 3);

  Serial.print("GyroOff X/Y/Z: ");
  Serial.print(gxOff, 2); Serial.print(" / ");
  Serial.print(gyOff, 2); Serial.print(" / ");
  Serial.println(gzOff, 2);
}

// =====================================================
//                    READ SENSORS
// =====================================================

void readSensors()
{
  unsigned long now = millis();

  readMPU();

  bool blanked = now < blankUntil;

  // ---- vibration: edges count karo (polarity se farak nahi) ----
  uint32_t e = vibEdges;
  vibEdges -= e;
  vibTotal += e;

  if (blanked) e = 0;

  if (e >= (uint32_t)VIB_MIN_EDGES)
    vibHoldUntil = now + HOLD_MS;

  vibrationDetected = now < vibHoldUntil;

  // ---- motion / gyro with hold ----
  if (!blanked && mpuOk)
  {
    if (rawMotion) motionHoldUntil = now + HOLD_MS;
    if (rawGyro)   gyroHoldUntil   = now + HOLD_MS;
  }

  motionDetected = now < motionHoldUntil;
  gyroDetected   = now < gyroHoldUntil;

  // ---- IR: fixed polarity ----
  bool irRaw = (digitalRead(IR_PIN) == IR_ACTIVE_LEVEL);

  if (irRaw)
  {
    if (irCount < 255) irCount++;
  }
  else
  {
    irCount = 0;
  }

  irDetected = irCount >= 2;
}

// =====================================================
//                    LOCK / UNLOCK
// =====================================================

void lockBike()
{
  lockServo.write(LOCK_ANGLE);
  Serial.println("LOCK: LOCKED");
}

void unlockBike()
{
  lockServo.write(UNLOCK_ANGLE);
  Serial.println("LOCK: UNLOCKED");
}

// =====================================================
//                    ARM / DISARM
// =====================================================

void armSystem()
{
  // already armed (ya arming chal rahi) aur theft nahi -> ignore
  if (armed && !theftAlert)
    return;

  armed = true;
  arming = true;
  armingStart = millis();

  theftAlert = false;
  warningActive = false;
  testAlarm = false;

  lockBike();
  beep(150);

  blankUntil = millis() + ARM_SETTLE_MS;

  lastEvent = "ARMING - KEEP BIKE STILL";

  Serial.println();
  Serial.println("==============================");
  Serial.println("   ARMING... keep bike still");
  Serial.println("==============================");
}

void finishArming()
{
  // servo/buzzer shaant, ab fresh baseline lo
  calibrateMPU(60, 5);

  vibEdges = 0;
  vibHoldUntil = 0;
  motionHoldUntil = 0;
  gyroHoldUntil = 0;

  arming = false;
  lastEvent = "SYSTEM ARMED";

   Serial.println(digitalRead(IR_PIN) == IR_ACTIVE_LEVEL ? "IR: OBJECT in front at arm time!" : "IR: clear");

  Serial.println("==============================");
  Serial.println("       SYSTEM ARMED");
  Serial.println("==============================");
}

void updateArming()
{
  if (!arming)
    return;

  if (millis() - armingStart < ARM_SETTLE_MS)
    return;

  finishArming();
}

void disarmSystem()
{
  armed = false;
  arming = false;

  theftAlert = false;
  warningActive = false;
  testAlarm = false;

  unlockBike();
  beep(80);

  lastEvent = "SYSTEM DISARMED";

  Serial.println();
  Serial.println("==============================");
  Serial.println("       SYSTEM DISARMED");
  Serial.println("==============================");
}

// =====================================================
//                    THEFT / WARNING
// =====================================================

void triggerTheft(String reason)
{
  if (!armed || arming) return;
  if (theftAlert) return;

  theftAlert = true;
  warningActive = false;
  lastEvent = reason;

  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("       THEFT DETECTED");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.print("Reason: ");
  Serial.println(reason);
}

void triggerWarning(String reason)
{
  if (!armed || arming) return;
  if (theftAlert) return;
  if (warningActive) return;

  warningActive = true;
  warningStart = millis();
  lastEvent = "WARNING: " + reason;

  beep(120);

  Serial.print("WARNING: ");
  Serial.println(reason);
}

void securityLogic()
{
  if (!armed || arming)
    return;

  // ---- strong confirmation (combo) ----

  if (vibrationDetected && motionDetected)
  {
    triggerTheft("VIBRATION + MOTION");
    return;
  }

  if (vibrationDetected && gyroDetected)
  {
    triggerTheft("VIBRATION + GYRO");
    return;
  }

  if (irDetected && vibrationDetected)
  {
    triggerTheft("IR + VIBRATION");
    return;
  }

  if (motionDetected && gyroDetected)
  {
    triggerTheft("MOTION + GYRO");
    return;
  }

  // ---- extreme single sensor ----

  if (mpuOk && movementAmount >= STRONG_MOTION)
  {
    triggerTheft("STRONG MOTION");
    return;
  }

  if (mpuOk &&
      (fabs(gx) >= STRONG_GYRO ||
       fabs(gy) >= STRONG_GYRO ||
       fabs(gz) >= STRONG_GYRO))
  {
    triggerTheft("STRONG ROTATION");
    return;
  }

  // ---- warnings ----

  if (vibrationDetected)
    triggerWarning("VIBRATION");
  else if (motionDetected)
    triggerWarning("MOTION");
  else if (irDetected)
    triggerWarning("IR OBJECT");

  // ---- clear warning ----

  if (warningActive && millis() - warningStart > WARNING_SHOW_MS)
  {
    warningActive = false;
  }
}

// =====================================================
//                    TEST ALARM
// =====================================================

void activateTestAlarm()
{
  testAlarm = true;
  testAlarmStart = millis();
  lastEvent = "TEST ALARM";
  blankUntil = millis() + 5500;
}

void updateTestAlarm()
{
  if (!testAlarm)
    return;

  if (millis() - testAlarmStart >= 5000)
  {
    testAlarm = false;
    lastEvent = "TEST ALARM COMPLETE";
  }
}

// =====================================================
//                    BUTTONS
// =====================================================

void checkButtons()
{
  if (millis() - lastButtonTime < 300)
    return;

  if (digitalRead(ARM_BUTTON) == LOW)
  {
    lastButtonTime = millis();
    armSystem();
  }

  if (digitalRead(DISARM_BUTTON) == LOW)
  {
    lastButtonTime = millis();
    disarmSystem();
  }
}

// =====================================================
//                    OLED
// =====================================================

void oledHeader(String title)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.println("---------------------");
}

// Har screen ke neeche website IP
void oledFooter()
{
  display.setCursor(0, 56);
  display.print(ipStr());
}

void showSystemScreen()
{
  if (!oledOk) return;

  oledHeader("RIDEGUARD");

  display.print("STATUS: ");

  if (theftAlert)       display.println("THEFT!");
  else if (arming)      display.println("ARMING..");
  else if (armed)       display.println("ARMED");
  else                  display.println("DISARMED");

  display.print("LOCK: ");
  display.println(armed ? "LOCKED" : "UNLOCKED");

  display.print("ALARM: ");

  if (theftAlert || testAlarm) display.println("ON");
  else if (warningActive)      display.println("WARN");
  else                         display.println("SILENT");

  display.print("WIFI: ");
  display.println(WiFi.status() == WL_CONNECTED ? "OK" : "OFF");

  oledFooter();
  display.display();
}

void showSensorScreen()
{
  if (!oledOk) return;

  oledHeader("SENSORS");

  display.print("AX:");  display.print(ax, 2);
  display.print(" AY:"); display.println(ay, 2);

  display.print("AZ:");  display.println(az, 2);

  display.print("GX:");  display.print(gx, 0);
  display.print(" GY:"); display.println(gy, 0);

  display.print("GZ:");  display.println(gz, 0);

  display.print("VIB:");
  display.print(vibrationDetected ? "YES " : "OK  ");

  display.print("IR:");
  display.println(irDetected ? "OBJ" : "CLR");

  oledFooter();
  display.display();
}

void showSecurityScreen()
{
  if (!oledOk) return;

  oledHeader("SECURITY");

  display.print("MOTION: ");
  display.println(motionDetected ? "YES" : "NORMAL");

  display.print("GYRO: ");
  display.println(gyroDetected ? "HIGH" : "NORMAL");

  display.print("MOVE: ");
  display.print(movementAmount, 2);
  display.println("g");

  display.print("VIB: ");
  display.println(vibrationDetected ? "YES" : "NORMAL");

  display.print("IR: ");
  display.println(irDetected ? "OBJECT" : "CLEAR");

  oledFooter();
  display.display();
}

void showGPSScreen()
{
  if (!oledOk) return;

  oledHeader(gpsFix ? "GPS (LIVE)" : "GPS (DEMO)");

  display.print("SAT: ");
  display.println(curSats);

  display.print("LAT:");
  display.println(curLat, 4);

  display.print("LON:");
  display.println(curLng, 4);

  display.print("ALT:");
  display.print(curAlt, 1);
  display.println("m");

  oledFooter();
  display.display();
}

void showIPScreen()
{
  if (!oledOk) return;

  oledHeader("WEB SERVER");

  display.println("OPEN IN BROWSER:");
  display.println();

  display.print("http://");
  display.println(ipStr());

  display.println();

  display.print("WIFI: ");

  if (WiFi.status() == WL_CONNECTED)
  {
    display.print("OK ");
    display.print(WiFi.RSSI());
    display.println("dBm");
  }
  else
  {
    display.println("OFF");
  }

  display.display();
}

void showAlertScreen()
{
  if (!oledOk) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println("!!! SECURITY !!!");
  display.println("---------------------");
  display.println("THEFT DETECTED!");
  display.println(lastEvent);

  display.print("VIB:");
  display.print(vibrationDetected ? "Y" : "N");
  display.print(" MOV:");
  display.println(motionDetected ? "Y" : "N");

  display.print("GYR:");
  display.print(gyroDetected ? "Y" : "N");
  display.print(" IR:");
  display.println(irDetected ? "Y" : "N");

  oledFooter();
  display.display();
}

void updateOLED()
{
  if (!oledOk || selfTesting)
    return;

  if (theftAlert)
  {
    showAlertScreen();
    return;
  }

  // screen badalna har 2.5 sec, par redraw har 0.5 sec (live data)
  if (millis() - lastScreenChange >= OLED_SCREEN_TIME)
  {
    lastScreenChange = millis();
    oledScreen++;
    if (oledScreen > 4) oledScreen = 0;
  }

  switch (oledScreen)
  {
    case 0: showSystemScreen();   break;
    case 1: showSensorScreen();   break;
    case 2: showSecurityScreen(); break;
    case 3: showGPSScreen();      break;
    case 4: showIPScreen();       break;
  }
}

// =====================================================
//                    SELF TEST
// =====================================================

void oledMsg(String l1, String l2, String l3)
{
  if (!oledOk) return;

  oledHeader("SELF TEST");
  display.println(l1);
  display.println(l2);
  display.println(l3);
  oledFooter();
  display.display();
}

String runSelfTest()
{
  if (armed)
    return "DISARM FIRST - self test sirf disarmed state me chalta hai";

  selfTesting = true;

  String r = "";

  Serial.println();
  Serial.println("========== SELF TEST ==========");

  // ---- MPU ----
  bool m = readMPU();

  if (m)
  {
    float mag = sqrt(ax * ax + ay * ay + az * az);
    bool magOk = (mag > 0.8 && mag < 1.2);

    r += "MPU6050   : OK  (|g| = " + String(mag, 2) + (magOk ? ", PASS)\n" : ", odd - expected ~1.00)\n");
    r += "MPU TEMP  : " + String(tempC, 1) + " C\n";
  }
  else
  {
    r += "MPU6050   : FAIL - check SDA(21)/SCL(22)/VCC/GND\n";
  }

  // ---- OLED ----
  r += oledOk ? "OLED      : OK  (0x3C)\n" : "OLED      : FAIL - check I2C wiring/address\n";

  // ---- actuators ----
  oledMsg("LED + BUZZER", "SERVO LOCK/UNLOCK", "");

  digitalWrite(STATUS_LED, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(250);
  digitalWrite(BUZZER_PIN, LOW);

  lockBike();
  delay(700);
  unlockBike();
  delay(700);

  digitalWrite(STATUS_LED, LOW);

  r += "LED/BUZZER : fired (dekh/sun ke confirm kar)\n";
  r += "SERVO      : moved LOCK -> UNLOCK (dekh ke confirm kar)\n";

  // ---- interactive window ----
  oledMsg("TAP THE BIKE NOW", "WAVE HAND ON IR", "5 seconds...");

  Serial.println("TAP the bike and WAVE a hand over the IR sensor (5 sec)");

  delay(300);

  vibEdges = 0;

  float peakMove = 0;
  float peakGyro = 0;

  int irStart = digitalRead(IR_PIN);
  bool irChanged = false;

  unsigned long t0 = millis();

  while (millis() - t0 < 5000)
  {
    if (readMPU())
    {
      if (movementAmount > peakMove) peakMove = movementAmount;

      float g = max(fabs(gx), max(fabs(gy), fabs(gz)));
      if (g > peakGyro) peakGyro = g;
    }

    if (digitalRead(IR_PIN) != irStart)
      irChanged = true;

    feedGPS();
    delay(20);
  }

  uint32_t edges = vibEdges;
  vibEdges = 0;

  r += edges > 0
       ? "SW-420 VIB : OK  (" + String(edges) + " pulses)\n"
       : "SW-420 VIB : NO RESPONSE - zyada zor se tap kar / pot ghuma / wiring check\n";

  if (m)
  {
    r += peakMove >= MOTION_THRESHOLD
         ? "MPU MOTION : OK  (peak " + String(peakMove, 2) + " g)\n"
         : "MPU MOTION : low (peak " + String(peakMove, 2) + " g) - bike ko hila ke dekh\n";

    r += peakGyro >= GYRO_THRESHOLD
         ? "MPU GYRO   : OK  (peak " + String(peakGyro, 0) + " deg/s)\n"
         : "MPU GYRO   : low (peak " + String(peakGyro, 0) + " deg/s) - bike ghuma ke dekh\n";
  }

  r += irChanged
       ? "IR SENSOR  : OK  (output badla)\n"
       : "IR SENSOR  : NO CHANGE - haath paas laa / pot adjust / wiring check\n";

  // ---- GPS ----
  feedGPS();
  gpsChars = gps.charsProcessed();

  if (gpsChars > 10)
    r += "GPS NEO-6M : NMEA data aa raha hai (" + String(gpsChars) + " chars)" +
         (gpsFix ? ", FIX mil gaya\n" : ", fix nahi (indoors normal hai)\n");
  else
    r += "GPS NEO-6M : koi data nahi - TX->GPIO16, RX->GPIO17, VCC, GND check kar\n";

  // ---- WiFi ----
  r += (WiFi.status() == WL_CONNECTED)
       ? "WIFI       : OK  (" + String(WiFi.RSSI()) + " dBm)\n"
       : "WIFI       : DISCONNECTED\n";

  Serial.println(r);
  Serial.println("===============================");

  // cleanup
  vibEdges = 0;
  vibHoldUntil = 0;
  motionHoldUntil = 0;
  gyroHoldUntil = 0;
  irCount = 0;
  blankUntil = millis() + 500;

  lastEvent = "SELF TEST DONE";
  selfTesting = false;

  return r;
}

// =====================================================
//                    WEB PAGE
// =====================================================

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RideGuard</title>
<style>
*{box-sizing:border-box}
:root{--c:#8491a7}
body.st-armed{--c:#00f5a0}
body.st-arming{--c:#ffd166}
body.st-theft{--c:#ff4d6d}
body{margin:0;font-family:Arial,Helvetica,sans-serif;background:radial-gradient(circle at top,#172554,#05070d 65%) fixed;color:#fff;padding:16px}
.wrap{max-width:720px;margin:auto}
.header{text-align:center;padding:14px 0 4px}
.logo{font-size:36px;font-weight:900;letter-spacing:6px}
.sub{color:#8491a7;font-size:11px;letter-spacing:3px;margin-top:6px}
.card{background:rgba(16,22,35,.92);border:1px solid #26344d;border-radius:20px;padding:18px;margin-bottom:14px;box-shadow:0 15px 35px rgba(0,0,0,.35)}
.card h3{margin:0 0 12px;font-size:13px;letter-spacing:2px;color:#9fb0cc}
.hero{text-align:center}
.status{font-size:26px;font-weight:bold;margin:6px 0;color:var(--c)}
.event{color:#738099;font-size:12px;min-height:16px}
.ring{width:230px;height:230px;margin:14px auto;border-radius:50%;display:flex;flex-direction:column;align-items:center;justify-content:center;border:3px solid var(--c);background:radial-gradient(circle,#0b101b,#05070d);box-shadow:0 0 25px var(--c),inset 0 0 25px rgba(255,255,255,.04)}
body.st-armed .ring{animation:pulse 2.4s infinite}
body.st-theft .ring{animation:pulse .5s infinite}
body.st-arming .ring{animation:pulse 1s infinite}
@keyframes pulse{50%{box-shadow:0 0 5px var(--c),inset 0 0 10px rgba(255,255,255,.02)}}
.bike{color:var(--c);width:170px}
.ringtxt{font-size:11px;letter-spacing:4px;color:var(--c);margin-top:4px;font-weight:bold}
.spin{animation:none}
body.st-armed .spin{animation:rot 14s linear infinite}
body.st-arming .spin{animation:rot 2s linear infinite}
body.st-theft .spin{animation:rot .6s linear infinite}
@keyframes rot{to{transform:rotate(360deg)}}
.chips{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:10px 0}
.chip{background:#0b101b;border:1px solid #202d43;border-radius:14px;padding:12px;font-weight:bold;font-size:15px}
.buttons{display:grid;grid-template-columns:1fr 1fr;gap:10px}
button{border:0;border-radius:14px;padding:15px;font-size:15px;font-weight:bold;cursor:pointer}
button:disabled{opacity:.5}
.arm{background:#00f5a0;color:#03100b}
.disarm{background:#ff4d6d;color:#fff}
.test{width:100%;margin-top:10px;background:#ffd166;color:#151000}
.st{width:100%;margin-top:10px;background:#1e2b45;color:#cfe0ff;border:1px solid #2f4268}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.sensor{background:#0b101b;border:1px solid #202d43;border-radius:15px;padding:14px}
.label{color:#7d8ba2;font-size:11px;letter-spacing:1px;margin-bottom:6px}
.val{font-size:17px;font-weight:bold}
.mono{font-family:Consolas,monospace;font-size:15px;line-height:1.55}
.ok{color:#00f5a0}.bad{color:#ff4d6d}
.bar{height:8px;border-radius:6px;background:#162033;margin-top:8px;overflow:hidden}
.bar i{display:block;height:100%;width:0;background:linear-gradient(90deg,#00f5a0,#ffd166,#ff4d6d);transition:width .3s}
.map{display:block;text-align:center;padding:13px;background:#172554;border-radius:12px;color:#fff;text-decoration:none;margin-top:12px;font-weight:bold}
.tag{display:inline-block;font-size:10px;letter-spacing:1px;padding:3px 8px;border-radius:8px;background:#162033;color:#9fb0cc;margin-left:6px}
pre{white-space:pre-wrap;font-size:12px;color:#b8c6e0;margin:10px 0 0;font-family:Consolas,monospace;line-height:1.5}
table{width:100%;border-collapse:collapse;font-size:13px}
td,th{padding:8px 6px;border-bottom:1px solid #202d43;text-align:left;vertical-align:top}
th{color:#7d8ba2;font-size:11px;letter-spacing:1px}
td b{color:#cfe0ff}
.small{color:#8491a7;font-size:12px;line-height:1.6}
.flow{display:grid;gap:8px}
.flow div{background:#0b101b;border:1px solid #202d43;border-radius:12px;padding:10px;font-size:13px}
.flow .t{border-color:#ff4d6d;color:#ff9db0}
.flow .w{border-color:#ffd166;color:#ffe29a}
hr{border:0;border-top:1px solid #26344d;margin:22px 0}
.foot{text-align:center;color:#55627a;font-size:11px;padding:10px 0 30px}
</style>
</head>
<body class="st-disarmed">
<div class="wrap">

<div class="header">
 <div class="logo">RIDEGUARD</div>
 <div class="sub">SMART BICYCLE SECURITY SYSTEM</div>
</div>

<div class="card hero">
 <div class="ring">
  <svg class="bike" viewBox="0 0 200 120" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round">
   <g class="spin" style="transform-origin:45px 82px"><circle cx="45" cy="82" r="30"/><path d="M45 52V112M15 82H75M24 61L66 103M66 61L24 103" stroke-width="1" opacity=".6"/></g>
   <g class="spin" style="transform-origin:155px 82px"><circle cx="155" cy="82" r="30"/><path d="M155 52V112M125 82H185M134 61L176 103M176 61L134 103" stroke-width="1" opacity=".6"/></g>
   <path d="M45 82L80 48L138 50L95 82Z M80 48L95 82 M138 50L155 82 M72 44H90 M130 40L146 42 M138 50L136 40"/>
   <circle cx="95" cy="82" r="5"/>
  </svg>
  <div class="ringtxt">RIDEGUARD</div>
 </div>
 <div id="status" class="status">CONNECTING...</div>
 <div id="event" class="event">Loading...</div>

 <div class="chips">
  <div class="chip" id="lock">--</div>
  <div class="chip" id="alarm">--</div>
 </div>

 <div class="buttons">
  <button class="arm" onclick="ctl('/arm')">🔒 ARM</button>
  <button class="disarm" onclick="ctl('/disarm')">🔓 DISARM</button>
 </div>
 <button class="test" onclick="ctl('/test')">🚨 TEST ALARM</button>
 <button class="st" id="stbtn" onclick="selfTest()">🧪 SELF TEST ALL SENSORS</button>
 <pre id="stout"></pre>
</div>

<div class="grid" style="margin-bottom:14px">
 <div class="card" style="margin:0">
  <h3>MPU6050 · ACCEL</h3>
  <div class="mono">X <span id="ax">--</span><br>Y <span id="ay">--</span><br>Z <span id="az">--</span></div>
  <div class="label" style="margin-top:8px">MOVEMENT</div>
  <div class="val" id="mv">--</div>
  <div class="bar"><i id="mvbar"></i></div>
 </div>
 <div class="card" style="margin:0">
  <h3>VIBRATION · SW-420</h3>
  <div class="val" id="vib" style="font-size:20px;margin-top:14px">--</div>
  <div class="label" style="margin-top:14px">MOTION</div>
  <div class="val" id="mot">--</div>
 </div>
 <div class="card" style="margin:0">
  <h3>GYROSCOPE</h3>
  <div class="mono">GX <span id="gx">--</span><br>GY <span id="gy">--</span><br>GZ <span id="gz">--</span></div>
  <div class="label" style="margin-top:8px">GYRO STATE</div>
  <div class="val" id="gyr">--</div>
 </div>
 <div class="card" style="margin:0">
  <h3>IR SENSOR</h3>
  <div class="val" id="ir" style="font-size:20px;margin-top:14px">--</div>
  <div class="label" style="margin-top:14px">MPU TEMP</div>
  <div class="val" id="tmp">--</div>
 </div>
</div>

<div class="card">
 <h3>📍 GPS LOCATION <span class="tag" id="gsrc">--</span></h3>
 <div class="grid">
  <div class="sensor"><div class="label">SATELLITES</div><div class="val" id="sat">--</div></div>
  <div class="sensor"><div class="label">ALTITUDE</div><div class="val" id="alt">--</div></div>
  <div class="sensor"><div class="label">LATITUDE</div><div class="val" id="lat">--</div></div>
  <div class="sensor"><div class="label">LONGITUDE</div><div class="val" id="lng">--</div></div>
 </div>
 <a id="map" class="map" target="_blank">📍 VIEW ON MAP</a>
</div>

<div class="card">
 <h3>SYSTEM HEALTH</h3>
 <div class="grid">
  <div class="sensor"><div class="label">MPU6050</div><div class="val" id="h_mpu">--</div></div>
  <div class="sensor"><div class="label">OLED</div><div class="val" id="h_oled">--</div></div>
  <div class="sensor"><div class="label">GPS MODULE</div><div class="val" id="h_gps">--</div></div>
  <div class="sensor"><div class="label">WIFI SIGNAL</div><div class="val" id="h_wifi">--</div></div>
  <div class="sensor"><div class="label">VIB PULSES (TOTAL)</div><div class="val" id="h_vib">--</div></div>
  <div class="sensor"><div class="label">UPTIME</div><div class="val" id="h_up">--</div></div>
 </div>
</div>

<hr>

<div class="card">
 <h3>RIDEGUARD HARDWARE ARCHITECTURE</h3>
 <table>
  <tr><th>COMPONENT</th><th>PIN</th><th>ROLE</th></tr>
  <tr><td><b>ESP32 DevKit</b><br>(ESP32 Dev Module)</td><td>—</td><td>Main security controller + Wi-Fi web server</td></tr>
  <tr><td><b>MPU6050</b></td><td>SDA → GPIO 21<br>SCL → GPIO 22</td><td>Accelerometer + Gyroscope (I2C 0x68)</td></tr>
  <tr><td><b>SW-420</b></td><td>GPIO 27 (digital out)</td><td>Vibration detection (interrupt, edge counting)</td></tr>
  <tr><td><b>IR Obstacle Sensor</b></td><td>GPIO 34 (digital out)</td><td>Object / proximity detection</td></tr>
  <tr><td><b>SG90 Servo</b></td><td>GPIO 18</td><td>Electronic locking mechanism</td></tr>
  <tr><td><b>Buzzer</b></td><td>GPIO 25</td><td>Audible security alarm</td></tr>
  <tr><td><b>Status LED</b></td><td>GPIO 2</td><td>System / alarm indication</td></tr>
  <tr><td><b>ARM Button</b></td><td>GPIO 32</td><td>Arm system (INPUT_PULLUP)</td></tr>
  <tr><td><b>DISARM Button</b></td><td>GPIO 33</td><td>Disarm system (INPUT_PULLUP)</td></tr>
  <tr><td><b>NEO-6M GPS</b></td><td>TX → GPIO 16<br>RX → GPIO 17</td><td>Location (UART2, 9600 baud)</td></tr>
  <tr><td><b>OLED SSD1306</b></td><td>SDA → GPIO 21<br>SCL → GPIO 22</td><td>Local display (I2C 0x3C), shows website IP</td></tr>
 </table>
</div>

<div class="card">
 <h3>SECURITY LOGIC</h3>
 <div class="flow">
  <div class="w">VIBRATION alone → ⚠️ WARNING (short beep)</div>
  <div class="w">MOTION alone → ⚠️ WARNING</div>
  <div class="w">IR OBJECT alone → ⚠️ WARNING</div>
  <div class="t">VIBRATION + MOTION → 🚨 THEFT DETECTED</div>
  <div class="t">VIBRATION + GYRO → 🚨 THEFT DETECTED</div>
  <div class="t">IR + VIBRATION → 🚨 THEFT DETECTED</div>
  <div class="t">MOTION + GYRO → 🚨 THEFT DETECTED</div>
  <div class="t">STRONG MOTION / STRONG ROTATION → 🚨 THEFT DETECTED</div>
 </div>
 <p class="small">When the **ARM button** is pressed, the servo locks the system, and a **4-second “ARMING” phase** begins to allow the bike to stabilize. After this period, a fresh **MPU6050 baseline** is recorded. Only after the baseline calibration is completed do the sensors begin evaluating theft-related activity. This prevents vibrations caused by the servo or buzzer during the arming process from triggering false alarms.</p>
 <p> ### 1. MPU6050 Accelerometer – Motion Detection

The MPU6050 accelerometer is used to detect changes in the device's acceleration relative to its calibrated reference position.

A **Motion Threshold of 0.35 g** is defined for detecting significant movement.

- If the change in acceleration is **less than 0.35 g**, the system considers the device to be in a **NORMAL** state.
- If the change in acceleration is **greater than or equal to 0.35 g**, the system considers the condition as **MOTION DETECTED**.

The threshold is applied to the deviation from the calibrated baseline acceleration rather than the absolute acceleration value.

### 2. MPU6050 Gyroscope – Rotational Movement Detection

The gyroscope component of the MPU6050 is used to detect significant rotational movement along the three axes.

A **Gyroscope Threshold of 150 °/s** is defined.

Rotational movement is considered significant when any one of the following conditions is satisfied:

- |GX| ≥ 150 °/s
- |GY| ≥ 150 °/s
- |GZ| ≥ 150 °/s

If any axis exceeds the specified threshold, the system identifies the condition as **STRONG ROTATIONAL MOVEMENT**.

### 3. SW-420 – Vibration Detection

The SW-420 vibration sensor module provides a **digital output** rather than a continuous numerical vibration measurement.

The sensor operates according to the following logic:

- **LOW** → Vibration detected
- **HIGH** → No vibration detected / Normal condition

The sensitivity of the SW-420 module is adjusted physically using the onboard potentiometer. Therefore, the threshold for triggering the digital output can be modified through hardware adjustment.

The system interprets a LOW digital signal from the SW-420 as a **VIBRATION DETECTED** condition.

### 4. IR Sensor – Object Detection

The IR obstacle sensor is used to detect the presence or absence of an object within its sensing range.

The sensor provides a digital output with the following logic:

- **LOW** → Object detected
- **HIGH** → No object detected / Clear

Accordingly, a LOW output is interpreted by the system as an **OBJECT DETECTED** condition, while a HIGH output represents a **CLEAR** condition. </p>
</div>

<div class="card">
 <h3>HOW WE BUILT IT</h3>
 <p class="small">
 <b>1. Sensing:</b> MPU6050 I2C par har 100 ms padha jata hai. Accelerometer ka baseline se farak "movement" (g) banata hai, gyroscope deg/s deta hai (offset calibrated). SW-420 ka output interrupt se pulses count karta hai, isliye module ki polarity se farak nahi padta. IR sensor ka output LOW aate hi object detect hota hai.<br><br>
 <b>2. Decision:</b> Har sensor ka flag ~400 ms tak "hold" hota hai taaki alag-alag sensors ke events combine ho sakein (jaise vibration + motion = theft).<br><br>
 <b>3. Action:</b> Theft par buzzer + LED + OLED alert, servo lock ON. Warning par sirf short beep.<br><br>
 <b>4. Dashboard:</b> ESP32 par chal raha WebServer yeh page (<code>/</code>) deta hai aur browser har 1 sec <code>/api</code> se JSON le kar sab live update karta hai. Controls: <code>/arm</code>, <code>/disarm</code>, <code>/test</code>, <code>/selftest</code>.<br><br>
 <b>5. GPS:</b> NEO-6M ka NMEA TinyGPS++ parse karta hai. Fix na mile (indoors) to demo coordinates dikhte hain, aur upar tag me "DEMO DATA" aata hai.
 </p>
</div>

<div class="card">
 <h3>NETWORK</h3>
 <table>
  <tr><td>Wi-Fi SSID</td><td><b id="n_ssid">--</b></td></tr>
  <tr><td>ESP32 IP</td><td><b id="n_ip">--</b></td></tr>
  <tr><td>Open in phone</td><td><b id="n_url">--</b></td></tr>
  <tr><td>Endpoints</td><td>/ · /api · /arm · /disarm · /test · /selftest</td></tr>
 </table>
</div>

<div class="foot">RIDEGUARD · ESP32 · Smart Bicycle Security</div>
</div>

<script>
const $=id=>document.getElementById(id);
function t(id,v){$(id).textContent=v;}
function pill(id,bad,badTxt,okTxt){
 const e=$(id);
 e.textContent=(bad?'🔴 ':'🟢 ')+(bad?badTxt:okTxt);
 e.className='val '+(bad?'bad':'ok');
}
function okpill(id,good,goodTxt,badTxt){
 const e=$(id);
 e.textContent=(good?'🟢 ':'🔴 ')+(good?goodTxt:badTxt);
 e.className='val '+(good?'ok':'bad');
}
function ctl(u){fetch(u).then(update).catch(()=>{});}
function selfTest(){
 const b=$('stbtn');
 b.disabled=true;
 t('stout','Running self-test... buzzer/servo chalega, phir 5 sec me bike thapthapao aur IR ke aage haath hilao.');
 fetch('/selftest').then(r=>r.text()).then(x=>{t('stout',x);b.disabled=false;}).catch(()=>{t('stout','Self-test response nahi aaya');b.disabled=false;});
}
function fmtUp(s){
 const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=s%60;
 return h+'h '+m+'m '+x+'s';
}
function update(){
 fetch('/api').then(r=>r.json()).then(d=>{
  let s='disarmed',label='⚪ SYSTEM DISARMED';
  if(d.theft){s='theft';label='🚨 THEFT DETECTED';}
  else if(d.arming){s='arming';label='🟡 ARMING... KEEP STILL';}
  else if(d.armed){s='armed';label='🟢 SYSTEM ARMED';}
  document.body.className='st-'+s;
  t('status',label);
  t('event',d.event);
  t('lock',d.lock?'🔒 LOCKED':'🔓 UNLOCKED');
  t('alarm',d.alarm?'🔊 ALARM ON':(d.warn?'⚠️ WARNING':'🔇 SILENT'));

  t('ax',d.ax+' g');t('ay',d.ay+' g');t('az',d.az+' g');
  t('mv',d.movement+' g');
  $('mvbar').style.width=Math.min(100,d.movement/0.7*100)+'%';
  t('gx',d.gx+' °/s');t('gy',d.gy+' °/s');t('gz',d.gz+' °/s');
  t('tmp',d.temp+' °C');

  pill('vib',d.vibration,'VIBRATING','NORMAL');
  pill('mot',d.motion,'MOTION','NORMAL');
  pill('gyr',d.gyro,'HIGH','NORMAL');
  pill('ir',d.ir,'OBJECT','CLEAR');

  t('sat',d.sat);t('alt',d.alt+' m');t('lat',d.lat);t('lng',d.lng);
  t('gsrc',d.gpsfix?'LIVE FIX':'DEMO DATA');
  $('map').href='https://www.google.com/maps?q='+d.lat+','+d.lng;

  okpill('h_mpu',d.mpu,'OK','NOT FOUND');
  okpill('h_oled',d.oled,'OK','NOT FOUND');
  okpill('h_gps',d.gpschars>10,'NMEA OK','NO DATA');
  t('h_wifi',d.rssi+' dBm');
  t('h_vib',d.vibtotal);
  t('h_up',fmtUp(d.up));

  t('n_ssid',d.ssid);t('n_ip',d.ip);t('n_url','http://'+d.ip);
 }).catch(()=>{t('status','⚠️ OFFLINE');});
}
setInterval(update,1000);
update();
</script>
</body>
</html>
)rawliteral";

void handleRoot()
{
  server.send_P(200, "text/html", PAGE);
}

// =====================================================
//                    API
// =====================================================

String b(bool v)
{
  return v ? "true" : "false";
}

void handleAPI()
{
  String j = "{";

  j += "\"armed\":" + b(armed);
  j += ",\"arming\":" + b(arming);
  j += ",\"theft\":" + b(theftAlert);
  j += ",\"alarm\":" + b(theftAlert || testAlarm);
  j += ",\"warn\":" + b(warningActive);
  j += ",\"lock\":" + b(armed);

  j += ",\"motion\":" + b(motionDetected);
  j += ",\"vibration\":" + b(vibrationDetected);
  j += ",\"gyro\":" + b(gyroDetected);
  j += ",\"ir\":" + b(irDetected);

  j += ",\"ax\":" + String(ax, 2);
  j += ",\"ay\":" + String(ay, 2);
  j += ",\"az\":" + String(az, 2);
  j += ",\"gx\":" + String(gx, 0);
  j += ",\"gy\":" + String(gy, 0);
  j += ",\"gz\":" + String(gz, 0);
  j += ",\"movement\":" + String(movementAmount, 2);
  j += ",\"temp\":" + String(tempC, 1);

  j += ",\"sat\":" + String(curSats);
  j += ",\"lat\":" + String(curLat, 6);
  j += ",\"lng\":" + String(curLng, 6);
  j += ",\"alt\":" + String(curAlt, 1);
  j += ",\"gpsfix\":" + b(gpsFix);
  j += ",\"gpschars\":" + String(gpsChars);

  j += ",\"mpu\":" + b(mpuOk);
  j += ",\"oled\":" + b(oledOk);
  j += ",\"vibtotal\":" + String(vibTotal);
  j += ",\"rssi\":" + String(WiFi.RSSI());
  j += ",\"up\":" + String(millis() / 1000);

  j += ",\"ssid\":\"" + String(ssid) + "\"";
  j += ",\"ip\":\"" + ipStr() + "\"";
  j += ",\"event\":\"" + lastEvent + "\"";

  j += "}";

  server.send(200, "application/json", j);
}

void handleArm()
{
  armSystem();
  server.send(200, "text/plain", "ARMING");
}

void handleDisarm()
{
  disarmSystem();
  server.send(200, "text/plain", "DISARMED");
}

void handleTest()
{
  activateTestAlarm();
  server.send(200, "text/plain", "TEST ALARM");
}

void handleSelfTest()
{
  String r = runSelfTest();
  server.send(200, "text/plain", r);
}

// =====================================================
//                    SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("       RIDEGUARD v2");
  Serial.println("==============================");

  // ---------------- GPIO ----------------

  pinMode(VIBRATION_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ARM_BUTTON, INPUT_PULLUP);
  pinMode(DISARM_BUTTON, INPUT_PULLUP);

  digitalWrite(STATUS_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // vibration interrupt (dono edges)
  attachInterrupt(digitalPinToInterrupt(VIBRATION_PIN), vibISR, CHANGE);

  // ---------------- I2C ----------------

  Wire.begin(21, 22);
  Wire.setClock(400000);

  // MPU6050 wake
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Accel +-2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyro +-250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ---------------- OLED ----------------

  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  if (!oledOk)
  {
    Serial.println("OLED NOT FOUND!");
  }
  else
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(5, 10);
    display.println("RIDE");
    display.setCursor(5, 35);
    display.println("GUARD");
    display.display();
    delay(1500);
  }

  // ---------------- SERVO ----------------

  lockServo.attach(SERVO_PIN);
  unlockBike();

  // ---------------- GPS ----------------

  randomSeed(analogRead(0) + millis());
  loadReferencePoint();

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // ---------------- MPU CALIBRATION ----------------

  delay(1000);
  calibrateMPU(100, 10);

  if (!mpuOk)
    Serial.println("WARNING: MPU6050 not responding!");

  // ---------------- WIFI ----------------

  WiFi.mode(WIFI_STA);

  if (WiFi.config(local_IP, gateway, subnet, dns))
    Serial.println("Static IP configured successfully!");
  else
    Serial.println("Static IP configuration FAILED!");

  Serial.println();
  Serial.print("Connecting to: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("================================");
    Serial.println("       WIFI CONNECTED!");
    Serial.println("================================");

    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());

    Serial.print("Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
  else
  {
    Serial.println("WIFI CONNECTION FAILED!");
  }

  // ---------------- WEB ROUTES ----------------

  server.on("/", handleRoot);
  server.on("/api", handleAPI);
  server.on("/arm", handleArm);
  server.on("/disarm", handleDisarm);
  server.on("/test", handleTest);
  server.on("/selftest", handleSelfTest);

  server.begin();

  Serial.println();
  Serial.println("RideGuard Web Server Started!");
  Serial.print("Open: http://");
  Serial.println(ipStr());

  // pehli screen: website IP
  showIPScreen();
  lastScreenChange = millis();
  oledScreen = 4;
}

// =====================================================
//                    LOOP
// =====================================================

void loop()
{
  server.handleClient();

  checkButtons();

  updateArming();

  if (millis() - lastSensorRead >= 100)
  {
    lastSensorRead = millis();

    readSensors();
    securityLogic();
  }

  updateGPS();

  updateTestAlarm();

  if (millis() - lastOLEDUpdate >= 500)
  {
    lastOLEDUpdate = millis();
    updateOLED();
  }

  // ---------------- WIFI AUTO RECONNECT ----------------

  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > 10000)
  {
    lastWifiTry = millis();
    WiFi.reconnect();
  }

  // ---------------- SERIAL DEBUG ----------------

  if (SERIAL_DEBUG && millis() - lastDebug >= 1000)
  {
    lastDebug = millis();

    Serial.print("[");
    Serial.print(theftAlert ? "THEFT" : arming ? "ARMING" : armed ? "ARMED" : "DISARMED");
    Serial.print("] VIB:");
    Serial.print(vibrationDetected ? 1 : 0);
    Serial.print(" (pulses ");
    Serial.print(vibTotal);
    Serial.print(")  MOT:");
    Serial.print(motionDetected ? 1 : 0);
    Serial.print("  GYR:");
    Serial.print(gyroDetected ? 1 : 0);
    Serial.print("  IR:");
    Serial.print(irDetected ? 1 : 0);
    Serial.print("  move=");
    Serial.print(movementAmount, 2);
    Serial.print("g  IRpin=");
    Serial.println(digitalRead(IR_PIN));
  }

  // ---------------- ALARM OUTPUT ----------------

  bool buzz = theftAlert || testAlarm || (millis() < beepUntil);

  digitalWrite(BUZZER_PIN, buzz ? HIGH : LOW);

  bool led;

  if (theftAlert || testAlarm)
    led = ((millis() / 100) % 2) == 0;
  else if (arming)
    led = ((millis() / 150) % 2) == 0;
  else
    led = armed;

  if (!selfTesting)
    digitalWrite(STATUS_LED, led ? HIGH : LOW);

  delay(5);
}