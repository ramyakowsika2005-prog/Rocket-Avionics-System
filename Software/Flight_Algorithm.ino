/*
-------------------------------------------------------------
Project : Rocket Avionics System
Competition : TEKNOFEST Rocket Competition
Subsystem : Indigenous Flight Control Computer (I-FCC)

Description:
This software reads sensor data, estimates altitude and
velocity, detects launch, processes GPS data, and
transmits telemetry at 10 Hz.

Project Status:
Preliminary Design Review (PDR)

Team Members:
- Abineshwaran
- Ramya Kowsika M
-------------------------------------------------------------
*/#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>

/* ================= MPU6050 ================= */
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT 0x3B

/* ================= GPS ================= */
#define GPS_RX 16
#define GPS_TX 17

Adafruit_BMP280 bmp;
TinyGPSPlus gps;

bool launchDetected = false;
bool pyroStatus = false;

unsigned long launchMillis = 0;
unsigned long lastTime = 0;
unsigned long lastSend = 0;

float groundAltitude = 0;

float accFiltered = 0;
float altitudeFiltered = 0;
float altitudePrev = 0;
float velocity = 0;

/* ================= FILTER CONSTANTS ================= */
const float alphaAcc = 0.18;
const float alphaAlt = 0.10;

/* ================= KALMAN FILTER ================= */
float vel_est = 0;     // Estimated velocity
float P = 1;           // Estimation covariance
float Q = 0.05;        // Process noise (tune 0.02–0.08)
float R = 2.0;         // Measurement noise (tune 1–5)

/* ================================================== */
void setup() {

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Wire.begin(21, 22);

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();

  bmp.begin(0x76);

  delay(1000);
  groundAltitude = bmp.readAltitude(1013.25);

  lastTime = millis();
}

/* ================================================== */
void loop() {

  while (Serial2.available())
    gps.encode(Serial2.read());

  int sats = gps.satellites.value();

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0) return;

  /* ================= MPU6050 READ ================= */
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() != 14) return;

  Wire.read(); Wire.read();     // AX
  Wire.read(); Wire.read();     // AY
  int16_t az = Wire.read() << 8 | Wire.read();  // AZ
  for (int i = 0; i < 6; i++) Wire.read();

  float az_net = (az / 16384.0) * 9.80665 - 9.80665;

  /* ================= BAROMETER ================= */
  float altitude = bmp.readAltitude(1013.25) - groundAltitude;
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;

  /* ================= LOW PASS FILTERS ================= */
  altitudeFiltered = alphaAlt * altitude +
                     (1 - alphaAlt) * altitudeFiltered;

  accFiltered = alphaAcc * az_net +
                (1 - alphaAcc) * accFiltered;

  /* ================= BARO VELOCITY ================= */
  float velBaro = (altitudeFiltered - altitudePrev) / dt;
  altitudePrev = altitudeFiltered;

  /* ================= KALMAN FILTER ================= */

  // ---- Prediction ----
  vel_est = vel_est + accFiltered * dt;
  P = P + Q;

  // ---- Update ----
  float K = P / (P + R);         // Kalman Gain
  vel_est = vel_est + K * (velBaro - vel_est);
  P = (1 - K) * P;

  velocity = vel_est;

  /* ================= LAUNCH DETECTION ================= */
  static unsigned long trig = 0;

  if (!launchDetected && (accFiltered > 6 || velocity > 3)) {
    if (trig == 0) trig = millis();
    if (millis() - trig > 80) {
      launchDetected = true;
      launchMillis = millis();
    }
  } else {
    trig = 0;
  }

  float Tplus = launchDetected ?
                (millis() - launchMillis) / 1000.0 :
                0;

  /* ================= GPS ================= */
  static double lat = 0;
  static double lon = 0;

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  float descentVelocity = (velocity < 0) ? velocity : 0;

  /* ================= 10 Hz TELEMETRY ================= */
  if (millis() - lastSend >= 100) {

    lastSend += 100;

    Serial.print("T:");
    Serial.print(Tplus, 2);

    Serial.print(",ALT:");
    Serial.print(altitudeFiltered, 2);

    Serial.print(",PRS:");
    Serial.print(pressure, 2);

    Serial.print(",TMP:");
    Serial.print(temperature, 2);

    Serial.print(",VEL:");
    Serial.print(velocity, 2);

    Serial.print(",DV:");
    Serial.print(descentVelocity, 2);

    Serial.print(",LAT:");
    Serial.print(lat, 6);

    Serial.print(",LON:");
    Serial.print(lon, 6);

    Serial.print(",SAT:");
    Serial.print(sats);

    Serial.print(",PY:");
    Serial.println(pyroStatus ? 1 : 0);
  }
}
