#include <Wire.h>

#define MPU_ADDR 0x68

const float Ts = 0.004f;        // 250 Hz
const uint32_t MINTAK = 15000;  // 60 s * 250 Hz

float GyroOffsetRoll = 0;
float GyroOffsetPitch = 0;


// =============================
// Futó statisztika (Welford)
// =============================
struct Statisztika {

  uint32_t n = 0;

  double atlag = 0;
  double M2 = 0;

  double minimum = 999999.0;
  double maximum = -999999.0;


  void hozzaad(double x) {

    n++;

    // minimum
    if (x < minimum) {
      minimum = x;
    }

    // maximum
    if (x > maximum) {
      maximum = x;
    }


    // átlag + szórás
    double delta = x - atlag;

    atlag += delta / n;

    double delta2 = x - atlag;

    M2 += delta * delta2;
  }


  double szoras() {

    if (n < 2) return 0;

    return sqrt(M2 / (n - 1));
  }
};


Statisztika GyroRollStat;
Statisztika GyroPitchStat;

Statisztika AngleRollStat;
Statisztika AnglePitchStat;


// =============================
// MPU regiszterek
// =============================
void MPU_Beallitas() {

  // MPU felébresztése
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // DLPF_CFG = 3
  // Gyro ~42 Hz, Acc ~44 Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  // Gyro ±500 °/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Accelerometer ±8 g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}


// =============================
// Szenzor kiolvasás
// =============================
void MPU_Olvasas(float &RateRoll,
                 float &RatePitch,
                 float &AccX,
                 float &AccY,
                 float &AccZ) {

  // Accelerometer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6);

  int16_t AccXRaw = Wire.read() << 8 | Wire.read();
  int16_t AccYRaw = Wire.read() << 8 | Wire.read();
  int16_t AccZRaw = Wire.read() << 8 | Wire.read();

  AccX = (float)AccXRaw / 4096.0f - 0.0454f;
  AccY = (float)AccYRaw / 4096.0f + 0.0279f;
  AccZ = (float)AccZRaw / 4096.0f - 0.10150f;


  // Gyroscope
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6);

  int16_t GyroXRaw = Wire.read() << 8 | Wire.read();
  int16_t GyroYRaw = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();  // Yaw most nem kell

  RateRoll = (float)GyroXRaw / 65.5f;
  RatePitch = (float)GyroYRaw / 65.5f;
}


// =============================
// SETUP
// =============================
void setup() {

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  MPU_Beallitas();

  delay(1000);

  Serial.println("Gyro kalibracio...");
  Serial.println("A dron legyen teljesen mozdulatlan.");


  // =============================
  // Gyro offset kalibráció
  // =============================

  const int KalibraciosMintak = 2000;

  for (int i = 0; i < KalibraciosMintak; i++) {

    float RateRoll, RatePitch;
    float AccX, AccY, AccZ;

    MPU_Olvasas(
      RateRoll,
      RatePitch,
      AccX,
      AccY,
      AccZ);

    GyroOffsetRoll += RateRoll;
    GyroOffsetPitch += RatePitch;

    delay(4);
  }

  GyroOffsetRoll /= KalibraciosMintak;
  GyroOffsetPitch /= KalibraciosMintak;


  Serial.print("Gyro Roll offset: ");
  Serial.println(GyroOffsetRoll, 6);

  Serial.print("Gyro Pitch offset: ");
  Serial.println(GyroOffsetPitch, 6);


  Serial.println();
  Serial.println("Meres indul...");
  Serial.println("60 masodpercig NE MOZGASD a dront.");
}


// =============================
// LOOP
// =============================
void loop() {

  static uint32_t MeresSzam = 0;
  static uint32_t LoopTimer = micros();
  static bool Kesz = false;


  if (Kesz) {
    return;
  }


  float RateRoll, RatePitch;
  float AccX, AccY, AccZ;

  MPU_Olvasas(
    RateRoll,
    RatePitch,
    AccX,
    AccY,
    AccZ);


  // Kalibrált gyro
  RateRoll -= GyroOffsetRoll;
  RatePitch -= GyroOffsetPitch;


  // Accelerometerből számított szögek
  float AngleRoll =
    atan2(
      AccY,
      sqrt(AccX * AccX + AccZ * AccZ))
    * 180.0f / PI;


  float AnglePitch =
    -atan2(
      AccX,
      sqrt(AccY * AccY + AccZ * AccZ))
    * 180.0f / PI;


  // Statisztikák
  GyroRollStat.hozzaad(RateRoll);
  GyroPitchStat.hozzaad(RatePitch);

  AngleRollStat.hozzaad(AngleRoll);
  AnglePitchStat.hozzaad(AnglePitch);


  MeresSzam++;


  // =============================
  // Mérési sorozat vége
  // =============================

  if (MeresSzam >= MINTAK) {

    Kesz = true;

    double SigmaGyroRoll = GyroRollStat.szoras();
    double SigmaGyroPitch = GyroPitchStat.szoras();

    double SigmaAngleRoll = AngleRollStat.szoras();
    double SigmaAnglePitch = AnglePitchStat.szoras();

    double QRoll = pow(Ts * SigmaGyroRoll, 2);
    double QPitch = pow(Ts * SigmaGyroPitch, 2);

    double RRoll = pow(SigmaAngleRoll, 2);
    double RPitch = pow(SigmaAnglePitch, 2);


    Serial.println();
    Serial.println("========== GYRO ROLL ==========");

    Serial.print("Atlag [deg/s]: ");
    Serial.println(GyroRollStat.atlag, 6);

    Serial.print("Sigma [deg/s]: ");
    Serial.println(SigmaGyroRoll, 6);

    Serial.print("Minimum [deg/s]: ");
    Serial.println(GyroRollStat.minimum, 6);

    Serial.print("Maximum [deg/s]: ");
    Serial.println(GyroRollStat.maximum, 6);


    Serial.println();
    Serial.println("========== GYRO PITCH ==========");

    Serial.print("Atlag [deg/s]: ");
    Serial.println(GyroPitchStat.atlag, 6);

    Serial.print("Sigma [deg/s]: ");
    Serial.println(SigmaGyroPitch, 6);

    Serial.print("Minimum [deg/s]: ");
    Serial.println(GyroPitchStat.minimum, 6);

    Serial.print("Maximum [deg/s]: ");
    Serial.println(GyroPitchStat.maximum, 6);


    Serial.println();
    Serial.println("========== ACC ANGLE ROLL ==========");

    Serial.print("Atlag [deg]: ");
    Serial.println(AngleRollStat.atlag, 6);

    Serial.print("Sigma [deg]: ");
    Serial.println(SigmaAngleRoll, 6);

    Serial.print("Minimum [deg]: ");
    Serial.println(AngleRollStat.minimum, 6);

    Serial.print("Maximum [deg]: ");
    Serial.println(AngleRollStat.maximum, 6);


    Serial.println();
    Serial.println("========== ACC ANGLE PITCH ==========");

    Serial.print("Atlag [deg]: ");
    Serial.println(AnglePitchStat.atlag, 6);

    Serial.print("Sigma [deg]: ");
    Serial.println(SigmaAnglePitch, 6);

    Serial.print("Minimum [deg]: ");
    Serial.println(AnglePitchStat.minimum, 6);

    Serial.print("Maximum [deg]: ");
    Serial.println(AnglePitchStat.maximum, 6);


    Serial.println();
    Serial.println("========== KALMAN KIINDULO ERTEKEK ==========");

    Serial.print("Q Roll: ");
    Serial.println(QRoll, 10);

    Serial.print("Q Pitch: ");
    Serial.println(QPitch, 10);

    Serial.print("R Roll: ");
    Serial.println(RRoll, 10);

    Serial.print("R Pitch: ");
    Serial.println(RPitch, 10);
  }

  // 250 Hz
  while (micros() - LoopTimer < 4000)
    ;

  LoopTimer = micros();
}