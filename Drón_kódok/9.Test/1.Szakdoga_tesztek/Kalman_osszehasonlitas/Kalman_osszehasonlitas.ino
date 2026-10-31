#include <Wire.h>
#include <math.h>

// Propellerek nelkul, levalasztott LiPo-val, csak USB-taprol.
const uint32_t Mintaveteli_ido_us = 4000;
const uint32_t Mintak_szama = 15000;  // Egy teszt kb. 60 masodperc.
const float Ts = 0.004f;

// A masodik nyugalmi sorozatbol: sqrt(a harom mintavariancia atlaga).
const float Sigma_gyro_mert[2] = {0.045410153f, 0.043824320f};  // Roll, Pitch; fok/s.
const float Sigma_szog_mert[2] = {0.085989697f, 0.082895460f};  // Roll, Pitch; fok.

float Rate_Roll, Rate_Pitch, Angle_Roll, Angle_Pitch;
float Rate_Roll_Offset = 0, Rate_Pitch_Offset = 0;
float Szog_eredeti[2], P_eredeti[2];
float Szog_mert[2], P_mert[2];

void hiba(const char* uzenet) {
  Serial.print("# HIBA: ");
  Serial.println(uzenet);
  Serial.println("# Ellenorzes utan reset szukseges.");
  while (true) delay(1000);
}

void regiszter_iras(uint8_t cim, uint8_t ertek) {
  Wire.beginTransmission(0x68);
  Wire.write(cim);
  Wire.write(ertek);
  if (Wire.endTransmission() != 0) hiba("MPU6050 beallitasi hiba");
}

int16_t adat16() {
  uint8_t felso = Wire.read();
  uint8_t also = Wire.read();
  return (int16_t)(((uint16_t)felso << 8) | also);
}

void gyro_acc_adat_olvasas() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) hiba("I2C cimzesi hiba");
  if (Wire.requestFrom((uint8_t)0x68, (uint8_t)14) != 14) hiba("Hianyos MPU6050 adat");

  int16_t Acc_XLSB = adat16();
  int16_t Acc_YLSB = adat16();
  int16_t Acc_ZLSB = adat16();
  adat16();  // Homerseklet.
  int16_t Gyro_X = adat16();
  int16_t Gyro_Y = adat16();
  adat16();  // Yaw.

  // Az eredeti repulesi kod skalazasa, offsetjei es szogkepletei.
  Rate_Roll = (float)Gyro_X / 65.5;
  Rate_Pitch = (float)Gyro_Y / 65.5;
  float Acc_X = (float)Acc_XLSB / 4096 - 0.0454;
  float Acc_Y = (float)Acc_YLSB / 4096 + 0.0279;
  float Acc_Z = (float)Acc_ZLSB / 4096 - 0.10150;
  Angle_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  Angle_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);
  if (!isfinite(Angle_Roll) || !isfinite(Angle_Pitch)) hiba("Ervenytelen szog");
}

void gyro_kalibracio() {
  Serial.println("# Kalibracio: kozel vizszintesen, mozdulatlanul tartsd!");
  double Roll_osszeg = 0, Pitch_osszeg = 0;
  for (int i = 0; i < 2000; i++) {
    gyro_acc_adat_olvasas();
    Roll_osszeg += Rate_Roll;
    Pitch_osszeg += Rate_Pitch;
    delay(1);
  }
  Rate_Roll_Offset = Roll_osszeg / 2000;
  Rate_Pitch_Offset = Pitch_osszeg / 2000;
  Serial.printf("# Offset [fok/s]: Roll=%.6f; Pitch=%.6f\n", Rate_Roll_Offset, Rate_Pitch_Offset);
}

void kalman_1d(float& szog, float& P, float rate, float meres,
               float sigma_gyro, float sigma_szog) {
  szog += Ts * rate;
  P += Ts * Ts * sigma_gyro * sigma_gyro;
  float K = P / (P + sigma_szog * sigma_szog);
  szog += K * (meres - szog);
  P = (1.0f - K) * P;
}

void osszehasonlitas() {
  gyro_kalibracio();
  for (int j = 0; j < 2; j++) {
    // Azonos kezdeti allapot mindket valtozatnal.
    Szog_eredeti[j] = Szog_mert[j] = 0;
    P_eredeti[j] = P_mert[j] = 4;
  }
  Serial.println("# START: 10 s nyugalom, dontesek, a vegen ismet 10 s nyugalom.");
  Serial.println("n,t_us,rate_roll,rate_pitch,acc_roll,kalman_eredeti_roll,kalman_mert_roll,acc_pitch,kalman_eredeti_pitch,kalman_mert_pitch");
  Serial.flush();

  uint32_t Kezdet = micros(), Elozo_ido = Kezdet;
  uint32_t Dt_min = UINT32_MAX, Dt_max = 0, Lassu_ciklusok = 0;
  uint32_t N = 0, Kiirt_sorok = 0;
  uint64_t Dt_osszeg = 0;

  for (uint32_t n = 1; n <= Mintak_szama; n++) {
    if (Serial.available() && Serial.read() == 'x') break;
    while ((uint32_t)(micros() - Elozo_ido) < Mintaveteli_ido_us) {}
    uint32_t Most = micros();
    uint32_t Dt = Most - Elozo_ido;
    Elozo_ido = Most;
    N = n;
    Dt_osszeg += Dt;
    if (Dt < Dt_min) Dt_min = Dt;
    if (Dt > Dt_max) Dt_max = Dt;
    if (Dt > 4500) Lassu_ciklusok++;

    gyro_acc_adat_olvasas();
    Rate_Roll -= Rate_Roll_Offset;
    Rate_Pitch -= Rate_Pitch_Offset;
    float Rate[2] = {Rate_Roll, Rate_Pitch};
    float Angle[2] = {Angle_Roll, Angle_Pitch};
    for (int j = 0; j < 2; j++) {
      // Ugyanaz a minta; csak a ket zajparameter kulonbozik.
      kalman_1d(Szog_eredeti[j], P_eredeti[j], Rate[j], Angle[j], 4.0f, 3.0f);
      kalman_1d(Szog_mert[j], P_mert[j], Rate[j], Angle[j], Sigma_gyro_mert[j], Sigma_szog_mert[j]);
    }

    // A szuro 250 Hz-en fut, csak minden otodik eredmenyt irjuk ki: 50 Hz.
    if (n % 5 == 0) {
      Serial.printf("%lu,%lu,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    (unsigned long)n, (unsigned long)(Most - Kezdet),
                    Rate_Roll, Rate_Pitch, Angle_Roll, Szog_eredeti[0], Szog_mert[0],
                    Angle_Pitch, Szog_eredeti[1], Szog_mert[1]);
      Kiirt_sorok++;
    }
  }

  Serial.printf("# VEGE: N=%lu; CSV_sorok=%lu\n", (unsigned long)N, (unsigned long)Kiirt_sorok);
  if (N > 0) {
    Serial.printf("# dt [us]: atlag=%.2f; min=%lu; max=%lu; 4500 us felett=%lu\n",
                  (double)Dt_osszeg / N, (unsigned long)Dt_min,
                  (unsigned long)Dt_max, (unsigned long)Lassu_ciklusok);
  }
  Serial.println("# Uj teszt: s. Minden inditas elott uj gyro-kalibracio van.");
}

void setup() {
  const uint8_t Motor_pinek[] = {23, 19, 18, 17};
  for (uint8_t pin : Motor_pinek) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  delay(250);
  regiszter_iras(0x6B, 0x00);
  regiszter_iras(0x1A, 0x03);  // DLPF: gyro 42 Hz.
  regiszter_iras(0x1B, 0x08);  // Gyro: +/-500 fok/s.
  regiszter_iras(0x1C, 0x10);  // Acc: +/-8 g.
  regiszter_iras(0x19, 0x00);  // Belso 1 kHz, kiolvasas 250 Hz.
  delay(100);
  Serial.println("# Kalman osszehasonlitas; 115200 baud; csak USB, motorvezerles nelkul.");
  Serial.println("# Eredeti sigma_gyro=4 fok/s, sigma_szog=3 fok.");
  Serial.printf("# Mert sigma: Roll=%.8f/%.8f; Pitch=%.8f/%.8f [fok/s / fok]\n",
                Sigma_gyro_mert[0], Sigma_szog_mert[0], Sigma_gyro_mert[1], Sigma_szog_mert[1]);
  Serial.println("# Mindket szuro: kezdeti szog=0; P=4 fok^2; Ts=4000 us.");
  Serial.println("# Bemelegedes 3-5 perc. Inditas: s; korabbi befejezes: x.");
}

void loop() {
  if (Serial.available() && Serial.read() == 's') osszehasonlitas();
  delay(1);
}
