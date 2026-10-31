#include <Wire.h>
#include <math.h>

// Csak USB-tap, propellerek nelkul. Nincs motorvezerles.
const uint32_t Mintaveteli_ido_us = 4000;
const uint32_t Mintak_szama = 15000;  // Meresenkent kb. 60 masodperc.
const uint8_t Meresek_szama = 3;
const bool CSV_naplo = false;  // true: minden minta kiirasa is.

float Rate_Roll, Rate_Pitch, Angle_Roll, Angle_Pitch;
float Rate_Roll_Offset = 0, Rate_Pitch_Offset = 0;
double Atlag[4], M2[4];
const char* Jelnevek[4] = {
  "Rate_Roll_fok_s", "Rate_Pitch_fok_s",
  "Angle_Roll_fok", "Angle_Pitch_fok"
};

void hiba(const char* uzenet) {
  Serial.print("# HIBA: ");
  Serial.println(uzenet);
  Serial.println("# A merest ne hasznald fel. Ellenorzes utan reset.");
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
  if (Wire.requestFrom((uint8_t)0x68, (uint8_t)14) != 14) {
    hiba("Hianyos MPU6050 adat");
  }

  int16_t Acc_XLSB = adat16();
  int16_t Acc_YLSB = adat16();
  int16_t Acc_ZLSB = adat16();
  adat16();  // Homerseklet.
  int16_t Gyro_X = adat16();
  int16_t Gyro_Y = adat16();
  adat16();  // Yaw.

  // Az eredeti repulesi kod skalazasa es gyorsulasmero-offsetjei.
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
  Serial.println("# Gyro nullpontkalibracio: ne mozgasd!");
  double Roll_osszeg = 0, Pitch_osszeg = 0;
  for (int i = 0; i < 2000; i++) {
    gyro_acc_adat_olvasas();
    Roll_osszeg += Rate_Roll;
    Pitch_osszeg += Rate_Pitch;
    delay(1);
  }
  Rate_Roll_Offset = Roll_osszeg / 2000;
  Rate_Pitch_Offset = Pitch_osszeg / 2000;
  Serial.printf("# Gyro offset [fok/s]: Roll=%.6f; Pitch=%.6f\n",
                Rate_Roll_Offset, Rate_Pitch_Offset);
}

void szoras_meres(uint8_t meres) {
  for (int j = 0; j < 4; j++) {
    Atlag[j] = 0;
    M2[j] = 0;
  }
  Serial.printf("# MERES %u/%u: ne mozgasd a dront!\n", meres, Meresek_szama);
  Serial.flush();

  uint32_t Elozo_ido = micros();
  uint32_t Elso_ido = 0;
  uint32_t Dt_min = UINT32_MAX, Dt_max = 0, Lassu_ciklusok = 0;
  uint64_t Dt_osszeg = 0;

  for (uint32_t n = 1; n <= Mintak_szama; n++) {
    while ((uint32_t)(micros() - Elozo_ido) < Mintaveteli_ido_us) {}
    uint32_t Most = micros();
    uint32_t Dt = Most - Elozo_ido;
    Elozo_ido = Most;
    if (n == 1) Elso_ido = Most;
    else {
      Dt_osszeg += Dt;
      if (Dt < Dt_min) Dt_min = Dt;
      if (Dt > Dt_max) Dt_max = Dt;
      if (Dt > 4500) Lassu_ciklusok++;
    }

    gyro_acc_adat_olvasas();
    // A Kalman ezekbol kap bemenetet; a PID 0.7/0.3 szuroje ide nem kell.
    double Ertekek[4] = {
      Rate_Roll - Rate_Roll_Offset, Rate_Pitch - Rate_Pitch_Offset,
      Angle_Roll, Angle_Pitch
    };

    // Welford: atlag es az atlagtol vett negyzetes elteresek osszege.
    for (int j = 0; j < 4; j++) {
      double Elteres = Ertekek[j] - Atlag[j];
      Atlag[j] += Elteres / n;
      M2[j] += Elteres * (Ertekek[j] - Atlag[j]);
    }

    if (CSV_naplo) {
      Serial.printf("%u,%lu,%.6f,%.6f,%.6f,%.6f\n", meres,
                    (unsigned long)(Most - Elso_ido),
                    Ertekek[0], Ertekek[1], Ertekek[2], Ertekek[3]);
    }
  }

  double Variancia[4];
  Serial.printf("# EREDMENY %u: N=%lu; elso-utolso minta=%.3f s\n",
                meres, (unsigned long)Mintak_szama, Dt_osszeg / 1000000.0);
  Serial.println("# Jel; atlag; mintaszoras; mintavariancia");
  for (int j = 0; j < 4; j++) {
    Variancia[j] = M2[j] / (Mintak_szama - 1);
    Serial.printf("# %s; %.6f; %.6f; %.9f\n", Jelnevek[j],
                  Atlag[j], sqrt(Variancia[j]), Variancia[j]);
  }
  const double Ts = Mintaveteli_ido_us / 1000000.0;
  Serial.println("# Nyugalmi kiindulo varianciak [fok^2], nem repulesi optimalas:");
  Serial.printf("# Roll:  R_v=%.12f; R_w=%.9f\n", Ts * Ts * Variancia[0], Variancia[2]);
  Serial.printf("# Pitch: R_v=%.12f; R_w=%.9f\n", Ts * Ts * Variancia[1], Variancia[3]);
  Serial.printf("# dt [us]: atlag=%.2f; min=%lu; max=%lu; 4500 us felett=%lu\n",
                (double)Dt_osszeg / (Mintak_szama - 1),
                (unsigned long)Dt_min, (unsigned long)Dt_max,
                (unsigned long)Lassu_ciklusok);
  Serial.println("# Mozgas vagy idozitesi hiba eseten ismeteld meg a merest.");
}

void setup() {
  // Az eredeti kod motorlabai alacsony szinten maradnak, PWM nelkul.
  const uint8_t Motor_pinek[] = {23, 19, 18, 17};
  for (uint8_t pin : Motor_pinek) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.begin(230400);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  delay(250);

  regiszter_iras(0x6B, 0x00);  // Ebresztes, mint a repulesi kodban.
  regiszter_iras(0x1A, 0x03);  // Gyro DLPF: 42 Hz.
  regiszter_iras(0x1B, 0x08);  // Gyro: +/-500 fok/s.
  regiszter_iras(0x1C, 0x10);  // Acc: +/-8 g.
  regiszter_iras(0x19, 0x00);  // Alapertek: belso 1 kHz, kiolvasas 250 Hz.
  delay(100);
  Serial.println("# MPU6050 nyugalmi szorasmeres, soros port: 230400 baud.");
  Serial.println("# Beallitas: DLPF=3; gyro=500; acc=8g; Ts=4000 us.");
  Serial.println("# Hagyd 3-5 percig bekapcsolva, stabil feluleten, csak USB-tappal.");
  Serial.println("# Inditas: kuldj s betut. 1 kalibracio, majd 3 x kb. 60 s meres.");
}

void loop() {
  if (Serial.available() && Serial.read() == 's') {
    gyro_kalibracio();
    if (CSV_naplo) Serial.println("meres,t_us,rate_roll,rate_pitch,angle_roll,angle_pitch");
    for (uint8_t m = 1; m <= Meresek_szama; m++) szoras_meres(m);
    Serial.println("# KESZ. Uj s betu: uj kalibracio es uj 3 meres.");
  }
  delay(1);
}
