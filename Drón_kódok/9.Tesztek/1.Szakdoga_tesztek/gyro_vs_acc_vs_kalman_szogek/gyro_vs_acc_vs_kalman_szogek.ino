#include <Wire.h>  //I2C könyvtár

//számított szögsebességeknek változó
float Rate_Roll, Rate_Pitch, Rate_Yaw;
float Angle_Gyro_Roll, Angle_Gyro_Pitch;

//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Offset = 0;
float Rate_Pitch_Offset = 0;
float Rate_Yaw_Offset = 0;

//számított szög változó
float Angle_Roll, Angle_Pitch;
float Acc_X, Acc_Y, Acc_Z;
float Kalman_Angle_Roll = 0;
float Kalman_Bizonytalansag_Angle_Roll = 2 * 2;  //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° szórás

float Kalman_Angle_Pitch = 0;
float Kalman_Bizonytalansag_Angle_Pitch = 2 * 2;  //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° szórás

float Kalman_1D_kimenet[] = { 0, 0 };

// 250 Hz-es loop vizsgálatához
uint32_t Loop_Timer, Debug_Ido;
float Ts = 0.004;  //250 Hz-es loop miatt

void mpu_6050_regiszterek_beallitasa(void) {

  //MPU aktiválása (felébesztése)
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x6B);              //kérés küldése a regiszer indítására
  Wire.write(0x00);              //MPU felébesztésének beleírása HEX-be
  Wire.endTransmission();        //küldés

  // 42 Hz aluláteresztő beállítása
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x1A);              //kérés küldése a regiszer indítására
  Wire.write(0x03);              //42 Hz beleírása HEX-be
  Wire.endTransmission();        //küldés

  // Girszkóp +-500 °/s beállítása 65.5 LSB/°/s
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x1B);              //kérés küldése a regiszer indítására
  Wire.write(0x08);              //+-500 °/s beleírása HEX-be
  Wire.endTransmission();        //küldés

  //Gyorsulásmérő +-8g beállítása 4096 LSB/g
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x1C);              //kérés küldése a regiszer indítására
  Wire.write(0x10);              //MPU felébesztésének beleírása HEX-be
  Wire.endTransmission();        //küldés
}

void gyro_acc_adat_olvasas(void) {

  //Girszkóp adatainak kiolvasása
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x3B);              //Acc adatok itt keződnek
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);  // 14 bytot kérünk

  while (Wire.available() < 14) {}  // vár a 14 byte-ra

  //MPU 16 bites értékeit bele bitshiftelni egy 16 bites változóba mivel I2C rigsztereken csak 8 bites byte-okban kommunikál
  // << bitshift 8 bittel feltoljuk a 16 bites változó 15-8 bitjébe
  // | (bitwise OR) összekapcsolja a byte.okat
  int16_t Acc_XLSB = Wire.read() << 8 | Wire.read();
  int16_t Acc_YLSB = Wire.read() << 8 | Wire.read();
  int16_t Acc_ZLSB = Wire.read() << 8 | Wire.read();
  int16_t Temp = Wire.read() << 8 | Wire.read();
  int16_t Gyro_X = Wire.read() << 8 | Wire.read();
  int16_t Gyro_Y = Wire.read() << 8 | Wire.read();
  int16_t Gyro_Z = Wire.read() << 8 | Wire.read();


  // °/s szögsebességek kiszámolás mivel +-500°/s LSB 65.5/°/s
  // MPU 16 bites adat olvasás 2^16, +-500 = 1000 ------> 2^16/1000=65.536 ~ 65.5
  Rate_Roll = (float)Gyro_X / 65.5;
  Rate_Pitch = (float)Gyro_Y / 65.5;
  Rate_Yaw = (float)Gyro_Z / 65.5;

  //Alakítsd át a mért értékeket fizikai (valós) egységekbe.
  //a gyorsulásmérő nyers LSB értékeiből csinálsz g-t vagy m/s²-t (ha m/s^2 akkor * 9.81)
  Acc_X = (float)Acc_XLSB / 4096 - 0.0454;
  Acc_Y = (float)Acc_YLSB / 4096 + 0.0279;
  Acc_Z = (float)Acc_ZLSB / 4096 - 0.10150;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Angle_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  Angle_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);
}

void kalman_1d(float Kalman_allapot, float Kalman_Bizonytalansag, float Kalman_bemenet, float Kalman_meres) {
  Kalman_allapot = Kalman_allapot + Kalman_bemenet * Ts;
  Kalman_Bizonytalansag = Kalman_Bizonytalansag + Ts * Ts * 4 * 4;
  float Kalman_erosites = Kalman_Bizonytalansag * 1 / (1 * Kalman_Bizonytalansag + 3 * 3);
  Kalman_allapot = Kalman_allapot + Kalman_erosites * (Kalman_meres - Kalman_allapot);
  Kalman_Bizonytalansag = (1 - Kalman_erosites * 1) * Kalman_Bizonytalansag;
  Kalman_1D_kimenet[0] = Kalman_allapot;
  Kalman_1D_kimenet[1] = Kalman_Bizonytalansag;
}

void setup() {

  Serial.begin(115200);

  Wire.begin();           //SDA, SCL
  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  delay(250);
  mpu_6050_regiszterek_beallitasa();

  //2000 mintavételes kalibráció
  for (int Kalibracios_int = 0; Kalibracios_int < 2000; Kalibracios_int++) {
    gyro_acc_adat_olvasas();
    Rate_Roll_Offset += Rate_Roll;
    Rate_Pitch_Offset += Rate_Pitch;
    Rate_Yaw_Offset += Rate_Yaw;
    delay(1);
  }

  //Mintavételi számmal való vissza osztás
  Rate_Roll_Offset /= (float)2000;
  Rate_Pitch_Offset /= (float)2000;
  Rate_Yaw_Offset /= (float)2000;

  Loop_Timer = micros();
}

void loop() {

  //Gyro adat olvasó fgv meghívása
  gyro_acc_adat_olvasas();

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  // Gyro integrálás
  Angle_Gyro_Roll += Rate_Roll * Ts;
  Angle_Gyro_Pitch += Rate_Pitch * Ts;

  /*
  // Yaw miatti roll/pitch tengelykorrekció
  float yaw_rad = Rate_Yaw * Ts * (PI / 180.0);

  float Roll_Elozo = Angle_Gyro_Roll;
  float Pitch_Elozo = Angle_Gyro_Pitch;

  Angle_Gyro_Pitch = Pitch_Elozo - Roll_Elozo * sin(yaw_rad);
  Angle_Gyro_Roll = Roll_Elozo + Pitch_Elozo * sin(yaw_rad);*/

  //Kalman Angle Roll számítás
  kalman_1d(Kalman_Angle_Roll, Kalman_Bizonytalansag_Angle_Roll, Rate_Roll, Angle_Roll);
  Kalman_Angle_Roll = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Angle_Roll = Kalman_1D_kimenet[1];

  //Kalman Angle Pitch számítás
  kalman_1d(Kalman_Angle_Pitch, Kalman_Bizonytalansag_Angle_Pitch, Rate_Pitch, Angle_Pitch);
  Kalman_Angle_Pitch = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Angle_Pitch = Kalman_1D_kimenet[1];

  const bool Roll_megjelenites = true;  // false: Pitch

  if (millis() - Debug_Ido >= 20) {
    Debug_Ido = millis();

    if (Roll_megjelenites) {
      Serial.printf("Gyro_Roll:%.3f\tGyro_Roll:%.3f\tAcc_Roll:%.3f\tKalman_Roll:%.3f\n",
                    Angle_Gyro_Roll, Angle_Roll, Kalman_Angle_Roll);
    } else {
      Serial.printf("Gyro_Pitch:%.3f\tAcc_Pitch:%.3f\tKalman_Pitch:%.3f\n",
                    Angle_Gyro_Pitch, Angle_Pitch, Kalman_Angle_Pitch);
    }
  }

  while (micros() - Loop_Timer < 4000)
    ;
  Loop_Timer = micros();
}
