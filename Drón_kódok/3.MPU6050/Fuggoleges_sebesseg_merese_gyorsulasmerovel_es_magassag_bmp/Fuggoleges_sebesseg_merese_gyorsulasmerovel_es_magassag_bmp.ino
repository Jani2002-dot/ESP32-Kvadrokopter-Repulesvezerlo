#include <Wire.h>                //I2C könyvtár
#include <BasicLinearAlgebra.h>  //vektorokhoz és mátrixokhoz könyvtár
using namespace BLA;

//számított szögsebességeknek változó
float Rate_Roll;
float Rate_Pitch;
float Rate_Yaw;
float Rate_Roll_Offset, Rate_Pitch_Offset, Rate_Yaw_Offset;

//számított szög változó
float Szog_Acc_Roll;
float Szog_Acc_Pitch;

//kiszámmolt Acc értékek
float Acc_X;
float Acc_Y;
float Acc_Z;

float Acc_Z_gyorsulas;
float Fuggoleges_sebesseg;

uint32_t Loop_idozito;  // 250 Hz-es loop vizsgálatához
uint32_t Print_meres;
uint32_t BMP_meres;
int Print_CTU = 0;
int BMP_Print_CTU = 0;
uint32_t BMP_elozo_ido = 0;

//16 bites kalibrációs adatok mentésére változók
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

float Baro_magassag;
float MPU_magassag = 0;
float Baro_magassag_elozo = 0;
float Baro_sebesseg;
float Baro_magassag_kezdetben = 0.0;
int Baro_kalibracios_szam;
float Kalman_Magassag, Kalman_Fuggoleges_sebesseg;

//Kalman
float Kalman_Angle_Roll = 0;                     //kezdeti szögbecslést 0°-ra állítottam, mivel az indítás közel vízszintes helyzetben történik
float Kalman_Bizonytalansag_Angle_Roll = 2 * 2;  //bizonytalanságát 2° kezdeti szórással, azaz 4°^2 hibavarianciával vettem figyelembe

float Kalman_Angle_Pitch = 0;                     //kezdeti szögbecslést 0°-ra állítottam, mivel az indítás közel vízszintes helyzetben történik
float Kalman_Bizonytalansag_Angle_Pitch = 2 * 2;  //bizonytalanságát 2° kezdeti szórással, azaz 4°^2 hibavarianciával vettem figyelembe

float Kalman_Szog_Kimenet[] = { 0, 0 };
float Ts = 0.004;  //250 Hz-es loop miatt
const int A_1d = 1, H_1d = 1, I_1d = 1;
float B_1d = Ts;
float Q_1d = Ts * Ts * 4 * 4;
float R_1d = 3 * 3;

//mátrixváltozók deklarálása <sor,oszlop>
BLA::Matrix<2, 1> xk;
BLA::Matrix<2, 2> A;
BLA::Matrix<2, 1> B;
BLA::Matrix<1, 1> u;
BLA::Matrix<2, 2> P;
BLA::Matrix<2, 2> Q;
BLA::Matrix<1, 1> L;
BLA::Matrix<2, 1> K;
BLA::Matrix<1, 2> H;
BLA::Matrix<1, 1> R;
BLA::Matrix<2, 2> I;
BLA::Matrix<1, 1> zk;

void Kalman_Szog(float x, float P, float u, float z) {
  x = A_1d * x + B_1d * u;
  P = A_1d * P * A_1d + Q_1d;
  float K = P * H_1d / (H_1d * P * H_1d + R_1d);
  x = x + K * (z - H_1d * x);
  P = (I_1d - K * H_1d) * P;
  Kalman_Szog_Kimenet[0] = x;  //Korrigált szög PID-nek és következő iteráccióba
  Kalman_Szog_Kimenet[1] = P;  //Korrigált hibakovariancia következő iterációba
}

void MPU_regiszterek_beallitasa(void) {

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

void MPU_adat_olvasas(void) {

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
  Acc_X = (float)Acc_XLSB / 4096.0f - 0.0454f;
  Acc_Y = (float)Acc_YLSB / 4096.0f + 0.0279f;
  Acc_Z = (float)Acc_ZLSB / 4096.0f - 0.10150f;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Szog_Acc_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  Szog_Acc_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);
}

void Gyro_kalibracio(void) {

  //2500 mintavételes kalibráció Gyronak
  for (int i = 0; i < 2500; i++) {
    MPU_adat_olvasas();
    Rate_Roll_Offset += Rate_Roll;
    Rate_Pitch_Offset += Rate_Pitch;
    Rate_Yaw_Offset += Rate_Yaw;
    delay(1);
  }

  //Mintavételi számmal való vissza osztás
  Rate_Roll_Offset /= (float)2500;
  Rate_Pitch_Offset /= (float)2500;
  Rate_Yaw_Offset /= (float)2500;
}


bool BMP_adat_olvasas(void) {

  Wire.beginTransmission(0x77);
  Wire.write(0xF7);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((uint8_t)0x77, (uint8_t)6) != 6) {
    return false;
  }

  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();

  // Nyers, kompenzálatlan nyomás és hőmérséklet
  int32_t adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  int32_t adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

  // Hőmérséklet-kompenzáció (Bosch 32 bites képlet)
  int32_t var1_T, var2_T;

  var1_T = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1)))
            * ((int32_t)dig_T2))
           >> 11;

  var2_T = (((((adc_T >> 4) - ((int32_t)dig_T1))
              * ((adc_T >> 4) - ((int32_t)dig_T1)))
             >> 12)
            * ((int32_t)dig_T3))
           >> 14;

  int32_t t_fine = var1_T + var2_T;

  // Nyomáskompenzáció (Bosch 64 bites képlet)
  int64_t var1, var2, p;

  var1 = ((int64_t)t_fine) - 128000;

  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);

  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8)
         + ((var1 * (int64_t)dig_P2) << 12);

  var1 = (((((int64_t)1 << 47) + var1)
           * (int64_t)dig_P1)
          >> 33);

  if (var1 == 0) {
    return false;
  }

  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;

  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8)
      + (((int64_t)dig_P7) << 4);

  // A Bosch eredménye Q24.8 formátumú.
  // Osztás 256-tal: Pa, majd 100-zal: hPa.
  uint32_t p_q24_8 = (uint32_t)p;
  double nyomas = (double)p_q24_8 / 25600.0;

  // A BMP280 adatlap szerinti nyomástartomány
  if (!isfinite(nyomas) || nyomas < 300.0 || nyomas > 1100.0) {
    return false;
  }

  // Barometrikus magasság számítása [cm]
  Baro_magassag = 44330.0 * (1.0 - pow(nyomas / 1013.25, 1.0 / 5.255)) * 100.0;

  if (!isfinite(Baro_magassag)) {
    return false;
  }

  return true;
}


void BMP_regiszterek_beallitasa(void) {

  // config:
  Wire.beginTransmission(0x77);
  Wire.write(0xF5);
  Wire.write(0x10);  //00010000 binárisan
  Wire.endTransmission();

  // ctrl_meas:
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x57);  //01010111 binárisan
  Wire.endTransmission();
}

void BMP_kalibracio(void) {

  uint8_t adat[24];

  Wire.beginTransmission(0x77);
  Wire.write(0x88);

  if (Wire.endTransmission(false) != 0) {
    return;
  }

  if (Wire.requestFrom((uint8_t)0x77, (uint8_t)24) != 24) {
    return;
  }

  for (int i = 0; i < 24; i++) {
    adat[i] = Wire.read();
  }

  // LSB/MSB szerint jönnek az adatok Wire.read()-nél ezért kell így bitshiftelni
  dig_T1 = (adat[1] << 8) | adat[0];
  dig_T2 = (adat[3] << 8) | adat[2];
  dig_T3 = (adat[5] << 8) | adat[4];
  dig_P1 = (adat[7] << 8) | adat[6];
  dig_P2 = (adat[9] << 8) | adat[8];
  dig_P3 = (adat[11] << 8) | adat[10];
  dig_P4 = (adat[13] << 8) | adat[12];
  dig_P5 = (adat[15] << 8) | adat[14];
  dig_P6 = (adat[17] << 8) | adat[16];
  dig_P7 = (adat[19] << 8) | adat[18];
  dig_P8 = (adat[21] << 8) | adat[20];
  dig_P9 = (adat[23] << 8) | adat[22];
  delay(250);

  Baro_magassag_kezdetben = 0;

  int Ervenyes_mintak = 0;

  while (Ervenyes_mintak < 50) {
    if (BMP_adat_olvasas()) {
      Baro_magassag_kezdetben += Baro_magassag;
      Ervenyes_mintak++;
    }

    delay(40);
  }

  Baro_magassag_kezdetben /= Ervenyes_mintak;
}

/*void kalman_2d(void) {

  // Előrejelzés gyorsulásmérőből
  u = { Acc_Z_gyorsulas };

  xk = A * xk + B * u;
  P = A * P * ~A + Q;  // ~ Transzponáltat jelenti
  L = H * P * ~H + R;
  K = P * ~H * Inverse(L);  // Inverse(L) -1 hatványát veszi L nek

  // Barometrikus korrekció
  zk = { Baro_magassag };
  xk = xk + K * (zk - H * xk);
  P = (I - K * H) * P;

  //Frissítés
  Kalman_Magassag = xk(0, 0);             //(sor,oszlop)
  Kalman_Fuggoleges_sebesseg = xk(1, 0);  //(sor,oszlop)
}*/

void kalman_2d() {

  // Előrejelzés: minden 4 ms-os ciklusban
  u = { Acc_Z_gyorsulas };

  xk = A * xk + B * u;
  P = A * P * ~A + Q;  // ~: transzponálás

  // Korrekció: csak újonnan átvett barométeres mérésnél
  zk = { Baro_magassag };

  L = H * P * ~H + R;
  K = P * ~H * Inverse(L);  // A mérési innováció kovarianciájának inverze

  xk = xk + K * (zk - H * xk);
  P = (I - K * H) * P;


  // A kimenetek korrekció nélkül is frissülnek az előrejelzésből
  Kalman_Magassag = xk(0, 0);
  Kalman_Fuggoleges_sebesseg = xk(1, 0);
}


void setup() {

  Serial.begin(230400);   //soros vevőnek
  Wire.begin();           //SDA, SCL
  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén

  delay(250);
  MPU_regiszterek_beallitasa();
  BMP_regiszterek_beallitasa();
  BMP_kalibracio();

  A = { 1, 0.004,
        0, 1 };
  B = { 0.5 * 0.004 * 0.004,
        0.004 };
  H = { 1, 0 };
  I = { 1, 0,
        0, 1 };
  Q = B * ~B * 5.001915f;  //10.0f * 10.0f
  R = { 181.319173f };
  P = { 0, 0,
        0, 0 };
  xk = { 0,
         0 };

  Gyro_kalibracio();

  Serial.println("# BMP;ido_us;Baro_magassag_cm;Baro_sebesseg_cm_s;mintaszam");
  Serial.println("# KF;ido_us;Acc_fugg_cm_s2;MPU_sebesseg_cm_s;MPU_magassag_cm;Kalman_magassag_cm;Kalman_sebesseg_cm_s;mintaszam");

  Loop_idozito = micros();
}

void loop() {
  bool uj_baro_meres = false;

  MPU_adat_olvasas();
  uint32_t MPU_minta_ido = micros();

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  //Kalman Angle Roll számítás
  Kalman_Szog(Kalman_Angle_Roll, Kalman_Bizonytalansag_Angle_Roll, Rate_Roll, Szog_Acc_Roll);
  Kalman_Angle_Roll = Kalman_Szog_Kimenet[0];
  Kalman_Bizonytalansag_Angle_Roll = Kalman_Szog_Kimenet[1];

  //Kalman Angle Pitch számítás
  Kalman_Szog(Kalman_Angle_Pitch, Kalman_Bizonytalansag_Angle_Pitch, Rate_Pitch, Szog_Acc_Pitch);
  Kalman_Angle_Pitch = Kalman_Szog_Kimenet[0];
  Kalman_Bizonytalansag_Angle_Pitch = Kalman_Szog_Kimenet[1];

  //Az inerciális Z tengely irányába eső gyorsulás kiszámítása
  Acc_Z_gyorsulas = -sin(Kalman_Angle_Pitch * (PI / 180)) * Acc_X + cos(Kalman_Angle_Pitch * (PI / 180)) * sin(Kalman_Angle_Roll * (PI / 180)) * Acc_Y + cos(Kalman_Angle_Pitch * (PI / 180)) * cos(Kalman_Angle_Roll * (PI / 180)) * Acc_Z;
  //gyorsulás átváltása cm/s^2-re
  Acc_Z_gyorsulas = (Acc_Z_gyorsulas - 1) * 9.81 * 100;

  MPU_magassag += Fuggoleges_sebesseg * Ts
                  + 0.5f * Acc_Z_gyorsulas * Ts * Ts;

  Fuggoleges_sebesseg += Acc_Z_gyorsulas * Ts;



  if (BMP_adat_olvasas()) {

    Baro_magassag -= Baro_magassag_kezdetben;

    uint32_t most = micros();

    if (BMP_elozo_ido != 0) {
      float dt = (most - BMP_elozo_ido) / 1000000.0f;
      Baro_sebesseg = (Baro_magassag - Baro_magassag_elozo) / dt;
    }

    Baro_magassag_elozo = Baro_magassag;
    BMP_elozo_ido = most;

    if (millis() - BMP_meres >= 40 && BMP_Print_CTU < 7500) {
      BMP_meres = millis();
      BMP_Print_CTU++;

      Serial.print("BMP;");
      Serial.print(most);
      Serial.print(';');
      Serial.print(Baro_magassag, 5);
      Serial.print(';');
      Serial.print(Baro_sebesseg, 5);
      Serial.print(';');
      Serial.println(BMP_Print_CTU);
    }
  }


  kalman_2d();

  uint32_t ido_us = micros();

  if (millis() - Print_meres >= 24 && Print_CTU < 12500) {
    Print_meres = millis();
    Print_CTU++;

    Serial.print("KF;");
    Serial.print(micros());
    Serial.print(';');
    Serial.print(Acc_Z_gyorsulas, 5);
    Serial.print(';');
    Serial.print(Fuggoleges_sebesseg, 5);
    Serial.print(';');
    Serial.print(MPU_magassag, 5);
    Serial.print(';');
    Serial.print(Kalman_Magassag, 5);
    Serial.print(';');
    Serial.print(Kalman_Fuggoleges_sebesseg, 5);
    Serial.print(';');
    Serial.println(Print_CTU);
  }

  while (micros() - Loop_idozito < 4000)
    ;
  Loop_idozito = micros();
}
