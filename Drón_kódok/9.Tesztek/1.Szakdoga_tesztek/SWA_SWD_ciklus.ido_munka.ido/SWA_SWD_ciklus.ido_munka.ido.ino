#pragma region 1.Valtozok

#pragma region 01_Konyvtarak
#include <Wire.h>                //I2C könyvtár
#include <BasicLinearAlgebra.h>  //vektorokhoz és mátrixokhoz könyvtár
using namespace BLA;
#pragma endregion

#pragma region 02_Pinek_es_PWM
#define RX_PIN 16
#define TX_PIN -1

#define Motorok_ESC_PWM_felbontas 12  //12 bit ---> 0-4095

#define Motorok_ESC_PWM_frekvencia 250  //250Hz=4000us

#define Motor_1_ESC_Pin 23
#define Motor_1_ESC_Csatorna 0

#define Motor_2_ESC_Pin 19
#define Motor_2_ESC_Csatorna 1

#define Motor_3_ESC_Pin 18
#define Motor_3_ESC_Csatorna 2

#define Motor_4_ESC_Pin 17
#define Motor_4_ESC_Csatorna 3

#define feszultseg_meres_PIN 36
#define aram_meres_PIN 39
#define Piros_LED 13
#define Zold_LED 4
#pragma endregion

#pragma region 03_IBUS_es_Failsafe
uint8_t IBUS_Buffer[32];
uint8_t IBUS_Index = 0;

uint32_t Utolso_IBUS_Csomag_Ido = 0;

bool IBUS_Kapcsolat_El = false;
bool Failsafe = true;

uint16_t Vevo_Ertekek[] = { 1500, 1500, 1000, 1500, 1000, 1000 };
bool EStop_Allapot = false;
bool Stabilization_Mode_Aktiv = true;
bool Elozo_Stabilization_Mode_Aktiv = true;
bool EStop_LED = false;
#pragma endregion

#pragma region 04_Rate_adatok
//számított szögsebességeknek változó
float Rate_Roll, Rate_Pitch, Rate_Yaw;
float Szurt_Roll_Rate, Szurt_Pitch_Rate, Szurt_Yaw_Rate;

//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Offset = 0;
float Rate_Pitch_Offset = 0;
float Rate_Yaw_Offset = 0;

float Kivant_Roll_Rate, Kivant_Pitch_Rate, Kivant_Yaw_Rate;
float Hiba_Roll_Rate, Hiba_Pitch_Rate, Hiba_Yaw_Rate;
float Elozo_Hiba_Roll_Rate, Elozo_Hiba_Pitch_Rate, Elozo_Hiba_Yaw_Rate;
float Elozo_Iterm_Roll_Rate, Elozo_Iterm_Pitch_Rate, Elozo_Iterm_Yaw_Rate;
#pragma endregion

#pragma region 05_Angle_adatok
//számított szög változó
float Angle_Roll, Angle_Pitch;
float Acc_X, Acc_Y, Acc_Z;

float Kalman_Angle_Roll = 0;
float Kalman_Bizonytalansag_Angle_Roll = 2 * 2;  //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_Angle_Pitch = 0;
float Kalman_Bizonytalansag_Angle_Pitch = 2 * 2;  //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_1D_kimenet[] = { 0, 0 };

//fly sky távirányítóval beáállított szög -50°-50°
float Kivant_Angle_Roll, Kivant_Angle_Pitch;

//kívánt szög - kálmán által mért szög
float Hiba_Angle_Roll, Hiba_Angle_Pitch;
float Elozo_Hiba_Angle_Roll, Elozo_Hiba_Angle_Pitch;
float Elozo_Iterm_Angle_Roll, Elozo_Iterm_Angle_Pitch;
#pragma endregion

#pragma region 06_Velocity_adatok
//16 bites kalibrációs adatok mentésére változók
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

float Acc_Z_gyorsulas;
float Baro_magassag;
float Baro_magassag_kezdetben = 0.0;
int Baro_kalibracios_szam;
float Kalman_Magassag, Kalman_Fuggoleges_sebesseg;
float Kivant_Fuggoleges_sebesseg, Hiba_Fuggoleges_sebesseg, Elozo_Hiba_Fuggoleges_sebesseg, Elozo_Iterm_Fuggoleges_sebesseg;
uint32_t Utolso_Baro_Frissites = 0;

//mátrixváltozók deklarálása <sor,oszlop>
BLA::Matrix<2, 2> F;
BLA::Matrix<2, 1> G;
BLA::Matrix<2, 2> P;
BLA::Matrix<2, 2> Q;
BLA::Matrix<2, 1> S;
BLA::Matrix<1, 2> H;
BLA::Matrix<2, 2> I;
BLA::Matrix<1, 1> U;
BLA::Matrix<2, 1> K;
BLA::Matrix<1, 1> R;
BLA::Matrix<1, 1> L;
BLA::Matrix<1, 1> M;
#pragma endregion

#pragma region 07_PID_adatok
float PIDReturn[] = { 0, 0, 0 };
bool Iterm_Aktiv = false;

//PID Rate
float P_Rate_Roll = 0.485, P_Rate_Pitch = P_Rate_Roll, P_Rate_Yaw = 0.55;
float I_Rate_Roll = 1.435, I_Rate_Pitch = I_Rate_Roll, I_Rate_Yaw = 1.4;
float D_Rate_Roll = 0.01075, D_Rate_Pitch = D_Rate_Roll, D_Rate_Yaw = 0;
//PID Angle
float P_Angle_Roll = 2, P_Angle_Pitch = P_Angle_Roll;
float I_Angle_Roll = 0, I_Angle_Pitch = I_Angle_Roll;
float D_Angle_Roll = 0, D_Angle_Pitch = D_Angle_Roll;
//PID Velocity
float P_Fuggoleges_sebesseg = 3.5;
float I_Fuggoleges_sebesseg = 0.0015;
float D_Fuggoleges_sebesseg = 0.01;
#pragma endregion

#pragma region 08_Motorvaltozok_es_Loop_valtozok
float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

// 250 Hz-es loop vizsgálatához
uint32_t Loop_Timer;
float Ts = 0.004;  //250 Hz-es loop miatt
uint32_t Elozo_Loop_Kezdet = 0;
uint32_t Ciklus_Ido = 0;
uint32_t Munka_Ido = 0;
uint32_t Utolso_Kiiras = 0;
#pragma endregion

#pragma region 09_Akkumulator_meres
float Feszultseg;
float Aram;
uint32_t Aksi_meres;
float Aksi_maradek;                 // %-ban
float Aksi_kezdetben;               // mAh-ban
float Elfogyasztott_kapacitas = 0;  // mAh-ban
float Alap_aksi_kapacitas = 2000;   // 2000mAh
#pragma endregion
#pragma endregion

#pragma region 2.Fuggvenyek

void kalman_2d_elorejelzes(void) {
  // Előrejelzés
  U = { Acc_Z_gyorsulas };

  S = F * S + G * U;
  P = F * P * ~F + Q;
}

void kalman_2d_baro_korrekcio(float Baro_relativ_cm) {
  // Barometrikus korrekció
  M = { Baro_relativ_cm };

  L = H * P * ~H + R;
  K = P * ~H * Inverse(L);

  S = S + K * (M - H * S);
  P = (I - K * H) * P;
}

void kalman_2d_kimenet_frissites(void) {
  Kalman_Magassag = S(0, 0);
  Kalman_Fuggoleges_sebesseg = S(1, 0);
}

bool baro_adat_olvasas(void) {

  Wire.beginTransmission(0x77);
  Wire.write(0xF7);  //itt kezdődik az adatok kiolvasása
  Wire.endTransmission();
  Wire.requestFrom(0x77, 6);          //6 bájt kérése
  uint32_t press_msb = Wire.read();   //0xF7
  uint32_t press_lsb = Wire.read();   //0xF8
  uint32_t press_xlsb = Wire.read();  //0xF9
  uint32_t temp_msb = Wire.read();    //0xFA
  uint32_t temp_lsb = Wire.read();    //0xFB
  uint32_t temp_xlsb = Wire.read();   //0xFC

  //nyers, kompennzálatlan és kalibrálatlan nyomás
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  //nyers, kompennzálatlan és kalibrálatlan hőmérséklet
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

  // A hőmérsékletet °C-ban adja vissza, a felbontás 0,01 °C.
  // Az „5123” kimeneti érték 51,23 °C-nak felel meg.
  // A t_fine globális változóként a pontosított hőmérsékletértéket tárolja.
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;

  // A nyomást Pa-ban adja vissza, előjel nélküli 32 bites egész számként.
  // A „96386” kimeneti érték 96386 Pa-nak felel meg = 963,86 hPa.
  unsigned long int p;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0) {
    return false;
  }  // Kerüld el a nullával való osztás miatt fellépő kivételt.
  p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((unsigned long int)var1);
  } else {
    p = (p / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
  p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));  //eredmény Pa-ban
  double nyomas = (double)p / 100;                                              //átváltjuk hPa-ra

  //A BMP280 adatlap szerinti nyomástartomány körülbelül 300–1100 hPa
  if (!isfinite(nyomas) || nyomas < 300.0 || nyomas > 1100.0) {
    return false;
  }

  Baro_magassag = 44330 * (1 - pow(nyomas / 1013.25, 1.0 / 5.255)) * 100;  //magasság kiszámítása (*100 m és cm közötti áltváltás)

  //Az isfinite() azt ellenőrzi, hogy egy lebegőpontos szám valódi, véges szám-e.
  if (!isfinite(Baro_magassag)) {
    return false;
  }
  return true;
}

void baro_regiszterek_beallitasa(void) {

  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x57);  //01010111 binárisan
  Wire.endTransmission();

  Wire.beginTransmission(0x77);
  Wire.write(0xF5);
  Wire.write(0x14);  //00010100 binárisan
  Wire.endTransmission();
}

void baro_kalibracio_olvasas(void) {
  uint8_t adat[24], i = 0;

  //kalibrációs adatok kiolvasása
  Wire.beginTransmission(0x77);
  Wire.write(0x88);  //itt kezdődik a dig_T1
  Wire.endTransmission();
  Wire.requestFrom(0x77, 24);  //kérünk 24 byte-ot

  while (Wire.available()) {
    adat[i] = Wire.read();  //24 byte mentése tömbe
    i++;
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
    if (baro_adat_olvasas()) {
      Baro_magassag_kezdetben += Baro_magassag;
      Ervenyes_mintak++;
    }

    delay(40);
  }

  Baro_magassag_kezdetben /= Ervenyes_mintak;
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

void aksi_feszultseg(void) {
  Feszultseg = (float)analogRead(feszultseg_meres_PIN) / 206.818182 + 1.2;  //4095/(3.3*6)=206.8182
  Aram = (float)analogRead(aram_meres_PIN) / 37.227273;                     // ADC / ((4095/3.3V)*0.03 V/A)=37.227273 ---> ADC*0,02686203
}

void IBUS_Olvasas(void) {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (IBUS_Index == 0) {
      if (b != 0x20) {
        continue;
      }
    }

    if (IBUS_Index == 1) {
      if (b != 0x40) {
        IBUS_Index = 0;
        continue;
      }
    }

    IBUS_Buffer[IBUS_Index++] = b;

    if (IBUS_Index >= 32) {
      IBUS_Index = 0;

      uint16_t Checksum = 0xFFFF;

      for (uint8_t i = 0; i < 30; i++) {
        Checksum -= IBUS_Buffer[i];
      }

      uint16_t Kapott_Checksum = IBUS_Buffer[30] | (IBUS_Buffer[31] << 8);

      if (Checksum == Kapott_Checksum) {
        uint16_t Temp[6];

        Temp[0] = IBUS_Buffer[2] | (IBUS_Buffer[3] << 8);    // CH1 Roll
        Temp[1] = IBUS_Buffer[4] | (IBUS_Buffer[5] << 8);    // CH2 Pitch
        Temp[2] = IBUS_Buffer[6] | (IBUS_Buffer[7] << 8);    // CH3 Gáz
        Temp[3] = IBUS_Buffer[8] | (IBUS_Buffer[9] << 8);    // CH4 Yaw
        Temp[4] = IBUS_Buffer[10] | (IBUS_Buffer[11] << 8);  // CH5 SWA EStop
        Temp[5] = IBUS_Buffer[12] | (IBUS_Buffer[13] << 8);  // CH6 SWD Módváltó

        bool Ervenyes = true;

        if (Temp[0] < 900 || Temp[0] > 2100) Ervenyes = false;
        if (Temp[1] < 900 || Temp[1] > 2100) Ervenyes = false;
        if (Temp[2] < 900 || Temp[2] > 2100) Ervenyes = false;
        if (Temp[3] < 900 || Temp[3] > 2100) Ervenyes = false;
        if (Temp[4] < 900 || Temp[4] > 2100) Ervenyes = false;
        if (Temp[5] < 900 || Temp[5] > 2100) Ervenyes = false;

        if (Ervenyes) {
          Vevo_Ertekek[0] = Temp[0];
          Vevo_Ertekek[1] = Temp[1];
          Vevo_Ertekek[2] = Temp[2];
          Vevo_Ertekek[3] = Temp[3];
          Vevo_Ertekek[4] = Temp[4];
          Vevo_Ertekek[5] = Temp[5];

          Utolso_IBUS_Csomag_Ido = millis();

          IBUS_Kapcsolat_El = true;
          Failsafe = false;

          EStop_Allapot = Vevo_Ertekek[4] > 1500;

          if (Vevo_Ertekek[5] < 1400) {
            Stabilization_Mode_Aktiv = true;
          } else if (Vevo_Ertekek[5] > 1600) {
            Stabilization_Mode_Aktiv = false;
          }
        }
      }
    }
  }
}

void IBUS_Failsafe_Ellenorzes(void) {
  uint32_t most = millis();

  if (most - Utolso_IBUS_Csomag_Ido > 100) {
    IBUS_Kapcsolat_El = false;
    Failsafe = true;
  }

  if (Failsafe) {
    Vevo_Ertekek[0] = 1500;
    Vevo_Ertekek[1] = 1500;
    Vevo_Ertekek[2] = 1000;
    Vevo_Ertekek[3] = 1500;
    Vevo_Ertekek[4] = 1000;
    Vevo_Ertekek[5] = 1000;
    EStop_Allapot = false;
    Stabilization_Mode_Aktiv = false;
  }
}

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

void PID_egyenlet(float Hiba, float P, float I, float D, float Elozo_Hiba, float Elozo_Iterm) {
  float Pterm = P * Hiba;
  float Iterm = Elozo_Iterm + (I * (Hiba + Elozo_Hiba)) * (Ts / 2);

  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;

  float Dterm = D * ((Hiba - Elozo_Hiba) / Ts);
  float PID_Kimenet = Pterm + Iterm + Dterm;

  if (PID_Kimenet > 400) PID_Kimenet = 400;
  else if (PID_Kimenet < -400) PID_Kimenet = -400;

  PIDReturn[0] = PID_Kimenet;
  PIDReturn[1] = Hiba;
  PIDReturn[2] = Iterm;
}

void Reset_PID(void) {
  //Rate reset
  Elozo_Hiba_Roll_Rate = 0;
  Elozo_Hiba_Pitch_Rate = 0;
  Elozo_Hiba_Yaw_Rate = 0;

  Elozo_Iterm_Roll_Rate = 0;
  Elozo_Iterm_Pitch_Rate = 0;
  Elozo_Iterm_Yaw_Rate = 0;

  //Angle reset
  Elozo_Hiba_Angle_Roll = 0;
  Elozo_Hiba_Angle_Pitch = 0;

  Elozo_Iterm_Angle_Roll = 0;
  Elozo_Iterm_Angle_Pitch = 0;

  //Velocity reset
  Elozo_Hiba_Fuggoleges_sebesseg = 0;
  Elozo_Iterm_Fuggoleges_sebesseg = 0;
}

void Reset_Angle_PID() {
  Elozo_Hiba_Angle_Roll = 0;
  Elozo_Hiba_Angle_Pitch = 0;

  Elozo_Iterm_Angle_Roll = 0;
  Elozo_Iterm_Angle_Pitch = 0;
}

uint32_t usToDuty(float motor_bemenet_us) {
  motor_bemenet_us = constrain(motor_bemenet_us, 1000, 2000);
  return (uint32_t)(motor_bemenet_us * 4095.0f / 4000.0f);
}

int RC_Deadband(int ertek) {
  if (ertek > 1508) return ertek - 1508;
  if (ertek < 1492) return ertek - 1492;
  return 0;
}
#pragma endregion

void setup() {

  Serial.begin(115200);
  pinMode(RX_PIN, INPUT_PULLUP);

  Serial2.setRxBufferSize(512);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  analogReadResolution(12);
  analogSetPinAttenuation(feszultseg_meres_PIN, ADC_11db);
  analogSetPinAttenuation(aram_meres_PIN, ADC_11db);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Piros_LED, OUTPUT);
  pinMode(Zold_LED, OUTPUT);

  digitalWrite(Piros_LED, HIGH);
  digitalWrite(Zold_LED, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  //Motor 1 (Jobb hátsó CW)
  ledcSetup(Motor_1_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_1_ESC_Pin, Motor_1_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //Motor 2 (Jobb első CCW)
  ledcSetup(Motor_2_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_2_ESC_Pin, Motor_2_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //Motor 3 (Bal hátsó CCW)
  ledcSetup(Motor_3_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_3_ESC_Pin, Motor_3_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //Motor 4 (Bal első CW)
  ledcSetup(Motor_4_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_4_ESC_Pin, Motor_4_ESC_Csatorna);                                    //ESP32 core 2.0.17

  // 250Hz=4ms=4000us
  // 12 bites felbontás = 0-4095
  // 0 ---> 0us        4095--->4000us
  // PWM jel küldése us-ban az ESC nek akkor 4095/4000=1.024-el kell majd szorozi 250Hz, 12 bit esetén

  ledcWrite(Motor_1_ESC_Csatorna, usToDuty(1000));
  ledcWrite(Motor_2_ESC_Csatorna, usToDuty(1000));
  ledcWrite(Motor_3_ESC_Csatorna, usToDuty(1000));
  ledcWrite(Motor_4_ESC_Csatorna, usToDuty(1000));

  digitalWrite(LED_BUILTIN, HIGH);

  delay(3000);

  Wire.begin();           //SDA, SCL
  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  delay(250);
  mpu_6050_regiszterek_beallitasa();
  baro_regiszterek_beallitasa();
  baro_kalibracio_olvasas();

  F = { 1, 0.004,
        0, 1 };
  G = { 0.5 * 0.004 * 0.004,
        0.004 };
  H = { 1, 0 };
  I = { 1, 0,
        0, 1 };
  Q = G * ~G * 10.0f * 10.0f;
  R = { 30.0f * 30.0f };
  P = { 0, 0,
        0, 0 };
  S = { 0,
        0 };

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

  aksi_feszultseg();
  if (Feszultseg > 16.6) {                 // terhelés nélkűl mérjük az aksit így valós értéket kapunk mivel terhelés alatt beesik a feszülség szintje és nem lesz valós érték ezért terhlés alatt áramot mmérünk
    digitalWrite(Piros_LED, LOW);          //piros led kikapcs
    Aksi_kezdetben = Alap_aksi_kapacitas;  //felöltve tehát 1300mAh
  } else if (Feszultseg < 15) {
    digitalWrite(Piros_LED, HIGH);
    Aksi_kezdetben = (float)30 / 100 * Alap_aksi_kapacitas;  //30% át vesszük
  } else {
    digitalWrite(Piros_LED, LOW);
    Aksi_kezdetben = (41 * Feszultseg - 580) / 100 * Alap_aksi_kapacitas;
  }

  digitalWrite(Zold_LED, HIGH);

  // kontrollálatlan motorindítás elkerülése
  /*while (Vevo_Ertekek[2] < 1020 || Vevo_Ertekek[2] > 1050 || Failsafe) {  //Vevo_Ertekek[2] ---> Gáz kar értéke  kontrollálatlan motorindítás elkerülése
    IBUS_Olvasas();
    IBUS_Failsafe_Ellenorzes();
    delay(4);
  }*/

  Loop_Timer = micros();
  Elozo_Loop_Kezdet = Loop_Timer;
}

void loop() {
  uint32_t Loop_Kezdet = micros();

  Ciklus_Ido = Loop_Kezdet - Elozo_Loop_Kezdet;
  Elozo_Loop_Kezdet = Loop_Kezdet;

  digitalWrite(LED_BUILTIN, EStop_LED);

  //Gyro adat olvasó fgv meghívása
  gyro_acc_adat_olvasas();

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  //elsőrendű exponenciális IIR aluláteresztő szűrés RATE MODE PID-hez hiba számoláshoz
  Szurt_Roll_Rate = Szurt_Roll_Rate * 0.7 + Rate_Roll * 0.3;
  Szurt_Pitch_Rate = Szurt_Pitch_Rate * 0.7 + Rate_Pitch * 0.3;
  Szurt_Yaw_Rate = Szurt_Yaw_Rate * 0.7 + Rate_Yaw * 0.3;

  //Kalman Angle Roll számítás
  kalman_1d(Kalman_Angle_Roll, Kalman_Bizonytalansag_Angle_Roll, Rate_Roll, Angle_Roll);
  Kalman_Angle_Roll = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Angle_Roll = Kalman_1D_kimenet[1];

  //Kalman Angle Pitch számítás
  kalman_1d(Kalman_Angle_Pitch, Kalman_Bizonytalansag_Angle_Pitch, Rate_Pitch, Angle_Pitch);
  Kalman_Angle_Pitch = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Angle_Pitch = Kalman_1D_kimenet[1];

  //Az inerciális Z tengely irányába eső gyorsulás kiszámítása
  Acc_Z_gyorsulas = -sin(Kalman_Angle_Pitch * (PI / 180)) * Acc_X + cos(Kalman_Angle_Pitch * (PI / 180)) * sin(Kalman_Angle_Roll * (PI / 180)) * Acc_Y + cos(Kalman_Angle_Pitch * (PI / 180)) * cos(Kalman_Angle_Roll * (PI / 180)) * Acc_Z;
  //gyorsulás átváltása cm/s^2-re
  Acc_Z_gyorsulas = (Acc_Z_gyorsulas - 1) * 9.81 * 100;

  // Gyorsulásos előrejelzés minden 4 ms-ban 250Hz-en
  kalman_2d_elorejelzes();

  uint32_t Most_us = micros();

  //Baro korrekció 25Hz-enként mivel a beállítások mellett a BMP280 körülbelül 26,3 Hz sebességgel készít új mintát
  if (Most_us - Utolso_Baro_Frissites >= 40000) {  // 40 000us = 40ms = 25Hz
    Utolso_Baro_Frissites = Most_us;

    if (baro_adat_olvasas()) {
      float Baro_relativ_cm = Baro_magassag - Baro_magassag_kezdetben;

      kalman_2d_baro_korrekcio(Baro_relativ_cm);
    }
  }

  kalman_2d_kimenet_frissites();

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  if (Failsafe) {
    Iterm_Aktiv = false;

    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;

    ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));
    ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));
    ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));
    ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));

    Reset_PID();

    Serial.print("Failsafe: ");
    Serial.print(Failsafe);
    Serial.print(" SWA: ");
    Serial.print(EStop_LED);
    Serial.print(" SWD: ");
    Serial.println(Stabilization_Mode_Aktiv);

    while (micros() - Loop_Timer < 4000)
      ;
    Loop_Timer = micros();
    return;
  }

  if (EStop_Allapot == 1) {
    EStop_LED = true;
    Iterm_Aktiv = false;

    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;

    ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));
    ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));
    ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));
    ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));

    Reset_PID();

    Serial.print("Failsafe: ");
    Serial.print(Failsafe);
    Serial.print(" SWA: ");
    Serial.print(EStop_LED);
    Serial.print(" SWD: ");
    Serial.println(Stabilization_Mode_Aktiv);

    while (micros() - Loop_Timer < 4000)
      ;

    Loop_Timer = micros();
    return;
  }

  EStop_LED = false;

  //ANGLE -> RATE módváltás érzékelése
  if (!Stabilization_Mode_Aktiv && Elozo_Stabilization_Mode_Aktiv) {
    Reset_Angle_PID();
  }
  // Aktuális állapot eltárolása következő ciklusra
  Elozo_Stabilization_Mode_Aktiv = Stabilization_Mode_Aktiv;

  if (Stabilization_Mode_Aktiv) {
    // ANGLE MÓD
    Kivant_Angle_Roll = 0.10 * RC_Deadband(Vevo_Ertekek[0]);
    Kivant_Angle_Pitch = 0.10 * RC_Deadband(Vevo_Ertekek[1]);
  } else {
    // RATE MÓD
    Kivant_Roll_Rate = 0.15 * RC_Deadband(Vevo_Ertekek[0]);
    Kivant_Pitch_Rate = 0.15 * RC_Deadband(Vevo_Ertekek[1]);
  }

  //Számítsd ki a kívánt gázt és Yaw-t a vevő (receiver) értékeiből.
  // VELOCITY MÓD
  Kivant_Fuggoleges_sebesseg = 0.3 * (Vevo_Ertekek[2] - 1500);

  // SIMA GÁZ PARANCS
  //Bemenet_Gaz = Vevo_Ertekek[2];

  // YAW PARANCS
  Kivant_Yaw_Rate = 0.15 * RC_Deadband(Vevo_Ertekek[3]);

  if (Vevo_Ertekek[2] > 1130) Iterm_Aktiv = true;
  //ternary operator: feltétel ? érték ha igaz : érték ha hamis

  //Angle
  float I_Angle_Roll_Aktiv = Iterm_Aktiv ? I_Angle_Roll : 0.0;
  float I_Angle_Pitch_Aktiv = Iterm_Aktiv ? I_Angle_Pitch : 0.0;
  //Rate
  float I_Rate_Roll_Aktiv = Iterm_Aktiv ? I_Rate_Roll : 0.0;
  float I_Rate_Pitch_Aktiv = Iterm_Aktiv ? I_Rate_Pitch : 0.0;
  float I_Rate_Yaw_Aktiv = Iterm_Aktiv ? I_Rate_Yaw : 0.0;
  //Velocity
  float I_Fuggoleges_sebesseg_Aktiv = Iterm_Aktiv ? I_Fuggoleges_sebesseg : 0.0;

  Hiba_Fuggoleges_sebesseg = Kivant_Fuggoleges_sebesseg - Kalman_Fuggoleges_sebesseg;

  //Velocity PID
  PID_egyenlet(Hiba_Fuggoleges_sebesseg, P_Fuggoleges_sebesseg, I_Fuggoleges_sebesseg_Aktiv, D_Fuggoleges_sebesseg, Elozo_Hiba_Fuggoleges_sebesseg, Elozo_Iterm_Fuggoleges_sebesseg);
  Bemenet_Gaz = 1500 + PIDReturn[0];  //Mivel a nulla sebességhez, tehát a lebegéshez tartozó pont várhatóan a gázkar középállása körül lesz — 1500 µs-nál —, ezt az értéket add hozzá a PID kimenetéhez.
  Elozo_Hiba_Fuggoleges_sebesseg = PIDReturn[1];
  Elozo_Iterm_Fuggoleges_sebesseg = PIDReturn[2];

  if (Stabilization_Mode_Aktiv) {
    //Számítsd ki a különbséget a kívánt és a tényleges roll és pitch szögek között.
    Hiba_Angle_Roll = Kivant_Angle_Roll - Kalman_Angle_Roll;
    Hiba_Angle_Pitch = Kivant_Angle_Pitch - Kalman_Angle_Pitch;

    //PID Angle Roll
    PID_egyenlet(Hiba_Angle_Roll, P_Angle_Roll, I_Angle_Roll_Aktiv, D_Angle_Roll, Elozo_Hiba_Angle_Roll, Elozo_Iterm_Angle_Roll);
    Kivant_Roll_Rate = PIDReturn[0];
    Kivant_Roll_Rate = constrain(Kivant_Roll_Rate, -220, 220);
    Elozo_Hiba_Angle_Roll = PIDReturn[1];
    Elozo_Iterm_Angle_Roll = PIDReturn[2];

    //PID Angle Pitch
    PID_egyenlet(Hiba_Angle_Pitch, P_Angle_Pitch, I_Angle_Pitch_Aktiv, D_Angle_Pitch, Elozo_Hiba_Angle_Pitch, Elozo_Iterm_Angle_Pitch);
    Kivant_Pitch_Rate = PIDReturn[0];
    Kivant_Pitch_Rate = constrain(Kivant_Pitch_Rate, -220, 220);
    Elozo_Hiba_Angle_Pitch = PIDReturn[1];
    Elozo_Iterm_Angle_Pitch = PIDReturn[2];
  }

  Hiba_Roll_Rate = Kivant_Roll_Rate - Szurt_Roll_Rate;
  Hiba_Pitch_Rate = Kivant_Pitch_Rate - Szurt_Pitch_Rate;
  Hiba_Yaw_Rate = Kivant_Yaw_Rate - Szurt_Yaw_Rate;

  // PID Rate Roll
  PID_egyenlet(Hiba_Roll_Rate, P_Rate_Roll, I_Rate_Roll_Aktiv, D_Rate_Roll, Elozo_Hiba_Roll_Rate, Elozo_Iterm_Roll_Rate);
  Bemenet_Roll = PIDReturn[0];
  Elozo_Hiba_Roll_Rate = PIDReturn[1];
  Elozo_Iterm_Roll_Rate = PIDReturn[2];

  //PID Rate Pitch
  PID_egyenlet(Hiba_Pitch_Rate, P_Rate_Pitch, I_Rate_Pitch_Aktiv, D_Rate_Pitch, Elozo_Hiba_Pitch_Rate, Elozo_Iterm_Pitch_Rate);
  Bemenet_Pitch = PIDReturn[0];
  Elozo_Hiba_Pitch_Rate = PIDReturn[1];
  Elozo_Iterm_Pitch_Rate = PIDReturn[2];

  //PID Rate Yaw
  PID_egyenlet(Hiba_Yaw_Rate, P_Rate_Yaw, I_Rate_Yaw_Aktiv, D_Rate_Yaw, Elozo_Hiba_Yaw_Rate, Elozo_Iterm_Yaw_Rate);
  Bemenet_Yaw = PIDReturn[0];
  Elozo_Hiba_Yaw_Rate = PIDReturn[1];
  Elozo_Iterm_Yaw_Rate = PIDReturn[2];

  //PID-nek hagyni korrekciós lehetőséget
  if (Bemenet_Gaz > 1800) Bemenet_Gaz = 1800;

  //250Hz 12 bit felbontás
  Motor_1_bemenet = (Bemenet_Gaz - Bemenet_Roll + Bemenet_Pitch + Bemenet_Yaw);  // 100 %
  Motor_2_bemenet = (Bemenet_Gaz - Bemenet_Roll - Bemenet_Pitch - Bemenet_Yaw);  //   0 %
  Motor_3_bemenet = (Bemenet_Gaz + Bemenet_Roll + Bemenet_Pitch - Bemenet_Yaw);  // 100 %
  Motor_4_bemenet = (Bemenet_Gaz + Bemenet_Roll - Bemenet_Pitch + Bemenet_Yaw);  // 100 %

  if (Motor_1_bemenet > 2000) Motor_1_bemenet = 2000;
  if (Motor_2_bemenet > 2000) Motor_2_bemenet = 2000;
  if (Motor_3_bemenet > 2000) Motor_3_bemenet = 2000;
  if (Motor_4_bemenet > 2000) Motor_4_bemenet = 2000;

  //18%-os alapjárat a motoroknak
  if (Motor_1_bemenet < Motorok_alapjarat) Motor_1_bemenet = Motorok_alapjarat;
  if (Motor_2_bemenet < Motorok_alapjarat) Motor_2_bemenet = Motorok_alapjarat;
  if (Motor_3_bemenet < Motorok_alapjarat) Motor_3_bemenet = Motorok_alapjarat;
  if (Motor_4_bemenet < Motorok_alapjarat) Motor_4_bemenet = Motorok_alapjarat;

  //ha leszállni szeretnénk
  if (Vevo_Ertekek[2] < 1030) {
    Iterm_Aktiv = false;
    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;
    Reset_PID();
  }

  //Motor parancsok (ESP32 core 2.0.17)
  ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));
  ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));
  ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));
  ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));

  //Aksi mérés
  if (millis() - Aksi_meres >= 100) {
    Aksi_meres = millis();
    aksi_feszultseg();

    Elfogyasztott_kapacitas = Aram * 1000 * 0.1 / 3600 + Elfogyasztott_kapacitas;
    Aksi_maradek = (Aksi_kezdetben - Elfogyasztott_kapacitas) / Alap_aksi_kapacitas * 100;

    if (Aksi_maradek <= 30) digitalWrite(Piros_LED, HIGH);
    else digitalWrite(Piros_LED, LOW);

    /*Serial.print("Failsafe: ");
    Serial.print(Failsafe);
    Serial.print(" SWA: ");
    Serial.print(EStop_LED); */

    /*Serial.print("R: ");
    Serial.print(Kivant_Roll_Rate);
    Serial.print(" P: ");
    Serial.print(Kivant_Pitch_Rate);
    Serial.print(" Y: ");
    Serial.print(Kivant_Yaw_Rate);
    Serial.print(" SWD: ");*/

    /*Serial.print("R: ");
    Serial.print(Kivant_Angle_Roll);
    Serial.print(" P: ");
    Serial.print(Kivant_Angle_Pitch);
    Serial.print(" Y: ");
    Serial.print(Kivant_Yaw_Rate);*/

    /*Serial.print("M1: ");
    Serial.print(Motor_1_bemenet);
    Serial.print(" M2: ");
    Serial.print(Motor_2_bemenet);
    Serial.print(" M3: ");
    Serial.print(Motor_3_bemenet);
    Serial.print(" M4: ");
    Serial.print(Motor_4_bemenet);
    Serial.print(" GAZ: ");
    Serial.print(Bemenet_Gaz);*/

    /*Serial.print("X: ");
    Serial.print(Acc_X,4);
    Serial.print(" Y: ");
    Serial.print(Acc_Y,4);
    Serial.print(" Z: ");
    Serial.print(Acc_Z,4);*/

    /*Serial.print("R: ");
    Serial.print(Rate_Roll);
    Serial.print(" P: ");
    Serial.print(Rate_Pitch);
    Serial.print(" Y: ");
    Serial.print(Rate_Yaw);*/

    Serial.print(" Szog_R: ");
    Serial.print(Angle_Roll);
    Serial.print(" Szog_P: ");
    Serial.println(Angle_Pitch);

    /*Serial.print(" Szog_R: ");
    Serial.print(Kalman_Angle_Roll);
    Serial.print(" Szog_P: ");
    Serial.print(Kalman_Angle_Pitch);*/

    /*Serial.print("Kivant_fg_v: ");
    Serial.print(Kivant_Fuggoleges_sebesseg);
    Serial.print(" Kalman_fg_v: ");
    Serial.print(Kalman_Fuggoleges_sebesseg);
    Serial.print(" Gaz: ");
    Serial.print(Bemenet_Gaz);
    Serial.print(" Gaz_kar: ");
    Serial.print(Vevo_Ertekek[2]);
    Serial.print(" Kalman_h: ");
    Serial.print(Kalman_Magassag);

    Serial.print(" SWD: ");
    Serial.println(Stabilization_Mode_Aktiv);*/
  }

  Munka_Ido = micros() - Loop_Kezdet;

  /*if (millis() - Utolso_Kiiras >= 50) {

    Utolso_Kiiras = millis();

    Serial.print("Ciklusido: ");
    Serial.print(Ciklus_Ido);
    Serial.print(" us | Munkaido: ");
    Serial.print(Munka_Ido);
    Serial.println(" us");
  }*/


  while (micros() - Loop_Timer < 4000)
    ;
  Loop_Timer = micros();
}