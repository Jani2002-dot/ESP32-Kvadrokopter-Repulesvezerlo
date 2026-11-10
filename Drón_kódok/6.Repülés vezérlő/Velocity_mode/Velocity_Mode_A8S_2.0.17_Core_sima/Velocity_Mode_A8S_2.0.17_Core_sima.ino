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

uint16_t Vevo_Ertekek[] = { 1500, 1500, 1000, 1500, 1000, 1000, 1000 };
bool EStop_Allapot = false;
bool Angle_Mode_Aktiv = false;
bool Velocity_Mode_Aktiv = false;
bool Elozo_Angle_Mode_Aktiv = false;
bool EStop_LED = false;
bool Arm_engedelyezve = false;
uint32_t Arm_Tartas_Kezdete = 0;
bool Arm_Tartas_Folyamatban = false;
#pragma endregion

#pragma region 04_Rate_adatok
//számított szögsebességeknek változó
float Rate_Roll, Rate_Pitch, Rate_Yaw;
float Szurt_Rate_Roll, Szurt_Rate_Pitch, Szurt_Rate_Yaw;

//2500 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Offset = 0;
float Rate_Pitch_Offset = 0;
float Rate_Yaw_Offset = 0;

float Kivant_Rate_Roll, Kivant_Rate_Pitch, Kivant_Rate_Yaw;
float Hiba_Rate_Roll, Hiba_Rate_Pitch, Hiba_Rate_Yaw;
float Elozo_Hiba_Rate_Roll, Elozo_Hiba_Rate_Pitch, Elozo_Hiba_Rate_Yaw;
float Elozo_Iterm_Rate_Roll, Elozo_Iterm_Rate_Pitch, Elozo_Iterm_Rate_Yaw;
#pragma endregion

#pragma region 05_Angle_adatok
//számított szög változó
float Angle_Roll, Angle_Pitch;
float Acc_X, Acc_Y, Acc_Z;

float Kalman_Angle_Roll = 0;                     //kezdeti szögbecslést 0°-ra állítottam, mivel az indítás közel vízszintes helyzetben történik
float Kalman_Bizonytalansag_Angle_Roll = 2 * 2;  //bizonytalanságát 2° kezdeti szórással, azaz 4°^2 hibavarianciával vettem figyelembe

float Kalman_Angle_Pitch = 0;                     //kezdeti szögbecslést 0°-ra állítottam, mivel az indítás közel vízszintes helyzetben történik
float Kalman_Bizonytalansag_Angle_Pitch = 2 * 2;  //bizonytalanságát 2° kezdeti szórással, azaz 4°^2 hibavarianciával vettem figyelembe

float Kalman_Szog_Kimenet[] = { 0, 0 };

//fly sky távirányítóval beáállított szög -50°-50°
float Kivant_Angle_Roll, Kivant_Angle_Pitch;

//kívánt szög - kálmán által mért szög
float Hiba_Angle_Roll, Hiba_Angle_Pitch;
float Elozo_Hiba_Angle_Roll, Elozo_Hiba_Angle_Pitch;
float Elozo_Iterm_Angle_Roll, Elozo_Iterm_Angle_Pitch;

float Ts = 0.004;  //250 Hz-es loop miatt 4ms

const int A_szog = 1, H_szog = 1, I_szog = 1;
float B_szog = Ts;
float Q_Szog = Ts * Ts * 4 * 4;
float R_szog = 3 * 3;
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
uint32_t BMP_meres;
bool Baro_frissites = false;

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
#pragma endregion

#pragma region 07_PID_adatok
float PID_kimenetek[] = { 0, 0, 0 };
bool Iterm_Aktiv = false;

//PID Rate
float P_Rate_Roll = 0.485, P_Rate_Pitch = P_Rate_Roll, P_Rate_Yaw = 0.55;
float I_Rate_Roll = 1.435, I_Rate_Pitch = I_Rate_Roll, I_Rate_Yaw = 1.4;
float D_Rate_Roll = 0.01075, D_Rate_Pitch = D_Rate_Roll, D_Rate_Yaw = 0;
//PID Angle
float P_Angle_Roll = 0.755, P_Angle_Pitch = P_Angle_Roll;
float I_Angle_Roll = 1.115, I_Angle_Pitch = I_Angle_Roll;
float D_Angle_Roll = 0, D_Angle_Pitch = D_Angle_Roll;
//PID Velocity
float P_Fuggoleges_sebesseg = 0;
float I_Fuggoleges_sebesseg = 0;
float D_Fuggoleges_sebesseg = 0;
#pragma endregion

#pragma region 08_Motorvaltozok_es_Ciklus_valtozok
float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;
int Motorok_max = 2000;

// 250 Hz-es loop vizsgálatához
uint32_t Ciklus_Idozito;
uint32_t Ciklus_Ido = 0;
uint32_t Ciklus_Ido_Max = 0;
uint32_t Elozo_Ciklus_Kezdet = 0;
uint32_t Piros_villogas = 0;
bool Piros_LED_Allapot = false;
#pragma endregion

#pragma region 09_Akkumulator_meres
float Feszultseg;
float Aram;
uint32_t Aksi_meres;
float Aksi_maradek;                 // %-ban
float Aksi_kezdetben;               // mAh-ban
float Elfogyasztott_kapacitas = 0;  // mAh-ban
float Alap_aksi_kapacitas = 2000;   // 2000mAh
bool Aksi_OK;
bool Aksi_Alacsony = false;
#pragma endregion
#pragma endregion

#pragma region 2.Fuggvenyek

void kalman_2d(void) {

  // Előrejelzés gyorsulásmérőből
  u = { Acc_Z_gyorsulas };

  xk = A * xk + B * u;
  P = A * P * ~A + Q;  // ~ Transzponáltat jelenti
  L = H * P * ~H + R;
  K = P * ~H * Inverse(L);  // Inverse(L) -1 hatványát veszi L nek

  // 25 Hz-es kálmán baro korrekció
  if (Baro_frissites) {
    // Barometrikus korrekció
    zk = { Baro_magassag };
    xk = xk + K * (zk - H * xk);
    P = (I - K * H) * P;
  }

  //Frissítés
  Kalman_Magassag = xk(0, 0);             //(sor,oszlop)
  Kalman_Fuggoleges_sebesseg = xk(1, 0);  //(sor,oszlop)
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

void Kalman_Szog(float x, float P, float u, float z) {
  x = A_szog * x + B_szog * u;
  P = A_szog * P * A_szog + Q_szog;
  float K = P * H_szog / (H_szog * P * H_szog + R_szog);
  x = x + K * (z - H_szog * x);
  P = (I_szog - K * H_szog) * P;
  Kalman_Szog_Kimenet[0] = x;  //Korrigált szög PID-nek és következő iteráccióba
  Kalman_Szog_Kimenet[1] = P;  //Korrigált hibakovariancia következő iterációba
}

void ADC_meres(void) {
  Feszultseg = analogReadMilliVolts(feszultseg_meres_PIN) / 1000.0f * 6.0f;  // Feszültség [V]
  Aram = analogReadMilliVolts(aram_meres_PIN) / 15.0f;                       // Áram [A] (30mV / A)
}


void IBUS_Olvasas(void) {
  while (Serial2.available() > 0) {
    uint8_t b = Serial2.read();

    // Keretkezdet: 0x20, majd 0x40.
    if (IBUS_Index == 0) {
      if (b != 0x20) continue;

      IBUS_Buffer[0] = b;
      IBUS_Index = 1;
      continue;
    }

    if (IBUS_Index == 1 && b != 0x40) {
      // Újabb 0x20 esetén új keret kezdődhet.
      IBUS_Index = (b == 0x20) ? 1 : 0;
      continue;
    }

    IBUS_Buffer[IBUS_Index++] = b;

    // A részleges keret a következő hívásig megmarad.
    if (IBUS_Index < 32) continue;
    IBUS_Index = 0;

    // Ellenőrző összeg számítása.
    uint16_t Checksum = 0xFFFF;

    for (uint8_t i = 0; i < 30; i++) {
      Checksum -= IBUS_Buffer[i];
    }

    uint16_t Kapott_Checksum =
      (uint16_t)IBUS_Buffer[30] | ((uint16_t)IBUS_Buffer[31] << 8);

    if (Checksum != Kapott_Checksum) continue;

    // A Velocity program hét csatornát használ.
    uint16_t Temp[7];
    bool Ervenyes = true;

    for (uint8_t i = 0; i < 7; i++) {
      uint8_t also = 2 + 2 * i;

      Temp[i] = (uint16_t)IBUS_Buffer[also] | ((uint16_t)IBUS_Buffer[also + 1] << 8);

      if (Temp[i] < 900 || Temp[i] > 2100) {
        Ervenyes = false;
      }
    }

    if (!Ervenyes) continue;

    // Csatornaértékek átvétele.
    for (uint8_t i = 0; i < 7; i++) {
      Vevo_Ertekek[i] = Temp[i];
    }

    // Csak elfogadott keret frissíti a kapcsolatot.
    Utolso_IBUS_Csomag_Ido = millis();
    IBUS_Kapcsolat_El = true;
    Failsafe = false;

    // CH5 - SWA: motorleállítás.
    if (Vevo_Ertekek[4] > 1600) {
      EStop_Allapot = true;
    } else if (Vevo_Ertekek[4] < 1400) {
      EStop_Allapot = false;
    }

    // CH6 - SWD: RATE / ANGLE mód.
    if (Vevo_Ertekek[5] > 1600) {
      Angle_Mode_Aktiv = true;
    } else if (Vevo_Ertekek[5] < 1400) {
      Angle_Mode_Aktiv = false;
    }

    // CH7 - SWB: VELOCITY mód.
    if (Vevo_Ertekek[6] > 1600) {
      Velocity_Mode_Aktiv = true;
    } else if (Vevo_Ertekek[6] < 1400) {
      Velocity_Mode_Aktiv = false;
    }
  }
}

void IBUS_Failsafe_Ellenorzes(void) {  // Paraméter és visszatérési érték nélküli függvény az iBUS-kapcsolat időtúllépésének ellenőrzésére.
  uint32_t most = millis();            // Eltárolja az aktuális időt milliszekundumban, 32 bites előjel nélküli változóban.

  if (most - Utolso_IBUS_Csomag_Ido > 100) {  // Ha az utolsó elfogadott csomag feldolgozása óta több mint 100 ms telt el:
    IBUS_Kapcsolat_El = false;                // A kapcsolatot inaktívnak jelöli.
    Failsafe = true;                          // Aktiválja a failsafe állapotot, amelyet a főciklus motorleállítási feltételként használ.
  }                                           // Az időtúllépés ellenőrzésének vége; itt nincs automatikus visszaállítás, azt az érvényes csomag fogadása végzi.

  if (Failsafe) {                 // Aktív failsafe esetén alapértékekre állítja a csatornákat és az üzemmódjelzőket.
    Vevo_Ertekek[0] = 1500;       // CH1: a roll-parancsot középállásra állítja.
    Vevo_Ertekek[1] = 1500;       // CH2: a pitch-parancsot középállásra állítja.
    Vevo_Ertekek[2] = 1000;       // CH3: a gázparancsot minimumra állítja.
    Vevo_Ertekek[3] = 1500;       // CH4: a yaw-parancsot középállásra állítja.
    Vevo_Ertekek[4] = 1000;       // CH5: az SWA csatornához az inaktív leállítókapcsolónak megfelelő értéket rendeli.
    Vevo_Ertekek[5] = 1000;       // CH6: az SWD csatornához a RATE módnak megfelelő értéket rendeli.
    Vevo_Ertekek[6] = 1000;       // CH7: az SWB csatornához a kikapcsolt velocity módnak megfelelő értéket rendeli.
    EStop_Allapot = false;        // Törli a kapcsolóból származó leállítási jelzőt; a Failsafe ettől még aktív marad.
    Angle_Mode_Aktiv = false;     // Kikapcsolja az ANGLE mód jelzőjét; ez nem engedélyezi a motorokat.
    Velocity_Mode_Aktiv = false;  // Kikapcsolja a függőleges sebesség szabályozásának üzemmódjelzőjét.
  }                               // A failsafe esetén végrehajtott alaphelyzetbe állítás vége.
}  // Visszatér a hívó programrészhez; a motorok leállítását, az ARM és a PID-állapotok törlését a főciklus végzi.

void Arm_Ellenorzes(void) {
  bool Arm_Feltetel =
    !Failsafe &&                //nincs failsafe
    !EStop_Allapot &&           //nincs estop
    Vevo_Ertekek[0] >= 1995 &&  // Roll: jobb végállás
    Vevo_Ertekek[1] <= 1005 &&  // Pitch: alsó végállás
    Vevo_Ertekek[2] <= 1005 &&  // Gáz: alsó végállás
    Vevo_Ertekek[3] >= 1995;    // Yaw: bal végállás

  if (!Arm_Feltetel) {
    // Ha megszakad a feltétel, újra kell kezdeni a tartást.
    Arm_Tartas_Folyamatban = false;
    return;
  }

  uint32_t Most = millis();

  if (!Arm_Tartas_Folyamatban) {
    Arm_Tartas_Kezdete = Most;
    Arm_Tartas_Folyamatban = true;
  }

  // 1 mp-ig tartani kell arm feltételeket
  if (Most - Arm_Tartas_Kezdete >= 1000) {
    Arm_engedelyezve = true;
    Arm_Tartas_Folyamatban = false;
    Ciklus_Ido_Max = 0;
  }
}

void MPU_regiszterek_beallitasa(void) {

  //MPU aktiválása (felébesztése)
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x6B);              //kérés küldése a regiszer indítására
  Wire.write(0x01);              //MPU felébesztésének beleírása HEX-be
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
  Wire.write(0x10);              //+-8 g beleírása HEX-be
  Wire.endTransmission();        //küldés
}

void MPU_olvasas(void) {

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

  // Gyorsulásvektor nagysága [g]
  float a_norm = sqrtf(Acc_X * Acc_X + Acc_Y * Acc_Y + Acc_Z * Acc_Z);

  if (a_norm > 0.0001f) {
    Angle_Roll = asinf(Acc_Y / a_norm) * (180.0f / PI);
    Angle_Pitch = asinf(-Acc_X / a_norm) * (180.0f / PI);
  }

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  //Angle_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  //Angle_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);
}

void Gyro_kalibracio(void) {

  //2500 mintavételes kalibráció Gyronak
  for (int i = 0; i < 2500; i++) {
    MPU_olvasas();
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

void PID_egyenlet(float Hiba, float P, float I, float D, float Elozo_Hiba, float Elozo_Iterm) {
  float Pterm = P * Hiba;
  float Iterm = Elozo_Iterm + (I * (Hiba + Elozo_Hiba)) * (Ts / 2);

  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;

  float Dterm = D * ((Hiba - Elozo_Hiba) / Ts);
  float PID_eredmeny = Pterm + Iterm + Dterm;

  if (PID_eredmeny > 400) PID_eredmeny = 400;
  else if (PID_eredmeny < -400) PID_eredmeny = -400;

  PID_kimenetek[0] = PID_eredmeny;
  PID_kimenetek[1] = Hiba;
  PID_kimenetek[2] = Iterm;
}

void Reset_PID(void) {
  //Rate reset
  Elozo_Hiba_Rate_Roll = 0;
  Elozo_Hiba_Rate_Pitch = 0;
  Elozo_Hiba_Rate_Yaw = 0;

  Elozo_Iterm_Rate_Roll = 0;
  Elozo_Iterm_Rate_Pitch = 0;
  Elozo_Iterm_Rate_Yaw = 0;

  //Angle reset
  Elozo_Hiba_Angle_Roll = 0;
  Elozo_Hiba_Angle_Pitch = 0;

  Elozo_Iterm_Angle_Roll = 0;
  Elozo_Iterm_Angle_Pitch = 0;

  //Velocity reset
  Elozo_Hiba_Fuggoleges_sebesseg = 0;
  Elozo_Iterm_Fuggoleges_sebesseg = 0;
}

void Reset_Angle_PID(void) {
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
  Q = B * ~B * 10.0f * 10.0f;
  R = { 30.0f * 30.0f };
  P = { 0, 0,
        0, 0 };
  xk = { 0,
         0 };

  Gyro_kalibracio();

  // Indulasi akkufeszultseg meghatarozasa 100 meres atlagabol (multisampling)
  float Feszultseg_Osszeg = 0.0f;

  for (int j = 0; j < 100; j++) {
    ADC_meres();
    Feszultseg_Osszeg += Feszultseg;
    delay(1);
  }

  Feszultseg = Feszultseg_Osszeg / 100.0f;

  // A kezdeti töltöttséget álló motorok mellett mért feszültségből becsüljük.
  if (Feszultseg >= 16.8f) {

    digitalWrite(Piros_LED, LOW);
    Aksi_kezdetben = Alap_aksi_kapacitas;  // A becslés felső korlátja: 2000 mAh.
    Aksi_OK = true;

  } else if (Feszultseg <= 14.0f) {

    // Alacsony indulási feszültség: az indulást tiltjuk.
    digitalWrite(Piros_LED, HIGH);
    Aksi_kezdetben = 0.0f;
    Aksi_OK = false;

  } else {

    // Közelítő lineáris becslés a 14–16,8 V tartományban.
    Aksi_kezdetben = ((Feszultseg - 14.0f) / (16.8f - 14.0f)) * Alap_aksi_kapacitas;
    Aksi_OK = true;

    // Alacsony indulási feszültség jelzése.
    if (Feszultseg <= 14.8f) {
      digitalWrite(Piros_LED, HIGH);
      Aksi_Alacsony = true;
    } else {
      digitalWrite(Piros_LED, LOW);
    }
  }

  digitalWrite(Zold_LED, HIGH);

  // kontrollálatlan motorindítás és alacsony aksi szintel történő repülés elkerülése
  while (!Arm_engedelyezve || !Aksi_OK) {

    IBUS_Olvasas();
    IBUS_Failsafe_Ellenorzes();

    Arm_Ellenorzes();

    delay(4);
  }

  Ciklus_Idozito = micros();
  Elozo_Ciklus_Kezdet = Ciklus_Idozito;
}

void loop() {

  //Ciklus idő mérés
  uint32_t Ciklus_Kezdet = micros();

  Ciklus_Ido = Ciklus_Kezdet - Elozo_Ciklus_Kezdet;
  Elozo_Ciklus_Kezdet = Ciklus_Kezdet;
  //

  //elmentjük mindig a legnagyobb ciklus időt 4000-4049 nem gond 250Hz-246,98Hz
  if (Ciklus_Ido > Ciklus_Ido_Max) {
    Ciklus_Ido_Max = Ciklus_Ido;
  }

  // A főciklusban kizárólag ez a blokk vezérli a piros LED-et.
  if (Ciklus_Ido_Max >= 4050) {
    // Az időzítési hibajelzés elsőbbséget kap.
    uint32_t Most = millis();

    if (Most - Piros_villogas >= 500) {
      Piros_villogas = Most;
      Piros_LED_Allapot = !Piros_LED_Allapot;
    }

    digitalWrite(Piros_LED, Piros_LED_Allapot ? HIGH : LOW);

  } else {
    // Időzítési hiba nélkül az akkumulátor állapotát jelezzük.
    digitalWrite(Piros_LED, Aksi_Alacsony ? HIGH : LOW);
  }

  //Gyro adat olvasó fgv meghívása
  MPU_olvasas();

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  //Ezt most nem használom
  /*Szurt_Rate_Roll = Szurt_Rate_Roll * 0.7 + Rate_Roll * 0.3;
  Szurt_Rate_Pitch = Szurt_Rate_Pitch * 0.7 + Rate_Pitch * 0.3;
  Szurt_Rate_Yaw = Szurt_Rate_Yaw * 0.7 + Rate_Yaw * 0.3;*/

  //Kalman Angle Roll számítás
  Kalman_Szog(Kalman_Angle_Roll, Kalman_Bizonytalansag_Angle_Roll, Rate_Roll, Angle_Roll);
  Kalman_Angle_Roll = Kalman_Szog_Kimenet[0];
  Kalman_Bizonytalansag_Angle_Roll = Kalman_Szog_Kimenet[1];

  //Kalman Angle Pitch számítás
  Kalman_Szog(Kalman_Angle_Pitch, Kalman_Bizonytalansag_Angle_Pitch, Rate_Pitch, Angle_Pitch);
  Kalman_Angle_Pitch = Kalman_Szog_Kimenet[0];
  Kalman_Bizonytalansag_Angle_Pitch = Kalman_Szog_Kimenet[1];

  // Dőlésszögek átváltása radiánba
  float Roll_rad = Kalman_Angle_Roll * (PI / 180);
  float Pitch_rad = Kalman_Angle_Pitch * (PI / 180);

  // Trigonometrikus értékek
  float Sin_Roll = sinf(Roll_rad);
  float Cos_Roll = cosf(Roll_rad);
  float Sin_Pitch = sinf(Pitch_rad);
  float Cos_Pitch = cosf(Pitch_rad);

  // Gyorsulás függőleges irányú komponense [g]
  float Acc_Z_inercialis = -Sin_Pitch * Acc_X + Cos_Pitch * Sin_Roll * Acc_Y + Cos_Pitch * Cos_Roll * Acc_Z;

  // Gravitáció kompenzálása, átváltás cm/s²-re
  Acc_Z_gyorsulas = (Acc_Z_inercialis - 1.0f) * 981.0f;  // * 9.81 * 100

  Baro_frissites = false;

  uint32_t Most_us = micros();

  if ((uint32_t)(Most_us - BMP_meres) >= 40000UL) {
    BMP_meres = Most_us;

    if (BMP_adat_olvasas()) {
      Baro_magassag -= Baro_magassag_kezdetben;
      Baro_frissites = true;
    }
  }

  kalman_2d();

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  // Biztonsági leállítás és LED állapota
  bool Leallitas = Failsafe || !Arm_engedelyezve || EStop_Allapot;

  EStop_LED = Leallitas;
  digitalWrite(LED_BUILTIN, EStop_LED);

  if (Leallitas) {
    Iterm_Aktiv = false;
    Arm_engedelyezve = false;

    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;

    ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));
    ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));
    ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));
    ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));

    Reset_PID();

    Arm_Ellenorzes();

    while (micros() - Ciklus_Idozito < 4000)
      ;
    Ciklus_Idozito = micros();
    return;
  }

  //RATE -> ANGLE módváltás érzékelése
  if (Angle_Mode_Aktiv && !Elozo_Angle_Mode_Aktiv) {
    Reset_Angle_PID();
  }
  // Aktuális állapot eltárolása következő ciklusra
  Elozo_Angle_Mode_Aktiv = Angle_Mode_Aktiv;


  //Felszállás elött Iterm gyűlődésének elkerülése
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

  //Velocity PID
  if (Velocity_Mode_Aktiv) {
    Kivant_Fuggoleges_sebesseg = 0.3 * (Vevo_Ertekek[2] - 1500);
    Hiba_Fuggoleges_sebesseg = Kivant_Fuggoleges_sebesseg - Kalman_Fuggoleges_sebesseg;

    PID_egyenlet(Hiba_Fuggoleges_sebesseg, P_Fuggoleges_sebesseg, I_Fuggoleges_sebesseg_Aktiv, D_Fuggoleges_sebesseg, Elozo_Hiba_Fuggoleges_sebesseg, Elozo_Iterm_Fuggoleges_sebesseg);
    Bemenet_Gaz = 1500 + PID_kimenetek[0];  //Mivel a nulla sebességhez, tehát a lebegéshez tartozó pont várhatóan a gázkar középállása körül lesz — 1500 µs-nál —, ezt az értéket add hozzá a PID kimenetéhez.
    Elozo_Hiba_Fuggoleges_sebesseg = PID_kimenetek[1];
    Elozo_Iterm_Fuggoleges_sebesseg = PID_kimenetek[2];
  } else {
    Bemenet_Gaz = Vevo_Ertekek[2];
  }

  //ANGLE PID
  if (Angle_Mode_Aktiv) {
    Kivant_Angle_Roll = 0.10 * RC_Deadband(Vevo_Ertekek[0]);
    Kivant_Angle_Pitch = 0.10 * RC_Deadband(Vevo_Ertekek[1]);
    //Számítsd ki a különbséget a kívánt és a tényleges roll és pitch szögek között.
    Hiba_Angle_Roll = Kivant_Angle_Roll - Kalman_Angle_Roll;
    Hiba_Angle_Pitch = Kivant_Angle_Pitch - Kalman_Angle_Pitch;

    //PID Angle Roll
    PID_egyenlet(Hiba_Angle_Roll, P_Angle_Roll, I_Angle_Roll_Aktiv, D_Angle_Roll, Elozo_Hiba_Angle_Roll, Elozo_Iterm_Angle_Roll);
    Kivant_Rate_Roll = PID_kimenetek[0];
    Kivant_Rate_Roll = constrain(Kivant_Rate_Roll, -220, 220);
    Elozo_Hiba_Angle_Roll = PID_kimenetek[1];
    Elozo_Iterm_Angle_Roll = PID_kimenetek[2];

    //PID Angle Pitch
    PID_egyenlet(Hiba_Angle_Pitch, P_Angle_Pitch, I_Angle_Pitch_Aktiv, D_Angle_Pitch, Elozo_Hiba_Angle_Pitch, Elozo_Iterm_Angle_Pitch);
    Kivant_Rate_Pitch = PID_kimenetek[0];
    Kivant_Rate_Pitch = constrain(Kivant_Rate_Pitch, -220, 220);
    Elozo_Hiba_Angle_Pitch = PID_kimenetek[1];
    Elozo_Iterm_Angle_Pitch = PID_kimenetek[2];
  } else {
    Kivant_Rate_Roll = 0.15 * RC_Deadband(Vevo_Ertekek[0]);
    Kivant_Rate_Pitch = 0.15 * RC_Deadband(Vevo_Ertekek[1]);
  }

  // YAW PARANCS
  Kivant_Rate_Yaw = 0.15 * RC_Deadband(Vevo_Ertekek[3]);

  //Hiba_Rate_Roll = Kivant_Rate_Roll - Szurt_Rate_Roll;
  //Hiba_Rate_Pitch = Kivant_Rate_Pitch - Szurt_Rate_Pitch;
  //Hiba_Rate_Yaw = Kivant_Rate_Yaw - Szurt_Rate_Yaw;

  Hiba_Rate_Roll = Kivant_Rate_Roll - Rate_Roll;
  Hiba_Rate_Pitch = Kivant_Rate_Pitch - Rate_Pitch;
  Hiba_Rate_Yaw = Kivant_Rate_Yaw - Rate_Yaw;

  // PID Rate Roll
  PID_egyenlet(Hiba_Rate_Roll, P_Rate_Roll, I_Rate_Roll_Aktiv, D_Rate_Roll, Elozo_Hiba_Rate_Roll, Elozo_Iterm_Rate_Roll);
  Bemenet_Roll = PID_kimenetek[0];
  Elozo_Hiba_Rate_Roll = PID_kimenetek[1];
  Elozo_Iterm_Rate_Roll = PID_kimenetek[2];

  //PID Rate Pitch
  PID_egyenlet(Hiba_Rate_Pitch, P_Rate_Pitch, I_Rate_Pitch_Aktiv, D_Rate_Pitch, Elozo_Hiba_Rate_Pitch, Elozo_Iterm_Rate_Pitch);
  Bemenet_Pitch = PID_kimenetek[0];
  Elozo_Hiba_Rate_Pitch = PID_kimenetek[1];
  Elozo_Iterm_Rate_Pitch = PID_kimenetek[2];

  //PID Rate Yaw
  PID_egyenlet(Hiba_Rate_Yaw, P_Rate_Yaw, I_Rate_Yaw_Aktiv, D_Rate_Yaw, Elozo_Hiba_Rate_Yaw, Elozo_Iterm_Rate_Yaw);
  Bemenet_Yaw = PID_kimenetek[0];
  Elozo_Hiba_Rate_Yaw = PID_kimenetek[1];
  Elozo_Iterm_Rate_Yaw = PID_kimenetek[2];

  //PID-nek hagyni korrekciós lehetőséget
  if (Bemenet_Gaz > 1800) Bemenet_Gaz = 1800;

  // Motorkeverés és motorvezérlő jelek korlátozása
  Motor_1_bemenet = constrain(Bemenet_Gaz - Bemenet_Roll + Bemenet_Pitch + Bemenet_Yaw, Motorok_alapjarat, Motorok_max);
  Motor_2_bemenet = constrain(Bemenet_Gaz - Bemenet_Roll - Bemenet_Pitch - Bemenet_Yaw, Motorok_alapjarat, Motorok_max);
  Motor_3_bemenet = constrain(Bemenet_Gaz + Bemenet_Roll + Bemenet_Pitch - Bemenet_Yaw, Motorok_alapjarat, Motorok_max);
  Motor_4_bemenet = constrain(Bemenet_Gaz + Bemenet_Roll - Bemenet_Pitch + Bemenet_Yaw, Motorok_alapjarat, Motorok_max);

  //ha leszállni szeretnék
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
    ADC_meres();

    Elfogyasztott_kapacitas = Aram * 1000 * 0.1 / 3600 + Elfogyasztott_kapacitas;           // mAh ban
    Aksi_maradek = (Aksi_kezdetben - Elfogyasztott_kapacitas) / Alap_aksi_kapacitas * 100;  // %-ban

    Aksi_maradek = constrain(Aksi_maradek, 0.0f, 100.0f);
    Aksi_Alacsony = (Aksi_maradek <= 30.0f);
  }

  //250 Hz, 4ms ciklus biztosítása
  while (micros() - Ciklus_Idozito < 4000)
    ;
  Ciklus_Idozito = micros();
}