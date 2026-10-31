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
byte Biztonsagos_inditas = 0;
uint32_t Arm_Tartas_Kezdete = 0;
bool Arm_Tartas_Folyamatban = false;
#pragma endregion

#pragma region 04_Rate_adatok
//számított szögsebességeknek változó
float Rate_Roll, Rate_Pitch, Rate_Yaw;
float Szurt_Roll_Rate, Szurt_Pitch_Rate, Szurt_Yaw_Rate;

//2500 mintavételt ezekhez adjuk hozzá
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

float Kalman_Angle_Roll = 0;                     //kezdeti szögbecslést 0°-ra állítottam, mivel az indítás közel vízszintes helyzetben történik
float Kalman_Bizonytalansag_Angle_Roll = 2 * 2;  //bizonytalanságát 2° kezdeti szórással, azaz 4°^2 hibavarianciával vettem figyelembe

float Kalman_Angle_Pitch = 0;                     //kezdeti szögbecslést 0°-ra állítottam, mivel az indítás közel vízszintes helyzetben történik
float Kalman_Bizonytalansag_Angle_Pitch = 2 * 2;  //bizonytalanságát 2° kezdeti szórással, azaz 4°^2 hibavarianciával vettem figyelembe

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
float PIDReturn[] = { 0, 0, 0 };
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

#pragma region 08_Motorvaltozok_es_Loop_valtozok
float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

// 250 Hz-es loop vizsgálatához
uint32_t Loop_Timer;
uint32_t Ciklus_Ido = 0;
uint32_t Ciklus_Ido_Max = 0;
uint32_t Elozo_Loop_Kezdet = 0;
uint32_t RED_BLINK = 0;
bool Red_LED_State = false;
float Ts = 0.004;  //250 Hz-es loop miatt 4ms
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

void kalman_2d(void) {

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
}

bool BMP_adat_olvasas(void) {

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
  signed long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  //nyers, kompennzálatlan és kalibrálatlan hőmérséklet
  signed long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

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
    if (BMP_adat_olvasas()) {
      Baro_magassag_kezdetben += Baro_magassag;
      Ervenyes_mintak++;
    }

    delay(40);
  }

  Baro_magassag_kezdetben /= Ervenyes_mintak;
}

void kalman_1d(float Kalman_allapot, float Kalman_Bizonytalansag, float Kalman_bemenet, float Kalman_meres) {
  Kalman_allapot = 1 * Kalman_allapot + Ts * Kalman_bemenet;
  Kalman_Bizonytalansag = 1 * Kalman_Bizonytalansag * 1 + Ts * Ts * 4 * 4;
  float Kalman_erosites = Kalman_Bizonytalansag * 1 / (1 * Kalman_Bizonytalansag * 1 + 3 * 3);
  Kalman_allapot = Kalman_allapot + Kalman_erosites * (Kalman_meres - 1 * Kalman_allapot);
  Kalman_Bizonytalansag = (1 - Kalman_erosites * 1) * Kalman_Bizonytalansag;
  Kalman_1D_kimenet[0] = Kalman_allapot;         //Korrigált szög PID-nek és következő iteráccióba
  Kalman_1D_kimenet[1] = Kalman_Bizonytalansag;  //Korrigált hibakovariancia következő iterációba
}

void ADC_meres(void) {
  Feszultseg = (float)analogRead(feszultseg_meres_PIN) / 206.818182 + 1.2;  //4095/(3.3*6)=206.8182
  Aram = (float)analogRead(aram_meres_PIN) / 37.227273;                     // ADC / ((4095/3.3V)*0.03 V/A)=37.227273 ---> ADC*0,02686203
}

void IBUS_Olvasas(void) {  // Paraméter és visszatérési érték nélküli függvény az iBUS-adatok feldolgozásához.

  while (Serial2.available()) {  // Addig ismétel, amíg van kiolvasható bájt a Serial2 vételi pufferében.
    uint8_t b = Serial2.read();  // Kiolvassa a következő bájtot, és eltárolja egy 8 bites, előjel nélküli változóban.

    if (IBUS_Index == 0) {  // Ha még nincs meg a keret első bájtja, a keretkezdetet keresi.
      if (b != 0x20) {      // Ellenőrzi, hogy a bájt eltér-e az elvárt 0x20 értéktől, amely 32-t jelent.
        continue;           // Ezt a bájtot eldobja, és a while következő körében a következő rendelkezésre álló bájttal folytatja.
      }                     // Az első bájt értékének ellenőrzése véget ér.
    }                       // A keret első bájtjához tartozó feltételes rész vége.

    if (IBUS_Index == 1) {  // Ha az első bájt már megvan, a második bájtot ellenőrzi.
      if (b != 0x40) {      // A második bájtnak 0x40 értékűnek kell lennie.
        IBUS_Index = 0;     // Hibás második bájtnál elölről kezdi a keret keresését.
        continue;           // A jelenlegi bájtot eldobja, és a következő rendelkezésre álló bájttal folytatja.
      }                     // A második bájt értékének ellenőrzése véget ér.
    }                       // A keret második bájtjához tartozó feltételes rész vége.

    IBUS_Buffer[IBUS_Index++] = b;  // Eltárolja a bájtot az aktuális indexen, majd eggyel növeli az indexet.

    if (IBUS_Index >= 32) {  // Ha összegyűlt a teljes, 32 bájtos keret, megkezdi annak ellenőrzését.
      IBUS_Index = 0;        // A következő keret fogadásához visszaállítja a puffer indexét; a puffer tartalmát nem törli.

      uint16_t Checksum = 0xFFFF;  // Az ellenőrző összeg kezdőértéke 65535, 16 bites előjel nélküli változóban.

      for (uint8_t i = 0; i < 30; i++) {  // Végigmegy a keret első 30 bájtján, a 0–29. indexeken.
        Checksum -= IBUS_Buffer[i];       // Az aktuális bájt értékét kivonja az ellenőrző összegből.
      }                                   // A két záró ellenőrző bájt kimarad a számításból.

      uint16_t Kapott_Checksum = IBUS_Buffer[30] | (IBUS_Buffer[31] << 8);
      // A 30. indexű bájt az alsó, a 31. indexű a felső helyiértékű bájt.
      // A << 8 a felső bájtot nyolc bittel balra tolja, a | bitenkénti VAGY pedig összekapcsolja a két bájtot.

      if (Checksum == Kapott_Checksum) {  // Csak egyező számított és kapott ellenőrző összegnél dolgozik tovább.
        uint16_t Temp[7];                 // Ideiglenes tömb hét darab 16 bites csatornaérték tárolására.

        Temp[0] = IBUS_Buffer[2] | (IBUS_Buffer[3] << 8);    // CH1: a roll-parancs összeállítása az alsó és felső bájtból.
        Temp[1] = IBUS_Buffer[4] | (IBUS_Buffer[5] << 8);    // CH2: a pitch-parancs összeállítása.
        Temp[2] = IBUS_Buffer[6] | (IBUS_Buffer[7] << 8);    // CH3: a gázparancs összeállítása.
        Temp[3] = IBUS_Buffer[8] | (IBUS_Buffer[9] << 8);    // CH4: a yaw-parancs összeállítása.
        Temp[4] = IBUS_Buffer[10] | (IBUS_Buffer[11] << 8);  // CH5: az SWA leállítókapcsoló csatornaértékének összeállítása.
        Temp[5] = IBUS_Buffer[12] | (IBUS_Buffer[13] << 8);  // CH6: az SWD RATE/ANGLE módválasztó csatornaértékének összeállítása.
        Temp[6] = IBUS_Buffer[14] | (IBUS_Buffer[15] << 8);  // CH7: az SWB csatornaértékének összeállítása; a hozzá tartozó módváltás lent ki van kapcsolva.

        bool Ervenyes = true;  // Kiindulásként megfelelőnek tekinti a csatornaértékeket; bármelyik hibás érték ezt felülírja.

        if (Temp[0] < 900 || Temp[0] > 2100) Ervenyes = false;  // Érvénytelen, ha a CH1 értéke 900 alatti VAGY 2100 feletti.
        if (Temp[1] < 900 || Temp[1] > 2100) Ervenyes = false;  // Ugyanezt a tartományellenőrzést elvégzi a CH2 értékén.
        if (Temp[2] < 900 || Temp[2] > 2100) Ervenyes = false;  // Ellenőrzi a CH3, azaz a gázcsatorna értékét.
        if (Temp[3] < 900 || Temp[3] > 2100) Ervenyes = false;  // Ellenőrzi a CH4, azaz a yaw-csatorna értékét.
        if (Temp[4] < 900 || Temp[4] > 2100) Ervenyes = false;  // Ellenőrzi a CH5, azaz az SWA csatorna értékét.
        if (Temp[5] < 900 || Temp[5] > 2100) Ervenyes = false;  // Ellenőrzi a CH6, azaz az SWD csatorna értékét.
        if (Temp[6] < 900 || Temp[6] > 2100) Ervenyes = false;  // Ellenőrzi a CH7 értékét is, annak ellenére, hogy a módváltása inaktív.

        if (Ervenyes) {               // Csak akkor veszi át az adatokat, ha mind a hét érték a megengedett tartományon belül van.
          Vevo_Ertekek[0] = Temp[0];  // Frissíti a program által használt roll-parancsot.
          Vevo_Ertekek[1] = Temp[1];  // Frissíti a pitch-parancsot.
          Vevo_Ertekek[2] = Temp[2];  // Frissíti a gázparancsot.
          Vevo_Ertekek[3] = Temp[3];  // Frissíti a yaw-parancsot.
          Vevo_Ertekek[4] = Temp[4];  // Frissíti az SWA leállítókapcsoló csatornaértékét.
          Vevo_Ertekek[5] = Temp[5];  // Frissíti az SWD módválasztó csatornaértékét.
          Vevo_Ertekek[6] = Temp[6];  // Frissíti az SWB csatornaértékét; ettől még nem kapcsol be a velocity mód.

          Utolso_IBUS_Csomag_Ido = millis();  // Eltárolja az érvényes csomag feldolgozásának időpontját milliszekundumban.

          IBUS_Kapcsolat_El = true;  // Jelzi, hogy érkezett elfogadott iBUS-csomag.
          Failsafe = false;          // Törli a failsafe jelzőt; az ARM-engedélyt ez önmagában nem állítja vissza.

          if (Vevo_Ertekek[4] > 1600) {         // Ha az SWA csatorna értéke meghaladja az 1600-at:
            EStop_Allapot = true;               // Aktiválja a leállítási kérést; a motorokat a főciklus leállítási ága kezeli.
          } else if (Vevo_Ertekek[4] < 1400) {  // Egyébként, ha az SWA csatorna értéke 1400 alatti:
            EStop_Allapot = false;              // Megszünteti a kapcsolóból származó leállítási kérést.
          }                                     // 1400 és 1600 között, a határokat is beleértve, az EStop_Allapot nem változik.

          if (Vevo_Ertekek[5] > 1600) {         // Ha az SWD csatorna értéke meghaladja az 1600-at:
            Angle_Mode_Aktiv = true;            // Kiválasztja az ANGLE üzemmódot.
          } else if (Vevo_Ertekek[5] < 1400) {  // Egyébként, ha az SWD csatorna értéke 1400 alatti:
            Angle_Mode_Aktiv = false;           // Kikapcsolja az ANGLE módot, így a főciklus RATE módot használ.
          }                                     // 1400 és 1600 között az előző üzemmód marad érvényben.

          // Az alábbi blokk megjegyzésben van, ezért egyetlen sora sem hajtódik végre.

          if (Vevo_Ertekek[6] > 1600) {         // Aktív kódként az SWB 1600 feletti értékét vizsgálná.
            Velocity_Mode_Aktiv = true;         // Bekapcsolná a függőleges sebesség szabályozásának üzemmódját.
          } else if (Vevo_Ertekek[6] < 1400) {  // Az SWB 1400 alatti értékét vizsgálná.
            Velocity_Mode_Aktiv = false;        // Kikapcsolná a velocity módot, és visszaállna a közvetlen gázparancsra.
          }                                     // A köztes tartományban megtartaná az előző állapotot.


        }  // Az érvényes csatornaértékek átvételének és a kapcsolók kiértékelésének vége.
      }    // Az egyező ellenőrző összeg esetén végrehajtott rész vége.
    }      // A teljes, 32 bájtos keret feldolgozásának vége.
  }        // A vételi puffer feldolgozásának vége; üres puffernél kilép a while ciklusból.
}  // A függvény visszatér oda, ahonnan meghívták.

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
    !Failsafe && !EStop_Allapot && Vevo_Ertekek[2] <= 1005 &&  // Gáz lent
    Vevo_Ertekek[0] >= 1995 &&                                 // Roll végállás
    Vevo_Ertekek[3] >= 1995;                                   // Yaw végállás

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
    Biztonsagos_inditas = 1;
    Arm_Tartas_Folyamatban = false;
  }
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

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Angle_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  Angle_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);
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

  // kontrollálatlan motorindítás elkerülése Arm logika
  while (Biztonsagos_inditas == 0) {

    IBUS_Olvasas();
    IBUS_Failsafe_Ellenorzes();

    Arm_Ellenorzes();

    delay(4);
  }

  Loop_Timer = micros();
  Elozo_Loop_Kezdet = Loop_Timer;
}

void loop() {

  //Ciklus idő mérés
  uint32_t Loop_Kezdet = micros();

  Ciklus_Ido = Loop_Kezdet - Elozo_Loop_Kezdet;
  Elozo_Loop_Kezdet = Loop_Kezdet;
  //

  //elentjük mindig a legnagyobb ciklus időt 4000-4049 nem gond 250Hz-246,98Hz
  if (Ciklus_Ido > Ciklus_Ido_Max) {
    Ciklus_Ido_Max = Ciklus_Ido;
  }

  // Egyetlen legalább 4050 us-os ciklus után is villog 1Hzen resetig.
  if (Ciklus_Ido_Max >= 4050) {
    if (millis() - RED_BLINK >= 500) {
      RED_BLINK = millis();
      Red_LED_State = !Red_LED_State;
    }
  } else {
    Red_LED_State = false;
  }

  digitalWrite(Piros_LED, Red_LED_State);

  digitalWrite(LED_BUILTIN, EStop_LED);

  //Gyro adat olvasó fgv meghívása
  MPU_olvasas();

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

  if (BMP_adat_olvasas()) {
    Baro_magassag -= Baro_magassag_kezdetben;
  }

  kalman_2d();

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  if (Failsafe || Biztonsagos_inditas == 0) {
    Iterm_Aktiv = false;
    EStop_LED = true;
    Biztonsagos_inditas = 0;

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

    while (micros() - Loop_Timer < 4000)
      ;
    Loop_Timer = micros();
    return;
  }

  if (EStop_Allapot) {
    EStop_LED = true;
    Iterm_Aktiv = false;
    Biztonsagos_inditas = 0;

    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;

    ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));
    ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));
    ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));
    ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));

    Reset_PID();

    while (micros() - Loop_Timer < 4000)
      ;

    Loop_Timer = micros();
    return;
  }

  EStop_LED = false;

  //RATE -> ANGLE módváltás érzékelése
  if (Angle_Mode_Aktiv && !Elozo_Angle_Mode_Aktiv) {
    Reset_Angle_PID();
  }
  // Aktuális állapot eltárolása következő ciklusra
  Elozo_Angle_Mode_Aktiv = Angle_Mode_Aktiv;

  if (Angle_Mode_Aktiv) {
    // ANGLE MÓD
    Kivant_Angle_Roll = 0.10 * RC_Deadband(Vevo_Ertekek[0]);
    Kivant_Angle_Pitch = 0.10 * RC_Deadband(Vevo_Ertekek[1]);
  } else {
    // RATE MÓD
    Kivant_Roll_Rate = 0.15 * RC_Deadband(Vevo_Ertekek[0]);
    Kivant_Pitch_Rate = 0.15 * RC_Deadband(Vevo_Ertekek[1]);
  }

  // VELOCITY MÓD
  if (Velocity_Mode_Aktiv) {
    Kivant_Fuggoleges_sebesseg = 0.3 * (Vevo_Ertekek[2] - 1500);
  } else {
    // SIMA GÁZ PARANCS
    Bemenet_Gaz = Vevo_Ertekek[2];
  }
  // YAW PARANCS
  Kivant_Yaw_Rate = 0.15 * RC_Deadband(Vevo_Ertekek[3]);

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
    Hiba_Fuggoleges_sebesseg = Kivant_Fuggoleges_sebesseg - Kalman_Fuggoleges_sebesseg;

    PID_egyenlet(Hiba_Fuggoleges_sebesseg, P_Fuggoleges_sebesseg, I_Fuggoleges_sebesseg_Aktiv, D_Fuggoleges_sebesseg, Elozo_Hiba_Fuggoleges_sebesseg, Elozo_Iterm_Fuggoleges_sebesseg);
    Bemenet_Gaz = 1500 + PIDReturn[0];  //Mivel a nulla sebességhez, tehát a lebegéshez tartozó pont várhatóan a gázkar középállása körül lesz — 1500 µs-nál —, ezt az értéket add hozzá a PID kimenetéhez.
    Elozo_Hiba_Fuggoleges_sebesseg = PIDReturn[1];
    Elozo_Iterm_Fuggoleges_sebesseg = PIDReturn[2];
  }

  //ANGLE PID
  if (Angle_Mode_Aktiv) {
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
    ADC_meres();

    Elfogyasztott_kapacitas = Aram * 1000 * 0.1 / 3600 + Elfogyasztott_kapacitas;
    Aksi_maradek = (Aksi_kezdetben - Elfogyasztott_kapacitas) / Alap_aksi_kapacitas * 100;

    if (Aksi_maradek <= 30) digitalWrite(Piros_LED, HIGH);
    else digitalWrite(Piros_LED, LOW);
  }

  while (micros() - Loop_Timer < 4000)
    ;
  Loop_Timer = micros();
}