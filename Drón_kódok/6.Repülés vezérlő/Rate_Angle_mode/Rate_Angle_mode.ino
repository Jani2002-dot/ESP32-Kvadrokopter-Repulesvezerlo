#pragma region 1.Valtozok

#pragma region 01_Konyvtarak
#include <Wire.h>  //I2C könyvtár
#pragma endregion

#pragma region 02_Pinek_es_PWM
#define RX_PIN 16
#define TX_PIN -1

#define SDA 21
#define SCL 22

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
bool Angle_Mode_Aktiv = false;
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

const int A = 1, H = 1, I = 1;
float B = Ts;
float Q = Ts * Ts * 4 * 4;
float R = 3 * 3;
#pragma endregion

#pragma region 06_PID_adatok
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
#pragma endregion

#pragma region 07_Motorvaltozok_es_Ciklus_valtozok
float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;
int Motorok_max = 2000;

// 250 Hz-es loop vizsgálatához
uint32_t Ciklus_Idozito;
uint32_t Ciklus_Ido = 0;
uint32_t Ciklus_Ido_Max = 0;
uint32_t Elozo_Ciklus_kezdet = 0;
uint32_t Piros_villogas = 0;
bool Piros_LED_Allapot = false;
#pragma endregion

#pragma region 08_Akkumulator_meres
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

void Kalman_Szog(float x, float P, float u, float z) {
  x = A * x + B * u;
  P = A * P * A + Q;
  float K = P * H / (H * P * H + R);
  x = x + K * (z - H * x);
  P = (I - K * H) * P;
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
      // Egy újabb 0x20 már a következő keret kezdete lehet.
      IBUS_Index = (b == 0x20) ? 1 : 0;
      continue;
    }

    IBUS_Buffer[IBUS_Index++] = b;

    // A részleges keret megmarad a következő meghívásig.
    if (IBUS_Index < 32) continue;
    IBUS_Index = 0;

    // Az első 30 bájtból számított ellenőrző összeg.
    uint16_t Checksum = 0xFFFF;

    for (uint8_t i = 0; i < 30; i++) {
      Checksum -= IBUS_Buffer[i];
    }

    // A keret végén az alsó helyiértékű bájt érkezik előbb.
    uint16_t Kapott_Checksum =
      (uint16_t)IBUS_Buffer[30] | ((uint16_t)IBUS_Buffer[31] << 8);

    if (Checksum != Kapott_Checksum) continue;

    // A hat használt csatorna ellenőrzése az átvétel előtt.
    uint16_t Temp[6];
    bool Ervenyes = true;

    for (uint8_t i = 0; i < 6; i++) {
      uint8_t also = 2 + 2 * i;

      Temp[i] = (uint16_t)IBUS_Buffer[also] | ((uint16_t)IBUS_Buffer[also + 1] << 8);

      if (Temp[i] < 900 || Temp[i] > 2100) {
        Ervenyes = false;
      }
    }

    if (!Ervenyes) continue;

    // Csak akkor frissítem a csatornákat, ha mind megfelelő.
    for (uint8_t i = 0; i < 6; i++) {
      Vevo_Ertekek[i] = Temp[i];
    }

    // Csak elfogadott keret frissíti a kapcsolat állapotát.
    Utolso_IBUS_Csomag_Ido = millis();
    IBUS_Kapcsolat_El = true;
    Failsafe = false;

    // SWA: motorleállítási kérés, hiszterézissel.
    if (Vevo_Ertekek[4] > 1600) {
      EStop_Allapot = true;
    } else if (Vevo_Ertekek[4] < 1400) {
      EStop_Allapot = false;
    }

    // SWD: RATE / ANGLE módválasztás, hiszterézissel.
    if (Vevo_Ertekek[5] > 1600) {
      Angle_Mode_Aktiv = true;
    } else if (Vevo_Ertekek[5] < 1400) {
      Angle_Mode_Aktiv = false;
    }
  }
}

void IBUS_Failsafe_Ellenorzes(void) {
  uint32_t most = millis();

  // Saját időkorlát: több mint 100 ms érvényes keret nélkül.
  if ((uint32_t)(most - Utolso_IBUS_Csomag_Ido) > 100U) {
    IBUS_Kapcsolat_El = false;
    Failsafe = true;
  }

  if (Failsafe) {
    Vevo_Ertekek[0] = 1500;  // Roll: középállás.
    Vevo_Ertekek[1] = 1500;  // Pitch: középállás.
    Vevo_Ertekek[2] = 1000;  // Gáz: minimum.
    Vevo_Ertekek[3] = 1500;  // Yaw: középállás.
    Vevo_Ertekek[4] = 1000;  // SWA alapérték.
    Vevo_Ertekek[5] = 1000;  // SWD: RATE mód.

    EStop_Allapot = false;
    Angle_Mode_Aktiv = false;

    // A motorok leállítását és az ARM törlését
    // a főciklus Failsafe feltételű ága végzi.
  }
}

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

  // MPU6050 felébresztése, X-tengelyű giroszkópra épülő PLL-órajel
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x6B);              //kérés küldése a regiszer indítására
  Wire.write(0x01);              //MPU felébesztésének, X-tengelyű giroszkópra épülő PLL-órajel beleírása HEX-be
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

  // Gyorsulásvektor nagysága [g]
  float a_norm = sqrtf(Acc_X * Acc_X + Acc_Y * Acc_Y + Acc_Z * Acc_Z);

  if (a_norm > 0.0001f) {
    Angle_Roll = asinf(Acc_Y / a_norm) * (180.0f / PI);
    Angle_Pitch = asinf(-Acc_X / a_norm) * (180.0f / PI);
  }

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
  float PID_Kimenet = Pterm + Iterm + Dterm;

  if (PID_Kimenet > 400) PID_Kimenet = 400;
  else if (PID_Kimenet < -400) PID_Kimenet = -400;

  PIDReturn[0] = PID_Kimenet;
  PIDReturn[1] = Hiba;
  PIDReturn[2] = Iterm;
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

int RC_Holtsav(int ertek) {
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

  Wire.begin(SDA, SCL);   //SDA, SCL
  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  delay(250);
  MPU_regiszterek_beallitasa();

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
  Elozo_Ciklus_kezdet = Ciklus_Idozito;
}

void loop() {

  //Ciklus idő mérés
  uint32_t Ciklus_kezdet = micros();

  Ciklus_Ido = Ciklus_kezdet - Elozo_Ciklus_kezdet;
  Elozo_Ciklus_kezdet = Ciklus_kezdet;
  //

  //elentjük mindig a legnagyobb ciklus időt 4000-4049 nem gond 250Hz-246,98Hz
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

  //MPU 6050 olvasó fgv meghívása
  MPU_olvasas();

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  /*//elsőrendű exponenciális IIR aluláteresztő szűrés RATE MODE PID-hez hiba számoláshoz
  Ez most nincs használtban
  Szurt_Rate_Roll = Szurt_Rate_Roll * 0.7 + Rate_Roll * 0.3;
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

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  // Biztonsági leállítás és LED állapota
  bool Leallitas = Failsafe || !Arm_engedelyezve || EStop_Allapot;

  EStop_LED = Leallitas;
  digitalWrite(LED_BUILTIN, EStop_LED);

  if (Leallitas) {
    Iterm_Aktiv = false;
    EStop_LED = true;
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

  EStop_LED = false;

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

  //ANGLE PID
  if (Angle_Mode_Aktiv) {
    Kivant_Angle_Roll = 0.10 * RC_Holtsav(Vevo_Ertekek[0]);
    Kivant_Angle_Pitch = 0.10 * RC_Holtsav(Vevo_Ertekek[1]);
    //Számítsd ki a különbséget a kívánt és a tényleges roll és pitch szögek között.
    Hiba_Angle_Roll = Kivant_Angle_Roll - Kalman_Angle_Roll;
    Hiba_Angle_Pitch = Kivant_Angle_Pitch - Kalman_Angle_Pitch;

    //PID Angle Roll
    PID_egyenlet(Hiba_Angle_Roll, P_Angle_Roll, I_Angle_Roll_Aktiv, D_Angle_Roll, Elozo_Hiba_Angle_Roll, Elozo_Iterm_Angle_Roll);
    Kivant_Rate_Roll = PIDReturn[0];
    Kivant_Rate_Roll = constrain(Kivant_Rate_Roll, -220, 220);
    Elozo_Hiba_Angle_Roll = PIDReturn[1];
    Elozo_Iterm_Angle_Roll = PIDReturn[2];

    //PID Angle Pitch
    PID_egyenlet(Hiba_Angle_Pitch, P_Angle_Pitch, I_Angle_Pitch_Aktiv, D_Angle_Pitch, Elozo_Hiba_Angle_Pitch, Elozo_Iterm_Angle_Pitch);
    Kivant_Rate_Pitch = PIDReturn[0];
    Kivant_Rate_Pitch = constrain(Kivant_Rate_Pitch, -220, 220);
    Elozo_Hiba_Angle_Pitch = PIDReturn[1];
    Elozo_Iterm_Angle_Pitch = PIDReturn[2];
  } else {
    // RATE MÓD
    Kivant_Rate_Roll = 0.15 * RC_Holtsav(Vevo_Ertekek[0]);
    Kivant_Rate_Pitch = 0.15 * RC_Holtsav(Vevo_Ertekek[1]);
  }

  Bemenet_Gaz = Vevo_Ertekek[2];                         // SIMA GÁZ PARANCS
  Kivant_Rate_Yaw = 0.15 * RC_Holtsav(Vevo_Ertekek[3]);  // YAW PARANCS

  //Hiba_Rate_Roll = Kivant_Rate_Roll - Szurt_Rate_Roll;
  //Hiba_Rate_Pitch = Kivant_Rate_Pitch - Szurt_Rate_Pitch;
  //Hiba_Rate_Yaw = Kivant_Rate_Yaw - Szurt_Rate_Yaw;

  Hiba_Rate_Roll = Kivant_Rate_Roll - Rate_Roll;
  Hiba_Rate_Pitch = Kivant_Rate_Pitch - Rate_Pitch;
  Hiba_Rate_Yaw = Kivant_Rate_Yaw - Rate_Yaw;

  // PID Rate Roll
  PID_egyenlet(Hiba_Rate_Roll, P_Rate_Roll, I_Rate_Roll_Aktiv, D_Rate_Roll, Elozo_Hiba_Rate_Roll, Elozo_Iterm_Rate_Roll);
  Bemenet_Roll = PIDReturn[0];
  Elozo_Hiba_Rate_Roll = PIDReturn[1];
  Elozo_Iterm_Rate_Roll = PIDReturn[2];

  //PID Rate Pitch
  PID_egyenlet(Hiba_Rate_Pitch, P_Rate_Pitch, I_Rate_Pitch_Aktiv, D_Rate_Pitch, Elozo_Hiba_Rate_Pitch, Elozo_Iterm_Rate_Pitch);
  Bemenet_Pitch = PIDReturn[0];
  Elozo_Hiba_Rate_Pitch = PIDReturn[1];
  Elozo_Iterm_Rate_Pitch = PIDReturn[2];

  //PID Rate Yaw
  PID_egyenlet(Hiba_Rate_Yaw, P_Rate_Yaw, I_Rate_Yaw_Aktiv, D_Rate_Yaw, Elozo_Hiba_Rate_Yaw, Elozo_Iterm_Rate_Yaw);
  Bemenet_Yaw = PIDReturn[0];
  Elozo_Hiba_Rate_Yaw = PIDReturn[1];
  Elozo_Iterm_Rate_Yaw = PIDReturn[2];

  //PID-nek hagyni korrekciós lehetőséget
  if (Bemenet_Gaz > 1800) Bemenet_Gaz = 1800;

  // Motorkeverés és motorvezérlő jelek korlátozása
  Motor_1_bemenet = constrain(Bemenet_Gaz - Bemenet_Roll + Bemenet_Pitch + Bemenet_Yaw, Motorok_alapjarat, Motorok_max);
  Motor_2_bemenet = constrain(Bemenet_Gaz - Bemenet_Roll - Bemenet_Pitch - Bemenet_Yaw, Motorok_alapjarat, Motorok_max);
  Motor_3_bemenet = constrain(Bemenet_Gaz + Bemenet_Roll + Bemenet_Pitch - Bemenet_Yaw, Motorok_alapjarat, Motorok_max);
  Motor_4_bemenet = constrain(Bemenet_Gaz + Bemenet_Roll - Bemenet_Pitch + Bemenet_Yaw, Motorok_alapjarat, Motorok_max);

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

    Elfogyasztott_kapacitas = Aram * 1000 * 0.1 / 3600 + Elfogyasztott_kapacitas;           // mAh ban
    Aksi_maradek = (Aksi_kezdetben - Elfogyasztott_kapacitas) / Alap_aksi_kapacitas * 100;  // %-ban

    Aksi_maradek = constrain(Aksi_maradek, 0.0f, 100.0f);
    Aksi_Alacsony = (Aksi_maradek <= 30.0f);
  }

  while (micros() - Ciklus_Idozito < 4000)
    ;
  Ciklus_Idozito = micros();
}