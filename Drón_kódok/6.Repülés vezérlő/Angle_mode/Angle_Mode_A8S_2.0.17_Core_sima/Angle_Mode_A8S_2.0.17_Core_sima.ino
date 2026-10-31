#include <Wire.h>  //I2C könyvtár

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

//számított szögsebességeknek változó
float Rate_Roll, Rate_Pitch, Rate_Yaw;
float Szurt_Roll_Rate, Szurt_Pitch_Rate, Szurt_Yaw_Rate;

//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Offset = 0;
float Rate_Pitch_Offset = 0;
float Rate_Yaw_Offset = 0;

uint8_t IBUS_Buffer[32];
uint8_t IBUS_Index = 0;

uint32_t Utolso_IBUS_Csomag_Ido = 0;

bool IBUS_Kapcsolat_El = false;
bool Failsafe = true;

uint16_t Vevo_Ertekek[] = { 1500, 1500, 1000, 1500, 1000, 1000 };
bool SWA_Allapot = false;
bool Rezges_Aktiv = false;
bool Tune_LED = false;
bool Iterm_Aktiv = false;

float Feszultseg;
float Aram;
uint32_t Aksi_meres;
float Aksi_maradek;                 // %-ban
float Aksi_kezdetben;               // mAh-ban
float Elfogyasztott_kapacitas = 0;  // mAh-ban
float Alap_aksi_kapacitas = 2000;   // 2000mAh

// 250 Hz-es loop vizsgálatához
uint32_t Loop_Timer;

float Kivant_Roll_Rate, Kivant_Pitch_Rate, Kivant_Yaw_Rate;
float Hiba_Roll_Rate, Hiba_Pitch_Rate, Hiba_Yaw_Rate;
float Elozo_Hiba_Roll_Rate, Elozo_Hiba_Pitch_Rate, Elozo_Hiba_Yaw_Rate;
float Elozo_Iterm_Roll, Elozo_Iterm_Pitch, Elozo_Iterm_Yaw;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
float PIDReturn[] = { 0, 0, 0 };

float P_Rate_Roll = 0.485, P_Rate_Pitch = P_Rate_Roll, P_Rate_Yaw = 0.55;
float I_Rate_Roll = 1.435, I_Rate_Pitch = I_Rate_Roll, I_Rate_Yaw = 1.4;
float D_Rate_Roll = 0.01075, D_Rate_Pitch = D_Rate_Roll, D_Rate_Yaw = 0;

//számított szög változó
float Angle_Roll, Angle_Pitch;

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

float P_Angle_Roll = 0, P_Angle_Pitch = P_Angle_Roll;
float I_Angle_Roll = 0, I_Angle_Pitch = I_Angle_Roll;
float D_Angle_Roll = 0, D_Angle_Pitch = D_Angle_Roll;

float Ts = 0.004;  //250 Hz-es loop miatt

int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;

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
        Temp[4] = IBUS_Buffer[10] | (IBUS_Buffer[11] << 8);  // CH5 SWA
        Temp[5] = IBUS_Buffer[12] | (IBUS_Buffer[13] << 8);  // CH6 SWD

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

          SWA_Allapot = Vevo_Ertekek[4] > 1500;
          Rezges_Aktiv = Vevo_Ertekek[5] > 1500;
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
    SWA_Allapot = false;
    Rezges_Aktiv = false;
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
  float Acc_X = (float)Acc_XLSB / 4096 - 0.0454;
  float Acc_Y = (float)Acc_YLSB / 4096 + 0.0279;
  float Acc_Z = (float)Acc_ZLSB / 4096 - 0.10793;

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
  
  if (PID_Kimenet  > 400) PID_Kimenet = 400;
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

  Elozo_Iterm_Roll = 0;
  Elozo_Iterm_Pitch = 0;
  Elozo_Iterm_Yaw = 0;

  //Angle reset
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
  while (Vevo_Ertekek[2] < 1020 || Vevo_Ertekek[2] > 1050 || Failsafe) {  //Vevo_Ertekek[2] ---> Gáz kar értéke  kontrollálatlan motorindítás elkerülése
    IBUS_Olvasas();
    IBUS_Failsafe_Ellenorzes();
    delay(4);
  }

  Loop_Timer = micros();
}

void loop() {

  digitalWrite(LED_BUILTIN, Tune_LED);

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

    while (micros() - Loop_Timer < 4000)
      ;
    Loop_Timer = micros();
    return;
  }

  if (SWA_Allapot == 1) {
    Tune_LED = true;
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

    while (micros() - Loop_Timer < 4000);

    Loop_Timer = micros();
    return;
  }

  Tune_LED = false;

  //Számítsd ki a kívánt szögeket gázt és Yaw-t a vevő (receiver) értékeiből.
  Kivant_Angle_Roll = 0.10 * RC_Deadband(Vevo_Ertekek[0]);
  Kivant_Angle_Pitch = 0.10 * RC_Deadband(Vevo_Ertekek[1]);
  Bemenet_Gaz = Vevo_Ertekek[2];
  Kivant_Yaw_Rate = 0.15 * RC_Deadband(Vevo_Ertekek[3]);

  //Számítsd ki a különbséget a kívánt és a tényleges roll és pitch szögek között.
  Hiba_Angle_Roll = Kivant_Angle_Roll - Kalman_Angle_Roll;
  Hiba_Angle_Pitch = Kivant_Angle_Pitch - Kalman_Angle_Pitch;

  if (Vevo_Ertekek[2] > 1130) Iterm_Aktiv = true;
  //ternary operator: feltétel ? érték ha igaz : érték ha hamis

  //Angle
  float I_Angle_Roll_Aktiv = Iterm_Aktiv ? I_Angle_Roll : 0.0;
  float I_Angle_Pitch_Aktiv = Iterm_Aktiv ? I_Angle_Pitch : 0.0;
  //Rate
  float I_Rate_Roll_Aktiv = Iterm_Aktiv ? I_Rate_Roll : 0.0;
  float I_Rate_Pitch_Aktiv = Iterm_Aktiv ? I_Rate_Pitch : 0.0;
  float I_Rate_Yaw_Aktiv = Iterm_Aktiv ? I_Rate_Yaw : 0.0;

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

  Hiba_Roll_Rate = Kivant_Roll_Rate - Szurt_Roll_Rate;
  Hiba_Pitch_Rate = Kivant_Pitch_Rate - Szurt_Pitch_Rate;
  Hiba_Yaw_Rate = Kivant_Yaw_Rate - Szurt_Yaw_Rate;

  // PID Rate Roll
  PID_egyenlet(Hiba_Roll_Rate, P_Rate_Roll, I_Rate_Roll_Aktiv, D_Rate_Roll, Elozo_Hiba_Roll_Rate, Elozo_Iterm_Roll);
  Bemenet_Roll = PIDReturn[0];
  Elozo_Hiba_Roll_Rate = PIDReturn[1];
  Elozo_Iterm_Roll = PIDReturn[2];

  //PID Rate Pitch
  PID_egyenlet(Hiba_Pitch_Rate, P_Rate_Pitch, I_Rate_Pitch_Aktiv, D_Rate_Pitch, Elozo_Hiba_Pitch_Rate, Elozo_Iterm_Pitch);
  Bemenet_Pitch = PIDReturn[0];
  Elozo_Hiba_Pitch_Rate = PIDReturn[1];
  Elozo_Iterm_Pitch = PIDReturn[2];

  //PID Rate Yaw
  PID_egyenlet(Hiba_Yaw_Rate, P_Rate_Yaw, I_Rate_Yaw_Aktiv, D_Rate_Yaw, Elozo_Hiba_Yaw_Rate, Elozo_Iterm_Yaw);
  Bemenet_Yaw = PIDReturn[0];
  Elozo_Hiba_Yaw_Rate = PIDReturn[1];
  Elozo_Iterm_Yaw = PIDReturn[2];

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
  }

  while (micros() - Loop_Timer < 4000)
    ;
  Loop_Timer = micros();
}