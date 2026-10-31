#include <Wire.h>  //I2C könyvtár
#include "rom/rtc.h"

//ibus.readChannel(0-9) 10 csatorna
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
float Rate_Roll_Szurt, Rate_Pitch_Szurt, Rate_Yaw_Szurt;

//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Offset = 0;
float Rate_Pitch_Offset = 0;
float Rate_Yaw_Offset = 0;

uint8_t IBUS_Buffer[32];
uint8_t IBUS_Index = 0;

uint32_t Utolso_IBUS_Csomag_Ido = 0;

bool IBUS_Kapcsolat_El = false;
bool Failsafe = true;

uint16_t Vevo_Ertekek[] = { 1500, 1500, 1000, 1500, 1000 };
bool SWA_Allapot = false;
bool MPU_Hiba;

float Feszultseg;
float Aram;
uint32_t Aksi_meres;
float Aksi_maradek;                 // %-ban
float Aksi_kezdetben;               // mAh-ban
float Elfogyasztott_kapacitas = 0;  // mAh-ban
float Alap_aksi_kapacitas = 2000;   // 2000mAh

// 250 Hz-es loop vizsgálatához
uint32_t Loop_Start_Us = 0;
uint32_t Elozo_Loop_Start_Us = 0;

uint32_t Loop_Munka_Us = 0;
uint32_t Loop_Teljes_Us = 0;

uint32_t Min_Teljes_Us = 999999;
uint32_t Max_Teljes_Us = 0;
uint32_t Sum_Teljes_Us = 0;
uint32_t Minta_Szam = 0;

uint32_t Min_Munka_Us = 999999;
uint32_t Max_Munka_Us = 0;

uint32_t Tulcsordulas_Szam = 0;
uint32_t Debug_Ido = 0;

float Kivant_Roll_ertek, Kivant_Pitch_ertek, Kivant_Yaw_ertek;
float Hiba_Roll_ertek, Hiba_Pitch_ertek, Hiba_Yaw_ertek;
float Elozo_Hiba_Roll_ertek, Elozo_Hiba_Pitch_ertek, Elozo_Hiba_Yaw_ertek;
float Elozo_Iterm_Roll, Elozo_Iterm_Pitch, Elozo_Iterm_Yaw;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
float PIDReturn[] = { 0, 0, 0 };

float P_ertek_Roll = 0, P_ertek_Pitch = P_ertek_Roll, P_ertek_Yaw = 0;
float I_ertek_Roll = 0, I_ertek_Pitch = I_ertek_Roll, I_ertek_Yaw = 0;
float D_ertek_Roll = 0, D_ertek_Pitch = D_ertek_Roll, D_ertek_Yaw = 0;

float Ts = 0.004;  //250 Hz-es loop miatt

int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;

void aksi_feszultseg(void) {
  Feszultseg = (float)analogRead(feszultseg_meres_PIN) / 206.818182;  //4095/(3.3*6)=206.8182
  Aram = (float)analogRead(aram_meres_PIN) / 37.227273;               // ADC / ((4095/3.3V)*0.03 V/A)=37.227273 ---> ADC*0,02686203
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
        uint16_t Temp[5];

        Temp[0] = IBUS_Buffer[2] | (IBUS_Buffer[3] << 8);    // CH1 Roll
        Temp[1] = IBUS_Buffer[4] | (IBUS_Buffer[5] << 8);    // CH2 Pitch
        Temp[2] = IBUS_Buffer[6] | (IBUS_Buffer[7] << 8);    // CH3 Gáz
        Temp[3] = IBUS_Buffer[8] | (IBUS_Buffer[9] << 8);    // CH4 Yaw
        Temp[4] = IBUS_Buffer[10] | (IBUS_Buffer[11] << 8);  // CH5 SWA

        bool Ervenyes = true;

        if (Temp[0] < 900 || Temp[0] > 2100) Ervenyes = false;
        if (Temp[1] < 900 || Temp[1] > 2100) Ervenyes = false;
        if (Temp[2] < 900 || Temp[2] > 2100) Ervenyes = false;
        if (Temp[3] < 900 || Temp[3] > 2100) Ervenyes = false;
        if (Temp[4] < 900 || Temp[4] > 2100) Ervenyes = false;

        if (Ervenyes) {
          Vevo_Ertekek[0] = Temp[0];
          Vevo_Ertekek[1] = Temp[1];
          Vevo_Ertekek[2] = Temp[2];
          Vevo_Ertekek[3] = Temp[3];
          Vevo_Ertekek[4] = Temp[4];

          Utolso_IBUS_Csomag_Ido = millis();

          IBUS_Kapcsolat_El = true;
          Failsafe = false;

          SWA_Allapot = Vevo_Ertekek[4] > 1500;
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
    SWA_Allapot = false;
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
}

void gyro_adat_olvasas(void) {

  //Girszkóp adatainak kiolvasása
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x43);              //Gyro adatok itt keződnek
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);  // 6 bytot kérünk

  uint32_t I2C_Start = micros();

  while (Wire.available() < 6) {
    if (micros() - I2C_Start > 1000) {
      MPU_Hiba = true;
      return;
    }
  }
  MPU_Hiba = false;

  //MPU 16 bites értékeit bele bitshiftelni egy 16 bites változóba mivel I2C rigsztereken csak 8 bites byte-okban kommunikál
  // << bitshift 8 bittel feltoljuk a 16 bites változó 15-8 bitjébe
  // | (bitwise OR) összekapcsolja a byte.okat
  int16_t Gyro_X = Wire.read() << 8 | Wire.read();
  int16_t Gyro_Y = Wire.read() << 8 | Wire.read();
  int16_t Gyro_Z = Wire.read() << 8 | Wire.read();

  // °/s szögsebességek kiszámolás mivel +-500°/s LSB 65.5/°/s
  // MPU 16 bites adat olvasás 2^16, +-500 = 1000 ------> 2^16/1000=65.536 ~ 65.5
  Rate_Roll = (float)Gyro_X / 65.5;
  Rate_Pitch = (float)Gyro_Y / 65.5;
  Rate_Yaw = (float)Gyro_Z / 65.5;
}

void PID_egyenlet(float Hiba, float P, float I, float D, float Elozo_Hiba, float Elozo_Iterm) {
  float Pterm = P * Hiba;
  float Iterm = Elozo_Iterm + (I * (Hiba + Elozo_Hiba)) * (Ts / 2);
  if (Iterm > 400) {
    Iterm = 400;
  } else if (Iterm < -400) {
    Iterm = -400;
  }
  float Dterm = D * ((Hiba - Elozo_Hiba) / Ts);
  float PID_Kimenet = Pterm + Iterm + Dterm;
  if (PID_Kimenet > 400) {
    PID_Kimenet = 400;
  } else if (PID_Kimenet < -400) {
    PID_Kimenet = -400;
  }

  PIDReturn[0] = PID_Kimenet;
  PIDReturn[1] = Hiba;
  PIDReturn[2] = Iterm;
}

void Reset_PID(void) {
  Elozo_Hiba_Roll_ertek = 0;
  Elozo_Hiba_Pitch_ertek = 0;
  Elozo_Hiba_Yaw_ertek = 0;
  Elozo_Iterm_Roll = 0;
  Elozo_Iterm_Pitch = 0;
  Elozo_Iterm_Yaw = 0;
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

  Serial.begin(115200);
  delay(1000);

  Serial.print("Reset reason CPU0: ");
  Serial.println(rtc_get_reset_reason(0));

  Serial.print("Reset reason CPU1: ");
  Serial.println(rtc_get_reset_reason(1));

  Serial.print("MPU: ");
  Serial.println(MPU_Hiba);

  pinMode(RX_PIN, INPUT_PULLUP);

  Serial2.setRxBufferSize(512);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  /*analogReadResolution(12);
  analogSetPinAttenuation(feszultseg_meres_PIN, ADC_11db);
  analogSetPinAttenuation(aram_meres_PIN, ADC_11db);*/

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Piros_LED, OUTPUT);
  pinMode(Zold_LED, OUTPUT);

  digitalWrite(Piros_LED, HIGH);
  digitalWrite(Zold_LED, LOW);

  //ledcAttachChannel(Motor_1_ESC_Pin, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas, Motor_1_ESC_Csatorna); //ESP32 core 3.3.6
  ledcSetup(Motor_1_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_1_ESC_Pin, Motor_1_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //ledcAttachChannel(Motor_2_ESC_Pin, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas, Motor_2_ESC_Csatorna); //ESP32 core 3.3.6
  ledcSetup(Motor_2_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_2_ESC_Pin, Motor_2_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //ledcAttachChannel(Motor_3_ESC_Pin, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas, Motor_3_ESC_Csatorna); //ESP32 core 3.3.6
  ledcSetup(Motor_3_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_3_ESC_Pin, Motor_3_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //ledcAttachChannel(Motor_4_ESC_Pin, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas, Motor_4_ESC_Csatorna); //ESP32 core 3.3.6
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

  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  Wire.begin();           //SDA, SCL
  delay(250);
  mpu_6050_regiszterek_beallitasa();

  //2000 mintavételes kalibráció
  for (int Kalibracios_int = 0; Kalibracios_int < 2000; Kalibracios_int++) {
    gyro_adat_olvasas();
    Rate_Roll_Offset += Rate_Roll;
    Rate_Pitch_Offset += Rate_Pitch;
    Rate_Yaw_Offset += Rate_Yaw;
    delay(1);
  }

  //Mintavételi számmal való vissza osztás
  Rate_Roll_Offset /= (float)2000;
  Rate_Pitch_Offset /= (float)2000;
  Rate_Yaw_Offset /= (float)2000;

  digitalWrite(Zold_LED, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  /*aksi_feszultseg();
  if (Feszultseg > 16.6) {                 // terhelés nélkűl mérjük az aksit így valós értéket kapunk mivel terhelés alatt beesik a feszülség szintje és nem lesz valós érték ezért terhlés alatt áramot mmérünk
    digitalWrite(Piros_LED, LOW);          //piros led kikapcs
    Aksi_kezdetben = Alap_aksi_kapacitas;  //felöltve tehát 1300mAh
  } else if (Feszultseg < 15) {
    digitalWrite(Piros_LED, HIGH);
    Aksi_kezdetben = (float)30 / 100 * Alap_aksi_kapacitas;  //30% át vesszük
  } else {
    digitalWrite(Piros_LED, LOW);
    Aksi_kezdetben = (41 * Feszultseg - 580) / 100 * Alap_aksi_kapacitas;
  }*/

  // kontrollálatlan motorindítás elkerülése
  while (Vevo_Ertekek[2] < 1020 || Vevo_Ertekek[2] > 1050 || Failsafe) {
    IBUS_Olvasas();
    IBUS_Failsafe_Ellenorzes();
    delay(4);
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Loop_Start_Us = micros();
}

void loop() {

  Loop_Start_Us = micros();

  if (Elozo_Loop_Start_Us != 0) {
    Loop_Teljes_Us = Loop_Start_Us - Elozo_Loop_Start_Us;

    if (Loop_Teljes_Us < Min_Teljes_Us) Min_Teljes_Us = Loop_Teljes_Us;
    if (Loop_Teljes_Us > Max_Teljes_Us) Max_Teljes_Us = Loop_Teljes_Us;

    Sum_Teljes_Us += Loop_Teljes_Us;
    Minta_Szam++;
  }

  Elozo_Loop_Start_Us = Loop_Start_Us;

  gyro_adat_olvasas();  //Gyro adat olvasó fgv meghívása

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  //exponenciális low-pass / mozgóátlag jellegű szűrő RATE MODE PID-hez hiba számoláshoz
  Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw * 0.3;

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  Kivant_Roll_ertek = 0.06 * RC_Deadband(Vevo_Ertekek[0]);
  Kivant_Pitch_ertek = 0.06 * RC_Deadband(Vevo_Ertekek[1]);
  Bemenet_Gaz = Vevo_Ertekek[2];
  Kivant_Yaw_ertek = 0.04 * RC_Deadband(Vevo_Ertekek[3]);

  Hiba_Roll_ertek = Kivant_Roll_ertek - Rate_Roll_Szurt;
  Hiba_Pitch_ertek = Kivant_Pitch_ertek - Rate_Pitch_Szurt;
  Hiba_Yaw_ertek = Kivant_Yaw_ertek - Rate_Yaw_Szurt;

  // PID Roll
  PID_egyenlet(Hiba_Roll_ertek, P_ertek_Roll, I_ertek_Roll, D_ertek_Roll, Elozo_Hiba_Roll_ertek, Elozo_Iterm_Roll);
  Bemenet_Roll = PIDReturn[0];
  Elozo_Hiba_Roll_ertek = PIDReturn[1];
  Elozo_Iterm_Roll = PIDReturn[2];

  //PID Pitch
  PID_egyenlet(Hiba_Pitch_ertek, P_ertek_Pitch, I_ertek_Pitch, D_ertek_Pitch, Elozo_Hiba_Pitch_ertek, Elozo_Iterm_Pitch);
  Bemenet_Pitch = PIDReturn[0];
  Elozo_Hiba_Pitch_ertek = PIDReturn[1];
  Elozo_Iterm_Pitch = PIDReturn[2];

  //PID Yaw
  PID_egyenlet(Hiba_Yaw_ertek, P_ertek_Yaw, I_ertek_Yaw, D_ertek_Yaw, Elozo_Hiba_Yaw_ertek, Elozo_Iterm_Yaw);
  Bemenet_Yaw = PIDReturn[0];
  Elozo_Hiba_Yaw_ertek = PIDReturn[1];
  Elozo_Iterm_Yaw = PIDReturn[2];

  if (Bemenet_Gaz > 1200) Bemenet_Gaz = 1200;

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
    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;
    Reset_PID();
  }

  //ledcWriteChannel(Motor_1_ESC_Csatorna, Motor_1_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));  //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_2_ESC_Csatorna, Motor_2_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));  //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_3_ESC_Csatorna, Motor_3_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));  //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_4_ESC_Csatorna, Motor_4_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));  //ESP32 core 2.0.17

  /*if (millis() - Aksi_meres >= 100){ // 100 ms 10Hz
  Aksi_meres = millis();
  aksi_feszultseg();
  Elfogyasztott_kapacitas = Aram * 1000 * 0.1 / 3600 + Elfogyasztott_kapacitas;
  Aksi_maradek = (Aksi_kezdetben - Elfogyasztott_kapacitas) / Alap_aksi_kapacitas * 100;
  if (Aksi_maradek <= 30) digitalWrite(Piros_LED, HIGH);
  else digitalWrite(Piros_LED, LOW);
}*/

  Loop_Munka_Us = micros() - Loop_Start_Us;

  if (Loop_Munka_Us < Min_Munka_Us) Min_Munka_Us = Loop_Munka_Us;
  if (Loop_Munka_Us > Max_Munka_Us) Max_Munka_Us = Loop_Munka_Us;

  if (Loop_Munka_Us > 4000) {
    Tulcsordulas_Szam++;
  } else {
    while (micros() - Loop_Start_Us < 4000)
      ;
  }

  if (millis() - Debug_Ido >= 1000) {
    Debug_Ido = millis();

    uint32_t Atlag_Teljes_Us = 0;

    if (Minta_Szam > 0) {
      Atlag_Teljes_Us = Sum_Teljes_Us / Minta_Szam;
    }

    Serial.print("Loop teljes min/us: ");
    Serial.print(Min_Teljes_Us);

    Serial.print(" | atlag/us: ");
    Serial.print(Atlag_Teljes_Us);

    Serial.print(" | max/us: ");
    Serial.print(Max_Teljes_Us);

    Serial.print(" | munka min/us: ");
    Serial.print(Min_Munka_Us);

    Serial.print(" | munka max/us: ");
    Serial.print(Max_Munka_Us);

    Serial.print(" | MPU: ");
    Serial.print(MPU_Hiba);

    Serial.print(" | tulcsordulas: ");
    Serial.println(Tulcsordulas_Szam);

    Min_Teljes_Us = 999999;
    Max_Teljes_Us = 0;
    Sum_Teljes_Us = 0;
    Minta_Szam = 0;

    Min_Munka_Us = 999999;
    Max_Munka_Us = 0;
    Tulcsordulas_Szam = 0;
  }
}