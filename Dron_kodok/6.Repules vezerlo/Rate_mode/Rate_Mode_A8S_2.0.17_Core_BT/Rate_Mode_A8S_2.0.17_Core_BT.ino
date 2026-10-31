#include <Wire.h>  //I2C könyvtár
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

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

String BT_Buffer = "";

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

uint16_t Vevo_Ertekek[] = { 1500, 1500, 1000, 1500, 1000, 1000 };
bool SWA_Allapot = false;
bool Rezges_Aktiv = false;
bool Elozo_Rezges_Aktiv = false;
bool Tune_LED = false;
bool Iterm_Aktiv = false;

float Feszultseg;
float Aram;
uint32_t Aksi_meres;
uint32_t Debug_meres;
float Aksi_maradek;                 // %-ban
float Aksi_kezdetben;               // mAh-ban
float Elfogyasztott_kapacitas = 0;  // mAh-ban
float Alap_aksi_kapacitas = 2000;   // 2000mAh

// 250 Hz-es loop vizsgálatához
uint32_t Loop_Timer;
uint32_t Loop_Munka_us = 0;
uint32_t Loop_Max_Munka_us = 0;
uint32_t Loop_Tulcsordulas = 0;

float Kivant_Roll_ertek, Kivant_Pitch_ertek, Kivant_Yaw_ertek;
float Hiba_Roll_ertek, Hiba_Pitch_ertek, Hiba_Yaw_ertek;
float Elozo_Hiba_Roll_ertek, Elozo_Hiba_Pitch_ertek, Elozo_Hiba_Yaw_ertek;
float Elozo_Iterm_Roll, Elozo_Iterm_Pitch, Elozo_Iterm_Yaw;
float Dterm_Roll, Dterm_Pitch, Dterm_Yaw;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
float PIDReturn[] = { 0, 0, 0, 0 };

float P_ertek_Roll = 0.485, P_ertek_Pitch = P_ertek_Roll, P_ertek_Yaw = 0.55;
float I_ertek_Roll = 1.435, I_ertek_Pitch = I_ertek_Roll, I_ertek_Yaw = 1.1;
float D_ertek_Roll = 0.01075, D_ertek_Pitch = D_ertek_Roll, D_ertek_Yaw = 0;

float Ts = 0.004;  //250 Hz-es loop miatt

int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;

float Rate_Roll_Min = 9999;
float Rate_Roll_Max = -9999;

float Rate_Pitch_Min = 9999;
float Rate_Pitch_Max = -9999;

float Rate_Yaw_Min = 9999;
float Rate_Yaw_Max = -9999;

void PID_ertekek_kiirasa() {

  SerialBT.println("===== PID =====");
  SerialBT.print("P_R/P: ");
  SerialBT.println(P_ertek_Roll, 5);
  SerialBT.print("I_R/P: ");
  SerialBT.println(I_ertek_Roll, 5);
  SerialBT.print("D_R/P: ");
  SerialBT.println(D_ertek_Roll, 5);

  SerialBT.print("P_Y: ");
  SerialBT.println(P_ertek_Yaw, 5);
  SerialBT.print("I_Y: ");
  SerialBT.println(I_ertek_Yaw, 5);
  SerialBT.print("D_Y: ");
  SerialBT.println(D_ertek_Yaw, 5);
}

void BT_PID_parancs_feldolgozas(String parancs) {
  parancs.trim();

  if (parancs == "SAVE") {
    SerialBT.println("SAVE most kikapcsolva, mert resetet okoz.");
    return;
  }

  if (parancs == "LOAD") {
    SerialBT.println("LOAD most kikapcsolva, mert resetet okoz.");
    return;
  }

  if (parancs == "PRINT") {
    PID_ertekek_kiirasa();
    return;
  }

  int eq = parancs.indexOf('=');
  if (eq < 0) return;

  String kulcs = parancs.substring(0, eq);
  float ertek = parancs.substring(eq + 1).toFloat();

  if (kulcs == "PR") {
    P_ertek_Roll = ertek;
    P_ertek_Pitch = ertek;
    SerialBT.println("P Roll/Pitch beallitva.");
  } else if (kulcs == "IR") {
    I_ertek_Roll = ertek;
    I_ertek_Pitch = ertek;
    SerialBT.println("I Roll/Pitch beallitva.");
  } else if (kulcs == "DR") {
    D_ertek_Roll = ertek;
    D_ertek_Pitch = ertek;
    SerialBT.println("D Roll/Pitch beallitva.");
  } else if (kulcs == "PY") {
    P_ertek_Yaw = ertek;
    SerialBT.println("P Yaw beallitva.");
  } else if (kulcs == "IY") {
    I_ertek_Yaw = ertek;
    SerialBT.println("I Yaw beallitva.");
  } else if (kulcs == "DY") {
    D_ertek_Yaw = ertek;
    SerialBT.println("D Yaw beallitva.");
  } else {
    SerialBT.println("Ismeretlen kulcs.");
    return;
  }
  PID_ertekek_kiirasa();
}

void BT_PID_kezelese_limitelt() {
  uint8_t karakter_limit = 20;

  while (SerialBT.available() && karakter_limit > 0) {
    karakter_limit--;

    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      if (BT_Buffer.length() > 0) {
        BT_PID_parancs_feldolgozas(BT_Buffer);
        BT_Buffer = "";
      }
    } else {
      BT_Buffer += c;

      if (BT_Buffer.length() > 40) {
        BT_Buffer = "";
        SerialBT.println("Tul hosszu parancs, torolve.");
      }
    }
  }
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
}

void gyro_adat_olvasas(void) {

  //Girszkóp adatainak kiolvasása
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x43);              //Gyro adatok itt keződnek
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);  // 6 bytot kérünk


  while (Wire.available() < 6) {}

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
  if (Iterm > 300) {
    Iterm = 300;
  } else if (Iterm < -300) {
    Iterm = -300;
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
  PIDReturn[3] = Dterm;
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
  //Serial.begin(115200);
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

  SerialBT.begin("ESP32_PID_TUNING");
  delay(500);

  Wire.begin();           //SDA, SCL
  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
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
  gyro_adat_olvasas();

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  //exponenciális low-pass / mozgóátlag jellegű szűrő RATE MODE PID-hez hiba számoláshoz
  Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw * 0.3;

  Rate_Roll_Min = min(Rate_Roll_Min, Rate_Roll_Szurt);
  Rate_Roll_Max = max(Rate_Roll_Max, Rate_Roll_Szurt);

  Rate_Pitch_Min = min(Rate_Pitch_Min, Rate_Pitch_Szurt);
  Rate_Pitch_Max = max(Rate_Pitch_Max, Rate_Pitch_Szurt);

  Rate_Yaw_Min = min(Rate_Yaw_Min, Rate_Yaw_Szurt);
  Rate_Yaw_Max = max(Rate_Yaw_Max, Rate_Yaw_Szurt);

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  if (Rezges_Aktiv && !Elozo_Rezges_Aktiv) {
    Loop_Max_Munka_us = 0;
    Loop_Tulcsordulas = 0;
  }
  Elozo_Rezges_Aktiv = Rezges_Aktiv;

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

    if (Vevo_Ertekek[2] < 1030) {
      BT_PID_kezelese_limitelt();
    }

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
  Tune_LED = false;

  Kivant_Roll_ertek = 0.15 * RC_Deadband(Vevo_Ertekek[0]);
  Kivant_Pitch_ertek = 0.15 * RC_Deadband(Vevo_Ertekek[1]);
  Bemenet_Gaz = Vevo_Ertekek[2];
  Kivant_Yaw_ertek = 0.15 * RC_Deadband(Vevo_Ertekek[3]);

  Hiba_Roll_ertek = Kivant_Roll_ertek - Rate_Roll_Szurt;
  Hiba_Pitch_ertek = Kivant_Pitch_ertek - Rate_Pitch_Szurt;
  Hiba_Yaw_ertek = Kivant_Yaw_ertek - Rate_Yaw_Szurt;

  if (Vevo_Ertekek[2] > 1130) Iterm_Aktiv = true;

  //ternary operator
  //feltétel ? érték ha igaz : érték ha hamis
  float I_Roll_Aktiv = Iterm_Aktiv ? I_ertek_Roll : 0.0;
  float I_Pitch_Aktiv = Iterm_Aktiv ? I_ertek_Pitch : 0.0;
  float I_Yaw_Aktiv = Iterm_Aktiv ? I_ertek_Yaw : 0.0;

  // PID Roll
  PID_egyenlet(Hiba_Roll_ertek, P_ertek_Roll, I_Roll_Aktiv, D_ertek_Roll, Elozo_Hiba_Roll_ertek, Elozo_Iterm_Roll);
  Bemenet_Roll = PIDReturn[0];
  Elozo_Hiba_Roll_ertek = PIDReturn[1];
  Elozo_Iterm_Roll = PIDReturn[2];
  Dterm_Roll = PIDReturn[3];

  //PID Pitch
  PID_egyenlet(Hiba_Pitch_ertek, P_ertek_Pitch, I_Pitch_Aktiv, D_ertek_Pitch, Elozo_Hiba_Pitch_ertek, Elozo_Iterm_Pitch);
  Bemenet_Pitch = PIDReturn[0];
  Elozo_Hiba_Pitch_ertek = PIDReturn[1];
  Elozo_Iterm_Pitch = PIDReturn[2];
  Dterm_Pitch = PIDReturn[3];

  //PID Yaw
  PID_egyenlet(Hiba_Yaw_ertek, P_ertek_Yaw, I_Yaw_Aktiv, D_ertek_Yaw, Elozo_Hiba_Yaw_ertek, Elozo_Iterm_Yaw);
  Bemenet_Yaw = PIDReturn[0];
  Elozo_Hiba_Yaw_ertek = PIDReturn[1];
  Elozo_Iterm_Yaw = PIDReturn[2];
  Dterm_Yaw = PIDReturn[3];

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

  //Aksi mérés 10Hz
  if (millis() - Aksi_meres >= 100) {
    Aksi_meres = millis();
    aksi_feszultseg();

    Elfogyasztott_kapacitas += Aram * 1000 * 0.1 / 3600;
    Aksi_maradek = (Aksi_kezdetben - Elfogyasztott_kapacitas) / Alap_aksi_kapacitas * 100;
  }

  //Bluetooth Debug kb. 80–100 Hz körüli debug
  if (millis() - Debug_meres >= 8) {
    Debug_meres = millis();

    int Motor_Max = max(max((int)Motor_1_bemenet, (int)Motor_2_bemenet),
                        max((int)Motor_3_bemenet, (int)Motor_4_bemenet));

    int Motor_Min = min(min((int)Motor_1_bemenet, (int)Motor_2_bemenet),
                        min((int)Motor_3_bemenet, (int)Motor_4_bemenet));

    int Headroom_Fel = 2000 - Motor_Max;
    int Headroom_Le = Motor_Min - Motorok_alapjarat;

    int LowSat = 0;

    if (Vevo_Ertekek[2] > 1030) {
      if (Motor_1_bemenet <= Motorok_alapjarat + 2) LowSat = 1;
      if (Motor_2_bemenet <= Motorok_alapjarat + 2) LowSat = 1;
      if (Motor_3_bemenet <= Motorok_alapjarat + 2) LowSat = 1;
      if (Motor_4_bemenet <= Motorok_alapjarat + 2) LowSat = 1;
    }

    // Roll motorpár-különbség
    // Bal oldal - jobb oldal
    int Roll_Diff = ((int)Motor_3_bemenet + (int)Motor_4_bemenet)
                    - ((int)Motor_1_bemenet + (int)Motor_2_bemenet);

    // Pitch motorpár-különbség
    // Hátsó motorpár - első motorpár
    int Pitch_Diff = ((int)Motor_1_bemenet + (int)Motor_3_bemenet)
                     - ((int)Motor_2_bemenet + (int)Motor_4_bemenet);

    // Yaw motorpár-különbség
    // CW pár - CCW pár
    int Yaw_Diff = ((int)Motor_1_bemenet + (int)Motor_4_bemenet)
                   - ((int)Motor_2_bemenet + (int)Motor_3_bemenet);

    int RG_Roll = (int)(Rate_Roll_Max - Rate_Roll_Min);
    int RG_Pitch = (int)(Rate_Pitch_Max - Rate_Pitch_Min);
    int RG_Yaw = (int)(Rate_Yaw_Max - Rate_Yaw_Min);

    int IBUS_Age = millis() - Utolso_IBUS_Csomag_Ido;

    char sor[450];

    /*
    t = időbélyeg
    A log ideje milliszekundumban. Ebből látjuk, mikor történt valami, mennyi ideig tartott a remegés, és mennyi idő volt két remegés között.

    R = remegés jelölés
    A kapcsolóval jelölt állapot. R:1 amikor remegést észleltél, R:0 amikor nem.

    T = throttle / gáz
    A gázkar értéke. Ezzel nézzük, hogy a remegés adott gázállásnál jelentkezik-e.

    SP = kívánt Roll/Pitch/Yaw rate
    Amit te kérsz a bottal. Ha SP közel 0, akkor nem te kérsz mozgást.

    RT = mért/szűrt Roll/Pitch/Yaw rate
    A gyro által mért tényleges forgási sebesség. Ebből látjuk, hogy a drón melyik tengelyen mozog vagy remeg.

    ER = hiba
    A kívánt és a mért érték különbsége. ER = SP - RT. Ezt próbálja kijavítani a PID.

    M = motorparancsok
    A négy motorra kiküldött PWM érték. Ebből látjuk, hogy melyik motor mennyit dolgozik.

    PID = PID kimenetek
    A Roll/Pitch/Yaw PID szabályzó kimenete. Megmutatja, melyik tengelyen dolgozik legerősebben a szabályzás.

    I = I-termek
    A PID integráló része. Lassú, felgyűlő korrekciót mutat. Nagy érték súlypont-, trim- vagy tartós hiba gyanú lehet.

    D = D-termek
    A PID gyors változásra reagáló része. Ha remegéskor nagyot ugrál, akkor gyro-zaj vagy mechanikai rezonancia gyanús.

    B = akkufesz
    Az akkumulátor feszültsége ×100 formában. Például B:1510 = 15.10 V.

    HR = headroom fel/le
    Megmutatja, mennyi tartalék van a motoroknál felfelé 2000 us-ig és lefelé az alapjáratig.

    LS = alsó limit jelzés
    Azt jelzi, hogy valamelyik motor leért-e az alapjárati limit közelébe. LS:1 = alsó limit gyanú.

    MD = Roll/Pitch/Yaw motorpár-különbség
    A motorparancsokból számolt tengelyirányú eltérés. Megmutatja, Roll, Pitch vagy Yaw irányban dolgoznak-e jobban a motorok.

    RG = rate range 8 ms alatt
    A gyro rate kilengése 8 ms alatt. Ez mutatja legjobban a gyors rezgést. Ha RG Roll nagy, akkor Roll irányú rezgés gyanús.

    AGE = IBUS frissesség
    Megmutatja, hány ms telt el az utolsó érvényes vevőjel óta. Ha nagy, akkor vevőjel/táp/kábel probléma lehet.

    LM = max loop munkaidő
    A legnagyobb mért loop végrehajtási idő mikrosecundumban. 250 Hz-nél 4000 us alatt kell maradnia.

    O = loop túlcsordulás
    Azt számolja, hányszor csúszott túl a loop a 4000 us időkereten. O:0 = jó.
    */

    snprintf(sor, sizeof(sor),
             "t:%lu R:%d T:%d SP:%d,%d,%d RT:%d,%d,%d ER:%d,%d,%d M:%d,%d,%d,%d PID:%d,%d,%d I:%d,%d,%d D:%d,%d,%d B:%d HR:%d,%d LS:%d MD:%d,%d,%d RG:%d,%d,%d AGE:%d LM:%lu O:%lu\n",
             (unsigned long)millis(),

             Rezges_Aktiv,

             Vevo_Ertekek[2],

             (int)Kivant_Roll_ertek,
             (int)Kivant_Pitch_ertek,
             (int)Kivant_Yaw_ertek,

             (int)Rate_Roll_Szurt,
             (int)Rate_Pitch_Szurt,
             (int)Rate_Yaw_Szurt,

             (int)Hiba_Roll_ertek,
             (int)Hiba_Pitch_ertek,
             (int)Hiba_Yaw_ertek,

             (int)Motor_1_bemenet,
             (int)Motor_2_bemenet,
             (int)Motor_3_bemenet,
             (int)Motor_4_bemenet,

             (int)Bemenet_Roll,
             (int)Bemenet_Pitch,
             (int)Bemenet_Yaw,

             (int)Elozo_Iterm_Roll,
             (int)Elozo_Iterm_Pitch,
             (int)Elozo_Iterm_Yaw,

             (int)Dterm_Roll,
             (int)Dterm_Pitch,
             (int)Dterm_Yaw,

             (int)(Feszultseg * 100),

             Headroom_Fel,
             Headroom_Le,

             LowSat,

             Roll_Diff,
             Pitch_Diff,
             Yaw_Diff,

             RG_Roll,
             RG_Pitch,
             RG_Yaw,

             IBUS_Age,

             (unsigned long)Loop_Max_Munka_us,
             (unsigned long)Loop_Tulcsordulas);

    SerialBT.print(sor);

    Rate_Roll_Min = 9999;
    Rate_Roll_Max = -9999;

    Rate_Pitch_Min = 9999;
    Rate_Pitch_Max = -9999;

    Rate_Yaw_Min = 9999;
    Rate_Yaw_Max = -9999;
  }

  // ===== Loop idő mérés =====
  Loop_Munka_us = micros() - Loop_Timer;

  if (Loop_Munka_us > Loop_Max_Munka_us) {
    Loop_Max_Munka_us = Loop_Munka_us;
  }

  if (Loop_Munka_us > 4000) {
    Loop_Tulcsordulas++;
  }

  // ===== 250 Hz tartása =====
  while (micros() - Loop_Timer < 4000)
    ;

  Loop_Timer = micros();
}