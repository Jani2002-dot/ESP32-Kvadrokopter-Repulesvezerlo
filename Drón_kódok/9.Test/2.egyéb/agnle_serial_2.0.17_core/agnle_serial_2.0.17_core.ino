#include <Wire.h>  //I2C könyvtár
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

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

uint16_t Vevo_Ertekek[] = { 1500, 1500, 1000, 1500, 1000 };
bool SWA_Allapot = false;
bool Tune_LED = false;

float Feszultseg;
float Aram;
uint32_t Aksi_meres;
float Aksi_maradek;                 // %-ban
float Aksi_kezdetben;               // mAh-ban
float Elfogyasztott_kapacitas = 0;  // mAh-ban
float Alap_aksi_kapacitas = 2000;   // 2000mAh

// 250 Hz-es loop vizsgálatához
uint32_t Loop_Timer;
uint32_t Loop_Start_Us = 0;
uint32_t Elozo_Loop_Start_Us = 0;

uint32_t Loop_Munka_Us = 0;
uint32_t Loop_Teljes_Us = 0;

uint32_t Max_Munka_Us = 0;
uint32_t Max_Teljes_Us = 0;

uint32_t Debug_Ido = 0;
uint32_t Tulcsordulas_Szam = 0;

float Kivant_Roll_ertek, Kivant_Pitch_ertek, Kivant_Yaw_ertek;
float Hiba_Roll_ertek, Hiba_Pitch_ertek, Hiba_Yaw_ertek;
float Elozo_Hiba_Roll_ertek, Elozo_Hiba_Pitch_ertek, Elozo_Hiba_Yaw_ertek;
float Elozo_Iterm_Roll, Elozo_Iterm_Pitch, Elozo_Iterm_Yaw;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
float PIDReturn[] = { 0, 0, 0 };

float P_ertek_Roll = 0.5, P_ertek_Pitch = P_ertek_Roll, P_ertek_Yaw = 0.8;
float I_ertek_Roll = 1.625, I_ertek_Pitch = I_ertek_Roll, I_ertek_Yaw = 0;
float D_ertek_Roll = 0.0125, D_ertek_Pitch = D_ertek_Roll, D_ertek_Yaw = 0;

//kiszámmolt Acc értékek
float Acc_X, Acc_Y, Acc_Z;
//számított szög változó
float Szog_Roll, Szog_Pitch;

float Kalman_Szog_Roll = 0;
float Kalman_Szog_Roll_Korrigalt = 0;
float Kalman_Roll_Offset = 0;
float Kalman_Bizonytalansag_Szog_Roll = 2 * 2;  //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_Szog_Pitch = 0;
float Kalman_Szog_Pitch_Korrigalt = 0;
float Kalman_Pitch_Offset = 0;
float Kalman_Bizonytalansag_Szog_Pitch = 2 * 2;  //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_1D_kimenet[] = { 0, 0 };

//fly sky távirányítóval beáállított szög -50°-50°
float Kivant_Szog_Roll, Kivant_Szog_Pitch;

//kívánt szög - kálmán által mért szög
float Hiba_Szog_Roll, Hiba_Szog_Pitch;

float Elozo_Hiba_Szog_Roll, Elozo_Hiba_Szog_Pitch;

float Elozo_Iterm_Szog_Roll, Elozo_Iterm_Szog_Pitch;

float P_Szog_Roll = 2, P_Szog_Pitch = P_Szog_Roll;
float I_Szog_Roll = 0, I_Szog_Pitch = I_Szog_Roll;
float D_Szog_Roll = 0, D_Szog_Pitch = D_Szog_Roll;

float Ts = 0.004;  //250 Hz-es loop miatt

int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;

void PID_ertekek_kiirasa() {

  SerialBT.println("===== PID =====");
  SerialBT.print("P_R/P: ");
  SerialBT.println(P_Szog_Roll, 5);
  SerialBT.print("I_R/P: ");
  SerialBT.println(I_Szog_Roll, 5);
  SerialBT.print("D_R/P: ");
  SerialBT.println(D_Szog_Roll, 5);
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
    P_Szog_Roll = ertek;
    P_Szog_Pitch = ertek;
    SerialBT.println("P Roll/Pitch beallitva.");
  } else if (kulcs == "IR") {
    I_Szog_Roll = ertek;
    I_Szog_Pitch = ertek;
    SerialBT.println("I Roll/Pitch beallitva.");
  } else if (kulcs == "DR") {
    D_Szog_Roll = ertek;
    D_Szog_Pitch = ertek;
    SerialBT.println("D Roll/Pitch beallitva.");
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

void kalman_1d(float Kalman_allapot, float Kalman_Bizonytalansag, float Kalman_bemenet, float Kalman_meres) {
  Kalman_allapot = Kalman_allapot + Kalman_bemenet * Ts;
  Kalman_Bizonytalansag = Kalman_Bizonytalansag + Ts * Ts * 4 * 4;
  float Kalman_erosites = Kalman_Bizonytalansag * 1 / (1 * Kalman_Bizonytalansag + 3 * 3);
  Kalman_allapot = Kalman_allapot + Kalman_erosites * (Kalman_meres - Kalman_allapot);
  Kalman_Bizonytalansag = (1 - Kalman_erosites) * Kalman_Bizonytalansag;
  Kalman_1D_kimenet[0] = Kalman_allapot;
  Kalman_1D_kimenet[1] = Kalman_Bizonytalansag;
}

void Kalman_1D_szog_nullazas(void) {
  float roll_osszeg = 0;
  float pitch_osszeg = 0;

  for (int i = 0; i < 500; i++) {
    gyro_acc_adat_olvasas();

    Rate_Roll -= Rate_Roll_Offset;
    Rate_Pitch -= Rate_Pitch_Offset;
    Rate_Yaw -= Rate_Yaw_Offset;

    kalman_1d(Kalman_Szog_Roll, Kalman_Bizonytalansag_Szog_Roll, Rate_Roll, Szog_Roll);
    Kalman_Szog_Roll = Kalman_1D_kimenet[0];
    Kalman_Bizonytalansag_Szog_Roll = Kalman_1D_kimenet[1];

    kalman_1d(Kalman_Szog_Pitch, Kalman_Bizonytalansag_Szog_Pitch, Rate_Pitch, Szog_Pitch);
    Kalman_Szog_Pitch = Kalman_1D_kimenet[0];
    Kalman_Bizonytalansag_Szog_Pitch = Kalman_1D_kimenet[1];

    roll_osszeg += Kalman_Szog_Roll;
    pitch_osszeg += Kalman_Szog_Pitch;

    delay(4);
  }

  Kalman_Roll_Offset = roll_osszeg / 500.0;
  Kalman_Pitch_Offset = pitch_osszeg / 500.0;
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
  Acc_Z = (float)Acc_ZLSB / 4096 - 0.10790;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Szog_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  Szog_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);
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
  //Rate reset
  Elozo_Hiba_Roll_ertek = 0;
  Elozo_Hiba_Pitch_ertek = 0;
  Elozo_Hiba_Yaw_ertek = 0;
  Elozo_Iterm_Roll = 0;
  Elozo_Iterm_Pitch = 0;
  Elozo_Iterm_Yaw = 0;

  //Angle reset
  Elozo_Hiba_Szog_Roll = 0;
  Elozo_Hiba_Szog_Pitch = 0;

  Elozo_Iterm_Szog_Roll = 0;
  Elozo_Iterm_Szog_Pitch = 0;
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

void Loop_Meres_Debug_Varas(void) {
  Loop_Munka_Us = micros() - Loop_Start_Us;

  if (Loop_Munka_Us > Max_Munka_Us) {
    Max_Munka_Us = Loop_Munka_Us;
  }

  if (Loop_Munka_Us > 4000) {
    Tulcsordulas_Szam++;
  }

  if (millis() - Debug_Ido >= 500) {
    Debug_Ido = millis();

    Serial.print("Munka max/us: ");
    Serial.print(Max_Munka_Us);

    Serial.print(" | Teljes loop max/us: ");
    Serial.print(Max_Teljes_Us);

    Serial.print(" | Tulcsordulas: ");
    Serial.print(Tulcsordulas_Szam);

    /*Serial.print(" | Rate raw R/P/Y: ");
    Serial.print(Rate_Roll, 2);
    Serial.print(" / ");
    Serial.print(Rate_Pitch, 2);
    Serial.print(" / ");
    Serial.print(Rate_Yaw, 2);

    Serial.print(" | Rate szurt R/P/Y: ");
    Serial.print(Rate_Roll_Szurt, 2);
    Serial.print(" / ");
    Serial.print(Rate_Pitch_Szurt, 2);
    Serial.print(" / ");
    Serial.println(Rate_Yaw_Szurt, 2);*/

    /*Serial.print(" | Kivant R/P/Y/G: ");
    Serial.print(Kivant_Szog_Roll, 2);
    Serial.print(" / ");
    Serial.print(Kivant_Szog_Pitch, 2);
    Serial.print(" / ");
    Serial.print(Kivant_Yaw_ertek, 2);
    Serial.print(" / ");
    Serial.print(Vevo_Ertekek[2]);
    Serial.print(" / ");
    Serial.print(SWA_Allapot);
    Serial.print(" / ");
    Serial.println(Failsafe);*/

    Serial.print(" | Motorok: ");
    Serial.print(Motor_1_bemenet);
    Serial.print(" / ");
    Serial.print(Motor_2_bemenet);
    Serial.print(" / ");
    Serial.print(Motor_3_bemenet);
    Serial.print(" / ");
    Serial.println(Motor_4_bemenet);

    /*Serial.print(" | Acc: ");
    Serial.print(Acc_X, 5);
    Serial.print(" / ");
    Serial.print(Acc_Y, 5);
    Serial.print(" / ");
    Serial.println(Acc_Z, 5);*/

    /*Serial.print(" | Szogek: ");
    Serial.print(Kalman_Szog_Roll, 3);
    Serial.print(" / ");
    Serial.print(Kalman_Szog_Roll_Korrigalt, 3);
    Serial.print(" / ");
    Serial.print(Kalman_Szog_Pitch, 3);
    Serial.print(" / ");
    Serial.println(Kalman_Szog_Pitch_Korrigalt, 3);*/

    /*Serial.print(" | Szogek: ");
    Serial.print(Kalman_Roll_Offset, 3);
    Serial.print(" / ");
    Serial.println(Kalman_Pitch_Offset, 3);*/

    Max_Munka_Us = 0;
    Max_Teljes_Us = 0;
    Tulcsordulas_Szam = 0;
  }
  while (micros() - Loop_Timer < 4000)
    ;

  Loop_Timer = micros();
}

void setup() {

  Serial.begin(115200);
  delay(500);

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

  //Kalman_1D_szog_nullazas();

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

  Loop_Start_Us = micros();

  // Teljes loopidő: előző loop elejétől a mostani loop elejéig
  if (Elozo_Loop_Start_Us != 0) {
    Loop_Teljes_Us = Loop_Start_Us - Elozo_Loop_Start_Us;

    if (Loop_Teljes_Us > Max_Teljes_Us) {
      Max_Teljes_Us = Loop_Teljes_Us;
    }
  }

  Elozo_Loop_Start_Us = Loop_Start_Us;


  digitalWrite(LED_BUILTIN, Tune_LED);

  //Gyro adat olvasó fgv meghívása
  gyro_acc_adat_olvasas();

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  //exponenciális low-pass / mozgóátlag jellegű szűrő RATE MODE PID-hez hiba számoláshoz
  Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw * 0.3;

  kalman_1d(Kalman_Szog_Roll, Kalman_Bizonytalansag_Szog_Roll, Rate_Roll, Szog_Roll);
  Kalman_Szog_Roll = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Szog_Roll = Kalman_1D_kimenet[1];

  kalman_1d(Kalman_Szog_Pitch, Kalman_Bizonytalansag_Szog_Pitch, Rate_Pitch, Szog_Pitch);
  Kalman_Szog_Pitch = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Szog_Pitch = Kalman_1D_kimenet[1];

  Kalman_Szog_Roll_Korrigalt = Kalman_Szog_Roll - Kalman_Roll_Offset;
  Kalman_Szog_Pitch_Korrigalt = Kalman_Szog_Pitch - Kalman_Pitch_Offset;

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  if (Failsafe) {

    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;

    ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));
    ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));
    ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));
    ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));

    Reset_PID();

    Loop_Meres_Debug_Varas();
    return;
  }

  if (SWA_Allapot == 1) {
    Tune_LED = true;

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

    Loop_Meres_Debug_Varas();
    return;
  }

  Tune_LED = false;

  //Számítsd ki a kívánt szögeket a vevő (receiver) értékeiből.
  Kivant_Szog_Roll = 0.05 * RC_Deadband(Vevo_Ertekek[0]);
  Kivant_Szog_Pitch = 0.05 * RC_Deadband(Vevo_Ertekek[1]);
  Bemenet_Gaz = Vevo_Ertekek[2];
  Kivant_Yaw_ertek = 0.15 * RC_Deadband(Vevo_Ertekek[3]);

  //Számítsd ki a különbséget a kívánt és a tényleges roll és pitch szögek között.
  Hiba_Szog_Roll = Kivant_Szog_Roll - Kalman_Szog_Roll_Korrigalt;
  Hiba_Szog_Pitch = Kivant_Szog_Pitch - Kalman_Szog_Pitch_Korrigalt;

  //Számítsd ki a kívánt roll szögeket a külső hurok PID szabályozóján keresztül.
  PID_egyenlet(Hiba_Szog_Roll, P_Szog_Roll, I_Szog_Roll, D_Szog_Roll, Elozo_Hiba_Szog_Roll, Elozo_Iterm_Szog_Roll);
  Kivant_Roll_ertek = PIDReturn[0];
  Elozo_Hiba_Szog_Roll = PIDReturn[1];
  Elozo_Iterm_Szog_Roll = PIDReturn[2];

  //Számítsd ki a kívánt pitch szögeket a külső hurok PID szabályozóján keresztül.
  PID_egyenlet(Hiba_Szog_Pitch, P_Szog_Pitch, I_Szog_Pitch, D_Szog_Pitch, Elozo_Hiba_Szog_Pitch, Elozo_Iterm_Szog_Pitch);
  Kivant_Pitch_ertek = PIDReturn[0];
  Elozo_Hiba_Szog_Pitch = PIDReturn[1];
  Elozo_Iterm_Szog_Pitch = PIDReturn[2];

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

  Loop_Meres_Debug_Varas();
}