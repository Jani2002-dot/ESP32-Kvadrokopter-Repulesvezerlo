#include "esp_system.h"
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <Wire.h>  //I2C könyvtár
#include <IBusBM.h>

IBusBM ibus;
BluetoothSerial SerialBT;
Preferences preferences;

//ibus.readChannel(0-9) 10 csatorna
#define RX_PIN 16
#define TX_PIN 25

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

//static azt jelenti, hogy ez a változó “fájl-szintű / globális” élettartamú, vagyis a program teljes futása alatt létezik, és ugyanaz az egy példánya van.
//csak ebben a .cpp fájlban látható (internal linkage). Tehát ha több fájlod van, másik .cpp-ből nem tudod elérni név szerint.
uint16_t Vevo_Ertekek[] = { 0, 0, 0, 0, 0 };
bool SWA_Allapot;

float Feszultseg;
float Aram;
float Aksi_maradek;                 // %-ban
float Aksi_kezdetben;               // mAh-ban
float Elfogyasztott_kapacitas = 0;  // mAh-ban
float Alap_aksi_kapacitas = 3000;   // 2000mAh

uint32_t Loop_idozito;  // 250 Hz-es loop vizsgálatához

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

void Csatorna_olvasas() {
  for (uint8_t i = 0; i < 5; i++) {
    Vevo_Ertekek[i] = ibus.readChannel(i);
  }

  if (Vevo_Ertekek[4] > 1500) {
    SWA_Allapot = 1;
  } else {
    SWA_Allapot = 0;
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

  while (Wire.available() < 6) {}  // vár a 6 byte-ra

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
  return (uint32_t)(motor_bemenet_us * 1.02375);
}

int RC_Deadband(int ertek) {
  if (ertek > 1508) return ertek - 1508;
  if (ertek < 1492) return ertek - 1492;
  return 0;
}

/*void varakozas_250Hz(void) {
  while ((uint32_t)(micros() - Loop_idozito) < 4000) {
    delayMicroseconds(270); 
  }

  Loop_idozito = micros();
}*/

void setup() {

  Serial.begin(921600);
  delay(1000);

  Serial.println("BOOT");
  Serial.print("Reset reason: ");
  Serial.println(esp_reset_reason());
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  //soros vevőnek
  ibus.begin(Serial2);                                //ibus inicializálása

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Piros_LED, OUTPUT);
  pinMode(Zold_LED, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(Piros_LED, HIGH);
  digitalWrite(Zold_LED, LOW);

  //250Hz 12 bit felbontás mintha motor kapná de most nincs
  Motor_1_bemenet = usToDuty(1000);
  Motor_2_bemenet = usToDuty(1000);
  Motor_3_bemenet = usToDuty(1000);
  Motor_4_bemenet = usToDuty(1000);
  delay(3000);

  SerialBT.begin("ESP32_PID_TUNING");
  delay(500);

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
  digitalWrite(Piros_LED, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  // kontrollálatlan motorindítás elkerülése
  while (Vevo_Ertekek[2] < 1020 || Vevo_Ertekek[2] > 1050) {  //Vevo_Ertekek[2] ---> Gáz kar értéke  kontrollálatlan motorindítás elkerülése
    Csatorna_olvasas();
    delay(4);
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Loop_idozito = micros();
}

void loop() {

  gyro_adat_olvasas();  //Gyro adat olvasó fgv meghívása

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  //exponenciális low-pass / mozgóátlag jellegű szűrő RATE MODE PID-hez hiba számoláshoz
  Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw * 0.3;

  Csatorna_olvasas();

  if (SWA_Allapot == 1 && Vevo_Ertekek[2] < 1030) {
    digitalWrite(LED_BUILTIN, LOW);

    BT_PID_kezelese_limitelt();

    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;

    //250Hz 12 bit felbontás mintha motor kapná de most nincs
    Motor_1_bemenet = usToDuty(Motor_1_bemenet);
    Motor_2_bemenet = usToDuty(Motor_2_bemenet);
    Motor_3_bemenet = usToDuty(Motor_3_bemenet);
    Motor_4_bemenet = usToDuty(Motor_4_bemenet);

    Reset_PID();

    while (micros() - Loop_idozito < 4000) {
      delayMicroseconds(250);
    }

    Loop_idozito = micros();
    return;
  }

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

  //250Hz 12 bit felbontás mintha motor kapná de most nincs
  Motor_1_bemenet = usToDuty(Motor_1_bemenet);
  Motor_2_bemenet = usToDuty(Motor_2_bemenet);
  Motor_3_bemenet = usToDuty(Motor_3_bemenet);
  Motor_4_bemenet = usToDuty(Motor_4_bemenet);

static uint32_t Serial_Idozito = 0;

if (millis() - Serial_Idozito >= 100) {
  Serial_Idozito = millis();

  Serial.print("SetR:");
  Serial.print(Kivant_Roll_ertek, 2);

  Serial.print("\tRateR:");
  Serial.print(Rate_Roll_Szurt, 2);

  Serial.print("\tPID_R:");
  Serial.print(Bemenet_Roll, 2);

  Serial.print("\tM1:");
  Serial.print(Motor_1_bemenet, 0);

  Serial.print("\tM2:");
  Serial.print(Motor_2_bemenet, 0);

  Serial.print("\tM3:");
  Serial.print(Motor_3_bemenet, 0);

  Serial.print("\tM4:");
  Serial.print(Motor_4_bemenet, 0);

  Serial.print("\tLoop:");
  Serial.println((uint32_t)(micros() - Loop_idozito));
}

  while (micros() - Loop_idozito < 4000) {
    delayMicroseconds(250);
  }

  Loop_idozito = micros();
}