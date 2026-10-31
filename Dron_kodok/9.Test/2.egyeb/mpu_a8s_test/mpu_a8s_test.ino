#include <IBusBM.h>
#include <Wire.h>  //I2C könyvtár
#include "rom/rtc.h"

IBusBM ibus;

//ibus.readChannel(0-9) 10 csatorna
#define RX_PIN 16
#define TX_PIN -1

//static const uint8_t Csatorna_Szam = 8;
//static azt jelenti, hogy ez a változó “fájl-szintű / globális” élettartamú, vagyis a program teljes futása alatt létezik, és ugyanaz az egy példánya van.
//csak ebben a .cpp fájlban látható (internal linkage). Tehát ha több fájlod van, másik .cpp-ből nem tudod elérni név szerint.
uint16_t Vevo_Ertekek[] = { 0, 0, 0, 0, 0 };
bool SWA_Allapot;

uint32_t Utolso_IBUS_Csomag_Ido = 0;
uint8_t Elozo_IBUS_Csomag_Szam = 0;

bool IBUS_Kapcsolat_El = false;
bool Failsafe = true;
bool MPU_Hiba;

// 250 Hz-es loop vizsgálatához
uint32_t Loop_idozito;
uint32_t Loop_Start_Us;
uint32_t Loop_Start_Elozo_Us;
uint32_t Loop_Ido_Us;
uint32_t Debug_Ido;

float Rate_Roll, Rate_Pitch, Rate_Yaw;
float Rate_Roll_Szurt, Rate_Pitch_Szurt, Rate_Yaw_Szurt;
float Rate_Roll_Offset, Rate_Pitch_Offset, Rate_Yaw_Offset;

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

void IBUS_Failsafe_Ellenorzes() {
  uint32_t most = millis();

  // Ha érkezett új iBUS csomag, akkor frissítjük az időt
  if (ibus.cnt_rec != Elozo_IBUS_Csomag_Szam) {
    Elozo_IBUS_Csomag_Szam = ibus.cnt_rec;
    Utolso_IBUS_Csomag_Ido = most;
    IBUS_Kapcsolat_El = true;
  }

  // Ha 100 ms óta nincs új iBUS csomag, akkor kapcsolatvesztés
  if (most - Utolso_IBUS_Csomag_Ido > 100) {
    IBUS_Kapcsolat_El = false;
  }

  // Alapból nincs failsafe
  Failsafe = false;

  // Ha nincs élő iBUS kapcsolat
  if (!IBUS_Kapcsolat_El) {
    Failsafe = true;
  }

  // Ha a fontos csatornák értéke teljesen hülye
  if (Vevo_Ertekek[0] < 900 || Vevo_Ertekek[0] > 2100) Failsafe = true;  // Roll
  if (Vevo_Ertekek[1] < 900 || Vevo_Ertekek[1] > 2100) Failsafe = true;  // Pitch
  if (Vevo_Ertekek[2] < 900 || Vevo_Ertekek[2] > 2100) Failsafe = true;  // Throttle
  if (Vevo_Ertekek[3] < 900 || Vevo_Ertekek[3] > 2100) Failsafe = true;  // Yaw
  if (Vevo_Ertekek[4] < 900 || Vevo_Ertekek[4] > 2100) Failsafe = true;  // Arm kapcsoló
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

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);
  Serial.begin(115200);

  Serial.print("Reset reason CPU0: ");
  Serial.println(rtc_get_reset_reason(0));

  Serial.print("Reset reason CPU1: ");
  Serial.println(rtc_get_reset_reason(1));

  Serial.print("MPU: ");
  Serial.println(MPU_Hiba);

  //pinMode(RX_PIN, INPUT_PULLUP);

  Serial2.setRxBufferSize(256);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  ibus.begin(Serial2, 2);

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

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  Loop_Start_Us = micros();
  if (Loop_Start_Elozo_Us != 0) Loop_idozito = Loop_Start_Us - Loop_Start_Elozo_Us;
  Loop_Start_Elozo_Us = Loop_Start_Us;

  gyro_adat_olvasas();  //Gyro adat olvasó fgv meghívása

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Offset;
  Rate_Pitch -= Rate_Pitch_Offset;
  Rate_Yaw -= Rate_Yaw_Offset;

  //exponenciális low-pass / mozgóátlag jellegű szűrő RATE MODE PID-hez hiba számoláshoz
  Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw * 0.3;

  // olvasás + print
  Csatorna_olvasas();
  IBUS_Failsafe_Ellenorzes();

  if (Failsafe) Vevo_Ertekek[2] = 1000;


  if (millis() - Debug_Ido >= 1000) {
    Debug_Ido = millis();

    /*Serial.print("Roll: ");
    Serial.print(Vevo_Ertekek[0]);

    Serial.print(" Pitch: ");
    Serial.print(Vevo_Ertekek[1]);*/

    Serial.print(" Gaz: ");
    Serial.print(Vevo_Ertekek[2]);

    /*Serial.print(" Yaw: ");
    Serial.print(Vevo_Ertekek[3]);

    Serial.print(" SWA: ");
    Serial.print(SWA_Allapot);*/

    Serial.print(" Fail: ");
    Serial.print(Failsafe);

    Serial.print(" szam: ");
    Serial.print(Elozo_IBUS_Csomag_Szam);

    Serial.print(" MPU: ");
    Serial.print(MPU_Hiba);

    Serial.print(" Loop: ");
    Serial.println(Loop_idozito);
  }

  Loop_Ido_Us = micros() - Loop_Start_Us;

  if (Loop_Ido_Us < 4000) {
    delayMicroseconds(4000 - Loop_Ido_Us);
  }
}