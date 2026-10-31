#include <Wire.h>  //I2C könyvtár

#define Piros_LED 13
#define Zold_LED 4

uint32_t Loop_idozito;  // 250 Hz-es loop vizsgálatához
uint32_t Loop_dt_us;
uint32_t Serial_Idozito = 0;

//számított szögsebességeknek változó
float Rate_Roll, Rate_Pitch, Rate_Yaw;
float Rate_Roll_2, Rate_Pitch_2, Rate_Yaw_2;
float Rate_Roll_Szurt, Rate_Pitch_Szurt, Rate_Yaw_Szurt;
float Ts = 0.004;
bool Komplementer_Elindult = false;

//kiszámmolt Acc értékek
float Acc_X, Acc_Y, Acc_Z;

//számított szög változók
float Szog_Roll, Szog_Pitch;

float Szurt_Szog_Roll, Szurt_Szog_Pitch;

//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Kalibracios = 0;
float Rate_Pitch_Kalibracios = 0;
float Rate_Yaw_Kalibracios = 0;

//for ciklus változói
int Kalibracios_int;
int Kalibracio_mintavetel_szam = 2000;

float Kalman_Roll_Offset = 0;
float Kalman_Pitch_Offset = 0;

float Kalman_Szog_Roll_Korrigalt = 0;
float Kalman_Szog_Pitch_Korrigalt = 0;

float Kalman_Szog_Roll = 0;
float Kalman_Bizonytalansag_Szog_Roll = 2 * 2;  //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_Szog_Pitch = 0;
float Kalman_Bizonytalansag_Szog_Pitch = 2 * 2;  //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_1D_kimenet[] = { 0, 0 };

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

    Rate_Roll -= Rate_Roll_Kalibracios;
    Rate_Pitch -= Rate_Pitch_Kalibracios;
    Rate_Yaw -= Rate_Yaw_Kalibracios;

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

void mpu_6050_regiszterek_beallitasa(void) {

  //MPU aktiválása (felébesztése)
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x6B);              //kérés küldése a regiszer indítására
  Wire.write(0x00);              //MPU felébesztésének beleírása HEX-be
  Wire.endTransmission();        //küldés

  //42Hz aluláteresztő beállítása
  Wire.beginTransmission(0x68);  //MPU-val való komunikáció
  Wire.write(0x1A);              //kérés küldése a regiszer indítására
  Wire.write(0x03);              //42Hz beleírása HEX-be
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
  Acc_X = (float)Acc_XLSB / 4096 - 0.0404;
  Acc_Y = (float)Acc_YLSB / 4096 + 0.0266;
  Acc_Z = (float)Acc_ZLSB / 4096 - 0.11188;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Szog_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  Szog_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Piros_LED,OUTPUT);
  pinMode(Zold_LED,OUTPUT);

  digitalWrite(LED_BUILTIN,HIGH);
  digitalWrite(Piros_LED,HIGH);
  digitalWrite(Zold_LED,LOW);

  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  Wire.begin();           //SDA, SCL
  delay(250);
  mpu_6050_regiszterek_beallitasa();
  delay(5000);

  //2000 mintavételes kalibráció
  for (Kalibracios_int = 0; Kalibracios_int < Kalibracio_mintavetel_szam; Kalibracios_int++) {
    gyro_acc_adat_olvasas();
    Rate_Roll_Kalibracios += Rate_Roll;
    Rate_Pitch_Kalibracios += Rate_Pitch;
    Rate_Yaw_Kalibracios += Rate_Yaw;
    delay(1);
  }

  //Mintavételi számmal való vissza osztás
  Rate_Roll_Kalibracios /= (float)Kalibracio_mintavetel_szam;
  Rate_Pitch_Kalibracios /= (float)Kalibracio_mintavetel_szam;
  Rate_Yaw_Kalibracios /= (float)Kalibracio_mintavetel_szam;

  Kalman_1D_szog_nullazas();

  digitalWrite(Piros_LED,LOW);
  digitalWrite(Zold_LED,HIGH);

  Loop_idozito = micros();
}

void loop() {
  // put your main code here, to run repeatedly:

  gyro_acc_adat_olvasas();  //Gyro és Acc adat olvasó fgv meghívása

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Kalibracios;
  Rate_Pitch -= Rate_Pitch_Kalibracios;
  Rate_Yaw -= Rate_Yaw_Kalibracios;

  /*Rate_Roll_2 = Rate_Roll;
  Rate_Pitch_2 = Rate_Pitch;
  Rate_Yaw_2 = Rate_Yaw;*/

  //exponenciális low-pass / mozgóátlag jellegű szűrő RATE MODE PID-hez hiba számoláshoz
  /*Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll_2 * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch_2 * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw_2 * 0.3;*/

  if (!Komplementer_Elindult) {
    Szurt_Szog_Roll = 0;
    Szurt_Szog_Pitch = 0;

    Komplementer_Elindult = true;
  }

  // Gyro integrálás atan()
  Szurt_Szog_Roll += Rate_Roll * Ts;
  Szurt_Szog_Pitch += Rate_Pitch * Ts;

  // Yaw miatti roll/pitch tengelykorrekció
  float yaw_rad = Rate_Yaw * Ts * (PI / 180.0);

  float Roll_Elozo = Szurt_Szog_Roll;
  float Pitch_Elozo = Szurt_Szog_Pitch;

  Szurt_Szog_Pitch = Pitch_Elozo - Roll_Elozo * sin(yaw_rad);
  Szurt_Szog_Roll = Roll_Elozo + Pitch_Elozo * sin(yaw_rad);

  // Accelerometeres lassú korrekció komplementer szűrővel
  Szurt_Szog_Roll = Szurt_Szog_Roll * 0.9996 + Szog_Roll * 0.0004;
  Szurt_Szog_Pitch = Szurt_Szog_Pitch * 0.9996 + Szog_Pitch * 0.0004;

  kalman_1d(Kalman_Szog_Roll, Kalman_Bizonytalansag_Szog_Roll, Rate_Roll, Szog_Roll);
  Kalman_Szog_Roll = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Szog_Roll = Kalman_1D_kimenet[1];

  kalman_1d(Kalman_Szog_Pitch, Kalman_Bizonytalansag_Szog_Pitch, Rate_Pitch, Szog_Pitch);
  Kalman_Szog_Pitch = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Szog_Pitch = Kalman_1D_kimenet[1];

  Kalman_Szog_Roll_Korrigalt = Kalman_Szog_Roll - Kalman_Roll_Offset;
  Kalman_Szog_Pitch_Korrigalt = Kalman_Szog_Pitch - Kalman_Pitch_Offset;

  if (millis() - Serial_Idozito >= 50) {
    Serial_Idozito = millis();

    Serial.print("AccAtanRoll:");
    Serial.print(Szog_Roll, 2);
    Serial.print("\tAccAtanPitch:");
    Serial.print(Szog_Pitch, 2);

    Serial.print("\tCompAtanRoll:");
    Serial.print(Szurt_Szog_Roll, 2);
    Serial.print("\tCompAtanPitch:");
    Serial.print(Szurt_Szog_Pitch, 2);

    Serial.print("\tKalmanRoll:");
    Serial.print(Kalman_Szog_Roll_Korrigalt, 2);
    Serial.print("\tKalmanPitch:");
    Serial.print(Kalman_Szog_Pitch_Korrigalt, 2);

    Serial.print("\tDiffRoll:");
    Serial.print(Szurt_Szog_Roll - Kalman_Szog_Roll_Korrigalt, 2);
    Serial.print("\tDiffPitch:");
    Serial.print(Szurt_Szog_Pitch - Kalman_Szog_Pitch_Korrigalt, 2);

    Serial.print("\tRateRoll:");
    Serial.print(Rate_Roll, 2);
    Serial.print("\tRatePitch:");
    Serial.print(Rate_Pitch, 2);
    Serial.print("\tRateYaw:");
    Serial.println(Rate_Yaw, 2);
  }

  while ((micros() - Loop_idozito) < 4000);

  Loop_idozito = micros();
}
