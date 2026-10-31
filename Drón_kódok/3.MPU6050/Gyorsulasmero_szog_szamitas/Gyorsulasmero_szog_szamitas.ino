#include <Wire.h>  //I2C könyvtár

//számított szögsebességeknek változó
float Rate_Roll;
float Rate_Pitch;
float Rate_Yaw;

//kiszámmolt Acc értékek
float Acc_X;
float Acc_Y;
float Acc_Z;

float Acc_X_korrigalt;
float Acc_Y_korrigalt;
float Acc_Z_korrigalt;

float Acc_X_Kalibracios, Acc_Y_Kalibracios, Acc_Z_Kalibracios;

//számított szög változók
float Szog_Roll;
float Szog_Pitch;

uint32_t Loop_Timer;
uint32_t Print_meres;
int Print_CTU = 0;

void mpu_6050_regiszterek_beallitasa(void) {

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
  Acc_X = (float)Acc_XLSB / 4096.0f - 0.0454f;
  Acc_Y = (float)Acc_YLSB / 4096.0f + 0.0279f;
  Acc_Z = (float)Acc_ZLSB / 4096.0f - 0.10150f;

  // Ugyanazok a minták, korrekcióval:
  //Acc_X_korrigalt = Acc_X - 0.0454f;
  //Acc_Y_korrigalt = Acc_Y + 0.0279f;
  //Acc_Z_korrigalt = Acc_Z - 0.10150f;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359

  //Szog_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  //Szog_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);

  float a_norm = sqrt(Acc_X * Acc_X + Acc_Y * Acc_Y + Acc_Z * Acc_Z);

  Szog_Roll = asin(Acc_Y / a_norm) * (180.0 / PI);
  Szog_Pitch = asin(-Acc_X / a_norm) * (180.0 / PI);

  //Szog_Roll = atan2f(Acc_Y, Acc_Z) * (180.0f / PI);

  //Szog_Pitch = atan2f(-Acc_X, sqrtf(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0f / PI);
}


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  Wire.begin();           //SDA, SCL
  delay(250);
  mpu_6050_regiszterek_beallitasa();
  //delay(30000);

  /*/2000 mintavételes kalibráció 
  for (int Kalibracios_int=0 ; Kalibracios_int < 5000 ; Kalibracios_int++){
    gyro_acc_adat_olvasas();
    Acc_X_Kalibracios += Acc_X;
    Acc_Y_Kalibracios += Acc_Y;
    Acc_Z_Kalibracios += Acc_Z;
    delay(1);
  }
  //Mintavételi számmal való vissza osztás
  Acc_X_Kalibracios /= (float)5000.0;
  Acc_Y_Kalibracios /= (float)5000.0;
  Acc_Z_Kalibracios /= (float)5000.0;*/

  digitalWrite(LED_BUILTIN, HIGH);
  //Serial.println("ido_us;AccX_g;AccY_g;AccZ_g;AccX_korrigalt_g;AccY_korrigalt_g;AccZ_korrigalt_g;CTU");
  Loop_Timer = micros();
}

void loop() {
  gyro_acc_adat_olvasas();
  uint32_t ido_us = micros();

  if ((millis() - Print_meres >= 200) && Print_CTU < 3000) {
    Print_meres = millis();
    Print_CTU++;

    Serial.print("Roll: ");
    //Serial.print(';');
    Serial.print(Szog_Roll, 3);
    //Serial.print(';');
    Serial.print("\tPitch: ");
    //Serial.print(';');
    Serial.println(Szog_Pitch, 3);
    /*Serial.print(';');
    Serial.print(Acc_X_korrigalt, 5);
    Serial.print(';');
    Serial.print(Acc_Y_korrigalt, 5);
    Serial.print(';');
    Serial.print(Acc_Z_korrigalt, 5);
    Serial.print(';');
    Serial.println(Print_CTU);*/
  }

  while (micros() - Loop_Timer < 4000) {
  }
  Loop_Timer = micros();
}
