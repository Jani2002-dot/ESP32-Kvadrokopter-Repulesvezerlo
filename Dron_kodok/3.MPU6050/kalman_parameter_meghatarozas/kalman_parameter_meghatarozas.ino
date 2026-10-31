#include <Wire.h>  //I2C könyvtár

//számított szögsebességeknek változó
float Rate_Roll;
float Rate_Pitch;
float Rate_Yaw;

float Rate_Roll_cal;
float Rate_Pitch_cal;
float Rate_Yaw_cal;

float Angle_Gyro_Roll, Angle_Gyro_Pitch;

//kiszámmolt Acc értékek
float Acc_X;
float Acc_Y;
float Acc_Z;

//számított szög változók
float Szog_Acc_Roll;
float Szog_Acc_Pitch;

uint32_t Loop_Timer;
uint32_t Print_meres;
int Print_CTU = 0;

//Kalman
float Kalman_Angle_Roll = 0;                     //kezdeti szögbecslést 0°-ra állítottam, mivel az indítás közel vízszintes helyzetben történik
float Kalman_Bizonytalansag_Angle_Roll = 2 * 2;  //bizonytalanságát 2° kezdeti szórással, azaz 4°^2 hibavarianciával vettem figyelembe

float Kalman_Angle_Pitch = 0;                     //kezdeti szögbecslést 0°-ra állítottam, mivel az indítás közel vízszintes helyzetben történik
float Kalman_Bizonytalansag_Angle_Pitch = 2 * 2;  //bizonytalanságát 2° kezdeti szórással, azaz 4°^2 hibavarianciával vettem figyelembe

float Kalman_Szog_Kimenet[] = { 0, 0 };
float Ts = 0.004;  //250 Hz-es loop miatt
const int A = 1, H = 1, I = 1;
float B = Ts;
float Q = Ts * Ts * 4 * 4;
float R = 3 * 3;

void Kalman_Szog(float x, float P, float u, float z) {
  x = A * x + B * u;
  P = A * P * A + Q;
  float K = P * H / (H * P * H + R);
  x = x + K * (z - H * x);
  P = (I - K * H) * P;
  Kalman_Szog_Kimenet[0] = x;  //Korrigált szög PID-nek és következő iteráccióba
  Kalman_Szog_Kimenet[1] = P;  //Korrigált hibakovariancia következő iterációba
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
  Acc_X = (float)Acc_XLSB / 4096.0f - 0.0454f;
  Acc_Y = (float)Acc_YLSB / 4096.0f + 0.0279f;
  Acc_Z = (float)Acc_ZLSB / 4096.0f - 0.10150f;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Szog_Acc_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0 / PI);
  Szog_Acc_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0 / PI);
}


void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();           //SDA, SCL
  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén

  delay(250);
  mpu_6050_regiszterek_beallitasa();
  delay(5000);

  for (int Kalibracios_int = 0; Kalibracios_int < 2500; Kalibracios_int++) {
    MPU_olvasas();
    Rate_Roll_cal += Rate_Roll;
    Rate_Pitch_cal += Rate_Pitch;
    Rate_Yaw_cal += Rate_Yaw;
    delay(1);
  }
  //Mintavételi számmal való vissza osztás
  Rate_Roll_cal /= (float)2500.0;
  Rate_Pitch_cal /= (float)2500.0;
  Rate_Yaw_cal /= (float)2500.0;

  digitalWrite(LED_BUILTIN, HIGH);

  //Serial.println("ido_us;Angle_Gyro_Roll;Angle_Gyro_Pitch;Szog_Acc_Roll;Szog_Acc_Pitch;Kalman_Roll;Kalman_Pitch;CTU");
  //Serial.println("ido_us;Angle_Gyro_Roll;Szog_Acc_Roll;Kalman_Roll;CTU");
  Serial.println("ido_us;Angle_Gyro_Pitch;Szog_Acc_Pitch;;Kalman_Pitch;CTU");

  Loop_Timer = micros();
}

void loop() {

  MPU_olvasas();

  Rate_Roll -= Rate_Roll_cal;
  Rate_Pitch -= Rate_Pitch_cal;
  Rate_Yaw -= Rate_Yaw_cal;

  // Gyro integrálás
  Angle_Gyro_Roll += Rate_Roll * Ts;
  Angle_Gyro_Pitch += Rate_Pitch * Ts;

  //Kalman Angle Roll számítás
  Kalman_Szog(Kalman_Angle_Roll, Kalman_Bizonytalansag_Angle_Roll, Rate_Roll, Szog_Acc_Roll);
  Kalman_Angle_Roll = Kalman_Szog_Kimenet[0];
  Kalman_Bizonytalansag_Angle_Roll = Kalman_Szog_Kimenet[1];

  //Kalman Angle Pitch számítás
  Kalman_Szog(Kalman_Angle_Pitch, Kalman_Bizonytalansag_Angle_Pitch, Rate_Pitch, Szog_Acc_Pitch);
  Kalman_Angle_Pitch = Kalman_Szog_Kimenet[0];
  Kalman_Bizonytalansag_Angle_Pitch = Kalman_Szog_Kimenet[1];

  uint32_t ido_us = micros();

  if ((millis() - Print_meres >= 8) && Print_CTU < 20000) {
    Print_meres = millis();
    Print_CTU++;

    Serial.print(ido_us);
    Serial.print(';');

    //Serial.print(Angle_Gyro_Roll, 5);
    //Serial.print(';');
    Serial.print(Angle_Gyro_Pitch, 5);
    Serial.print(';');

    //Serial.print(Szog_Acc_Roll, 5);
    //Serial.print(';');
    Serial.print(Szog_Acc_Pitch, 5);
    Serial.print(';');

    //Serial.print(Kalman_Angle_Roll, 5);
    //Serial.print(';');
    Serial.print(Kalman_Angle_Pitch, 5);
    +Serial.print(';');

    Serial.println(Print_CTU);
  }

  while (micros() - Loop_Timer < 4000) {
  }
  Loop_Timer = micros();
}
