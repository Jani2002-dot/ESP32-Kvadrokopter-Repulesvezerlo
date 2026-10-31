#include <Wire.h>  //I2C könyvtár

//számított szögsebességeknek változó
float Rate_Roll;
float Rate_Pitch;
float Rate_Yaw;
float Rate_Roll_Szurt;
float Rate_Pitch_Szurt;
float Rate_Yaw_Szurt;
//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Kalibracios = 0;
float Rate_Pitch_Kalibracios = 0;
float Rate_Yaw_Kalibracios = 0;
//for ciklus változói
int Kalibracios_int;
int Kalibracio_mintavetel_szam = 2000;

bool MPU_Hiba = false;

// 250 Hz-es loop vizsgálatához
uint32_t Loop_idozito;
uint32_t Loop_Start_Us;
uint32_t Loop_Start_Elozo_Us;
uint32_t Loop_Ido_Us;
uint32_t Debug_Ido;

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
  MPU_Hiba = false;

  Wire.beginTransmission(0x68);
  Wire.write(0x43);

  uint8_t I2C_Hiba = Wire.endTransmission(false);

  if (I2C_Hiba != 0) {
    MPU_Hiba = true;
    return;
  }

  uint8_t Beolvasott_Byteok = Wire.requestFrom((uint8_t)0x68, (uint8_t)6, (uint8_t) true);

  if (Beolvasott_Byteok != 6) {
    MPU_Hiba = true;
    return;
  }

  int16_t Gyro_X = Wire.read() << 8 | Wire.read();
  int16_t Gyro_Y = Wire.read() << 8 | Wire.read();
  int16_t Gyro_Z = Wire.read() << 8 | Wire.read();

  Rate_Roll = (float)Gyro_X / 65.5;
  Rate_Pitch = (float)Gyro_Y / 65.5;
  Rate_Yaw = (float)Gyro_Z / 65.5;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(921600);
  Wire.setClock(400000);  //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  Wire.begin();           //SDA, SCL
  delay(250);
  mpu_6050_regiszterek_beallitasa();


  //2000 mintavételes kalibráció
  for (Kalibracios_int = 0; Kalibracios_int < Kalibracio_mintavetel_szam; Kalibracios_int++) {
    gyro_adat_olvasas();
    Rate_Roll_Kalibracios += Rate_Roll;
    Rate_Pitch_Kalibracios += Rate_Pitch;
    Rate_Yaw_Kalibracios += Rate_Yaw;
    delay(1);
  }
  //Mintavételi számmal való vissza osztás
  Rate_Roll_Kalibracios /= (float)2000;
  Rate_Pitch_Kalibracios /= (float)2000;
  Rate_Yaw_Kalibracios /= (float)2000;
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

  Loop_Start_Us = micros();
  Loop_idozito = Loop_Start_Us - Loop_Start_Elozo_Us;
  Loop_Start_Elozo_Us = Loop_Start_Us;

  gyro_adat_olvasas();  //Gyro adat olvasó fgv meghívása

  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Kalibracios;
  Rate_Pitch -= Rate_Pitch_Kalibracios;
  Rate_Yaw -= Rate_Yaw_Kalibracios;

  //exponenciális low-pass / mozgóátlag jellegű szűrő RATE MODE PID-hez hiba számoláshoz
  Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw * 0.3;

  if (millis() - Debug_Ido >= 1000) {
    Debug_Ido = millis();

    Serial.print("RawRoll: ");
    Serial.print(Rate_Roll, 2);

    Serial.print(" RawPitch: ");
    Serial.print(Rate_Pitch, 2);

    Serial.print(" RawYaw: ");
    Serial.print(Rate_Yaw, 2);

    Serial.print(" Hiba: ");
    Serial.print(MPU_Hiba);

    Serial.print(" Loop: ");
    Serial.println(Loop_idozito);
  }

  Loop_Ido_Us = micros() - Loop_Start_Us;

  if (Loop_Ido_Us < 4000) {
    delayMicroseconds(4000 - Loop_Ido_Us);
  }
}