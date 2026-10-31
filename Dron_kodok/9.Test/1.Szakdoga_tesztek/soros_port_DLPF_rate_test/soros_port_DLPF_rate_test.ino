#include <Wire.h> //I2C könyvtár

#define MPU_DLPF_CFG 3   // 5=10Hz, 4=20Hz, 3=42Hz, 2=98Hz

uint32_t Rate_Teszt_Ido = 0;
uint32_t Rate_Teszt_Minta = 0;

float Raw_Roll_SumSq = 0;
float Raw_Pitch_SumSq = 0;
float Raw_Yaw_SumSq = 0;

float Szurt_Roll_SumSq = 0;
float Szurt_Pitch_SumSq = 0;
float Szurt_Yaw_SumSq = 0;

float Raw_Roll_Peak = 0;
float Raw_Pitch_Peak = 0;
float Raw_Yaw_Peak = 0;

float Szurt_Roll_Peak = 0;
float Szurt_Pitch_Peak = 0;
float Szurt_Yaw_Peak = 0;

uint32_t Loop_Sum_Us = 0;
uint32_t Loop_Max_Us = 0;

uint32_t Loop_idozito; // 250 Hz-es loop vizsgálatához
uint32_t Loop_dt_us;

//számított szögsebességeknek változó
float Rate_Roll , Rate_Pitch , Rate_Yaw;
float Rate_Roll_2 , Rate_Pitch_2 , Rate_Yaw_2;
float Rate_Roll_Szurt , Rate_Pitch_Szurt , Rate_Yaw_Szurt;
float Ts = 0.004;
bool Gyro_Kalibracio_Kesz = false;

//kiszámmolt Acc értékek
float Acc_X , Acc_Y , Acc_Z;

//számított szög változók
float Szog_Roll;
float Szog_Pitch;
float acc_total_vector;
float angle_pitch_acc, angle_roll_acc;

//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Kalibracios=0;
float Rate_Pitch_Kalibracios=0;
float Rate_Yaw_Kalibracios=0;

//for ciklus változói
int Kalibracios_int;
int Kalibracio_mintavetel_szam=2000;

const char* DLPF_Nev() {
  if (MPU_DLPF_CFG == 5) return "10Hz";
  if (MPU_DLPF_CFG == 4) return "20Hz";
  if (MPU_DLPF_CFG == 3) return "42Hz";
  if (MPU_DLPF_CFG == 2) return "98Hz";
  if (MPU_DLPF_CFG == 1) return "188Hz";
  if (MPU_DLPF_CFG == 0) return "256Hz";
  return "Ismeretlen";
}

void Rate_Teszt_Reset() {
  Rate_Teszt_Minta = 0;

  Raw_Roll_SumSq = 0;
  Raw_Pitch_SumSq = 0;
  Raw_Yaw_SumSq = 0;

  Szurt_Roll_SumSq = 0;
  Szurt_Pitch_SumSq = 0;
  Szurt_Yaw_SumSq = 0;

  Raw_Roll_Peak = 0;
  Raw_Pitch_Peak = 0;
  Raw_Yaw_Peak = 0;

  Szurt_Roll_Peak = 0;
  Szurt_Pitch_Peak = 0;
  Szurt_Yaw_Peak = 0;

  Loop_Sum_Us = 0;
  Loop_Max_Us = 0;
}

void Rate_DLPF_Teszt_Kiiras() {
  Rate_Teszt_Minta++;

  Raw_Roll_SumSq += Rate_Roll_2 * Rate_Roll_2;
  Raw_Pitch_SumSq += Rate_Pitch_2 * Rate_Pitch_2;
  Raw_Yaw_SumSq += Rate_Yaw_2 * Rate_Yaw_2;

  Szurt_Roll_SumSq += Rate_Roll_Szurt * Rate_Roll_Szurt;
  Szurt_Pitch_SumSq += Rate_Pitch_Szurt * Rate_Pitch_Szurt;
  Szurt_Yaw_SumSq += Rate_Yaw_Szurt * Rate_Yaw_Szurt;

  if (abs(Rate_Roll_2) > Raw_Roll_Peak) Raw_Roll_Peak = abs(Rate_Roll_2);
  if (abs(Rate_Pitch_2) > Raw_Pitch_Peak) Raw_Pitch_Peak = abs(Rate_Pitch_2);
  if (abs(Rate_Yaw_2) > Raw_Yaw_Peak) Raw_Yaw_Peak = abs(Rate_Yaw_2);

  if (abs(Rate_Roll_Szurt) > Szurt_Roll_Peak) Szurt_Roll_Peak = abs(Rate_Roll_Szurt);
  if (abs(Rate_Pitch_Szurt) > Szurt_Pitch_Peak) Szurt_Pitch_Peak = abs(Rate_Pitch_Szurt);
  if (abs(Rate_Yaw_Szurt) > Szurt_Yaw_Peak) Szurt_Yaw_Peak = abs(Rate_Yaw_Szurt);

  Loop_Sum_Us += Loop_dt_us;
  if (Loop_dt_us > Loop_Max_Us) Loop_Max_Us = Loop_dt_us;

  if (millis() - Rate_Teszt_Ido >= 1000) {
    Rate_Teszt_Ido = millis();

    float Raw_Roll_RMS = sqrt(Raw_Roll_SumSq / Rate_Teszt_Minta);
    float Raw_Pitch_RMS = sqrt(Raw_Pitch_SumSq / Rate_Teszt_Minta);
    float Raw_Yaw_RMS = sqrt(Raw_Yaw_SumSq / Rate_Teszt_Minta);

    float Szurt_Roll_RMS = sqrt(Szurt_Roll_SumSq / Rate_Teszt_Minta);
    float Szurt_Pitch_RMS = sqrt(Szurt_Pitch_SumSq / Rate_Teszt_Minta);
    float Szurt_Yaw_RMS = sqrt(Szurt_Yaw_SumSq / Rate_Teszt_Minta);

    float Loop_Atlag_Us = (float)Loop_Sum_Us / Rate_Teszt_Minta;

    Serial.print("DLPF:");
    Serial.print(DLPF_Nev());

    Serial.print("\tMinta:");
    Serial.print(Rate_Teszt_Minta);

    Serial.print("\tLoopAtlag:");
    Serial.print(Loop_Atlag_Us, 1);

    Serial.print("\tLoopMax:");
    Serial.print(Loop_Max_Us);

    Serial.print("\tRawRollRMS:");
    Serial.print(Raw_Roll_RMS, 2);

    Serial.print("\tRawPitchRMS:");
    Serial.print(Raw_Pitch_RMS, 2);

    Serial.print("\tRawYawRMS:");
    Serial.print(Raw_Yaw_RMS, 2);

    Serial.print("\tSzurtRollRMS:");
    Serial.print(Szurt_Roll_RMS, 2);

    Serial.print("\tSzurtPitchRMS:");
    Serial.print(Szurt_Pitch_RMS, 2);

    Serial.print("\tSzurtYawRMS:");
    Serial.print(Szurt_Yaw_RMS, 2);

    Serial.print("\tRawRollPeak:");
    Serial.print(Raw_Roll_Peak, 2);

    Serial.print("\tRawPitchPeak:");
    Serial.print(Raw_Pitch_Peak, 2);

    Serial.print("\tRawYawPeak:");
    Serial.print(Raw_Yaw_Peak, 2);

    Serial.print("\tSzurtRollPeak:");
    Serial.print(Szurt_Roll_Peak, 2);

    Serial.print("\tSzurtPitchPeak:");
    Serial.print(Szurt_Pitch_Peak, 2);

    Serial.print("\tSzurtYawPeak:");
    Serial.println(Szurt_Yaw_Peak, 2);

    Rate_Teszt_Reset();
  }
}

void mpu_6050_regiszterek_beallitasa(void){

  //MPU aktiválása (felébesztése)
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x6B); //kérés küldése a regiszer indítására 
  Wire.write(0x00); //MPU felébesztésének beleírása HEX-be 
  Wire.endTransmission(); //küldés 

  //42Hz aluláteresztő beállítása
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x1A); //kérés küldése a regiszer indítására 
  Wire.write(0x03); //20Hz beleírása HEX-be 
  Wire.endTransmission(); //küldés 

  // Girszkóp +-500 °/s beállítása 65.5 LSB/°/s
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x1B); //kérés küldése a regiszer indítására 
  Wire.write(0x08); //+-500 °/s beleírása HEX-be 
  Wire.endTransmission(); //küldés 

  //Gyorsulásmérő +-8g beállítása 4096 LSB/g
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x1C); //kérés küldése a regiszer indítására 
  Wire.write(0x10); //MPU felébesztésének beleírása HEX-be 
  Wire.endTransmission(); //küldés 
}

void gyro_acc_adat_olvasas(void){

  //Girszkóp adatainak kiolvasása
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x3B); //Acc adatok itt keződnek
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14); // 14 bytot kérünk

  while (Wire.available() < 14) {} // vár a 14 byte-ra

  //MPU 16 bites értékeit bele bitshiftelni egy 16 bites változóba mivel I2C rigsztereken csak 8 bites byte-okban kommunikál 
  // << bitshift 8 bittel feltoljuk a 16 bites változó 15-8 bitjébe
  // | (bitwise OR) összekapcsolja a byte.okat
  int16_t Acc_XLSB = Wire.read()<<8 | Wire.read();
  int16_t Acc_YLSB = Wire.read()<<8 | Wire.read();
  int16_t Acc_ZLSB = Wire.read()<<8 | Wire.read();
  int16_t Temp = Wire.read()<<8 | Wire.read();
  int16_t Gyro_X = Wire.read()<<8 | Wire.read();
  int16_t Gyro_Y = Wire.read()<<8 | Wire.read();
  int16_t Gyro_Z = Wire.read()<<8 | Wire.read();


  // °/s szögsebességek kiszámolás mivel +-500°/s LSB 65.5/°/s 
  // MPU 16 bites adat olvasás 2^16, +-500 = 1000 ------> 2^16/1000=65.536 ~ 65.5
  Rate_Roll=(float)Gyro_X/65.5;
  Rate_Pitch=(float)Gyro_Y/65.5;
  Rate_Yaw=(float)Gyro_Z/65.5;

  if (Gyro_Kalibracio_Kesz) {
    //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
    Rate_Roll -= Rate_Roll_Kalibracios;
    Rate_Pitch -= Rate_Pitch_Kalibracios;
    Rate_Yaw -= Rate_Yaw_Kalibracios;

    Rate_Roll_2 = Rate_Roll;
    Rate_Pitch_2 = Rate_Pitch;
    Rate_Yaw_2 = Rate_Yaw;
  }

  //Alakítsd át a mért értékeket fizikai (valós) egységekbe.
  //a gyorsulásmérő nyers LSB értékeiből csinálsz g-t vagy m/s²-t (ha m/s^2 akkor * 9.81)
  Acc_X = (float)Acc_XLSB / 4096 - 0.0404;
  Acc_Y = (float)Acc_YLSB / 4096 + 0.0266;
  Acc_Z = (float)Acc_ZLSB / 4096 - 0.11188;
  
  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Szog_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0/PI);
  Szog_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0/PI);

  //Accelerometer angle calculations
  acc_total_vector = sqrt((Acc_XLSB*Acc_XLSB)+(Acc_YLSB*Acc_YLSB)+(Acc_ZLSB*Acc_ZLSB));       //Calculate the total accelerometer vector.
  
  if (abs(Acc_YLSB) < acc_total_vector) {
    angle_roll_acc = asin((float)Acc_YLSB / acc_total_vector) * (180.0 / PI);
  }

  if (abs(Acc_XLSB) < acc_total_vector) {
    angle_pitch_acc = asin((float)-Acc_XLSB / acc_total_vector) * (180.0 / PI);
  }

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_roll_acc -= -0.95; 
  angle_pitch_acc -= -1.25;                                                   //Accelerometer calibration value for pitch.  

}

void varakozas_250Hz(void) {
  while ((uint32_t)(micros() - Loop_idozito) < 4000) {
    //delayMicroseconds(250); 
  }

  Loop_dt_us = micros() - Loop_idozito;
  Loop_idozito = micros();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.setClock(400000); //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  Wire.begin(); //SDA, SCL
  delay(250);
  mpu_6050_regiszterek_beallitasa();

  digitalWrite(LED_BUILTIN, LOW);

  //2000 mintavételes kalibráció 
  for (Kalibracios_int=0 ; Kalibracios_int < Kalibracio_mintavetel_szam ; Kalibracios_int++){
    gyro_acc_adat_olvasas();
    Rate_Roll_Kalibracios += Rate_Roll;
    Rate_Pitch_Kalibracios += Rate_Pitch;
    Rate_Yaw_Kalibracios += Rate_Yaw;
    delay(1);
  }

  //Mintavételi számmal való vissza osztás
  Rate_Roll_Kalibracios /= (float)Kalibracio_mintavetel_szam ;
  Rate_Pitch_Kalibracios /= (float)Kalibracio_mintavetel_szam ;
  Rate_Yaw_Kalibracios /= (float)Kalibracio_mintavetel_szam ;

  Gyro_Kalibracio_Kesz = true;

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("DLPF rate teszt indul");
  Serial.println("DLPF\tMinta\tLoopAtlag\tLoopMax\tRawRollRMS\tRawPitchRMS\tRawYawRMS\tSzurtRollRMS\tSzurtPitchRMS\tSzurtYawRMS\tRawRollPeak\tRawPitchPeak\tRawYawPeak\tSzurtRollPeak\tSzurtPitchPeak\tSzurtYawPeak");

  Rate_Teszt_Ido = millis();
  Rate_Teszt_Reset();

  Loop_idozito = micros();

}

void loop() {
  // put your main code here, to run repeatedly:

  gyro_acc_adat_olvasas(); //Gyro és Acc adat olvasó fgv meghívása

  //exponenciális low-pass / mozgóátlag jellegű szűrő
  Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll_2 * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch_2 * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw_2 * 0.3;

  Rate_DLPF_Teszt_Kiiras();
  varakozas_250Hz();
}
