#include <BluetoothSerial.h>
#include <Wire.h> //I2C könyvtár
#include <IBusBM.h>

IBusBM ibus;
BluetoothSerial SerialBT;

//ibus.readChannel(0-9) 10 csatorna
#define RX_PIN 16
#define TX_PIN 25

#define Motorok_ESC_PWM_felbontas  12     //12 bit ---> 0-4095

#define Motorok_ESC_PWM_frekvencia 250    //250Hz=4000us

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

//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Kalibracios=0;
float Rate_Pitch_Kalibracios=0;
float Rate_Yaw_Kalibracios=0;

//for ciklus változói
int Kalibracios_int;
int Kalibracio_mintavetel_szam=2000;

float Rate_Roll, Rate_Pitch, Rate_Yaw;
float Acc_X, Acc_Y, Acc_Z;
float Szog_Roll, Szog_Pitch;

String BT_Buffer = "";
float Acc_Total_G = 0;
float Rezges_mg = 0;

float Rezges_sum = 0;
float Rezges_sum_sq = 0;
uint16_t Rezges_minta = 0;

int Teszt_Motor = 0; // 0=STOP, 1=M1, 2=M2, 3=M3, 4=M4, 5=ALL

int Motor_max_rezges = 1300;

uint32_t BT_kiiras_ido = 0;


uint16_t Vevo_Ertekek[]={0,0,0,0,0};
bool SWA_Allapot;
float Bemenet_Gaz;

uint32_t Loop_idozito;

float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;

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

void mpu_6050_regiszterek_beallitasa(void){

  //MPU aktiválása (felébesztése)
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x6B); //kérés küldése a regiszer indítására 
  Wire.write(0x00); //MPU felébesztésének beleírása HEX-be 
  Wire.endTransmission(); //küldés 

  // 20 Hz aluláteresztő beállítása
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x1A); //kérés küldése a regiszer indítására 
  Wire.write(0x04); //20 Hz beleírása HEX-be 
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

  //MPU adatainak kiolvasása
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

  //Alakítsd át a mért értékeket fizikai (valós) egységekbe.
  //a gyorsulásmérő nyers LSB értékeiből csinálsz g-t vagy m/s²-t (ha m/s^2 akkor * 9.81)
  Acc_X = (float)Acc_XLSB / 4096 - 0.0175;
  Acc_Y = (float)Acc_YLSB / 4096 + 0.023; 
  Acc_Z = (float)Acc_ZLSB / 4096 - 0.108;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Szog_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0/PI);
  Szog_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0/PI);
}

uint32_t usToDuty(float motor_bemenet_us) {
  return (uint32_t)(motor_bemenet_us * 1.02375);
}

void varakozas_250Hz(void) {
  while ((uint32_t)(micros() - Loop_idozito) < 4000) {
    delay(1); 
  }

  Loop_idozito = micros();
}

void Rezges_Reset() {
  Rezges_sum = 0;
  Rezges_sum_sq = 0;
  Rezges_minta = 0;
  Rezges_mg = 0;
}

void Rezges_szamitas() {
  Acc_Total_G = sqrt(Acc_X * Acc_X + Acc_Y * Acc_Y + Acc_Z * Acc_Z);

  Rezges_sum += Acc_Total_G;
  Rezges_sum_sq += Acc_Total_G * Acc_Total_G;
  Rezges_minta++;

  if (Rezges_minta >= 100) {
    float atlag = Rezges_sum / Rezges_minta;
    float atlag_negyzet = Rezges_sum_sq / Rezges_minta;

    float variancia = atlag_negyzet - atlag * atlag;
    if (variancia < 0) variancia = 0;

    Rezges_mg = sqrt(variancia) * 1000.0;

    Rezges_sum = 0;
    Rezges_sum_sq = 0;
    Rezges_minta = 0;
  }
}

void Motorok_Stop() {
  Motor_1_bemenet = 1000;
  Motor_2_bemenet = 1000;
  Motor_3_bemenet = 1000;
  Motor_4_bemenet = 1000;

  ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));
  ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));
  ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));
  ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));
}

void BT_parancs_feldolgozas(String p) {
  p.trim();
  p.toUpperCase();

  if (p == "M1") {
    Teszt_Motor = 1;
    Rezges_Reset();
    SerialBT.println("M1");
  }
  else if (p == "M2") {
    Teszt_Motor = 2;
    Rezges_Reset();
    SerialBT.println("M2");
  }
  else if (p == "M3") {
    Teszt_Motor = 3;
    Rezges_Reset();
    SerialBT.println("M3");
  }
  else if (p == "M4") {
    Teszt_Motor = 4;
    Rezges_Reset();
    SerialBT.println("M4");
  }
  else if (p == "ALL") {
    Teszt_Motor = 5;
    Rezges_Reset();
    SerialBT.println("ALL");
  }
  else if (p == "STOP") {
    Teszt_Motor = 0;
    Rezges_Reset();
    Motorok_Stop();
    SerialBT.println("STOP");
  }
  else {
    SerialBT.println("M1 M2 M3 M4 ALL STOP");
  }
}

void BT_kezelese() {
  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      if (BT_Buffer.length() > 0) {
        BT_parancs_feldolgozas(BT_Buffer);
        BT_Buffer = "";
      }
    } 
    else {
      BT_Buffer += c;

      if (BT_Buffer.length() > 20) {
        BT_Buffer = "";
      }
    }
  }
}

void Motor_Teszt_Frissites() {
  Bemenet_Gaz = Vevo_Ertekek[2];

  if (Bemenet_Gaz > Motor_max_rezges) {
    Bemenet_Gaz = Motor_max_rezges;
  }

  // ha gáz lent, vagy nincs kiválasztva motor, vagy SWA=1, akkor minden áll
  if (Bemenet_Gaz < 1030 || Teszt_Motor == 0 || SWA_Allapot == 1) {
    Motorok_Stop();
    return;
  }

  Motor_1_bemenet = 1000;
  Motor_2_bemenet = 1000;
  Motor_3_bemenet = 1000;
  Motor_4_bemenet = 1000;

  if (Teszt_Motor == 1 || Teszt_Motor == 5) Motor_1_bemenet = Bemenet_Gaz;
  if (Teszt_Motor == 2 || Teszt_Motor == 5) Motor_2_bemenet = Bemenet_Gaz;
  if (Teszt_Motor == 3 || Teszt_Motor == 5) Motor_3_bemenet = Bemenet_Gaz;
  if (Teszt_Motor == 4 || Teszt_Motor == 5) Motor_4_bemenet = Bemenet_Gaz;

  ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));
  ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));
  ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));
  ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));
}


void Rezges_BT_kiiras() {
  if (millis() - BT_kiiras_ido >= 500) {
    BT_kiiras_ido = millis();

    SerialBT.print("Rezges=");
    SerialBT.print(Rezges_mg, 1);
    SerialBT.println(" mg");

    SerialBT.print("Roll=");
    SerialBT.print(Rate_Roll, 1);
    SerialBT.print(" Pitch=");
    SerialBT.print(Rate_Pitch, 1);
    SerialBT.print(" Yaw=");
    SerialBT.println(Rate_Yaw, 1);    

    SerialBT.print("Gaz=");
    SerialBT.println(Bemenet_Gaz);    
  }
}

void setup() {

  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); //soros vevőnek
  ibus.begin(Serial2); //ibus inicializálása

  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(Piros_LED,OUTPUT);
  pinMode(Zold_LED,OUTPUT);

  digitalWrite(LED_BUILTIN,LOW);
  digitalWrite(Piros_LED,HIGH);
  digitalWrite(Zold_LED,LOW);

  //ledcAttachChannel(Motor_1_ESC_Pin, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas, Motor_1_ESC_Csatorna); //ESP32 core 3.3.6
  ledcSetup(Motor_1_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas); //ESP32 core 2.0.17
  ledcAttachPin(Motor_1_ESC_Pin, Motor_1_ESC_Csatorna); //ESP32 core 2.0.17

  //ledcAttachChannel(Motor_2_ESC_Pin, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas, Motor_2_ESC_Csatorna); //ESP32 core 3.3.6
  ledcSetup(Motor_2_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas); //ESP32 core 2.0.17
  ledcAttachPin(Motor_2_ESC_Pin, Motor_2_ESC_Csatorna); //ESP32 core 2.0.17

  //ledcAttachChannel(Motor_3_ESC_Pin, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas, Motor_3_ESC_Csatorna); //ESP32 core 3.3.6
  ledcSetup(Motor_3_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas); //ESP32 core 2.0.17
  ledcAttachPin(Motor_3_ESC_Pin, Motor_3_ESC_Csatorna); //ESP32 core 2.0.17

  //ledcAttachChannel(Motor_4_ESC_Pin, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas, Motor_4_ESC_Csatorna); //ESP32 core 3.3.6
  ledcSetup(Motor_4_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas); //ESP32 core 2.0.17
  ledcAttachPin(Motor_4_ESC_Pin, Motor_4_ESC_Csatorna); //ESP32 core 2.0.17

  Motorok_Stop();

  delay(3000);  

  SerialBT.begin("ESP32_REZGES_TESZT");
  delay(500);

  Wire.setClock(400000); //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  Wire.begin(); //SDA, SCL
  delay(250);
  mpu_6050_regiszterek_beallitasa();

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

  digitalWrite(Zold_LED,HIGH);
  digitalWrite(Piros_LED,LOW);
  digitalWrite(LED_BUILTIN,HIGH);
  Loop_idozito = micros();
}

void loop() {
  Csatorna_olvasas();

  // csak akkor lehet BT-n motort választani, ha SWA=1 és gáz lent
  if (SWA_Allapot == 1 && Vevo_Ertekek[2] < 1030) {
    BT_kezelese();
  }

  gyro_acc_adat_olvasas();

  Rate_Roll -= Rate_Roll_Kalibracios;
  Rate_Pitch -= Rate_Pitch_Kalibracios;
  Rate_Yaw -= Rate_Yaw_Kalibracios;

  Rezges_szamitas();

  Motor_Teszt_Frissites();

  Rezges_BT_kiiras();

  varakozas_250Hz();
}