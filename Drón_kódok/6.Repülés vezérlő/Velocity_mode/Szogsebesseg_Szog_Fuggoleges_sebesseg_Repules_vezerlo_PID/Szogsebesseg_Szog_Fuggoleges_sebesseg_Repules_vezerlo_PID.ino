#include <Wire.h> //I2C könyvtár

//nyers Gyro értékeknek változó
int16_t Gyro_X;
int16_t Gyro_Y;
int16_t Gyro_Z;
//számított szögsebességeknek változó
float Rate_Roll;
float Rate_Pitch;
float Rate_Yaw;
//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Kalibracios=0;
float Rate_Pitch_Kalibracios=0;
float Rate_Yaw_Kalibracios=0;
//for ciklus változói
int Kalibracios_int;
int Kalibracio_mintavetel_szam=2000;

#include <IBusBM.h>
IBusBM ibus;

//ibus.readChannel(0-9) 10 csatorna
#define RX_PIN 16
#define TX_PIN -1

#define Motorok_ESC_PWM_felbontas  12  //12    //12 bit ---> 0-4095
//#define Motorok_ESC_PWM_felbontas  16  //16    //16 bit ---> 0-65 535
#define Motorok_ESC_PWM_frekvencia 250 //250  //250Hz=4000us
//#define Motorok_ESC_PWM_frekvencia 50 //50   //50Hz=20 000us

#define Motor_1_ESC_Pin 23
#define Motor_1_ESC_Csatorna 0

#define Motor_2_ESC_Pin 19
#define Motor_2_ESC_Csatorna 1

#define Motor_3_ESC_Pin 18
#define Motor_3_ESC_Csatorna 2

#define Motor_4_ESC_Pin 17
#define Motor_4_ESC_Csatorna 3


//static azt jelenti, hogy ez a változó “fájl-szintű / globális” élettartamú, vagyis a program teljes futása alatt létezik, és ugyanaz az egy példánya van.
//csak ebben a .cpp fájlban látható (internal linkage). Tehát ha több fájlod van, másik .cpp-ből nem tudod elérni név szerint.
uint16_t Vevo_Ertekek[]={0,0,0,0,0,0,0,0};

#define feszultseg_meres_PIN 36
#define aram_meres_PIN 34
#define Piros_LED 4
#define Zold_LED 13
float Feszultseg;
float Aram;
float Aksi_maradek;               // %-ban 
float Aksi_kezdetben;             // mAh-ban
float Elfogyasztott_kapacitas=0;  // mAh-ban 
float Alap_aksi_kapacitas=2000;   // 2000mAh

uint32_t Loop_idozito; // 250 Hz-es loop vizsgálatához

float Kivant_Roll_ertek, Kivant_Pitch_ertek, Kivant_Yaw_ertek;
float Hiba_Roll_ertek, Hiba_Pitch_ertek, Hiba_Yaw_ertek;
float Elozo_Hiba_Roll_ertek, Elozo_Hiba_Pitch_ertek, Elozo_Hiba_Yaw_ertek;
float Elozo_Iterm_Roll, Elozo_Iterm_Pitch, Elozo_Iterm_Yaw;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;

float PIDReturn[] = {0,0,0};

//Rate PID
float P_ertek_Roll=0, P_ertek_Pitch=0, P_ertek_Yaw=0;
float I_ertek_Roll=0, I_ertek_Pitch=0, I_ertek_Yaw=0;
float D_ertek_Roll=0, D_ertek_Pitch=0, D_ertek_Yaw=0;

float Ts = 0.004; //250 Hz-es loop miatt 
int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;

//nyers Acc értékeknek változó
int16_t Acc_XLSB;
int16_t Acc_YLSB;
int16_t Acc_ZLSB;
//kiszámmolt Acc értékek
float Acc_X;
float Acc_Y;
float Acc_Z;
//számított szög változó
float Szog_Roll;
float Szog_Pitch;

float Kalman_Szog_Roll=0;
float Kalman_Bizonytalansag_Szog_Roll=2*2; //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_Szog_Pitch=0;
float Kalman_Bizonytalansag_Szog_Pitch=2*2; //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_1D_kimenet[] = {0,0};

int16_t Temp;

//fly sky távirányítóval beáállított szög -50°-50°
float Kivant_Szog_Roll;
float Kivant_Szog_Pitch;
//kívánt szög - kálmán által mért szög
float Hiba_Szog_Roll;
float Hiba_Szog_Pitch;

float Elozo_Hiba_Szog_Roll;
float Elozo_Hiba_Szog_Pitch;

float Elozo_Iterm_Szog_Roll;
float Elozo_Iterm_Szog_Pitch;

//Szög PID
float P_Szog_Roll=2, P_Szog_Pitch=2;
float I_Szog_Roll=0, I_Szog_Pitch=0;
float D_Szog_Roll=0, D_Szog_Pitch=0;

float Acc_Z_gyorsulas;

//16 bites kalibrációs adatok mentésére változók
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
//20 bites nyomás és hőmérséklet nyers adatok mentésére 6 bájton
uint32_t press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb;

//20 bites nyomás hőmérséklet mentése egy vátozóba bitshiftel
unsigned long int adc_P, adc_T;


float Baro_magassag;
float Baro_magassag_kezdetben = 0.0;

int Baro_kalibracios_szam;

#include <BasicLinearAlgebra.h> //vektorokhoz és mátrixokhoz könyvtár
using namespace BLA;
float Magassag_Kalman, Fuggoleges_sebesseg_Kalman;
//mátrixváltozók deklarálása <sor,oszlop>
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> U;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;

float Kivant_Fuggoleges_sebesseg;
float Hiba_Fuggoleges_sebesseg;
float Elozo_Hiba_Fuggoleges_sebesseg;
float Elozo_Iterm_Fuggoleges_sebesseg;

//Fuggoleges_sebesseg PID
float P_Fuggoleges_sebesseg=3.5;
float I_Fuggoleges_sebesseg=0.0015;
float D_Fuggoleges_sebesseg=0.01;

void kalman_1d(float Kalman_allapot, float Kalman_Bizonytalansag, float Kalman_bemenet, float Kalman_meres){
  Kalman_allapot = Kalman_allapot + Kalman_bemenet * 0.004;
  Kalman_Bizonytalansag = Kalman_Bizonytalansag + 0.004 * 0.004 * 4 * 4;
  float Kalman_erosites = Kalman_Bizonytalansag * 1/(1 * Kalman_Bizonytalansag + 0.004 * 0.004 * 3 * 3);
  Kalman_allapot = Kalman_allapot + Kalman_erosites * (Kalman_meres - Kalman_allapot);
  Kalman_Bizonytalansag = (1 - Kalman_erosites * 1) * Kalman_Bizonytalansag;
  Kalman_1D_kimenet[0] = Kalman_allapot;
  Kalman_1D_kimenet[1] = Kalman_Bizonytalansag;
}

void kalman_2d(void){
  U = {Acc_Z_gyorsulas};
  S=F*S+G*U;
  P=F*P*~F+Q; //~F az a F transzponált
  L=H*P*~H+R;
  K=P*~H*Inverse(L);
  M={Baro_magassag};
  S=S+K*(M-H*S);
  Magassag_Kalman=S(0,0);
  Fuggoleges_sebesseg_Kalman=S(1,0);
  P=(I-K*H)*P;
}

void baro_adat_olvasas(void){

  Wire.beginTransmission(0x77);
  Wire.write(0xF7); //itt kezdődik az adatok kiolvasása 
  Wire.endTransmission();
  Wire.requestFrom(0x77,6); //6 bájt kérése
  press_msb = Wire.read(); //0xF7
  press_lsb = Wire.read(); //0xF8
  press_xlsb = Wire.read(); //0xF9
  temp_msb = Wire.read();
  temp_lsb = Wire.read();
  temp_xlsb = Wire.read(); //0xFC

  //nyers, kompennzálatlan és kalibrálatlan nyomás
  adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  //nyers, kompennzálatlan és kalibrálatlan hőmérséklet
  adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

  // A hőmérsékletet °C-ban adja vissza, a felbontás 0,01 °C.
  // Az „5123” kimeneti érték 51,23 °C-nak felel meg.
  // A t_fine globális változóként a pontosított hőmérsékletértéket tárolja.
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1<<1)))* ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;

  // A nyomást Pa-ban adja vissza, előjel nélküli 32 bites egész számként.
  // A „96386” kimeneti érték 96386 Pa-nak felel meg = 963,86 hPa.
  unsigned long int p;
  var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1*((signed long int )dig_P5))<<1);
  var2 = (var2>>2)+(((signed long int )dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13))>>3)+((((signed long int )dig_P2) *var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int )dig_P1))>>15);
  if (var1 == 0) { p=0;} // Kerüld el a nullával való osztás miatt fellépő kivételt.
  p = (((unsigned long int )(((signed long int )1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
  else { p = (p / (unsigned long int )var1) * 2; }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int )(p>>2)) *((signed long int )dig_P8))>>13;
  p = (unsigned long int)((signed long int )p +((var1 + var2+ dig_P7) >> 4)); //eredmény Pa-ban 
  double nyomas=(double)p/100; //átváltjuk hPa-ra 

  Baro_magassag=44330*(1-pow(nyomas/1013.25, 1.0/5.255))*100; //magasság kiszámítása (*100 m és cm közötti áltváltás)
}

void baro_regiszterek_beallitasa(void){

  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x57); //01010111 binárisan
  Wire.endTransmission();

  Wire.beginTransmission(0x77);
  Wire.write(0xF5);
  Wire.write(0x14); //00010100 binárisan
  Wire.endTransmission();
}

void baro_kalibracio_olvasas(void){
  uint8_t adat[24], i=0;

  //kalibrációs adatok kiolvasása
  Wire.beginTransmission(0x77);
  Wire.write(0x88); //itt kezdődik a dig_T1
  Wire.endTransmission();
  Wire.requestFrom(0x77,24); //kérünk 24 byte-ot

  while (Wire.available()){
    adat[i] = Wire.read(); //24 byte mentése tömbe
    i++;
  }

  // LSB/MSB szerint jönnek az adatok Wire.read()-nél ezért kell így bitshiftelni 
  dig_T1 = (adat[1]<<8) | adat[0];
  dig_T2 = (adat[3]<<8) | adat[2];
  dig_T3 = (adat[5]<<8) | adat[4];
  dig_P1 = (adat[7]<<8) | adat[6];
  dig_P2 = (adat[9]<<8) | adat[8];
  dig_P3 = (adat[11]<<8) | adat[10];
  dig_P4 = (adat[13]<<8) | adat[12];
  dig_P5 = (adat[15]<<8) | adat[14];
  dig_P6 = (adat[17]<<8) | adat[16];
  dig_P7 = (adat[19]<<8) | adat[18];
  dig_P8 = (adat[21]<<8) | adat[20];
  dig_P9 = (adat[23]<<8) | adat[22];
  delay(250);

  //mintavételezem 2000x a magasságot és átlagolom 
  for (Baro_kalibracios_szam = 0; Baro_kalibracios_szam < 2000; Baro_kalibracios_szam++){
    baro_adat_olvasas();
    Baro_magassag_kezdetben += Baro_magassag;
    delay(1);
  }
  Baro_magassag_kezdetben /= 2000;
}

void aksi_feszultseg(void){
  Feszultseg=(float)analogRead(feszultseg_meres_PIN)/206.8182; //4095/(3.3*6)=206.8182
  Aram=(float)analogRead(aram_meres_PIN)*0.02686203; // ADC / ((4095/3.3V)*0.03 V/A) ---> ADC*0,02686203
}

void Csatorna_olvasas() {
    for (uint8_t i = 0; i < 8; i++) {
    Vevo_Ertekek[i] = ibus.readChannel(i);
  }
}

void mpu_6050_regiszterek_beallitasa(void){

  //MPU aktiválása (felébesztése)
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x6B); //kérés küldése a regiszer indítására 
  Wire.write(0x00); //MPU felébesztésének beleírása HEX-be 
  Wire.endTransmission(); //küldés 

  // 10 Hz aluláteresztő beállítása
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x1A); //kérés küldése a regiszer indítására 
  Wire.write(0x05); //10 Hz beleírása HEX-be 
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

  uint32_t t0 = micros();
  while (Wire.available() < 14) {  // vár a 14 byte-ra 
    if (micros() - t0 > 3000) {  // 3000 us = 3 ms timeout
    // I2C hiba / nincs adat
    Serial.println("MPU6050 timeout!");
    return;  // kilép a függvényből
  }
}


  //MPU 16 bites értékeit bele bitshiftelni egy 16 bites változóba mivel I2C rigsztereken csak 8 bites byte-okban kommunikál 
  // << bitshift 8 bittel feltoljuk a 16 bites változó 15-8 bitjébe
  // | (bitwise OR) összekapcsolja a byte.okat
  Acc_XLSB = Wire.read()<<8 | Wire.read();
  Acc_YLSB = Wire.read()<<8 | Wire.read();
  Acc_ZLSB = Wire.read()<<8 | Wire.read();
  Temp = Wire.read()<<8 | Wire.read();
  Gyro_X = Wire.read()<<8 | Wire.read();
  Gyro_Y = Wire.read()<<8 | Wire.read();
  Gyro_Z = Wire.read()<<8 | Wire.read();


  // °/s szögsebességek kiszámolás mivel +-500°/s LSB 65.5/°/s 
  // MPU 16 bites adat olvasás 2^16, +-500 = 1000 ------> 2^16/1000=65.536 ~ 65.5
  Rate_Roll=(float)Gyro_X/65.5;
  Rate_Pitch=(float)Gyro_Y/65.5;
  Rate_Yaw=(float)Gyro_Z/65.5;

  //Alakítsd át a mért értékeket fizikai (valós) egységekbe.
  //a gyorsulásmérő nyers LSB értékeiből csinálsz g-t vagy m/s²-t (ha m/s^2 akkor * 9.81)
  Acc_X = (float)Acc_XLSB / 4096 - 0.0495;
  Acc_Y = (float)Acc_YLSB / 4096 + 0.023; 
  Acc_Z = (float)Acc_ZLSB / 4096 - 0.093;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Szog_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0/PI);
  Szog_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0/PI);

  /*Szog_Roll  = atan2(Acc_Y,  sqrt(Acc_X*Acc_X + Acc_Z*Acc_Z)) * (180.0/PI);
  Szog_Pitch = atan2(-Acc_X, sqrt(Acc_Y*Acc_Y + Acc_Z*Acc_Z)) * (180.0/PI);*/
}

void PID_egyenlet (float Hiba, float P, float I, float D, float Elozo_Hiba, float Elozo_Iterm){
  float Pterm = P * Hiba;
  float Iterm = Elozo_Iterm + I * (Hiba + Elozo_Hiba) * Ts / 2;
  if (Iterm  > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Hiba - Elozo_Hiba) / Ts;
  float PID_Kimenet = Pterm + Iterm + Dterm;
  if (PID_Kimenet  > 400) PID_Kimenet = 400;
  else if (PID_Kimenet < -400) PID_Kimenet = -400;

  PIDReturn[0] = PID_Kimenet;
  PIDReturn[1] = Hiba; 
  PIDReturn[2] = Iterm;
}

void Reset_PID(void){
  Elozo_Hiba_Roll_ertek = 0;
  Elozo_Hiba_Pitch_ertek = 0;
  Elozo_Hiba_Yaw_ertek = 0;
  Elozo_Iterm_Roll = 0;
  Elozo_Iterm_Pitch = 0;
  Elozo_Iterm_Yaw = 0;

  Elozo_Hiba_Szog_Roll = 0;
  Elozo_Hiba_Szog_Pitch = 0;

  Elozo_Iterm_Szog_Roll = 0;
  Elozo_Iterm_Szog_Pitch = 0;

  Elozo_Hiba_Fuggoleges_sebesseg = 0;
  Elozo_Iterm_Fuggoleges_sebesseg = 0;
}

void setup() {

  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); //soros vevőnek
  ibus.begin(Serial2); //ibus inicializálása

  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(Piros_LED,OUTPUT);
  pinMode(Zold_LED,OUTPUT);
  pinMode(feszultseg_meres_PIN,INPUT);
  pinMode(aram_meres_PIN,INPUT);

  digitalWrite(LED_BUILTIN,HIGH);
  digitalWrite(Piros_LED,HIGH);
  digitalWrite(Zold_LED,LOW);

  Wire.begin(21, 22); //SDA, SCL
  Wire.setClock(400000); //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  delay(250);
  mpu_6050_regiszterek_beallitasa();
  baro_regiszterek_beallitasa();
  baro_kalibracio_olvasas();

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

  F = {1, 0.004,
       0, 1};
  G = {0.5*0.004*0.004,
        0.004};
  H = {1, 0};
  I = {1, 0,
        0, 1};
  Q = G * ~G * 10.0f * 10.0f;
  R = {30.0f * 30.0f};
  P = {0, 0,
        0, 0};
  S = {0,
      0};

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

// 250Hz=4ms=4000us
// 50Hz=20ms=20 000us
// 12 bites felbontás = 0-4095 
// 0 ---> 0us        4095--->4000us
//16 bites felbontás = 0-65 536
// 0 ---> 0us        65 535--->20 000us
// PWM jel küldése us-ban az ESC nek akkor 4095/4000=1.024-el kell majd szorozi 250Hz, 12 bit esetén 
// PWM jel küldése us-ban az ESC nek akkor 65 535/20 000=3.27675-el kell majd szorozi 50 Hz, 16 bit esetén

  digitalWrite(Zold_LED,HIGH);

  aksi_feszultseg();
  if (Feszultseg > 16.7) {         // terhelés nélkűl mérjük az aksit így valós értéket kapunk mivel terhelés alatt beesik a feszülség szintje és nem lesz valós érték ezért terhlés alatt áramot mmérünk
    digitalWrite(Piros_LED, LOW); //piros led kikapcs
    Aksi_kezdetben = Alap_aksi_kapacitas;//felöltve tehát 2000mAh 
  }
  else if (Feszultseg < 14.5){
    Aksi_kezdetben=30.0/100.0*Alap_aksi_kapacitas; //30% át vesszük
  }
  else {
    digitalWrite(Piros_LED, LOW);
    Aksi_kezdetben = (31.8182 * Feszultseg - 431.364)/100 * Alap_aksi_kapacitas; 
  }

// kontrollálatlan motorindítás elkerülése 
  while (Vevo_Ertekek[2] < 1020 || Vevo_Ertekek[2] > 1050){  //Vevo_Ertekek[2] ---> Gáz kar értéke  kontrollálatlan motorindítás elkerülése 
    Csatorna_olvasas();
    delay(4);
  }
  Loop_idozito = micros();

}

void loop() {

  gyro_acc_adat_olvasas(); //Gyro adat olvasó fgv meghívása
  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Kalibracios;
  Rate_Pitch -= Rate_Pitch_Kalibracios;
  Rate_Yaw -= Rate_Yaw_Kalibracios;

  kalman_1d(Kalman_Szog_Roll, Kalman_Bizonytalansag_Szog_Roll, Rate_Roll, Szog_Roll);
  Kalman_Szog_Roll = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Szog_Roll = Kalman_1D_kimenet[1];

  kalman_1d(Kalman_Szog_Pitch, Kalman_Bizonytalansag_Szog_Pitch, Rate_Pitch, Szog_Pitch);
  Kalman_Szog_Pitch = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Szog_Pitch = Kalman_1D_kimenet[1];

  //Az inerciális Z tengely irányába eső gyorsulás kiszámítása
  Acc_Z_gyorsulas = -sin(Kalman_Szog_Pitch*(PI/180)) * Acc_X + cos(Kalman_Szog_Pitch*(PI/180)) * sin(Kalman_Szog_Roll*(PI/180)) * Acc_Y + cos(Kalman_Szog_Pitch*(PI/180)) * cos(Kalman_Szog_Roll*(PI/180)) * Acc_Z;
  //gyorsulás átváltása cm/s^2-re
  Acc_Z_gyorsulas = (Acc_Z_gyorsulas-1) * 9.81 * 100;

  baro_adat_olvasas();
  Baro_magassag -= Baro_magassag_kezdetben;
  kalman_2d();

  Csatorna_olvasas();
  //Számítsd ki a kívánt szögeket a vevő (receiver) értékeiből.
  Kivant_Szog_Roll = 0.10 * (Vevo_Ertekek[0] - 1500); 
  Kivant_Szog_Pitch = 0.10 * (Vevo_Ertekek[1] - 1500);
  Kivant_Fuggoleges_sebesseg = 0.3 * (Vevo_Ertekek[2] - 1500);  
  Kivant_Yaw_ertek = 0.15 * (Vevo_Ertekek[3] - 1500);

  Hiba_Fuggoleges_sebesseg = Kivant_Fuggoleges_sebesseg - Fuggoleges_sebesseg_Kalman;

  PID_egyenlet(Hiba_Fuggoleges_sebesseg, P_Fuggoleges_sebesseg, I_Fuggoleges_sebesseg, D_Fuggoleges_sebesseg, Elozo_Hiba_Fuggoleges_sebesseg, Elozo_Iterm_Fuggoleges_sebesseg);
  Bemenet_Gaz = 1500 + PIDReturn[0]; //Mivel a nulla sebességhez, tehát a lebegéshez tartozó pont várhatóan a gázkar középállása körül lesz — 1500 µs-nál —, ezt az értéket add hozzá a PID kimenetéhez.
  Elozo_Hiba_Fuggoleges_sebesseg = PIDReturn[1];
  Elozo_Iterm_Fuggoleges_sebesseg = PIDReturn[2];

  //Számítsd ki a különbséget a kívánt és a tényleges roll és pitch szögek között.
  Hiba_Szog_Roll = Kivant_Szog_Roll - Kalman_Szog_Roll;
  Hiba_Szog_Pitch = Kivant_Szog_Pitch - Kalman_Szog_Pitch;

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

  Hiba_Roll_ertek = Kivant_Roll_ertek - Rate_Roll;
  Hiba_Pitch_ertek = Kivant_Pitch_ertek - Rate_Pitch; 
  Hiba_Yaw_ertek = Kivant_Yaw_ertek - Rate_Yaw;

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
  Motor_1_bemenet =  1.02375 * (Bemenet_Gaz + Bemenet_Roll - Bemenet_Pitch + Bemenet_Yaw );
  Motor_2_bemenet =  1.02375 * (Bemenet_Gaz + Bemenet_Roll + Bemenet_Pitch - Bemenet_Yaw );
  Motor_3_bemenet =  1.02375 * (Bemenet_Gaz - Bemenet_Roll - Bemenet_Pitch - Bemenet_Yaw );
  Motor_4_bemenet =  1.02375 * (Bemenet_Gaz - Bemenet_Roll + Bemenet_Pitch + Bemenet_Yaw );

  /* //50Hz 16 bit felbontás
  Motor_1_bemenet =  3.27675 * (Bemenet_Gaz - Bemenet_Roll - Bemenet_Pitch - Bemenet_Yaw );
  Motor_2_bemenet =  3.27675 * (Bemenet_Gaz - Bemenet_Roll + Bemenet_Pitch + Bemenet_Yaw );
  Motor_3_bemenet =  3.27675 * (Bemenet_Gaz + Bemenet_Roll + Bemenet_Pitch - Bemenet_Yaw );
  Motor_4_bemenet =  3.27675 * (Bemenet_Gaz + Bemenet_Roll - Bemenet_Pitch + Bemenet_Yaw );
  */  

  if (Motor_1_bemenet > 2000) Motor_1_bemenet = 1999;
  if (Motor_2_bemenet > 2000) Motor_2_bemenet = 1999;
  if (Motor_3_bemenet > 2000) Motor_3_bemenet = 1999;
  if (Motor_4_bemenet > 2000) Motor_4_bemenet = 1999;

  //18%-os alapjárat a motoroknak
  if (Motor_1_bemenet < Motorok_alapjarat) Motor_1_bemenet = Motorok_alapjarat;
  if (Motor_2_bemenet < Motorok_alapjarat) Motor_2_bemenet = Motorok_alapjarat;
  if (Motor_3_bemenet < Motorok_alapjarat) Motor_3_bemenet = Motorok_alapjarat;
  if (Motor_4_bemenet < Motorok_alapjarat) Motor_4_bemenet = Motorok_alapjarat;

  //ha leszállni szeretnénk
  if (Vevo_Ertekek[2] < 1050){
    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;
    Reset_PID();
  }

  //ledcWriteChannel(Motor_1_ESC_Csatorna, Motor_1_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_1_ESC_Csatorna, Motor_1_bemenet);                       //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_2_ESC_Csatorna, Motor_2_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_2_ESC_Csatorna, Motor_2_bemenet);                       //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_3_ESC_Csatorna, Motor_3_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_3_ESC_Csatorna, Motor_3_bemenet);                       //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_4_ESC_Csatorna, Motor_4_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_4_ESC_Csatorna, Motor_4_bemenet);                       //ESP32 core 2.0.17

  aksi_feszultseg();
  Elfogyasztott_kapacitas = Aram * 1000 * 0.004 / 3600 + Elfogyasztott_kapacitas;
  Aksi_maradek = (Aksi_kezdetben - Elfogyasztott_kapacitas) / Alap_aksi_kapacitas * 100;
  if (Aksi_maradek <= 30) digitalWrite(Piros_LED, HIGH);
  else digitalWrite(Piros_LED, LOW);

  while (micros() - Loop_idozito < 4000);
  Loop_idozito = micros();
}
