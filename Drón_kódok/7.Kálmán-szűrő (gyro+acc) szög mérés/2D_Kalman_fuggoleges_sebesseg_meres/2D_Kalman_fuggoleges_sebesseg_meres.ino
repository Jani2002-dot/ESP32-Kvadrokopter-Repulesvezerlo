#include <Wire.h> //I2C könyvtár

//számított szögsebességeknek változó
float Rate_Roll;
float Rate_Pitch;
float Rate_Yaw;

//számított szög változó
float Szog_Roll;
float Szog_Pitch;

//kiszámmolt Acc értékek
float Acc_X;
float Acc_Y;
float Acc_Z;

uint32_t Loop_idozito; // 250 Hz-es loop vizsgálatához

float Acc_Z_gyorsulas;

//16 bites kalibrációs adatok mentésére változók
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

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
  uint32_t press_msb = Wire.read(); //0xF7
  uint32_t press_lsb = Wire.read(); //0xF8
  uint32_t press_xlsb = Wire.read(); //0xF9
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read(); //0xFC

  //nyers, kompennzálatlan és kalibrálatlan nyomás
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  //nyers, kompennzálatlan és kalibrálatlan hőmérséklet
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

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
  Acc_X = (float)Acc_XLSB / 4096 - 0.0495;
  Acc_Y = (float)Acc_YLSB / 4096 + 0.023; 
  Acc_Z = (float)Acc_ZLSB / 4096 - 0.093;

  //Szögek kiszámítása árkusz tangens fgv-el de  az Arduinoba Radiánt ad ezért meg kell szorzoni 180/pi vel pi=3.14159265359
  Szog_Roll = atan(Acc_Y / sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (180.0/PI);
  Szog_Pitch = atan(-Acc_X / sqrt(Acc_Y * Acc_Y + Acc_Z * Acc_Z)) * (180.0/PI);

  /*Szog_Roll  = atan2(Acc_Y,  sqrt(Acc_X*Acc_X + Acc_Z*Acc_Z)) * (180.0/PI);
  Szog_Pitch = atan2(-Acc_X, sqrt(Acc_Y*Acc_Y + Acc_Z*Acc_Z)) * (180.0/PI);*/
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(57600); //soros vevőnek
  Wire.begin(21, 22); //SDA, SCL
  Wire.setClock(400000); //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  delay(250);
  mpu_6050_regiszterek_beallitasa();
  baro_regiszterek_beallitasa();
  baro_kalibracio_olvasas();
  digitalWrite(LED_BUILTIN, LOW);
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

  Loop_idozito = micros();
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  gyro_acc_adat_olvasas();
  //Az inerciális Z tengely irányába eső gyorsulás kiszámítása
  Acc_Z_gyorsulas = -sin(Szog_Pitch*(PI/180)) * Acc_X + cos(Szog_Pitch*(PI/180)) * sin(Szog_Roll*(PI/180)) * Acc_Y + cos(Szog_Pitch*(PI/180)) * cos(Szog_Roll*(PI/180)) * Acc_Z;
  //gyorsulás átváltása cm/s^2-re
  Acc_Z_gyorsulas = (Acc_Z_gyorsulas-1) * 9.81 * 100;

  baro_adat_olvasas();
  Baro_magassag -= Baro_magassag_kezdetben;
  kalman_2d();

  Serial.print("Magassag: ");
  Serial.print(Magassag_Kalman);
  //Serial.print("\t");
  Serial.print("  Sebesseg: ");
  Serial.println(Fuggoleges_sebesseg_Kalman);

  while (micros() - Loop_idozito < 4000);
    Loop_idozito = micros();
}
