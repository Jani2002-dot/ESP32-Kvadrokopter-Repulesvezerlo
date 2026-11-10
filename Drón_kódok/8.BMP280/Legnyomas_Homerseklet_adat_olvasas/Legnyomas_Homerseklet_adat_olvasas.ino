#include <Wire.h>

//16 bites kalibrációs adatok mentésére változók
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

float Baro_magassag;
float Baro_magassag_kezdetben = 0.0;

int Baro_kalibracios_szam;

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

void setup() {
  Serial.begin(57600);
  //Serial.println("Setup kezdés");
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  baro_regiszterek_beallitasa();
  baro_kalibracio_olvasas();
  //Serial.println("Setup kész");
}

void loop() {
  baro_adat_olvasas();
  float magassag_relativ = Baro_magassag - Baro_magassag_kezdetben;
  //Serial.print("Magassag");
  Serial.print(25);
  Serial.print("\t");
  Serial.print(0);
  Serial.print("\t");
  Serial.print(-25);
  Serial.print("\t");
  Serial.println(magassag_relativ);
  delay(50);
}
