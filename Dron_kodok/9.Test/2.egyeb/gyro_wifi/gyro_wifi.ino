#include <Wire.h> //I2C könyvtár
#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

const char* ssid = "MPU_Monitor";
const char* password = "12345678";

uint32_t Loop_idozito; // 250 Hz-es loop vizsgálatához
uint32_t Loop_dt_us;

//számított szögsebességeknek változó
float Rate_Roll , Rate_Pitch , Rate_Yaw;
float Rate_Roll_2 , Rate_Pitch_2 , Rate_Yaw_2;
float Rate_Roll_Szurt , Rate_Pitch_Szurt , Rate_Yaw_Szurt;
bool Komplementer_Elindult = false;
float Szurt_Szog_Roll;
float Szurt_Szog_Pitch;
float Szurt_Szog_Roll_2;
float Szurt_Szog_Pitch_2;
float Ts = 0.004;
bool Gyro_Kalibracio_Kesz = false;
float acc_total_vector;
float angle_pitch_acc, angle_roll_acc;

//kiszámmolt Acc értékek
float Acc_X , Acc_Y , Acc_Z;

//számított szög változók
float Szog_Roll;
float Szog_Pitch;

//Függöleges sebesség változói
float Acc_Z_gyorsulas;
float Fuggoleges_sebesseg;

//2000 mintavételt ezekhez adjuk hozzá
float Rate_Roll_Kalibracios=0;
float Rate_Pitch_Kalibracios=0;
float Rate_Yaw_Kalibracios=0;

//for ciklus változói
int Kalibracios_int;
int Kalibracio_mintavetel_szam=2000;

float Kalman_Roll_Offset = 0;
float Kalman_Pitch_Offset = 0;

float Kalman_Szog_Roll_Korrigalt = 0;
float Kalman_Szog_Pitch_Korrigalt = 0;

float Kalman_Szog_Roll=0;
float Kalman_Bizonytalansag_Szog_Roll=2*2; //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_Szog_Pitch=0;
float Kalman_Bizonytalansag_Szog_Pitch=2*2; //mivel feltételezzük nem vízszintes talajról indulnuk ezért 2° variancia

float Kalman_1D_kimenet[] = {0,0};

//16 bites kalibrációs adatok mentésére változók
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

float Baro_magassag;
float Baro_magassag_kezdetben = 0.0;

int Baro_kalibracios_szam;

float Magassag_Kalman, Fuggoleges_sebesseg_Kalman;

#include <BasicLinearAlgebra.h> //vektorokhoz és mátrixokhoz könyvtár
using namespace BLA;

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

void kalman_1d(float Kalman_allapot, float Kalman_Bizonytalansag, float Kalman_bemenet, float Kalman_meres){
  Kalman_allapot = Kalman_allapot + Kalman_bemenet * 0.004;
  Kalman_Bizonytalansag = Kalman_Bizonytalansag + 0.004 * 0.004 * 4 * 4;
  float Kalman_erosites = Kalman_Bizonytalansag * 1/(1 * Kalman_Bizonytalansag + 3 * 3);
  Kalman_allapot = Kalman_allapot + Kalman_erosites * (Kalman_meres - Kalman_allapot);
  Kalman_Bizonytalansag = (1 - Kalman_erosites * 1) * Kalman_Bizonytalansag;
  Kalman_1D_kimenet[0] = Kalman_allapot;
  Kalman_1D_kimenet[1] = Kalman_Bizonytalansag;
}

void Kalman_1D_szog_nullazas(void) {
  float roll_osszeg = 0;
  float pitch_osszeg = 0;

  for (int i = 0; i < 500; i++) {
    gyro_acc_adat_olvasas();

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

void wifi_monitor_inditas() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", []() {
    server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>MPU Monitor</title>
</head>
<body>
  <h2>MPU6050 Monitor</h2>

  <h3>Nyers / kalibrált gyro rate</h3>
  <p>Rate Roll: <span id="roll">0</span> °/s</p>
  <p>Rate Pitch: <span id="pitch">0</span> °/s</p>
  <p>Rate Yaw: <span id="yaw">0</span> °/s</p>

  <hr>

  <h3>Szűrt gyro rate</h3>
  <p>Rate Roll szűrt: <span id="roll_szurt">0</span> °/s</p>
  <p>Rate Pitch szűrt: <span id="pitch_szurt">0</span> °/s</p>
  <p>Rate Yaw szűrt: <span id="yaw_szurt">0</span> °/s</p>

  <hr>

  <h3>Accelerometerből számolt szög</h3>
  <p>Acc Roll szög: <span id="szog_roll">0</span> °</p>
  <p>Acc Pitch szög: <span id="szog_pitch">0</span> °</p>

  <hr>

  <h3>Accelerometerből számolt szög 2</h3>
  <p>Acc Roll szög 2: <span id="szog_roll_2">0</span> °</p>
  <p>Acc Pitch szög 2: <span id="szog_pitch_2">0</span> °</p>  

  <hr>

  <h3>Komplementer szűrt szög</h3>
  <p>Szűrt Roll szög: <span id="szurt_szog_roll">0</span> °</p>
  <p>Szűrt Pitch szög: <span id="szurt_szog_pitch">0</span> °</p>

  <hr>

  <h3>Komplementer szűrt szög 2</h3>
  <p>Szűrt Roll szög 2: <span id="szurt_szog_roll_2">0</span> °</p>
  <p>Szűrt Pitch szög 2: <span id="szurt_szog_pitch_2">0</span> °</p>

  <hr>

  <h3>Kálmán szög</h3>
  <p>Kálmán Roll szög: <span id="kalman_szog_roll">0</span> °</p>
  <p>Kálmán Pitch szög: <span id="kalman_szog_pitch">0</span> °</p>

  <hr>

  <h3>Accelerometer értékek</h3>
  <p>Acc X: <span id="acc_x">0</span> g</p>
  <p>Acc Y: <span id="acc_y">0</span> g</p>
  <p>Acc Z: <span id="acc_z">0</span> g</p>

  <hr>

  <h3>Kálmán függőleges sebesség</h3>
  <p>Függőleges sebesség Kálmán: <span id="fuggoleges_sebesseg_kalman">0</span> cm/s</p>

  <hr>

  <h3>Loop</h3>
  <p>Loop idő: <span id="loop_dt_us">0</span> us</p>

  <script>
    setInterval(() => {
      fetch('/adat')
        .then(response => response.json())
        .then(data => {
          document.getElementById('roll').textContent = data.roll.toFixed(2);
          document.getElementById('pitch').textContent = data.pitch.toFixed(2);
          document.getElementById('yaw').textContent = data.yaw.toFixed(2);

          document.getElementById('roll_szurt').textContent = data.roll_szurt.toFixed(2);
          document.getElementById('pitch_szurt').textContent = data.pitch_szurt.toFixed(2);
          document.getElementById('yaw_szurt').textContent = data.yaw_szurt.toFixed(2);

          document.getElementById('szog_roll').textContent = data.szog_roll.toFixed(2);
          document.getElementById('szog_pitch').textContent = data.szog_pitch.toFixed(2);

          document.getElementById('szog_roll_2').textContent = data.szog_roll_2.toFixed(2);
          document.getElementById('szog_pitch_2').textContent = data.szog_pitch_2.toFixed(2);

          document.getElementById('szurt_szog_roll').textContent = data.szurt_szog_roll.toFixed(2);
          document.getElementById('szurt_szog_pitch').textContent = data.szurt_szog_pitch.toFixed(2);

          document.getElementById('szurt_szog_roll_2').textContent = data.szurt_szog_roll_2.toFixed(2);
          document.getElementById('szurt_szog_pitch_2').textContent = data.szurt_szog_pitch_2.toFixed(2);

          document.getElementById('kalman_szog_roll').textContent = data.kalman_szog_roll.toFixed(2);
          document.getElementById('kalman_szog_pitch').textContent = data.kalman_szog_pitch.toFixed(2);

          document.getElementById('acc_x').textContent = data.acc_x.toFixed(3);
          document.getElementById('acc_y').textContent = data.acc_y.toFixed(3);
          document.getElementById('acc_z').textContent = data.acc_z.toFixed(3);

          document.getElementById('fuggoleges_sebesseg_kalman').textContent = data.fuggoleges_sebesseg_kalman.toFixed(3);

          document.getElementById('loop_dt_us').textContent = data.loop_dt_us;
        })
        .catch(error => {
          console.log("Adat lekérési hiba:", error);
        });
    }, 200);
  </script>
</body>
</html>
)rawliteral");
  });

  server.on("/adat", []() {
    auto jsonFloat = [](float ertek, int tizedes) -> String {
      if (isnan(ertek) || isinf(ertek)) return "0";
      return String(ertek, tizedes);
    };

    String json = "{";

    json += "\"roll\":" + jsonFloat(Rate_Roll_2, 2) + ",";
    json += "\"pitch\":" + jsonFloat(Rate_Pitch_2, 2) + ",";
    json += "\"yaw\":" + jsonFloat(Rate_Yaw_2, 2) + ",";

    json += "\"roll_szurt\":" + jsonFloat(Rate_Roll_Szurt, 2) + ",";
    json += "\"pitch_szurt\":" + jsonFloat(Rate_Pitch_Szurt, 2) + ",";
    json += "\"yaw_szurt\":" + jsonFloat(Rate_Yaw_Szurt, 2) + ",";

    json += "\"szog_roll\":" + jsonFloat(Szog_Roll, 2) + ",";
    json += "\"szog_pitch\":" + jsonFloat(Szog_Pitch, 2) + ",";

    json += "\"szog_roll_2\":" + jsonFloat(angle_roll_acc, 2) + ",";
    json += "\"szog_pitch_2\":" + jsonFloat(angle_pitch_acc, 2) + ",";

    json += "\"szurt_szog_roll\":" + jsonFloat(Szurt_Szog_Roll, 2) + ",";
    json += "\"szurt_szog_pitch\":" + jsonFloat(Szurt_Szog_Pitch, 2) + ",";

    json += "\"szurt_szog_roll_2\":" + jsonFloat(Szurt_Szog_Roll_2, 2) + ",";
    json += "\"szurt_szog_pitch_2\":" + jsonFloat(Szurt_Szog_Pitch_2, 2) + ",";    

    json += "\"kalman_szog_roll\":" + jsonFloat(Kalman_Szog_Roll_Korrigalt, 2) + ",";
    json += "\"kalman_szog_pitch\":" + jsonFloat(Kalman_Szog_Pitch_Korrigalt, 2) + ",";

    json += "\"acc_x\":" + jsonFloat(Acc_X, 4) + ",";
    json += "\"acc_y\":" + jsonFloat(Acc_Y, 4) + ",";
    json += "\"acc_z\":" + jsonFloat(Acc_Z, 4) + ",";

    json += "\"fuggoleges_sebesseg_kalman\":" + jsonFloat(Fuggoleges_sebesseg_Kalman, 3) + ",";

    json += "\"loop_dt_us\":" + String(Loop_dt_us);

    json += "}";

    server.send(200, "application/json", json);
  });

  server.begin();
}

  TaskHandle_t WiFiTaskHandle = NULL;

  void WiFiTask(void * parameter) {
    for (;;) {
      server.handleClient();
      delay(10); // kb. 100 Hz web kiszolgálás, nem zavarja a 250 Hz loopot
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
  Wire.write(0x03); //10 Hz beleírása HEX-be 
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
  Acc_X = (float)Acc_XLSB / 4096 - 0.0217;
  Acc_Y = (float)Acc_YLSB / 4096 + 0.019; 
  Acc_Z = (float)Acc_ZLSB / 4096 - 0.109;

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
  //Serial.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.setClock(400000); //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  Wire.begin(); //SDA, SCL
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

  Kalman_1D_szog_nullazas();

  digitalWrite(LED_BUILTIN, HIGH);

  wifi_monitor_inditas();

  xTaskCreatePinnedToCore(
    WiFiTask,
    "WiFiTask",
    8192,
    NULL,
    1,
    &WiFiTaskHandle,
    0
  );

  Loop_idozito = micros();
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);

  gyro_acc_adat_olvasas(); //Gyro és Acc adat olvasó fgv meghívása

  //exponenciális low-pass / mozgóátlag jellegű szűrő
  Rate_Roll_Szurt = Rate_Roll_Szurt * 0.7 + Rate_Roll_2 * 0.3;
  Rate_Pitch_Szurt = Rate_Pitch_Szurt * 0.7 + Rate_Pitch_2 * 0.3;
  Rate_Yaw_Szurt = Rate_Yaw_Szurt * 0.7 + Rate_Yaw_2 * 0.3;

  if (!Komplementer_Elindult) {
    Szurt_Szog_Roll = Szog_Roll;
    Szurt_Szog_Pitch = Szog_Pitch;
    Szurt_Szog_Roll_2 = angle_roll_acc;
    Szurt_Szog_Pitch_2 = angle_pitch_acc;
    Komplementer_Elindult = true;
  }

  // Gyro integrálás atan()
  Szurt_Szog_Roll += Rate_Roll * Ts;
  Szurt_Szog_Pitch += Rate_Pitch * Ts;

  // Gyro integrálás asin()
  Szurt_Szog_Roll_2 += Rate_Roll * Ts;
  Szurt_Szog_Pitch_2 += Rate_Pitch * Ts;

  // Yaw miatti roll/pitch tengelykorrekció
  float yaw_rad = Rate_Yaw * Ts * (PI / 180.0);

  Szurt_Szog_Pitch = Szurt_Szog_Pitch - Szurt_Szog_Roll * sin(yaw_rad);
  Szurt_Szog_Roll  = Szurt_Szog_Roll + Szurt_Szog_Pitch * sin(yaw_rad);

  Szurt_Szog_Pitch_2 = Szurt_Szog_Pitch_2 - Szurt_Szog_Roll_2 * sin(yaw_rad);
  Szurt_Szog_Roll_2  = Szurt_Szog_Roll_2 + Szurt_Szog_Pitch_2 * sin(yaw_rad);
  
  // Accelerometeres lassú korrekció komplementer szűrővel
  Szurt_Szog_Roll = Szurt_Szog_Roll * 0.9996 + Szog_Roll * 0.0004;
  Szurt_Szog_Pitch = Szurt_Szog_Pitch * 0.9996 + Szog_Pitch * 0.0004;

  // Accelerometeres lassú korrekció komplementer szűrővel 2
  Szurt_Szog_Roll_2 = Szurt_Szog_Roll_2 * 0.9996 + angle_roll_acc * 0.0004;
  Szurt_Szog_Pitch_2 = Szurt_Szog_Pitch_2 * 0.9996 + angle_pitch_acc * 0.0004;

  kalman_1d(Kalman_Szog_Roll, Kalman_Bizonytalansag_Szog_Roll, Rate_Roll, Szog_Roll);
  Kalman_Szog_Roll = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Szog_Roll = Kalman_1D_kimenet[1];

  kalman_1d(Kalman_Szog_Pitch, Kalman_Bizonytalansag_Szog_Pitch, Rate_Pitch, Szog_Pitch);
  Kalman_Szog_Pitch = Kalman_1D_kimenet[0];
  Kalman_Bizonytalansag_Szog_Pitch = Kalman_1D_kimenet[1];

  Kalman_Szog_Roll_Korrigalt = Kalman_Szog_Roll - Kalman_Roll_Offset;
  Kalman_Szog_Pitch_Korrigalt = Kalman_Szog_Pitch - Kalman_Pitch_Offset;

  //Az inerciális Z tengely irányába eső gyorsulás kiszámítása
  Acc_Z_gyorsulas = -sin(Kalman_Szog_Pitch_Korrigalt*(PI/180)) * Acc_X + cos(Kalman_Szog_Pitch_Korrigalt*(PI/180)) * sin(Kalman_Szog_Roll_Korrigalt*(PI/180)) * Acc_Y + cos(Kalman_Szog_Pitch_Korrigalt*(PI/180)) * cos(Kalman_Szog_Roll_Korrigalt*(PI/180)) * Acc_Z;
  //gyorsulás átváltása cm/s^2-re
  Acc_Z_gyorsulas = (Acc_Z_gyorsulas-1) * 9.81 * 100;
  //függőleges sebesség  számítása, gyorsulás időszerinti integrálsával
  Fuggoleges_sebesseg  = Fuggoleges_sebesseg +  0.004 * Acc_Z_gyorsulas;

  baro_adat_olvasas();
  Baro_magassag -= Baro_magassag_kezdetben;
  kalman_2d();

  varakozas_250Hz();
}
