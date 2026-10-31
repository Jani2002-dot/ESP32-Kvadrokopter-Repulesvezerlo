#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Wire.h> //I2C könyvtár
//#include "esp_system.h"
#include <IBusBM.h>
IBusBM ibus;

 const char* ssid = "Telekom-pQhhNF";
 const char* password = "edu4hbeebb37";

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

#define feszultseg_meres_PIN 36
#define aram_meres_PIN 39
#define Piros_LED 13
#define Zold_LED 4

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

//static azt jelenti, hogy ez a változó “fájl-szintű / globális” élettartamú, vagyis a program teljes futása alatt létezik, és ugyanaz az egy példánya van.
//csak ebben a .cpp fájlban látható (internal linkage). Tehát ha több fájlod van, másik .cpp-ből nem tudod elérni név szerint.
uint16_t Vevo_Ertekek[]={0,0,0,0,0};
bool SWA_Allapot;
float Gaz_bemenet; //0% gaz=1000 us 100%gaz=2000us

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
float P_ertek_Roll=0, P_ertek_Pitch=0, P_ertek_Yaw=0;
float I_ertek_Roll=0, I_ertek_Pitch=0, I_ertek_Yaw=0;
float D_ertek_Roll=0, D_ertek_Pitch=0, D_ertek_Yaw=0;
float Ts = 0.004; //250 Hz-es loop miatt 
int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;

/*void ledVillog(int db, int onTime = 500, int offTime = 500) {
  for (int i = 0; i < db; i++) {
    digitalWrite(Piros_LED, HIGH);
    delay(onTime);
    digitalWrite(Piros_LED, LOW);
    delay(offTime);
  }
}

int resetKod() {
  esp_reset_reason_t reason = esp_reset_reason();

  switch (reason) {
    case ESP_RST_POWERON:
      return 1; // táp bekapcsolás / első indulás

    case ESP_RST_EXT:
      return 2; // külső reset pin

    case ESP_RST_SW:
      return 3; // szoftveres reset

    case ESP_RST_PANIC:
      return 4; // crash / exception / programhiba

    case ESP_RST_INT_WDT:
      return 5; // interrupt watchdog

    case ESP_RST_TASK_WDT:
      return 6; // task watchdog, pl. program beragad

    case ESP_RST_WDT:
      return 7; // általános watchdog reset

    case ESP_RST_BROWNOUT:
      return 8; // tápfesz beesés / brownout

    case ESP_RST_DEEPSLEEP:
      return 9; // deep sleepből ébredt

    default:
      return 10; // ismeretlen reset ok
  }
}

void resetOkVillogas() {
  int kod = resetKod();

  delay(1000);

  // 3x ismétli a reset ok kódját
  for (int kor = 0; kor < 3; kor++) {
    ledVillog(kod);
    delay(1200);
  }
}*/


void aksi_feszultseg(void){
  Feszultseg=(float)analogRead(feszultseg_meres_PIN)/206.818182  + 0.027; //4095/(3.3*6)=206.8182
  Aram=(float)analogRead(aram_meres_PIN)/37.227273; // ADC / ((4095/3.3V)*0.03 V/A)=37.227273 ---> ADC*0,02686203
}

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
}

void gyro_adat_olvasas(void){

  //Girszkóp adatainak kiolvasása
  Wire.beginTransmission(0x68); //MPU-val való komunikáció 
  Wire.write(0x43); //Gyro adatok itt keződnek
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6); // 6 bytot kérünk

  while (Wire.available()<6) {}; // vár a 6 byte-ra 

  //MPU 16 bites értékeit bele bitshiftelni egy 16 bites változóba mivel I2C rigsztereken csak 8 bites byte-okban kommunikál 
  // << bitshift 8 bittel feltoljuk a 16 bites változó 15-8 bitjébe
  // | (bitwise OR) összekapcsolja a byte.okat
  int16_t Gyro_X = Wire.read()<<8 | Wire.read();
  int16_t Gyro_Y = Wire.read()<<8 | Wire.read();
  int16_t Gyro_Z = Wire.read()<<8 | Wire.read();
 
  // °/s szögsebességek kiszámolás mivel +-500°/s LSB 65.5/°/s 
  // MPU 16 bites adat olvasás 2^16, +-500 = 1000 ------> 2^16/1000=65.536 ~ 65.5
  Rate_Roll=(float)Gyro_X/65.5;
  Rate_Pitch=(float)Gyro_Y/65.5;
  Rate_Yaw=(float)Gyro_Z/65.5;
}

void PID_egyenlet (float Hiba, float P, float I, float D, float Elozo_Hiba, float Elozo_Iterm){
  float Pterm = P * Hiba;
  float Iterm = Elozo_Iterm + (I  * (Hiba + Elozo_Hiba)) * (Ts / 2);
  if (Iterm  > 400) {
    Iterm = 400;
  }
  else if (Iterm < -400) {
    Iterm = -400;
  }
  float Dterm = D * ((Hiba - Elozo_Hiba) / Ts);
  float PID_Kimenet = Pterm + Iterm + Dterm;
  if (PID_Kimenet  > 400) {
    PID_Kimenet = 400;
  }
  else if (PID_Kimenet < -400) {
    PID_Kimenet = -400;
  }

  PIDReturn[0] = PID_Kimenet;
  PIDReturn[1] = Hiba; 
  PIDReturn[2] = Iterm;
}

//WIFI tuning global code
AsyncWebServer server(80);

const char* PARAM_P_GAIN = "pGain";   //For Pitch & Roll RATE
const char* PARAM_I_GAIN = "iGain";
const char* PARAM_D_GAIN = "dGain";

const char* PARAM_P_A_GAIN = "pAGain";   //For Pitch & Roll ANGLE
const char* PARAM_I_A_GAIN = "iAGain";
const char* PARAM_D_A_GAIN = "dAGain";

const char* PARAM_P_YAW = "pYaw";     //For Yaw
const char* PARAM_I_YAW = "iYaw";
const char* PARAM_D_YAW = "dYaw";

const char* PARAM_TIME_CYCLE = "tc";  //Computation time cycle

// HTML web page to handle 6 input fields of PID gains
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);
    }
  </script></head><body>

     <form action="/get" target="hidden-form"><br>
    ESP32 Webserver for PID Gain value tuning of Quadcopter 
  </form><br><br>

  <form action="/get" target="hidden-form">
    P Pitch & Roll Gain (current value %pGain%): <input type="number" step="any" name="pGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    I Pitch & Roll Gain (current value %iGain%): <input type="number" step="any" name="iGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    D Pitch & Roll Gain (current value %dGain%): <input type="number" step="any" name="dGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    P Pitch & Roll Angle Gain (current value %pAGain%): <input type="number" step="any" name="pAGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
    <form action="/get" target="hidden-form">
    I Pitch & Roll Angle Gain (current value %iAGain%): <input type="number" step="any" name="iAGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
    <form action="/get" target="hidden-form">
    D Pitch & Roll Angle Gain (current value %dAGain%): <input type="number" step="any" name="dAGain">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    P Yaw Gain (current value %pYaw%): <input type="number" step="any" name="pYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    I Yaw Gain (current value %iYaw%): <input type="number" step="any" name="iYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    D Yaw Gain (current value %dYaw%): <input type="number" step="any" name="dYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br><br>
    <form action="/get" target="hidden-form">
    Time cycle (current value %tc%): <input type="number" step="any" name="tc">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br><br>

  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
if(var == "pGain"){
    return readFile(SPIFFS, "/pGain.txt");
}
else if(var == "iGain"){
    return readFile(SPIFFS, "/iGain.txt");
}
else if(var == "dGain"){
    return readFile(SPIFFS, "/dGain.txt");
}
else if(var == "pAGain"){
    return readFile(SPIFFS, "/pAGain.txt");
}
else if(var == "iAGain"){
    return readFile(SPIFFS, "/iAGain.txt");
}
else if(var == "dAGain"){
    return readFile(SPIFFS, "/dAGain.txt");
}
else if(var == "pYaw"){
    return readFile(SPIFFS, "/pYaw.txt");
}
else if(var == "dYaw"){
    return readFile(SPIFFS, "/dYaw.txt");
}
else if(var == "iYaw"){
    return readFile(SPIFFS, "/iYaw.txt");
}
else if(var == "tc"){
    return readFile(SPIFFS, "/tc.txt");
}

}

void Reset_PID(void){
  Elozo_Hiba_Roll_ertek = 0;
  Elozo_Hiba_Pitch_ertek = 0;
  Elozo_Hiba_Yaw_ertek = 0;
  Elozo_Iterm_Roll = 0;
  Elozo_Iterm_Pitch = 0;
  Elozo_Iterm_Yaw = 0;
}

uint32_t usToDuty(float motor_bemenet_us) {
  return (uint32_t)(motor_bemenet_us * 1.02375);
}

void varakozas_250Hz(void) {
  while ((uint32_t)(micros() - Loop_idozito) < 4000) {
    delayMicroseconds(250); 
  }

  Loop_idozito = micros();
}

void setup() {

Serial.begin(115200);

//WIFI server setup START
// Initialize SPIFFS
  #ifdef ESP32
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #else
    if(!SPIFFS.begin()){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
  #endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");

  Serial.println(WiFi.localIP());
  delay(2000);
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET P Gain value on <ESP_IP>/get?pGain=<inputMessage>
if (request->hasParam(PARAM_P_GAIN)) {
    inputMessage = request->getParam(PARAM_P_GAIN)->value();
    writeFile(SPIFFS, "/pGain.txt", inputMessage.c_str());
}
// GET I Gain value on <ESP_IP>/get?iGain=<inputMessage>
else if (request->hasParam(PARAM_I_GAIN)) {
    inputMessage = request->getParam(PARAM_I_GAIN)->value();
    writeFile(SPIFFS, "/iGain.txt", inputMessage.c_str());
}
// GET D Gain value on <ESP_IP>/get?dGain=<inputMessage>
else if (request->hasParam(PARAM_D_GAIN)) {
    inputMessage = request->getParam(PARAM_D_GAIN)->value();
    writeFile(SPIFFS, "/dGain.txt", inputMessage.c_str());
}
else if (request->hasParam(PARAM_P_A_GAIN)) {
    inputMessage = request->getParam(PARAM_P_A_GAIN)->value();
    writeFile(SPIFFS, "/pAGain.txt", inputMessage.c_str());
}
else if (request->hasParam(PARAM_I_A_GAIN)) {
    inputMessage = request->getParam(PARAM_I_A_GAIN)->value();
    writeFile(SPIFFS, "/iAGain.txt", inputMessage.c_str());
}
else if (request->hasParam(PARAM_D_A_GAIN)) {
    inputMessage = request->getParam(PARAM_D_A_GAIN)->value();
    writeFile(SPIFFS, "/dAGain.txt", inputMessage.c_str());
}
else if (request->hasParam(PARAM_P_YAW)) {
    inputMessage = request->getParam(PARAM_P_YAW)->value();
    writeFile(SPIFFS, "/pYaw.txt", inputMessage.c_str());
  } else if (request->hasParam(PARAM_I_YAW)) {
    inputMessage = request->getParam(PARAM_I_YAW)->value();
    writeFile(SPIFFS, "/iYaw.txt", inputMessage.c_str());
  } else if (request->hasParam(PARAM_D_YAW)) {
    inputMessage = request->getParam(PARAM_D_YAW)->value();
    writeFile(SPIFFS, "/dYaw.txt", inputMessage.c_str());
  }
 else if (request->hasParam(PARAM_TIME_CYCLE)) {
    inputMessage = request->getParam(PARAM_TIME_CYCLE)->value();
    writeFile(SPIFFS, "/tc.txt", inputMessage.c_str());
  }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });

  server.onNotFound(notFound);
  server.begin();
//WIFI server setup END

  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); //soros vevőnek
  ibus.begin(Serial2); //ibus inicializálása

  analogReadResolution(12);
  analogSetPinAttenuation(feszultseg_meres_PIN, ADC_11db);
  analogSetPinAttenuation(aram_meres_PIN, ADC_11db);

  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(Piros_LED,OUTPUT);
  pinMode(Zold_LED,OUTPUT);

  //resetOkVillogas();
  //bootVillogas();

  digitalWrite(LED_BUILTIN,HIGH);
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

// 250Hz=4ms=4000us
// 12 bites felbontás = 0-4095 
// 0 ---> 0us        4095--->4000us
// PWM jel küldése us-ban az ESC nek akkor 4095/4000=1.024-el kell majd szorozi 250Hz, 12 bit esetén 

  ledcWrite(Motor_1_ESC_Csatorna, usToDuty(1000));
  ledcWrite(Motor_2_ESC_Csatorna, usToDuty(1000));
  ledcWrite(Motor_3_ESC_Csatorna, usToDuty(1000));
  ledcWrite(Motor_4_ESC_Csatorna, usToDuty(1000));

  delay(3000);  

  Wire.setClock(400000); //400kHz ClockSpeed MPU6050 adatlap írja I2C kommunikáció esetén
  Wire.begin(); //SDA, SCL
  delay(250);
  mpu_6050_regiszterek_beallitasa();

  //2000 mintavételes kalibráció 
  for (Kalibracios_int=0 ; Kalibracios_int < Kalibracio_mintavetel_szam ; Kalibracios_int++){
    gyro_adat_olvasas();
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

  aksi_feszultseg();
  if (Feszultseg > 16.6) {         // terhelés nélkűl mérjük az aksit így valós értéket kapunk mivel terhelés alatt beesik a feszülség szintje és nem lesz valós érték ezért terhlés alatt áramot mmérünk
    digitalWrite(Piros_LED, LOW); //piros led kikapcs
    Aksi_kezdetben = Alap_aksi_kapacitas;//felöltve tehát 1300mAh 
  }
  else if (Feszultseg < 15){
    digitalWrite(Piros_LED, HIGH);
    Aksi_kezdetben=(float)30/100*Alap_aksi_kapacitas; //30% át vesszük
  }
  else {
    digitalWrite(Piros_LED, LOW);
    Aksi_kezdetben = (41 * Feszultseg - 580)/100 * Alap_aksi_kapacitas; 
  }

// kontrollálatlan motorindítás elkerülése 
  while (Vevo_Ertekek[2] < 1020 || Vevo_Ertekek[2] > 1050){  //Vevo_Ertekek[2] ---> Gáz kar értéke  kontrollálatlan motorindítás elkerülése 
    Csatorna_olvasas();
    delay(4);
  }
  Loop_idozito = micros();
}

void loop() {

  digitalWrite(LED_BUILTIN,LOW);

  gyro_adat_olvasas(); //Gyro adat olvasó fgv meghívása
  //Levonjuk az offset hibát az újra olvasott értékekből így közel 0°/s lesz minden
  Rate_Roll -= Rate_Roll_Kalibracios;
  Rate_Pitch -= Rate_Pitch_Kalibracios;
  Rate_Yaw -= Rate_Yaw_Kalibracios;

  Csatorna_olvasas();

  if(SWA_Allapot == 1) //channel 5 for uploading values
  {
  P_ertek_Roll = readFile(SPIFFS, "/pGain.txt").toFloat();
  I_ertek_Roll = readFile(SPIFFS, "/iGain.txt").toFloat();
  D_ertek_Roll = readFile(SPIFFS, "/dGain.txt").toFloat();

  P_ertek_Pitch = P_ertek_Roll;
  I_ertek_Pitch = I_ertek_Roll;
  D_ertek_Pitch = D_ertek_Roll;

  /*PAngleRoll=readFile(SPIFFS, "/pAGain.txt").toFloat();; 
  IAngleRoll=readFile(SPIFFS, "/iAGain.txt").toFloat();; 
  DAngleRoll=readFile(SPIFFS, "/dAGain.txt").toFloat();; 

  PAnglePitch=PAngleRoll;
  IAnglePitch=IAngleRoll;
  DAnglePitch=DAngleRoll;*/

  P_ertek_Yaw = readFile(SPIFFS, "/pYaw.txt").toFloat();
  I_ertek_Yaw = readFile(SPIFFS, "/iYaw.txt").toFloat();
  D_ertek_Yaw = readFile(SPIFFS, "/dYaw.txt").toFloat();

  //t = readFile(SPIFFS, "/tc.txt").toFloat();
  }

  Kivant_Roll_ertek = 0.15 * (Vevo_Ertekek[0] - 1500); 
  Kivant_Pitch_ertek = 0.15 * (Vevo_Ertekek[1] - 1500);
  Bemenet_Gaz = Vevo_Ertekek[2];
  Kivant_Yaw_ertek = 0.15 * (Vevo_Ertekek[3] - 1500);

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
  Motor_1_bemenet =  (Bemenet_Gaz - Bemenet_Roll + Bemenet_Pitch + Bemenet_Yaw); // 100 %
  Motor_2_bemenet =  (Bemenet_Gaz - Bemenet_Roll - Bemenet_Pitch - Bemenet_Yaw); //   0 %
  Motor_3_bemenet =  (Bemenet_Gaz + Bemenet_Roll + Bemenet_Pitch - Bemenet_Yaw); // 100 %
  Motor_4_bemenet =  (Bemenet_Gaz + Bemenet_Roll - Bemenet_Pitch + Bemenet_Yaw); // 100 %

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
  if (Vevo_Ertekek[2] < 1030){
    Motor_1_bemenet = Motorok_leallitas;
    Motor_2_bemenet = Motorok_leallitas;
    Motor_3_bemenet = Motorok_leallitas;
    Motor_4_bemenet = Motorok_leallitas;
    Reset_PID();
  }

  //ledcWriteChannel(Motor_1_ESC_Csatorna, Motor_1_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Motor_1_bemenet));             //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_2_ESC_Csatorna, Motor_2_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Motor_2_bemenet));             //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_3_ESC_Csatorna, Motor_3_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Motor_3_bemenet));             //ESP32 core 2.0.17

  //ledcWriteChannel(Motor_4_ESC_Csatorna, Motor_4_bemenet);              //ESP32 core 3.3.6
  ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Motor_4_bemenet));             //ESP32 core 2.0.17

  aksi_feszultseg();
  Elfogyasztott_kapacitas = Aram * 1000 * 0.004 / 3600 + Elfogyasztott_kapacitas;
  Aksi_maradek = (Aksi_kezdetben - Elfogyasztott_kapacitas) / Alap_aksi_kapacitas * 100;
  if (Aksi_maradek <= 30) digitalWrite(Piros_LED, HIGH);
  else digitalWrite(Piros_LED, LOW);

  varakozas_250Hz();
}