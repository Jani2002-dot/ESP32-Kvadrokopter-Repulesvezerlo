#include <BluetoothSerial.h>
#include <Preferences.h>
#include <IBusBM.h>

IBusBM ibus;
BluetoothSerial SerialBT;
Preferences preferences;

//ibus.readChannel(0-9) 10 csatorna
#define RX_PIN 16
#define TX_PIN 25

float P_ertek_Roll=0, P_ertek_Pitch=P_ertek_Roll, P_ertek_Yaw=0;
float I_ertek_Roll=0, I_ertek_Pitch=I_ertek_Roll, I_ertek_Yaw=0;
float D_ertek_Roll=0, D_ertek_Pitch=D_ertek_Roll, D_ertek_Yaw=0;

String BT_Buffer = "";

uint16_t Vevo_Ertekek[]={0,0,0,0,0};
bool SWA_Allapot;
float Gaz_bemenet; //0% gaz=1000 us 100%gaz=2000us

float Kivant_Roll_ertek, Kivant_Pitch_ertek, Kivant_Yaw_ertek;

uint32_t Loop_idozito;
float Ts = 0.004;

void varakozas_250Hz(void) {
  while ((uint32_t)(micros() - Loop_idozito) < 4000) {
    delayMicroseconds(270); 
  }

  Loop_idozito = micros();
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


void PID_ertekek_kiirasa() {
  Serial.println("===== PID =====");
  Serial.print("P_R/P: "); Serial.println(P_ertek_Roll, 5);
  Serial.print("I_R/P: "); Serial.println(I_ertek_Roll, 5);
  Serial.print("D_R/P: "); Serial.println(D_ertek_Roll, 5);

  Serial.print("P_Y: "); Serial.println(P_ertek_Yaw, 5);
  Serial.print("I_Y: "); Serial.println(I_ertek_Yaw, 5);
  Serial.print("D_Y: "); Serial.println(D_ertek_Yaw, 5);

  SerialBT.println("===== PID =====");
  SerialBT.print("P_R/P: "); SerialBT.println(P_ertek_Roll, 5);
  SerialBT.print("I_R/P: "); SerialBT.println(I_ertek_Roll, 5);
  SerialBT.print("D_R/P: "); SerialBT.println(D_ertek_Roll, 5);

  SerialBT.print("P_Y: "); SerialBT.println(P_ertek_Yaw, 5);
  SerialBT.print("I_Y: "); SerialBT.println(I_ertek_Yaw, 5);
  SerialBT.print("D_Y: "); SerialBT.println(D_ertek_Yaw, 5);
}



void BT_PID_parancs_feldolgozas(String parancs) {
  parancs.trim();

  if (parancs == "SAVE") {
    SerialBT.println("SAVE most kikapcsolva, mert resetet okoz.");
    Serial.println("SAVE kikapcsolva.");
    return;
  }

  if (parancs == "LOAD") {
    SerialBT.println("LOAD most kikapcsolva, mert resetet okoz.");
    Serial.println("LOAD kikapcsolva.");
    return;
  }

  if (parancs == "PRINT") {
    PID_ertekek_kiirasa();
    return;
  }

  int eq = parancs.indexOf('=');
  if (eq < 0) return;

  String kulcs = parancs.substring(0, eq);
  float ertek = parancs.substring(eq + 1).toFloat();

if (kulcs == "PR") {
  P_ertek_Roll = ertek;
  P_ertek_Pitch = ertek;
  SerialBT.println("P Roll/Pitch beallitva.");
}
else if (kulcs == "IR") {
  I_ertek_Roll = ertek;
  I_ertek_Pitch = ertek;
  SerialBT.println("I Roll/Pitch beallitva.");
}
else if (kulcs == "DR") {
  D_ertek_Roll = ertek;
  D_ertek_Pitch = ertek;
  SerialBT.println("D Roll/Pitch beallitva.");
}
else if (kulcs == "PY") {
  P_ertek_Yaw = ertek;
  SerialBT.println("P Yaw beallitva.");
}
else if (kulcs == "IY") {
  I_ertek_Yaw = ertek;
  SerialBT.println("I Yaw beallitva.");
}
else if (kulcs == "DY") {
  D_ertek_Yaw = ertek;
  SerialBT.println("D Yaw beallitva.");
}
else {
  SerialBT.println("Ismeretlen kulcs.");
  return;
}
  PID_ertekek_kiirasa();
}

void BT_PID_kezelese_limitelt() {
  uint8_t karakter_limit = 20;

  while (SerialBT.available() && karakter_limit > 0) {
    karakter_limit--;

    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      if (BT_Buffer.length() > 0) {
        BT_PID_parancs_feldolgozas(BT_Buffer);
        BT_Buffer = "";
      }
    } else {
      BT_Buffer += c;

      if (BT_Buffer.length() > 40) {
        BT_Buffer = "";
        SerialBT.println("Tul hosszu parancs, torolve.");
      }
    }
  }
}

bool Tuning_Mod_Aktiv() {
  return (SWA_Allapot == 1 && Vevo_Ertekek[2] < 1030);
}

void setup() {
  Serial.begin(115200);

  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); //soros vevőnek
  ibus.begin(Serial2); //ibus inicializálása

  SerialBT.begin("ESP32_PID_TUNING");

  Serial.println("Bluetooth PID teszt indul.");
  Serial.println("Telefonon csatlakozz: ESP32_PID_TUNING");
  PID_ertekek_kiirasa(); 

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  Loop_idozito = micros();
}

void loop() {
  Csatorna_olvasas();

  bool Tuning_Mod = Tuning_Mod_Aktiv();

  if (Tuning_Mod) {
    digitalWrite(LED_BUILTIN, HIGH);

    BT_PID_kezelese_limitelt();

    varakozas_250Hz();
    return;
  }

  digitalWrite(LED_BUILTIN, LOW);

  // Normál mód, később ide jön:
  // gyro_adat_olvasas();
  // PID számítás
  // motor mixer
  // motor kimenet

  static uint32_t elozo_print = 0;
  if (millis() - elozo_print > 1000) {
    elozo_print = millis();

    Serial.println("===== NORMAL MOD =====");
    Serial.print("Gaz: "); Serial.println(Vevo_Ertekek[2]);
    Serial.print("SWA: "); Serial.println(Vevo_Ertekek[4]);
    Serial.print("SWA_Allapot: "); Serial.println(SWA_Allapot);

    Serial.print("P_R/P="); Serial.print(P_ertek_Roll, 5);
    Serial.print(" I_R/P="); Serial.print(I_ertek_Roll, 5);
    Serial.print(" D_R/P="); Serial.println(D_ertek_Roll, 5);

    Serial.print("P_Y="); Serial.print(P_ertek_Yaw, 5);
    Serial.print(" I_Y="); Serial.print(I_ertek_Yaw, 5);
    Serial.print(" D_Y="); Serial.println(D_ertek_Yaw, 5);
    Serial.println();
  }

  varakozas_250Hz();
}