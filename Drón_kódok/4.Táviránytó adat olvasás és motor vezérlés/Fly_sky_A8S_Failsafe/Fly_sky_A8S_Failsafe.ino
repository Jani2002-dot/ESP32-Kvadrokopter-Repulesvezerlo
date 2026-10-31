#include <IBusBM.h>
IBusBM ibus;

//ibus.readChannel(0-9) 10 csatorna
#define RX_PIN 16
#define TX_PIN 25

//static const uint8_t Csatorna_Szam = 8;
//static azt jelenti, hogy ez a változó “fájl-szintű / globális” élettartamú, vagyis a program teljes futása alatt létezik, és ugyanaz az egy példánya van.
//csak ebben a .cpp fájlban látható (internal linkage). Tehát ha több fájlod van, másik .cpp-ből nem tudod elérni név szerint.
uint16_t Vevo_Ertekek[] = { 0, 0, 0, 0, 0 };
bool SWA_Allapot;

uint32_t Utolso_IBUS_Csomag_Ido = 0;
uint8_t Elozo_IBUS_Csomag_Szam = 0;

bool IBUS_Kapcsolat_El = false;
bool Failsafe = true;

// 250 Hz-es loop vizsgálatához
uint32_t Loop_idozito;
uint32_t Loop_Start_Us;
uint32_t Loop_Start_Elozo_Us;
uint32_t Loop_Ido_Us;
uint32_t Debug_Ido;


void IBUS_Failsafe_Ellenorzes() {
  uint32_t most = millis();

  // Ha érkezett új iBUS csomag, akkor frissítjük az időt
  if (ibus.cnt_rec != Elozo_IBUS_Csomag_Szam) {
    Elozo_IBUS_Csomag_Szam = ibus.cnt_rec;
    Utolso_IBUS_Csomag_Ido = most;
    IBUS_Kapcsolat_El = true;
  }

  // Ha 100 ms óta nincs új iBUS csomag, akkor kapcsolatvesztés
  if (most - Utolso_IBUS_Csomag_Ido > 100) {
    IBUS_Kapcsolat_El = false;
  }

  // Alapból nincs failsafe
  Failsafe = false;

  // Ha nincs élő iBUS kapcsolat
  if (!IBUS_Kapcsolat_El) {
    Failsafe = true;
  }

  // Ha a fontos csatornák értéke teljesen hülye
  if (Vevo_Ertekek[0] < 900 || Vevo_Ertekek[0] > 2100) Failsafe = true;  // Roll
  if (Vevo_Ertekek[1] < 900 || Vevo_Ertekek[1] > 2100) Failsafe = true;  // Pitch
  if (Vevo_Ertekek[2] < 900 || Vevo_Ertekek[2] > 2100) Failsafe = true;  // Throttle
  if (Vevo_Ertekek[3] < 900 || Vevo_Ertekek[3] > 2100) Failsafe = true;  // Yaw
  if (Vevo_Ertekek[4] < 900 || Vevo_Ertekek[4] > 2100) Failsafe = true;  // Arm kapcsoló
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

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  ibus.begin(Serial2);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  Loop_Start_Us = micros();
  if (Loop_Start_Elozo_Us != 0) Loop_idozito = Loop_Start_Us - Loop_Start_Elozo_Us;
  Loop_Start_Elozo_Us = Loop_Start_Us;
  // olvasás + print
  Csatorna_olvasas();
  IBUS_Failsafe_Ellenorzes();

  if (Failsafe) Vevo_Ertekek[2] = 1000;


  if (millis() - Debug_Ido >= 1000) {
    Debug_Ido = millis();

    Serial.print("Roll: ");
    Serial.print(Vevo_Ertekek[0]);

    Serial.print(" Pitch: ");
    Serial.print(Vevo_Ertekek[1]);

    Serial.print(" Gaz: ");
    Serial.print(Vevo_Ertekek[2]);

    Serial.print(" Yaw: ");
    Serial.print(Vevo_Ertekek[3]);

    Serial.print(" SWA: ");
    Serial.print(SWA_Allapot);

    Serial.print(" Fail: ");
    Serial.print(Failsafe);

    Serial.print(" szam: ");
    Serial.print(Elozo_IBUS_Csomag_Szam);
    
    Serial.print(" Loop: ");
    Serial.println(Loop_idozito);
  }

  Loop_Ido_Us = micros() - Loop_Start_Us;

  if (Loop_Ido_Us < 4000) {
    delayMicroseconds(4000 - Loop_Ido_Us);
  }
}