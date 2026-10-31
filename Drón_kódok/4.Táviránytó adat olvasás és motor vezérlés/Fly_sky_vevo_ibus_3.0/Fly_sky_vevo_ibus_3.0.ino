#include <IBusBM.h>
IBusBM ibus;

//ibus.readChannel(0-9) 10 csatorna
#define RX_PIN 16
#define TX_PIN -1

//static const uint8_t Csatorna_Szam = 8;
//static azt jelenti, hogy ez a változó “fájl-szintű / globális” élettartamú, vagyis a program teljes futása alatt létezik, és ugyanaz az egy példánya van.
//csak ebben a .cpp fájlban látható (internal linkage). Tehát ha több fájlod van, másik .cpp-ből nem tudod elérni név szerint.
uint16_t Vevo_Ertekek[]={0,0,0,0,0};
bool SWA_Allapot;

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

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  ibus.begin(Serial2);
}

void loop() {
    // olvasás + print
    Csatorna_olvasas();

    Serial.print("Roll [µs]: ");
    Serial.print(Vevo_Ertekek[0]);
    Serial.print(" Pitch [µs]: ");
    Serial.print(Vevo_Ertekek[1]);
    Serial.print(" Throttle [µs]: ");
    Serial.print(Vevo_Ertekek[2]);
    Serial.print(" Yaw [µs]: ");
    Serial.print(Vevo_Ertekek[3]);
    Serial.print(" SWA: ");
    Serial.println(SWA_Allapot);
    /*Serial.print(" Vmi_2 [µs]: ");
    Serial.print(Vevo_Ertekek[5]);
    Serial.print(" Vmi_3 [µs]: ");
    Serial.print(Vevo_Ertekek[6]);
    Serial.print(" Vmi_4 [µs]: ");
    Serial.print(Vevo_Ertekek[7]);
    Serial.print(" Vmi_5 [µs]: ");
    Serial.print(Vevo_Ertekek[8]);
    Serial.print(" Vmi_6 [µs]: ");
    Serial.println(Vevo_Ertekek[9]);*/      
  delay(15);  
}
