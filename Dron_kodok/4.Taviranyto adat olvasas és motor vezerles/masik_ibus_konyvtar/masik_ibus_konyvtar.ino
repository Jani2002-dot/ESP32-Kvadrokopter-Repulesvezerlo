#include <FlyskyIBUS.h>

#define RX_PIN 16
#define TX_PIN -1

FlyskyIBUS IBUS;

uint16_t Vevo_Ertekek[5] = {1500, 1500, 1000, 1500, 1000};
bool Failsafe = true;
bool SWA_Allapot = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  //pinMode(RX_PIN, INPUT_PULLUP);

  IBUS.begin();

  Serial.println("FlyskyIBUS indul");
}

void loop() {
  Vevo_Ertekek[0] = IBUS.getChannel(0);
  Vevo_Ertekek[1] = IBUS.getChannel(1);
  Vevo_Ertekek[2] = IBUS.getChannel(2);
  Vevo_Ertekek[3] = IBUS.getChannel(3);
  Vevo_Ertekek[4] = IBUS.getChannel(4);

  Failsafe = IBUS.isFailsafe();

  if (Failsafe) {
    Vevo_Ertekek[0] = 1500;
    Vevo_Ertekek[1] = 1500;
    Vevo_Ertekek[2] = 1000;
    Vevo_Ertekek[3] = 1500;
    Vevo_Ertekek[4] = 1000;
  }

  SWA_Allapot = Vevo_Ertekek[4] > 1500;

  static uint32_t Debug_Ido = 0;
  if (millis() - Debug_Ido >= 500) {
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
    Serial.println(Failsafe);
  }

  delay(4);
}