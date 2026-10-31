#define feszultseg_meres_PIN 36
#define aram_meres_PIN 39
#define Piros_LED 13
#define Zold_LED 4

#pragma region 08_Akkumulator_meres
float Feszultseg;
float Aram;
uint32_t Aksi_meres;
float Aksi_maradek;                 // %-ban
float Aksi_kezdetben;               // mAh-ban
float Elfogyasztott_kapacitas = 0;  // mAh-ban
float Alap_aksi_kapacitas = 2000;   // 2000mAh
#pragma endregion

void ADC_meres(void) {
  Feszultseg = analogReadMilliVolts(feszultseg_meres_PIN) / 1000.0f * 6.0f;  // Feszültség [V]
  Aram = analogReadMilliVolts(aram_meres_PIN) / 30.0f;                       // Áram [A]
}

void setup() {

  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(feszultseg_meres_PIN, ADC_11db);
  analogSetPinAttenuation(aram_meres_PIN, ADC_11db);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Piros_LED, OUTPUT);
  pinMode(Zold_LED, OUTPUT);

  // Indulasi akkufeszultseg meghatarozasa 100 meres atlagabol (multisampling)
  float Feszultseg_Osszeg = 0.0f;

  for (int j = 0; j < 100; j++) {
    ADC_meres();
    Feszultseg_Osszeg += Feszultseg;
    delay(1);
  }

  Feszultseg = Feszultseg_Osszeg / 100.0f;

  digitalWrite(Zold_LED, HIGH);
}

void loop() {

  //Aksi mérés
  if (millis() - Aksi_meres >= 100) {
    Aksi_meres = millis();
    //ADC_meres();

    Serial.print("V: ");
    Serial.print(Feszultseg, 4);
    Serial.print(" A: ");
    Serial.println(Aram, 4);
  }
}
