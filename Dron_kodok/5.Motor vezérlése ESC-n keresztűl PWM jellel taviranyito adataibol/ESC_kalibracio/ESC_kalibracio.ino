#pragma region 01_Pinek_es_PWM
#define RX_PIN 16
#define TX_PIN -1

#define Motorok_ESC_PWM_felbontas 12  //12 bit ---> 0-4095

#define Motorok_ESC_PWM_frekvencia 250  //250Hz=4000us

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
#pragma endregion

#pragma region 02_IBUS_es_Failsafe
uint8_t IBUS_Buffer[32];
uint8_t IBUS_Index = 0;

uint32_t Utolso_IBUS_Csomag_Ido = 0;

bool IBUS_Kapcsolat_El = false;
bool Failsafe = true;

uint16_t Vevo_Ertekek[] = { 1500, 1500, 1000, 1500, 1000, 1000 };
bool EStop_Allapot = false;
bool Angle_Mode_Aktiv = false;
bool Elozo_Angle_Mode_Aktiv = false;
bool EStop_LED = false;
bool Biztonsagos_inditas = false;
uint32_t Arm_Tartas_Kezdete = 0;
bool Arm_Tartas_Folyamatban = false;
#pragma endregion

#pragma region 03_Motorvaltozok_es_Loop_valtozok
float Motor_1_bemenet, Motor_2_bemenet, Motor_3_bemenet, Motor_4_bemenet;
float Bemenet_Roll, Bemenet_Pitch, Bemenet_Yaw, Bemenet_Gaz;
int Motorok_alapjarat = 1180;
int Motorok_leallitas = 1000;

// 250 Hz-es loop vizsgálatához
uint32_t Loop_Timer;
uint32_t Ciklus_Ido = 0;
uint32_t Ciklus_Ido_Max = 0;
uint32_t Elozo_Loop_Kezdet = 0;
uint32_t Piros_Blink = 0;
bool Piros_LED_State = false;
float Ts = 0.004;  //250 Hz-es loop miatt 4ms
#pragma endregion

#pragma region 04_Akkumulator_meres
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

void IBUS_Olvasas(void) {  // Paraméter és visszatérési érték nélküli függvény az iBUS-adatok feldolgozásához.

  while (Serial2.available()) {  // Addig ismétel, amíg van kiolvasható bájt a Serial2 vételi pufferében.
    uint8_t b = Serial2.read();  // Kiolvassa a következő bájtot, és eltárolja egy 8 bites, előjel nélküli változóban.

    if (IBUS_Index == 0) {  // Ha még nincs meg a keret első bájtja, a keretkezdetet keresi.
      if (b != 0x20) {      // Ellenőrzi, hogy a bájt eltér-e az elvárt 0x20 értéktől, amely 32-t jelent.
        continue;           // Ezt a bájtot eldobja, és a while következő körében a következő rendelkezésre álló bájttal folytatja.
      }                     // Az első bájt értékének ellenőrzése véget ér.
    }                       // A keret első bájtjához tartozó feltételes rész vége.

    if (IBUS_Index == 1) {  // Ha az első bájt már megvan, a második bájtot ellenőrzi.
      if (b != 0x40) {      // A második bájtnak 0x40 értékűnek kell lennie.
        IBUS_Index = 0;     // Hibás második bájtnál elölről kezdi a keret keresését.
        continue;           // A jelenlegi bájtot eldobja, és a következő rendelkezésre álló bájttal folytatja.
      }                     // A második bájt értékének ellenőrzése véget ér.
    }                       // A keret második bájtjához tartozó feltételes rész vége.

    IBUS_Buffer[IBUS_Index++] = b;  // Eltárolja a bájtot az aktuális indexen, majd eggyel növeli az indexet.

    if (IBUS_Index >= 32) {  // Ha összegyűlt a teljes, 32 bájtos keret, megkezdi annak ellenőrzését.
      IBUS_Index = 0;        // A következő keret fogadásához visszaállítja a puffer indexét; a puffer tartalmát nem törli.

      uint16_t Checksum = 0xFFFF;  // Az ellenőrző összeg kezdőértéke 65535, 16 bites előjel nélküli változóban.

      for (uint8_t i = 0; i < 30; i++) {  // Végigmegy a keret első 30 bájtján, a 0–29. indexeken.
        Checksum -= IBUS_Buffer[i];       // Az aktuális bájt értékét kivonja az ellenőrző összegből.
      }                                   // A két záró ellenőrző bájt kimarad a számításból.

      uint16_t Kapott_Checksum = IBUS_Buffer[30] | (IBUS_Buffer[31] << 8);
      // A 30. indexű bájt az alsó, a 31. indexű a felső helyiértékű bájt.
      // A << 8 a felső bájtot nyolc bittel balra tolja, a | bitenkénti VAGY pedig összekapcsolja a két bájtot.

      if (Checksum == Kapott_Checksum) {  // Csak egyező számított és kapott ellenőrző összegnél dolgozik tovább.
        uint16_t Temp[6];                 // Ideiglenes tömb hat darab 16 bites csatornaérték tárolására.

        Temp[0] = IBUS_Buffer[2] | (IBUS_Buffer[3] << 8);    // CH1: a roll-parancs összeállítása az alsó és felső bájtból.
        Temp[1] = IBUS_Buffer[4] | (IBUS_Buffer[5] << 8);    // CH2: a pitch-parancs összeállítása.
        Temp[2] = IBUS_Buffer[6] | (IBUS_Buffer[7] << 8);    // CH3: a gázparancs összeállítása.
        Temp[3] = IBUS_Buffer[8] | (IBUS_Buffer[9] << 8);    // CH4: a yaw-parancs összeállítása.
        Temp[4] = IBUS_Buffer[10] | (IBUS_Buffer[11] << 8);  // CH5: az SWA leállítókapcsoló csatornaértékének összeállítása.
        Temp[5] = IBUS_Buffer[12] | (IBUS_Buffer[13] << 8);  // CH6: az SWD RATE/ANGLE módválasztó csatornaértékének összeállítása.

        bool Ervenyes = true;  // Kiindulásként megfelelőnek tekinti a csatornaértékeket; bármelyik hibás érték ezt felülírja.

        if (Temp[0] < 900 || Temp[0] > 2100) Ervenyes = false;  // Érvénytelen, ha a CH1 értéke 900 alatti VAGY 2100 feletti.
        if (Temp[1] < 900 || Temp[1] > 2100) Ervenyes = false;  // Ugyanezt a tartományellenőrzést elvégzi a CH2 értékén.
        if (Temp[2] < 900 || Temp[2] > 2100) Ervenyes = false;  // Ellenőrzi a CH3, azaz a gázcsatorna értékét.
        if (Temp[3] < 900 || Temp[3] > 2100) Ervenyes = false;  // Ellenőrzi a CH4, azaz a yaw-csatorna értékét.
        if (Temp[4] < 900 || Temp[4] > 2100) Ervenyes = false;  // Ellenőrzi a CH5, azaz az SWA csatorna értékét.
        if (Temp[5] < 900 || Temp[5] > 2100) Ervenyes = false;  // Ellenőrzi a CH6, azaz az SWD csatorna értékét.

        if (Ervenyes) {               // Csak akkor veszi át az adatokat, ha mind a hat  érték a megengedett tartományon belül van.
          Vevo_Ertekek[0] = Temp[0];  // Frissíti a program által használt roll-parancsot.
          Vevo_Ertekek[1] = Temp[1];  // Frissíti a pitch-parancsot.
          Vevo_Ertekek[2] = Temp[2];  // Frissíti a gázparancsot.
          Vevo_Ertekek[3] = Temp[3];  // Frissíti a yaw-parancsot.
          Vevo_Ertekek[4] = Temp[4];  // Frissíti az SWA leállítókapcsoló csatornaértékét.
          Vevo_Ertekek[5] = Temp[5];  // Frissíti az SWD módválasztó csatornaértékét.

          Utolso_IBUS_Csomag_Ido = millis();  // Eltárolja az érvényes csomag feldolgozásának időpontját milliszekundumban.

          IBUS_Kapcsolat_El = true;  // Jelzi, hogy érkezett elfogadott iBUS-csomag.
          Failsafe = false;          // Törli a failsafe jelzőt; az ARM-engedélyt ez önmagában nem állítja vissza.

          if (Vevo_Ertekek[4] > 1600) {         // Ha az SWA csatorna értéke meghaladja az 1600-at:
            EStop_Allapot = true;               // Aktiválja a leállítási kérést; a motorokat a főciklus leállítási ága kezeli.
          } else if (Vevo_Ertekek[4] < 1400) {  // Egyébként, ha az SWA csatorna értéke 1400 alatti:
            EStop_Allapot = false;              // Megszünteti a kapcsolóból származó leállítási kérést.
          }                                     // 1400 és 1600 között, a határokat is beleértve, az EStop_Allapot nem változik.

          if (Vevo_Ertekek[5] > 1600) {         // Ha az SWD csatorna értéke meghaladja az 1600-at:
            Angle_Mode_Aktiv = true;            // Kiválasztja az ANGLE üzemmódot.
          } else if (Vevo_Ertekek[5] < 1400) {  // Egyébként, ha az SWD csatorna értéke 1400 alatti:
            Angle_Mode_Aktiv = false;           // Kikapcsolja az ANGLE módot, így a főciklus RATE módot használ.
          }                                     // 1400 és 1600 között az előző üzemmód marad érvényben.

        }  // Az érvényes csatornaértékek átvételének és a kapcsolók kiértékelésének vége.
      }    // Az egyező ellenőrző összeg esetén végrehajtott rész vége.
    }      // A teljes, 32 bájtos keret feldolgozásának vége.
  }        // A vételi puffer feldolgozásának vége; üres puffernél kilép a while ciklusból.
}  // A függvény visszatér oda, ahonnan meghívták.

void IBUS_Failsafe_Ellenorzes(void) {  // Paraméter és visszatérési érték nélküli függvény az iBUS-kapcsolat időtúllépésének ellenőrzésére.
  uint32_t most = millis();            // Eltárolja az aktuális időt milliszekundumban, 32 bites előjel nélküli változóban.

  if (most - Utolso_IBUS_Csomag_Ido > 300) {  // Ha az utolsó elfogadott csomag feldolgozása óta több mint 100 ms telt el:
    IBUS_Kapcsolat_El = false;                // A kapcsolatot inaktívnak jelöli.
    Failsafe = true;                          // Aktiválja a failsafe állapotot, amelyet a főciklus motorleállítási feltételként használ.
  }                                           // Az időtúllépés ellenőrzésének vége; itt nincs automatikus visszaállítás, azt az érvényes csomag fogadása végzi.

  if (Failsafe) {              // Aktív failsafe esetén alapértékekre állítja a csatornákat és az üzemmódjelzőket.
    Vevo_Ertekek[0] = 1500;    // CH1: a roll-parancsot középállásra állítja.
    Vevo_Ertekek[1] = 1500;    // CH2: a pitch-parancsot középállásra állítja.
    Vevo_Ertekek[2] = 1000;    // CH3: a gázparancsot minimumra állítja.
    Vevo_Ertekek[3] = 1500;    // CH4: a yaw-parancsot középállásra állítja.
    Vevo_Ertekek[4] = 1000;    // CH5: az SWA csatornához az inaktív leállítókapcsolónak megfelelő értéket rendeli.
    Vevo_Ertekek[5] = 1000;    // CH6: az SWD csatornához a RATE módnak megfelelő értéket rendeli.
    EStop_Allapot = false;     // Törli a kapcsolóból származó leállítási jelzőt; a Failsafe ettől még aktív marad.
    Angle_Mode_Aktiv = false;  // Kikapcsolja az ANGLE mód jelzőjét; ez nem engedélyezi a motorokat.
  }                            // A failsafe esetén végrehajtott alaphelyzetbe állítás vége.
}  // Visszatér a hívó programrészhez; a motorok leállítását, az ARM és a PID-állapotok törlését a főciklus végzi.

uint32_t usToDuty(float motor_bemenet_us) {
  motor_bemenet_us = constrain(motor_bemenet_us, 1000, 2000);
  return (uint32_t)(motor_bemenet_us * 4095.0f / 4000.0f);
}

void setup() {

  //Serial.begin(115200);
  pinMode(RX_PIN, INPUT_PULLUP);

  Serial2.setRxBufferSize(512);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Piros_LED, OUTPUT);
  pinMode(Zold_LED, OUTPUT);

  digitalWrite(Piros_LED, HIGH);
  digitalWrite(Zold_LED, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  //Motor 1 (Jobb hátsó CW)
  ledcSetup(Motor_1_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_1_ESC_Pin, Motor_1_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //Motor 2 (Jobb első CCW)
  ledcSetup(Motor_2_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_2_ESC_Pin, Motor_2_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //Motor 3 (Bal hátsó CCW)
  ledcSetup(Motor_3_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_3_ESC_Pin, Motor_3_ESC_Csatorna);                                    //ESP32 core 2.0.17

  //Motor 4 (Bal első CW)
  ledcSetup(Motor_4_ESC_Csatorna, Motorok_ESC_PWM_frekvencia, Motorok_ESC_PWM_felbontas);  //ESP32 core 2.0.17
  ledcAttachPin(Motor_4_ESC_Pin, Motor_4_ESC_Csatorna);


  // 250Hz=4ms=4000us
  // 12 bites felbontás = 0-4095
  // 0 ---> 0us        4095--->4000us
  // PWM jel küldése us-ban az ESC nek akkor 4095/4000=1.024-el kell majd szorozi 250Hz esetén
  // PWM jel küldése us-ban az ESC nek akkor 65 535/20 000=3.27675-el kell majd szorozi 50 Hz esetén
  delay(250);

  digitalWrite(Piros_LED, LOW);
  digitalWrite(Zold_LED, HIGH);

  Loop_Timer = micros();
}

void loop() {

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  Bemenet_Gaz = (Failsafe || EStop_Allapot) ? 1000.0f : Vevo_Ertekek[2];

  ledcWrite(Motor_1_ESC_Csatorna, usToDuty(Bemenet_Gaz));
  ledcWrite(Motor_2_ESC_Csatorna, usToDuty(Bemenet_Gaz));
  ledcWrite(Motor_3_ESC_Csatorna, usToDuty(Bemenet_Gaz));
  ledcWrite(Motor_4_ESC_Csatorna, usToDuty(Bemenet_Gaz));

  if (millis() - Aksi_meres >= 100) {
    Aksi_meres = millis();

    ADC_meres();

    /*Serial.print("Gaz_us: ");
    Serial.print(Bemenet_Gaz, 0);

    Serial.print("; F: ");
    Serial.print(Failsafe);

    Serial.print("; U_raw: ");
    Serial.print(analogRead(feszultseg_meres_PIN));
    Serial.print("; U_mV: ");
    Serial.print(analogReadMilliVolts(feszultseg_meres_PIN));

    Serial.print("; I_raw: ");
    Serial.print(analogRead(aram_meres_PIN));
    Serial.print("; I_mV: ");
    Serial.println(analogReadMilliVolts(aram_meres_PIN));*/
  }

  while (micros() - Loop_Timer < 4000)
    ;
  Loop_Timer = micros();
}