#define RX_PIN 16
#define TX_PIN -1

#pragma region 03_IBUS_es_Failsafe
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
uint32_t Debug_Ido = 0;
#pragma endregion

void IBUS_Olvasas(void) {
  while (Serial2.available() > 0) {
    uint8_t b = Serial2.read();

    // Keretkezdet: 0x20, majd 0x40.
    if (IBUS_Index == 0) {
      if (b != 0x20) continue;

      IBUS_Buffer[0] = b;
      IBUS_Index = 1;
      continue;
    }

    if (IBUS_Index == 1 && b != 0x40) {
      // Egy újabb 0x20 már a következő keret kezdete lehet.
      IBUS_Index = (b == 0x20) ? 1 : 0;
      continue;
    }

    IBUS_Buffer[IBUS_Index++] = b;

    // A részleges keret megmarad a következő meghívásig.
    if (IBUS_Index < 32) continue;
    IBUS_Index = 0;

    // Az első 30 bájtból számított ellenőrző összeg.
    uint16_t Checksum = 0xFFFF;

    for (uint8_t i = 0; i < 30; i++) {
      Checksum -= IBUS_Buffer[i];
    }

    // A keret végén az alsó helyiértékű bájt érkezik előbb.
    uint16_t Kapott_Checksum =
      (uint16_t)IBUS_Buffer[30] | ((uint16_t)IBUS_Buffer[31] << 8);

    if (Checksum != Kapott_Checksum) continue;

    // A hat használt csatorna ellenőrzése az átvétel előtt.
    uint16_t Temp[6];
    bool Ervenyes = true;

    for (uint8_t i = 0; i < 6; i++) {
      uint8_t also = 2 + 2 * i;

      Temp[i] = (uint16_t)IBUS_Buffer[also] | ((uint16_t)IBUS_Buffer[also + 1] << 8);

      if (Temp[i] < 900 || Temp[i] > 2100) {
        Ervenyes = false;
      }
    }

    if (!Ervenyes) continue;

    // Csak akkor frissítem a csatornákat, ha mind megfelelő.
    for (uint8_t i = 0; i < 6; i++) {
      Vevo_Ertekek[i] = Temp[i];
    }

    // Csak elfogadott keret frissíti a kapcsolat állapotát.
    Utolso_IBUS_Csomag_Ido = millis();
    IBUS_Kapcsolat_El = true;
    Failsafe = false;

    // SWA: motorleállítási kérés, hiszterézissel.
    if (Vevo_Ertekek[4] > 1600) {
      EStop_Allapot = true;
    } else if (Vevo_Ertekek[4] < 1400) {
      EStop_Allapot = false;
    }

    // SWD: RATE / ANGLE módválasztás, hiszterézissel.
    if (Vevo_Ertekek[5] > 1600) {
      Angle_Mode_Aktiv = true;
    } else if (Vevo_Ertekek[5] < 1400) {
      Angle_Mode_Aktiv = false;
    }
  }
}

void IBUS_Failsafe_Ellenorzes(void) {
  uint32_t most = millis();

  // Saját időkorlát: több mint 100 ms érvényes keret nélkül.
  if ((uint32_t)(most - Utolso_IBUS_Csomag_Ido) > 100U) {
    IBUS_Kapcsolat_El = false;
    Failsafe = true;
  }

  if (Failsafe) {
    Vevo_Ertekek[0] = 1500;  // Roll: középállás.
    Vevo_Ertekek[1] = 1500;  // Pitch: középállás.
    Vevo_Ertekek[2] = 1000;  // Gáz: minimum.
    Vevo_Ertekek[3] = 1500;  // Yaw: középállás.
    Vevo_Ertekek[4] = 1000;  // SWA alapérték.
    Vevo_Ertekek[5] = 1000;  // SWD: RATE mód.

    EStop_Allapot = false;
    Angle_Mode_Aktiv = false;

    // A motorok leállítását és az ARM törlését
    // a főciklus Failsafe feltételű ága végzi.
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(RX_PIN, INPUT_PULLUP);

  Serial2.setRxBufferSize(512);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

}

void loop() {

  IBUS_Olvasas();
  IBUS_Failsafe_Ellenorzes();

  if (millis() - Debug_Ido >= 50) {
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
    Serial.print(EStop_Allapot);

    Serial.print(" SWD: ");
    Serial.print(Angle_Mode_Aktiv);

    Serial.print(" Fail: ");
    Serial.print(Failsafe);

    Serial.print(" Kapcs: ");
    Serial.println(IBUS_Kapcsolat_El);

  }
}