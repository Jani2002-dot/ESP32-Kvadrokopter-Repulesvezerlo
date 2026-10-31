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