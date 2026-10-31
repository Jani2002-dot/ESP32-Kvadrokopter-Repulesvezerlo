#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string.h>

// 64 bajtos, little-endian keret; CRC32 az elso 60 bajton.
enum USBTipus : uint8_t { USB_START = 1, USB_STAGE, USB_SAMPLE, USB_END, USB_STATUS, USB_TEXT };
struct USBKeret {
  uint32_t magic = 0x31565053;  // SPV1
  uint8_t tipus = 0, mod = 0;
  uint16_t szakasz = 0;
  uint32_t futas = 0, sorszam = 0;
  uint64_t ido_us = 0;
  uint32_t adat[9] = {};
  uint32_t crc = 0;
};
static_assert(sizeof(USBKeret) == 64, "USB keretmeret");
static_assert(offsetof(USBKeret, crc) == 60, "USB elrendezes");

TesztVezerles Teszt;
QueueHandle_t USB_Sor = nullptr;
TaskHandle_t USB_Kuldo = nullptr;
bool USB_Puffer_Hiba = false, USB_Vege_Var = false, USB_Cancel = false;
bool USB_Fuggo = false, USB_Elozo_Loop = false;
uint64_t USB_Utolso_Ping = 0, USB_Leallas_us = 0;
uint32_t USB_Futas = 0, USB_Mintak = 0, USB_Elveszett = 0, USB_Uzenet_Hiba = 0;
uint32_t USB_LM = 0, USB_CTM = 0, USB_C4050 = 0, USB_C4200 = 0, USB_O = 0;
USBKeret USB_Fuggo_Minta;

void USB_float(USBKeret &k, unsigned index, float value) {
  memcpy(&k.adat[index], &value, sizeof(value));
}

bool USB_sorba(const USBKeret &k) {
  return USB_Sor && xQueueSend(USB_Sor, &k, 0) == pdTRUE;
}

void USB_uzenet(const char *szoveg) {
  USBKeret k;
  k.tipus = USB_TEXT;
  strncpy((char *)k.adat, szoveg, sizeof(k.adat) - 1);
  if (!USB_sorba(k)) ++USB_Uzenet_Hiba;
}

uint32_t USB_crc32(const USBKeret &k) {
  uint32_t crc = 0xffffffff;
  const uint8_t *b = (const uint8_t *)&k;
  for (unsigned i = 0; i < 60; ++i) {
    crc ^= b[i];
    for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

void USB_kuldo_feladat(void *) {
  USBKeret k;
  for (;;) {
    if (xQueueReceive(USB_Sor, &k, portMAX_DELAY) != pdTRUE) continue;
    const uint8_t *b = (const uint8_t *)&k;
    k.crc = USB_crc32(k);
    size_t elkuldve = 0;
    while (elkuldve < sizeof(k)) {
      size_t n = Serial.write(b + elkuldve, sizeof(k) - elkuldve);
      elkuldve += n;
      if (!n) vTaskDelay(1);
    }
  }
}

void USB_naplo_inditas() {
  Serial.setRxBufferSize(512);
  Serial.begin(USB_BAUD);
  USB_Sor = xQueueCreate(512, sizeof(USBKeret));
  if (!USB_Sor || xTaskCreatePinnedToCore(USB_kuldo_feladat, "usb_naplo", 4096, nullptr, 1, &USB_Kuldo, 0) != pdPASS) {
    USB_Puffer_Hiba = true;
    return;
  }
  USB_uzenet("BOOT: kalibracio, majd vevo/gaz");
}

void USB_status() {
  USBKeret k;
  k.tipus = USB_STATUS; k.mod = Teszt.mod; k.futas = USB_Futas;
  k.sorszam = USB_Mintak; k.ido_us = esp_timer_get_time();
  k.adat[0] = Loop_Max_Munka_us; k.adat[1] = Ciklus_Ido_Max;
  k.adat[2] = Loop_Tulcsordulas; k.adat[3] = Ciklus_Ido_Max_Szamlalo;
  k.adat[4] = USB_Elveszett; k.adat[5] = USB_Uzenet_Hiba;
  k.adat[6] = (Teszt.fut ? 1 : 0) | (Teszt.retesz[0] ? 2 : 0) | (Teszt.retesz[1] ? 4 : 0) |
    (Monitor_Allapot ? 8 : 0) | (Failsafe ? 16 : 0) | (EStop_Allapot ? 32 : 0) |
    (Angle_Mode_Aktiv ? 64 : 0) | (Szenzor_Hiba ? 128 : 0) | (USB_Puffer_Hiba ? 256 : 0);
  k.adat[7] = USB_Sor ? uxQueueMessagesWaiting(USB_Sor) : 0;
  if (!USB_sorba(k)) ++USB_Uzenet_Hiba;
}

void USB_parancs(const char *s, uint64_t most) {
  if (!strcmp(s, "PING")) { USB_Utolso_Ping = most; return; }
  if (!strcmp(s, "STATUS")) { USB_status(); return; }
  if (!strcmp(s, "CANCEL")) { USB_Cancel = true; return; }
  TesztMod mod = NINCS;
  if (!strcmp(s, "TEST RATE ROLL")) mod = RATE_ROLL;
  if (!strcmp(s, "TEST RATE PITCH")) mod = RATE_PITCH;
  if (!strcmp(s, "TEST ANGLE ROLL")) mod = ANGLE_ROLL;
  if (!strcmp(s, "TEST ANGLE PITCH")) mod = ANGLE_PITCH;
  if (mod == NINCS) { USB_uzenet("ERROR ismeretlen parancs"); return; }
  if (Teszt.fut || USB_Vege_Var || Monitor_Allapot || Teszt.retesz[0] || Teszt.retesz[1]) {
    USB_uzenet("ERROR SWB KI, kar kozep, teszt vege"); return;
  }
  const TesztSzakasz *lista = mod >= ANGLE_ROLL ? Angle_Teszt : Rate_Teszt;
  size_t n = mod >= ANGLE_ROLL ? sizeof(Angle_Teszt) / sizeof(Angle_Teszt[0]) : sizeof(Rate_Teszt) / sizeof(Rate_Teszt[0]);
  if (!Teszt.beallit(mod, lista, n)) { USB_uzenet("ERROR ervenytelen tesztbeallitas"); return; }
  USB_Puffer_Hiba = false;
  USB_uzenet("READY: megfelelo mod, SWB KI->BE");
  USB_status();
}

void USB_parancsok(uint64_t most) {
  static char sor[48];
  static unsigned n = 0;
  static bool tulhosszu = false;
  for (unsigned limit = 0; limit < 32 && Serial.available(); ++limit) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (tulhosszu) USB_uzenet("ERROR tul hosszu parancs");
      else if (n) { sor[n] = 0; USB_parancs(sor, most); }
      n = 0; tulhosszu = false;
    } else if (!tulhosszu) {
      if (n < sizeof(sor) - 1) sor[n++] = c;
      else tulhosszu = true;
    }
  }
}

void USB_monitor_kezd(uint32_t most) {
  Ciklus_Ido = most - Elozo_Loop_Kezdet;
  Elozo_Loop_Kezdet = most;
  if (USB_Elozo_Loop) {
    if (Ciklus_Ido > Ciklus_Ido_Max) Ciklus_Ido_Max = Ciklus_Ido;
    if (Ciklus_Ido >= LOOP_SZAMLALO_US) ++Ciklus_Ido_Max_Szamlalo;
  }
  USB_Elozo_Loop = true;
  if (!USB_Fuggo) return;
  USB_Fuggo_Minta.adat[0] = Ciklus_Ido;
  if (Ciklus_Ido > USB_CTM) USB_CTM = Ciklus_Ido;
  if (Ciklus_Ido >= LOOP_SZAMLALO_US) ++USB_C4050;
  if (Ciklus_Ido >= LOOP_LED_US) ++USB_C4200;
  if (!USB_sorba(USB_Fuggo_Minta)) { ++USB_Elveszett; USB_Puffer_Hiba = true; }
  USB_Fuggo = false;
}

void USB_monitor_veg() {
  Loop_Munka_us = micros() - Loop_Timer;
  if (Loop_Munka_us > Loop_Max_Munka_us) Loop_Max_Munka_us = Loop_Munka_us;
  if (Loop_Munka_us > 4000) ++Loop_Tulcsordulas;
  if (USB_Fuggo) {
    USB_Fuggo_Minta.adat[1] = Loop_Munka_us;
    if (Loop_Munka_us > USB_LM) USB_LM = Loop_Munka_us;
    if (Loop_Munka_us > 4000) ++USB_O;
  }
}

void USB_kezdo_keretek() {
  ++USB_Futas;
  USB_Mintak = USB_Elveszett = USB_LM = USB_CTM = USB_C4050 = USB_C4200 = USB_O = 0;
  Loop_Max_Munka_us = Ciklus_Ido_Max = Loop_Tulcsordulas = Ciklus_Ido_Max_Szamlalo = 0;
  USBKeret k;
  k.tipus = USB_START; k.mod = Teszt.mod; k.futas = USB_Futas;
  k.szakasz = Teszt.darab; k.ido_us = Teszt.teljes_us;
  bool pitch = Teszt.tengely() == 1;
  float pid[] = {pitch ? P_Rate_Pitch : P_Rate_Roll, pitch ? I_Rate_Pitch : I_Rate_Roll,
    pitch ? D_Rate_Pitch : D_Rate_Roll, pitch ? P_Angle_Pitch : P_Angle_Roll,
    pitch ? I_Angle_Pitch : I_Angle_Roll, pitch ? D_Angle_Pitch : D_Angle_Roll,
    P_Rate_Yaw, I_Rate_Yaw, D_Rate_Yaw};
  for (unsigned i = 0; i < 9; ++i) USB_float(k, i, pid[i]);
  USB_sorba(k);
  for (size_t i = 0; i < Teszt.darab; ++i) {
    k = USBKeret(); k.tipus = USB_STAGE; k.mod = Teszt.mod; k.futas = USB_Futas;
    k.szakasz = i; k.ido_us = TesztVezerles::hossz(Teszt.szakaszok[i]);
    USB_float(k, 0, Teszt.szakaszok[i].sp);
    USB_sorba(k);
  }
}

void USB_vege_keret() {
  if (!USB_Vege_Var || USB_Fuggo) return;
  USBKeret k;
  k.tipus = USB_END; k.mod = Teszt.mod; k.futas = USB_Futas;
  k.sorszam = USB_Mintak; k.ido_us = USB_Leallas_us - Teszt.kezdet;
  k.adat[0] = USB_CTM; k.adat[1] = USB_LM; k.adat[2] = USB_C4050;
  k.adat[3] = USB_C4200; k.adat[4] = USB_O; k.adat[5] = USB_Elveszett;
  k.adat[6] = Teszt.ok; k.adat[7] = USB_Uzenet_Hiba;
  if (USB_sorba(k)) USB_Vege_Var = false;
}

void USB_teszt_frissit(uint64_t most) {
  static bool elozo_swb = true;
  USB_parancsok(most);
  TesztBemenet b = {};
  b.swb = Monitor_Allapot; b.angle = Angle_Mode_Aktiv; b.estop = EStop_Allapot;
  b.rx_ok = !Failsafe && IBUS_Kapcsolat_El; b.sensor_ok = !Szenzor_Hiba;
  b.usb_ok = USB_Kuldo && USB_Utolso_Ping && most - USB_Utolso_Ping < USB_HEARTBEAT_US && !USB_Vege_Var;
  if (!Teszt.fut && (!USB_Sor || uxQueueSpacesAvailable(USB_Sor) < Teszt.darab + 8)) b.usb_ok = false;
  b.cancel = USB_Cancel;
  for (unsigned i = 0; i < 2; ++i) b.kozepen[i] = Vevo_Ertekek[i] >= KAR_KOZEP_MIN && Vevo_Ertekek[i] <= KAR_KOZEP_MAX;
  Teszt.frissit(most, b);
  if (b.swb && !elozo_swb && !Teszt.fut && !Teszt.leallt) {
    if (Teszt.mod == NINCS) USB_uzenet("ERROR elobb TEST parancs");
    else if (b.angle != Teszt.angle()) USB_uzenet("ERROR nem megfelelo Rate/Angle mod");
    else if (!b.rx_ok || b.estop || !b.sensor_ok) USB_uzenet("ERROR EStop/failsafe/MPU hiba");
    else if (!b.usb_ok) USB_uzenet("ERROR USB nem kesz; SWB KI majd BE");
    else USB_uzenet("ERROR kar kozep, SWB KI majd BE");
  }
  elozo_swb = b.swb;
  USB_Cancel = false;
  if (Teszt.indult) USB_kezdo_keretek();
  if (Teszt.fut && USB_Puffer_Hiba) Teszt.leallit(PUFFER_TELE);
  if (Teszt.leallt) { USB_Leallas_us = most; USB_Vege_Var = true; }
  USB_vege_keret();
}

void USB_alapjel_feluliras() {
  for (unsigned i = 0; i < 2; ++i) {
    bool teszt_tengely = Teszt.fut && Teszt.tengely() == i;
    if (!teszt_tengely && !Teszt.retesz[i]) continue;
    float sp = teszt_tengely ? Teszt.sp : 0.0f;
    if (Angle_Mode_Aktiv) {
      if (!i) Kivant_Angle_Roll = sp; else Kivant_Angle_Pitch = sp;
    } else {
      if (!i) Kivant_Roll_Rate = sp; else Kivant_Pitch_Rate = sp;
    }
  }
}

void USB_mintavetel(uint64_t most) {
  if (!Teszt.fut) return;
  USBKeret &k = USB_Fuggo_Minta;
  k = USBKeret(); k.tipus = USB_SAMPLE; k.mod = Teszt.mod; k.futas = USB_Futas;
  k.szakasz = Teszt.szakasz; k.sorszam = USB_Mintak++; k.ido_us = most - Teszt.kezdet;
  bool pitch = Teszt.tengely() == 1;
  float sp = Teszt.angle() ? (pitch ? Kivant_Angle_Pitch : Kivant_Angle_Roll) : (pitch ? Kivant_Pitch_Rate : Kivant_Roll_Rate);
  float pv = Teszt.angle() ? (pitch ? Kalman_Angle_Pitch : Kalman_Angle_Roll) : (pitch ? Szurt_Pitch_Rate : Szurt_Roll_Rate);
  USB_float(k, 2, sp); USB_float(k, 3, pv);
  k.adat[4] = Vevo_Ertekek[2]; k.adat[5] = Iterm_Aktiv ? 1 : 0;
  USB_Fuggo = true;
}
