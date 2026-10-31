#pragma once
#include "Teszt_beallitasok.h"
#include <string.h>
#include <stdlib.h>

struct RAM_Minta {
  float pv;
  uint32_t ciklus_us, munka_us;
};
static_assert(sizeof(RAM_Minta) == 12, "RAM-minta meret");
// Egyszeri foglalas BT-inditas utan, kis blokkokban; meres kozben nincs malloc.
struct RAM_Tarolo {
  static constexpr uint32_t BLOKK = 500;
  static constexpr uint32_t DARAB = (RAM_MAX_MINTA + BLOKK - 1) / BLOKK;
  RAM_Minta *blokkok[DARAB] = {};
  bool kesz = false;
  bool foglal() {
    if (kesz) return true;
    for (uint32_t i = 0; i < DARAB; ++i) {
      uint32_t n = RAM_MAX_MINTA - i * BLOKK;
      if (n > BLOKK) n = BLOKK;
      blokkok[i] = (RAM_Minta *)malloc(n * sizeof(RAM_Minta));
      if (!blokkok[i]) {
        for (uint32_t j = 0; j < i; ++j) { free(blokkok[j]); blokkok[j] = nullptr; }
        return false;
      }
    }
    kesz = true; return true;
  }
  RAM_Minta &operator[](uint32_t i) { return blokkok[i / BLOKK][i % BLOKK]; }
} RAM_Adatok;
TesztVezerles Teszt;
uint32_t RAM_Darab = 0;
int32_t RAM_Fuggo = -1;
bool RAM_Elokeszitve = false, RAM_Kiadas_Zar = false, RAM_Elozo_Merendo = false;
uint32_t RAM_CTM = 0, RAM_LM = 0, RAM_C4050 = 0, RAM_C4200 = 0, RAM_O = 0;
float RAM_PID[9] = {};
const char *RAM_Ok = "ures";
uint32_t RAM_Szabad_Indulas = 0;

const char *RAM_mod_nev() {
  switch (Teszt.mod) {
    case RATE_ROLL: return "RATE_ROLL";
    case RATE_PITCH: return "RATE_PITCH";
    case ANGLE_ROLL: return "ANGLE_ROLL";
    case ANGLE_PITCH: return "ANGLE_PITCH";
    default: return "NONE";
  }
}

const char *RAM_ok_nev(TesztOk ok) {
  switch (ok) {
    case KESZ: return "kesz";
    case SWB_KI: return "swb_ki";
    case ESTOP: return "estop";
    case JELVESZTES: return "failsafe";
    case MODVALTAS: return "modvaltas";
    case PUFFER_TELE: return "puffer_tele";
    case SZENZOR_HIBA: return "mpu_hiba";
    default: return "megszakitas";
  }
}

void RAM_monitor_kezd(uint32_t most) {
  Ciklus_Ido = most - Elozo_Loop_Kezdet;
  Elozo_Loop_Kezdet = most;
  if (RAM_Elozo_Merendo) {
    if (Ciklus_Ido > Ciklus_Ido_Max) Ciklus_Ido_Max = Ciklus_Ido;
    if (Ciklus_Ido >= LOOP_SZAMLALO_US) ++Ciklus_Ido_Max_Szamlalo;
  }
  RAM_Elozo_Merendo = false;
  if (RAM_Fuggo < 0) return;
  RAM_Adatok[RAM_Fuggo].ciklus_us = Ciklus_Ido;
  if (Ciklus_Ido > RAM_CTM) RAM_CTM = Ciklus_Ido;
  if (Ciklus_Ido >= LOOP_SZAMLALO_US) ++RAM_C4050;
  if (Ciklus_Ido >= LOOP_LED_US) ++RAM_C4200;
  RAM_Fuggo = -1;
}

void RAM_monitor_veg() {
  RAM_Elozo_Merendo = true;
  Loop_Munka_us = micros() - Loop_Timer;
  if (Loop_Munka_us > Loop_Max_Munka_us) Loop_Max_Munka_us = Loop_Munka_us;
  if (Loop_Munka_us > 4000) ++Loop_Tulcsordulas;
  if (RAM_Fuggo < 0) return;
  RAM_Adatok[RAM_Fuggo].munka_us = Loop_Munka_us;
  if (Loop_Munka_us > RAM_LM) RAM_LM = Loop_Munka_us;
  if (Loop_Munka_us > 4000) ++RAM_O;
}

void RAM_frissit(uint64_t most) {
  TesztBemenet b = {};
  b.swb = Monitor_Allapot; b.angle = Angle_Mode_Aktiv; b.estop = EStop_Allapot;
  b.rx_ok = !Failsafe && IBUS_Kapcsolat_El; b.sensor_ok = !Szenzor_Hiba;
  b.engedely = RAM_Adatok.kesz && RAM_Elokeszitve && !RAM_Kiadas_Zar;
  for (unsigned i = 0; i < 2; ++i)
    b.kozepen[i] = Vevo_Ertekek[i] >= KAR_KOZEP_MIN && Vevo_Ertekek[i] <= KAR_KOZEP_MAX;
  Teszt.frissit(most, b);
  if (Teszt.indult) {
    RAM_Ok = "rogzites";
    Ciklus_Ido_Max = Ciklus_Ido_Max_Szamlalo = Loop_Max_Munka_us = Loop_Tulcsordulas = 0;
  }
  if (Teszt.fut && RAM_Darab >= RAM_MAX_MINTA) Teszt.leallit(PUFFER_TELE);
  if (Teszt.leallt) { RAM_Elokeszitve = false; RAM_Ok = RAM_ok_nev(Teszt.ok); }
}

void RAM_alapjel_feluliras() {
  for (unsigned i = 0; i < 2; ++i) {
    bool aktiv = Teszt.fut && Teszt.tengely() == i;
    if (!aktiv && !Teszt.retesz[i]) continue;
    float sp = aktiv ? Teszt.sp : 0;
    if (Angle_Mode_Aktiv) {
      if (i == 0) Kivant_Angle_Roll = sp; else Kivant_Angle_Pitch = sp;
    } else {
      if (i == 0) Kivant_Roll_Rate = sp; else Kivant_Pitch_Rate = sp;
    }
  }
}

void RAM_mintavetel() {
  if (!Teszt.fut || RAM_Darab >= RAM_MAX_MINTA) return;
  RAM_Minta &m = RAM_Adatok[RAM_Darab];
  bool pitch = Teszt.tengely() == 1;
  m.pv = Teszt.angle() ? (pitch ? Kalman_Angle_Pitch : Kalman_Angle_Roll)
                       : (pitch ? Szurt_Pitch_Rate : Szurt_Roll_Rate);
  m.ciklus_us = m.munka_us = 0;
  RAM_Fuggo = RAM_Darab++;
}

void RAM_status() {
  SerialBT.printf("# RAM ready=%d capacity=%lu bytes=%lu\n", RAM_Adatok.kesz,
    (unsigned long)RAM_MAX_MINTA, (unsigned long)(RAM_MAX_MINTA * sizeof(RAM_Minta)));
  SerialBT.printf("# HEAP after_buffer_bytes=%lu\n", (unsigned long)RAM_Szabad_Indulas);
  SerialBT.printf("# STATUS mode=%s samples=%lu prepared=%d recording=%d SWB=%d lock=%d MPU=%d reason=%s\n",
    RAM_mod_nev(), (unsigned long)RAM_Darab, RAM_Elokeszitve, Teszt.fut,
    Monitor_Allapot, RAM_Kiadas_Zar, Szenzor_Hiba, RAM_Ok);
  SerialBT.printf("# LOOP LM_us=%lu CTM_us=%lu O=%lu C4050=%lu\n",
    (unsigned long)Loop_Max_Munka_us, (unsigned long)Ciklus_Ido_Max,
    (unsigned long)Loop_Tulcsordulas, (unsigned long)Ciklus_Ido_Max_Szamlalo);
  SerialBT.printf("# RECORD LM_us=%lu CTM_us=%lu O=%lu C4050=%lu C4200=%lu\n",
    (unsigned long)RAM_LM, (unsigned long)RAM_CTM, (unsigned long)RAM_O,
    (unsigned long)RAM_C4050, (unsigned long)RAM_C4200);
}

bool RAM_sor(const char *s) {
  if (!SerialBT.hasClient()) return false;
  size_t n = strlen(s);
  return SerialBT.write((const uint8_t *)s, n) == n;
}

uint32_t RAM_crc(uint32_t crc, const char *s) {
  while (*s) {
    crc ^= (uint8_t)*s++;
    for (unsigned i = 0; i < 8; ++i) crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
  }
  return crc;
}

void RAM_data() {
  if (!RAM_Darab || Teszt.fut || RAM_Elokeszitve || RAM_Fuggo >= 0) {
    SerialBT.println("# ERROR nincs lezart meres"); return;
  }
  // Ez a fuggveny csak a motorstop agbol hivhato. A zar kapcsolatveszteskor is marad.
  RAM_Kiadas_Zar = true;
  char sor[240];
  snprintf(sor, sizeof(sor), "# BEGIN version=2 mode=%s samples=%lu nominal_hz=250 reason=%s\n",
    RAM_mod_nev(), (unsigned long)RAM_Darab, RAM_Ok);
  if (!RAM_sor(sor)) return;
  RAM_status();
  snprintf(sor, sizeof(sor), "# PID rate_P,I,D,angle_P,I,D,yaw_P,I,D=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
    RAM_PID[0], RAM_PID[1], RAM_PID[2], RAM_PID[3], RAM_PID[4], RAM_PID[5], RAM_PID[6], RAM_PID[7], RAM_PID[8]);
  if (!RAM_sor(sor)) return;
  for (size_t i = 0; i < Teszt.darab; ++i) {
    snprintf(sor, sizeof(sor), "# STAGE index=%u sp=%.6f duration_us=%llu\n", (unsigned)i,
      Teszt.szakaszok[i].sp, (unsigned long long)TesztVezerles::hossz(Teszt.szakaszok[i]));
    if (!RAM_sor(sor)) return;
  }
  const char *fejlec = "index,t_us,cycle_us,work_us,sp,pv\n";
  uint32_t crc = RAM_crc(0xffffffff, fejlec);
  if (!RAM_sor(fejlec)) return;
  uint64_t t = 0, hatar = TesztVezerles::hossz(Teszt.szakaszok[0]);
  size_t szakasz = 0;
  for (uint32_t i = 0; i < RAM_Darab; ++i) {
    while (szakasz + 1 < Teszt.darab && t >= hatar)
      hatar += TesztVezerles::hossz(Teszt.szakaszok[++szakasz]);
    const RAM_Minta &m = RAM_Adatok[i];
    snprintf(sor, sizeof(sor), "%lu,%llu,%lu,%lu,%.6f,%.6f\n", (unsigned long)i,
      (unsigned long long)t, (unsigned long)m.ciklus_us, (unsigned long)m.munka_us,
      Teszt.szakaszok[szakasz].sp, m.pv);
    if (!RAM_sor(sor)) return;
    crc = RAM_crc(crc, sor);
    t += m.ciklus_us;
    delay(1);
  }
  snprintf(sor, sizeof(sor), "# END samples=%lu duration_us=%llu crc32=%08lx motor_lock=1\n",
    (unsigned long)RAM_Darab, (unsigned long long)t, (unsigned long)~crc);
  RAM_sor(sor);
}

void RAM_parancs(const char *s) {
  if (!EStop_Allapot || Failsafe || !IBUS_Kapcsolat_El || Vevo_Ertekek[2] >= 1030 || Teszt.fut) return;
  if (!strcmp(s, "STATUS")) { RAM_status(); return; }
  if (!strcmp(s, "DATA")) { RAM_data(); return; }
  if (!strcmp(s, "CANCEL")) {
    RAM_Elokeszitve = false; RAM_Ok = "cancel";
    SerialBT.println("# OK elokeszites torolve; meresi adatok megmaradnak"); return;
  }
  if (Monitor_Allapot) { SerialBT.println("# ERROR SWB legyen KI"); return; }
  if (!strcmp(s, "RESUME")) {
    RAM_Kiadas_Zar = false;
    SerialBT.println("# OK adatkuldesi motorzar feloldva; EStop marad"); return;
  }
  if (!strcmp(s, "CLEAR")) {
    RAM_Elokeszitve = false; RAM_Darab = 0; RAM_Fuggo = -1;
    RAM_CTM = RAM_LM = RAM_C4050 = RAM_C4200 = RAM_O = 0;
    Teszt.mod = NINCS; RAM_Ok = "ures";
    SerialBT.println("# OK RAM torolve"); return;
  }
  TesztMod mod = NINCS;
  if (!strcmp(s, "RATE ROLL") || !strcmp(s, "TEST RATE ROLL") || !strcmp(s, "1")) mod = RATE_ROLL;
  if (!strcmp(s, "RATE PITCH") || !strcmp(s, "TEST RATE PITCH") || !strcmp(s, "2")) mod = RATE_PITCH;
  if (!strcmp(s, "ANGLE ROLL") || !strcmp(s, "TEST ANGLE ROLL") || !strcmp(s, "3")) mod = ANGLE_ROLL;
  if (!strcmp(s, "ANGLE PITCH") || !strcmp(s, "TEST ANGLE PITCH") || !strcmp(s, "4")) mod = ANGLE_PITCH;
  if (mod == NINCS) { SerialBT.println("# ERROR RATE/ANGLE ROLL/PITCH, STATUS, DATA, CLEAR, RESUME, CANCEL"); return; }
  if (!RAM_Adatok.kesz) { SerialBT.println("# ERROR RAM foglalas sikertelen; teszt nem indithato"); return; }
  if (RAM_Darab || RAM_Kiadas_Zar) { SerialBT.println("# ERROR elobb DATA, CLEAR es RESUME"); return; }
  const TesztSzakasz *lista = mod >= ANGLE_ROLL ? Angle_Teszt : Rate_Teszt;
  size_t n = mod >= ANGLE_ROLL ? sizeof(Angle_Teszt)/sizeof(Angle_Teszt[0]) : sizeof(Rate_Teszt)/sizeof(Rate_Teszt[0]);
  if (!Teszt.beallit(mod, lista, n)) { SerialBT.println("# ERROR tesztbeallitas vagy karretesz"); return; }
  if (Teszt.teljes_us > (uint64_t)RAM_MAX_MINTA * 4000) {
    RAM_Elokeszitve = false; SerialBT.println("# ERROR tul hosszu lista ehhez a RAM_MAX_MINTA ertekhez"); return;
  }
  bool p = Teszt.tengely() == 1;
  const float pid[] = {p ? P_Rate_Pitch : P_Rate_Roll, p ? I_Rate_Pitch : I_Rate_Roll,
    p ? D_Rate_Pitch : D_Rate_Roll, p ? P_Angle_Pitch : P_Angle_Roll,
    p ? I_Angle_Pitch : I_Angle_Roll, p ? D_Angle_Pitch : D_Angle_Roll,
    P_Rate_Yaw, I_Rate_Yaw, D_Rate_Yaw};
  memcpy(RAM_PID, pid, sizeof(pid));
  RAM_Elokeszitve = true; RAM_Ok = "varakozik";
  SerialBT.printf("# READY %s %.3f s; megfelelo mod, EStop KI, SWB KI->BE\n", RAM_mod_nev(), Teszt.teljes_us / 1e6);
}

void RAM_BT_fogadas(bool engedely) {
  static char sor[48];
  static unsigned n = 0;
  static bool tulhosszu = false, elozo_engedely = false;
  // Repules alatt erkezett parancs ne hajtodjon vegre kesobb a foldon.
  if (!engedely || !elozo_engedely) {
    n = 0; tulhosszu = false;
    unsigned limit = 128;
    while (limit-- && SerialBT.available()) SerialBT.read();
    elozo_engedely = engedely && !SerialBT.available();
    return;
  }
  for (unsigned i = 0; i < 20 && SerialBT.available(); ++i) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (tulhosszu) SerialBT.println("# ERROR tul hosszu parancs");
      else if (n) { sor[n] = 0; RAM_parancs(sor); }
      n = 0; tulhosszu = false;
    } else if (!tulhosszu) {
      if (c >= 'a' && c <= 'z') c -= 'a' - 'A';
      if (n < sizeof(sor)-1) sor[n++] = c; else tulhosszu = true;
    }
  }
}
