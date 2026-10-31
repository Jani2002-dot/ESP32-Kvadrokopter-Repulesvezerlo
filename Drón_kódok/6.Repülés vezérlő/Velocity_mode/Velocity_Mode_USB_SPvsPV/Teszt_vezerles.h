#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

struct TesztSzakasz {
  float sp;
  float ido_s;
};

enum TesztMod : uint8_t { NINCS, RATE_ROLL, RATE_PITCH, ANGLE_ROLL, ANGLE_PITCH };
enum TesztOk : uint32_t {
  KESZ = 1, SWB_KI, ESTOP, JELVESZTES, MODVALTAS, USB_HIANY,
  PUFFER_TELE, MEGSZAKITAS, SZENZOR_HIBA
};

struct TesztBemenet {
  bool swb, angle, estop, rx_ok, usb_ok, sensor_ok, cancel;
  bool kozepen[2];
};

class TesztVezerles {
 public:
  TesztMod mod = NINCS;
  const TesztSzakasz *szakaszok = nullptr;
  size_t darab = 0, szakasz = 0;
  bool fut = false, indult = false, leallt = false;
  bool retesz[2] = {false, false};
  uint64_t kezdet = 0, teljes_us = 0;
  float sp = 0;
  TesztOk ok = KESZ;

  bool angle() const { return mod == ANGLE_ROLL || mod == ANGLE_PITCH; }
  unsigned tengely() const { return mod == RATE_PITCH || mod == ANGLE_PITCH ? 1 : 0; }
  static uint64_t hossz(const TesztSzakasz &s) {
    return (uint64_t)((double)s.ido_s * 1000000.0 + 0.5);
  }

  bool beallit(TesztMod m, const TesztSzakasz *s, size_t n) {
    if (fut || retesz[0] || retesz[1] || m < RATE_ROLL || m > ANGLE_PITCH || !s || !n || n > 128) return false;
    uint64_t osszeg = 0;
    const float hatar = m >= ANGLE_ROLL ? 80.0f : 500.0f;
    for (size_t i = 0; i < n; ++i) {
      if (!isfinite(s[i].sp) || fabsf(s[i].sp) > hatar || !isfinite(s[i].ido_s) ||
          s[i].ido_s < 0.004f || s[i].ido_s > 604800.0f) return false;
      osszeg += hossz(s[i]);
    }
    // A sorszam 32 bites; egy het 250 Hz-en is boven belefer.
    if (osszeg > 604800000000ULL) return false;
    mod = m; szakaszok = s; darab = n; teljes_us = osszeg;
    alacsony_volt = false;
    return true;
  }

  void leallit(TesztOk miert) {
    if (!fut) return;
    fut = false; leallt = true; ok = miert; sp = 0;
    retesz[tengely()] = true;
    alacsony_volt = false;
  }

  void frissit(uint64_t most, const TesztBemenet &b) {
    indult = leallt = false;
    if (b.rx_ok) {
      for (unsigned i = 0; i < 2; ++i) if (b.kozepen[i]) retesz[i] = false;
    }
    if (fut) {
      if (!b.sensor_ok) leallit(SZENZOR_HIBA);
      else if (!b.rx_ok) leallit(JELVESZTES);
      else if (b.estop) leallit(ESTOP);
      else if (!b.usb_ok) leallit(USB_HIANY);
      else if (b.angle != angle()) leallit(MODVALTAS);
      else if (b.cancel) leallit(MEGSZAKITAS);
      else if (!b.swb) leallit(SWB_KI);
      else if (most - kezdet >= teljes_us) leallit(KESZ);
      if (!fut) return;
    } else {
      if (!b.rx_ok || b.estop || !b.usb_ok || !b.sensor_ok || b.cancel) {
        alacsony_volt = false;
        return;
      }
      if (!b.swb) { alacsony_volt = true; return; }
      if (!alacsony_volt) return;
      alacsony_volt = false;
      if (mod == NINCS || b.angle != angle() || retesz[0] || retesz[1]) return;
      fut = indult = true; kezdet = most; szakasz = 0;
    }
    uint64_t vege = 0;
    for (size_t i = 0; i < darab; ++i) {
      vege += hossz(szakaszok[i]);
      if (most - kezdet < vege) { szakasz = i; sp = szakaszok[i].sp; break; }
    }
  }

 private:
  bool alacsony_volt = false;
};
