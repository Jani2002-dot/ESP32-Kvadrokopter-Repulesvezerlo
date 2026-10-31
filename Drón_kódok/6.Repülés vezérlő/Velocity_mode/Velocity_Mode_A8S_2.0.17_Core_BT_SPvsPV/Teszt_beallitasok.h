#pragma once
#include "Teszt_vezerles.h"

// {alapjel, masodperc}; a sorok szama es minden ertek modosithato.
const TesztSzakasz Rate_Teszt[] = {
  {0, 7}, {20, 1}, {0, 6}, {-20, 2}, {0, 6}, {20, 1}, {0, 7}
};
const TesztSzakasz Angle_Teszt[] = {
  {0, 6}, {20, 6}, {0, 6}, {-20, 6}, {0, 6}
};
constexpr uint32_t RAM_MAX_MINTA = 7500; // 30 s, 250 Hz; 12 bajt/minta.
constexpr uint32_t LOOP_SZAMLALO_US = 4050;
constexpr uint32_t LOOP_LED_US = 4200;
constexpr int KAR_KOZEP_MIN = 1492, KAR_KOZEP_MAX = 1508;
static_assert(RAM_MAX_MINTA > 0, "Ures RAM-puffer");
