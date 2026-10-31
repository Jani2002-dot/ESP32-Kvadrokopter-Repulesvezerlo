#pragma once
#include "Teszt_vezerles.h"

// Egy sor: {alapjel, idotartam masodpercben}. Sorok szabadon hozzaadhatok/torolhetok.
// Rate alapjel: fok/s. A ket teszttengely ugyanazt a listat hasznalja.
const TesztSzakasz Rate_Teszt[] = {
  {  0, 7 },
  { 20, 1 },
  {  0, 6 },
  {-20, 2 },
  {  0, 6 },
  { 20, 1 },
  {  0, 7 }
};

// Angle alapjel: fok.
const TesztSzakasz Angle_Teszt[] = {
  {  0, 6 },
  { 20, 6 },
  {  0, 6 },
  {-20, 6 },
  {  0, 6 }
};

constexpr uint32_t USB_BAUD = 460800;
constexpr uint32_t USB_HEARTBEAT_US = 3000000;
constexpr uint32_t LOOP_SZAMLALO_US = 4050;
constexpr uint32_t LOOP_LED_US = 4200;
constexpr int KAR_KOZEP_MIN = 1492;
constexpr int KAR_KOZEP_MAX = 1508;

static_assert(sizeof(Rate_Teszt) / sizeof(Rate_Teszt[0]) <= 128, "Legfeljebb 128 Rate szakasz");
static_assert(sizeof(Angle_Teszt) / sizeof(Angle_Teszt[0]) <= 128, "Legfeljebb 128 Angle szakasz");
