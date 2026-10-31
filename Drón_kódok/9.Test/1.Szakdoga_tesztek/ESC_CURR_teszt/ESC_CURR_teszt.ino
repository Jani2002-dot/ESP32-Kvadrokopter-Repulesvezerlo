#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

// ESP32-WROOM / Arduino-ESP32 2.0.17. Measurement only, no motor PWM.
constexpr uint8_t CURR_PIN = 39;
constexpr double DIVIDER_GAIN = 1.0;  // Direct connection: 1.0; equal divider: 0.5.
constexpr unsigned SAMPLE_COUNT = 100;
constexpr unsigned MAX_POINTS = 24;
constexpr double ADC_MIN_MV = 150.0;
constexpr double ADC_MAX_MV = 3100.0;

struct Measurement {
  double mean;
  double sd;
  uint32_t minimum;
  uint32_t maximum;
};

struct Point {
  double amps;
  double adcMv;
  double externalMv;
  bool adcInRange;
  bool hasExternal;
};

Point points[MAX_POINTS];
unsigned pointCount = 0;
char command[80];
unsigned commandLength = 0;
bool overflow = false;

Measurement measureVoltage() {
  Measurement result = {0.0, 0.0, UINT32_MAX, 0};
  double m2 = 0.0;
  for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
    const uint32_t mv = analogReadMilliVolts(CURR_PIN);
    const double delta = mv - result.mean;
    result.mean += delta / (i + 1);
    m2 += delta * (mv - result.mean);
    if (mv < result.minimum) result.minimum = mv;
    if (mv > result.maximum) result.maximum = mv;
    delay(10);
  }
  result.sd = sqrt(m2 / (SAMPLE_COUNT - 1));
  return result;
}

bool inRange(const Measurement &m) {
  return m.minimum >= ADC_MIN_MV && m.maximum <= ADC_MAX_MV;
}

void printMeasurement(const Measurement &m) {
  Serial.printf("ADC: atlag=%.2f mV; min=%lu; max=%lu; szoras=%.2f mV\n",
                m.mean, (unsigned long)m.minimum,
                (unsigned long)m.maximum, m.sd);
  if (!inRange(m)) {
    Serial.println("FIGYELEM: ADC 11 dB nevleges 150..3100 mV tartomanyon kivul.");
    Serial.println("Ez a pont nem kerul az ADC-illesztesbe. A kod NEM tulfeszultseg-vedelem!");
  }
}

void printFit(bool external) {
  unsigned count = 0;
  double meanI = 0.0, meanU = 0.0;
  double minI = 1e9, maxI = -1e9;
  for (unsigned i = 0; i < pointCount; ++i) {
    const Point &p = points[i];
    if (external ? !p.hasExternal : !p.adcInRange) continue;
    const double u = external ? p.externalMv : p.adcMv / DIVIDER_GAIN;
    ++count;
    meanI += (p.amps - meanI) / count;
    meanU += (u - meanU) / count;
    if (p.amps < minI) minI = p.amps;
    if (p.amps > maxI) maxI = p.amps;
  }
  Serial.println(external ? "--- Kulso muszer: ESC CURR karakterisztika ---"
                          : "--- ESP32: teljes merolanc becslese ---");
  if (count < 3) {
    Serial.println("Legalabb 3 hasznalhato pont kell, eltero aramertekekkel.");
    return;
  }
  double sxx = 0.0, sxy = 0.0;
  for (unsigned i = 0; i < pointCount; ++i) {
    const Point &p = points[i];
    if (external ? !p.hasExternal : !p.adcInRange) continue;
    const double u = external ? p.externalMv : p.adcMv / DIVIDER_GAIN;
    sxx += (p.amps - meanI) * (p.amps - meanI);
    sxy += (p.amps - meanI) * (u - meanU);
  }
  if (sxx < 1e-9) {
    Serial.println("Nem elegendo aramkulonbseg az illeszteshez.");
    return;
  }
  const double slope = sxy / sxx;
  const double offset = meanU - slope * meanI;
  double squaredError = 0.0;
  for (unsigned i = 0; i < pointCount; ++i) {
    const Point &p = points[i];
    if (external ? !p.hasExternal : !p.adcInRange) continue;
    const double u = external ? p.externalMv : p.adcMv / DIVIDER_GAIN;
    const double error = u - (offset + slope * p.amps);
    squaredError += error * error;
  }
  Serial.printf("Pontok=%u; vizsgalt tartomany=%.3f..%.3f A\n", count, minI, maxI);
  Serial.printf("U_CURR[mV] = %.3f + %.3f * I[A]\n", offset, slope);
  Serial.printf("Erzekenyseg: %.3f mV/A; elteres 30-tol: %+.2f %%\n",
                slope, (slope / 30.0 - 1.0) * 100.0);
  Serial.printf("Illesztett U0: %.3f mV; maradek RMS: %.3f mV\n",
                offset, sqrt(squaredError / count));
  Serial.println("U0 extrapolalt lehet; az RMS nem teljes meresi bizonytalansag.");
  Serial.println("Az eredmeny nem igazolja a nagyaramu linearitast vagy a kimeneti maximumot.");
  if (slope <= 0) Serial.println("HIBAGYANU: nem pozitiv erzekenyseg, ellenorizd a merest!");
}

void help() {
  Serial.println("\nESC CURR teszt - GPIO39 - ADC 11 dB - nincs motorvezerles");
  Serial.println("v          : kb. 1 masodperces ADC-meres");
  Serial.println("p 0.45     : pont mentese, referenciaaram 0.45 A + uj ADC-meres");
  Serial.println("p 0.45 18.2: ugyanaz + kulso muszerrel mert CURR-feszultseg 18.2 mV");
  Serial.println("f          : kulon ADC es kulso muszeres egyenesillesztes");
  Serial.println("l          : mentett pontok CSV formatumban");
  Serial.println("r          : pontok torlese (RAM, ujrainditaskor is elvesznek)");
  Serial.println("h          : sugo; tizedespont es ujsor szukseges");
  Serial.println("A nyugalmi aram NEM nulla! A tenyleges referenciaaramot add meg.");
  Serial.println("PDM300 C3: >2 A eseten <10 s meres, >15 perc szunet; ellenorizd a sajat valtozatot!");
  Serial.println("Kulso feszultseg: mindig az ESC CURR labon, oszto ELOTT mert mV.");
}

void runCommand(char *line) {
  while (isspace((unsigned char)*line)) ++line;
  if (!*line) return;
  const char action = *line++;
  double args[2];
  unsigned count = 0;
  while (true) {
    while (isspace((unsigned char)*line)) ++line;
    if (!*line) break;
    if (count == 2) { Serial.println("Hibas parancs."); return; }
    char *end = nullptr;
    args[count] = strtod(line, &end);
    if (end == line || !isfinite(args[count])) { Serial.println("Hibas szam."); return; }
    ++count;
    line = end;
  }
  if (action != 'p' && count != 0) { Serial.println("Hibas parancs."); return; }
  if (action == 'v') {
    printMeasurement(measureVoltage());
  } else if (action == 'p') {
    if (count < 1 || args[0] < 0 || args[0] >= 10 ||
        (count == 2 && (args[1] < 0 || args[1] > 10000))) {
      Serial.println("Formatum: p aram_A [CURR_mV], 0 <= aram < 10.");
      Serial.println("A 10 A nem biztonsagos celaram; tartsd be a muszer korlatait!");
      return;
    }
    if (pointCount == MAX_POINTS) { Serial.println("Tele: 24 pont. Mentsd a listat, majd r."); return; }
    if (args[0] > 2.0) Serial.println("FIGYELEM: PDM300 C3 >2 A ido-/pihenokorlat! Nincs automatikus aramlekapcsolas.");
    Serial.println("Tartsd allandon a terhelest; kb. 1 s meres...");
    const Measurement m = measureVoltage();
    points[pointCount++] = {args[0], m.mean, count == 2 ? args[1] : 0.0,
                            inRange(m), count == 2};
    printMeasurement(m);
    Serial.printf("Mentve: %u. pont, I_ref=%.4f A\n", pointCount, args[0]);
  } else if (action == 'f') {
    printFit(false);
    printFit(true);
  } else if (action == 'l') {
    Serial.println("index,I_ref_A,U_ADC_mV,ADC_in_range,U_CURR_external_mV");
    for (unsigned i = 0; i < pointCount; ++i) {
      const Point &p = points[i];
      Serial.printf("%u,%.4f,%.3f,%u,", i + 1, p.amps, p.adcMv, p.adcInRange ? 1 : 0);
      if (p.hasExternal) Serial.print(p.externalMv, 3);
      Serial.println();
    }
  } else if (action == 'r') {
    pointCount = 0;
    Serial.println("Pontok torolve.");
  } else if (action == 'h') {
    help();
  } else {
    Serial.println("Ismeretlen parancs; h = sugo.");
  }
}

void setup() {
  // Both old (17) and V2 (4) motor-4 pins stay LOW. No arming pulses.
  const uint8_t motorPins[] = {23, 19, 18, 17, 4};
  for (uint8_t pin : motorPins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.begin(115200);
  pinMode(CURR_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(CURR_PIN, ADC_11db);
  delay(1000);
  help();
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (overflow) Serial.println("Tul hosszu parancs, eldobva.");
      else if (commandLength) {
        command[commandLength] = '\0';
        runCommand(command);
      }
      commandLength = 0;
      overflow = false;
    } else if (!overflow) {
      if (commandLength < sizeof(command) - 1) command[commandLength++] = c;
      else overflow = true;
    }
  }
  delay(1);
}
