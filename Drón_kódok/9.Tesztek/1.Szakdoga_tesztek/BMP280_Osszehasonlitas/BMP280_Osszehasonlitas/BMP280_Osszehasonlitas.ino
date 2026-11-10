#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ESP32 Arduino; no external sensor library required.
// Adjust to your board/wiring (classic ESP32 defaults shown).
constexpr int SDA_PIN = 21, SCL_PIN = 22;
constexpr uint8_t BMP_ADDR = 0x77;
constexpr uint32_t BAUD = 460800;
constexpr uint32_t SETTLE_MS = 10000, REFERENCE_MS = 10000, TEST_MS = 120000;
constexpr uint32_t POLL_US = 500;

struct Calibration {
  uint16_t t1, p1;
  int16_t t2, t3, p2, p3, p4, p5, p6, p7, p8, p9;
} cal;
struct Profile { uint8_t sleepCtrl, config; uint32_t maxUs; };
// Original F4 0x57 becomes 0x54 sleep / 0x55 forced.
// Original F5 0x14 retained exactly; actual byte read back and logged.
const Profile profiles[2] = {{0x54, 0x14, 44000}, {0x2C, 0x08, 14000}};
enum Phase { IDLE, SETTLE, REFERENCE, TEST };
Phase phase = IDLE;
bool sensorOK = false, pending = false, sawBusy = false, stepTest = false;
bool activeStep = false;
uint8_t profile = 1;
uint32_t runId = 0, phaseStart = 0, triggerUs = 0, pollUs = 0;
uint32_t originUs = 0, previousUs = 0, referenceN = 0, sampleN = 0, errors = 0;
uint32_t markN = 0;
double referenceSum = 0, referenceCm = 0, meanCm = 0, m2Cm = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMP_ADDR); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool readBytes(uint8_t reg, uint8_t *buf, size_t count) {
  Wire.beginTransmission(BMP_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BMP_ADDR, (uint8_t)count) != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i=0; i<count; ++i) buf[i] = (uint8_t)Wire.read();
  return true;
}
bool readReg(uint8_t reg, uint8_t &value) { return readBytes(reg, &value, 1); }
uint16_t u16(const uint8_t *b) { return uint16_t(b[0]) | (uint16_t(b[1]) << 8); }

// Only setup/profile changes block briefly. Measurement loop is non-blocking.
bool waitIdle(uint32_t timeoutMs) {
  uint32_t start = millis(); uint8_t status;
  do {
    if (!readReg(0xF3, status)) return false;
    if (!(status & 0x09)) return true;
    delay(1);
  } while (uint32_t(millis()-start) < timeoutMs);
  return false;
}
bool loadCalibration() {
  uint8_t b[24]; if (!readBytes(0x88,b,24)) return false;
  cal.t1=u16(b); cal.t2=(int16_t)u16(b+2); cal.t3=(int16_t)u16(b+4);
  cal.p1=u16(b+6); cal.p2=(int16_t)u16(b+8); cal.p3=(int16_t)u16(b+10);
  cal.p4=(int16_t)u16(b+12); cal.p5=(int16_t)u16(b+14); cal.p6=(int16_t)u16(b+16);
  cal.p7=(int16_t)u16(b+18); cal.p8=(int16_t)u16(b+20); cal.p9=(int16_t)u16(b+22);
  return cal.p1 != 0 && cal.p1 != 0xFFFF && cal.t1 != 0 && cal.t1 != 0xFFFF;
}
bool configure(uint8_t number) {
  profile=number; const Profile &p=profiles[profile-1];
  if (!writeReg(0xF4,p.sleepCtrl) || !waitIdle(100)) return false;
  if (!writeReg(0xF5,p.config)) return false;
  uint8_t ctrl,conf;
  if (!readReg(0xF4,ctrl) || !readReg(0xF5,conf)) return false;
  Serial.printf("#CONFIG,%u,F4_sleep=0x%02X,F5=0x%02X,mode=forced\n",profile,ctrl,conf);
  return ctrl==p.sleepCtrl && conf==p.config;
}
// Bosch datasheet Appendix 8.1 double-precision compensation.
// Using signed 20-bit ADC values avoids mixed signed/unsigned arithmetic.
bool compensate(int32_t adcT,int32_t adcP,double &temperature,double &pressure) {
  if (adcT==0x80000 || adcP==0x80000) return false;
  double v1=(adcT/16384.0-cal.t1/1024.0)*cal.t2;
  double v2=pow(adcT/131072.0-cal.t1/8192.0,2)*cal.t3;
  int32_t fine=(int32_t)(v1+v2);
  temperature=(v1+v2)/5120.0;
  v1=fine/2.0-64000.0;
  v2=v1*v1*cal.p6/32768.0;
  v2+=v1*cal.p5*2.0;
  v2=v2/4.0+cal.p4*65536.0;
  v1=(cal.p3*v1*v1/524288.0+cal.p2*v1)/524288.0;
  v1=(1.0+v1/32768.0)*cal.p1;
  if (v1==0.0) return false; // Exit BEFORE division, discard invalid sample.
  pressure=(1048576.0-adcP-v2/4096.0)*6250.0/v1;
  v1=cal.p9*pressure*pressure/2147483648.0;
  v2=pressure*cal.p8/32768.0;
  pressure+=(v1+v2+cal.p7)/16.0;
  return isfinite(pressure) && isfinite(temperature) && pressure>=30000 &&
    pressure<=110000 && temperature>=-40 && temperature<=85;
}
bool readSample(double &pressure,double &temperature,double &altitude) {
  uint8_t b[6]; if (!readBytes(0xF7,b,6)) return false;
  int32_t adcP=(int32_t(b[0])<<12)|(int32_t(b[1])<<4)|(b[2]>>4);
  int32_t adcT=(int32_t(b[3])<<12)|(int32_t(b[4])<<4)|(b[5]>>4);
  if (!compensate(adcT,adcP,temperature,pressure)) return false;
  altitude=44330.0*(1.0-pow(pressure/101325.0,1.0/5.255))*100.0;
  return isfinite(altitude);
}
void fail(const char *reason) {
  ++errors; Serial.printf("#ERROR,%lu,%s\n",(unsigned long)runId,reason);
  phase=IDLE; pending=false;
  writeReg(0xF4,profiles[profile-1].sleepCtrl);
  Serial.println("#ABORT; fix error then start a new run");
}
void trigger() {
  if (!writeReg(0xF4,profiles[profile-1].sleepCtrl|0x01)) { fail("trigger_i2c"); return; }
  triggerUs=micros(); pollUs=triggerUs; pending=true; sawBusy=false;
}
void finish() {
  double sd=sampleN>1?sqrt(m2Cm/(sampleN-1)):NAN;
  Serial.printf("#END,%lu,n=%lu,mean_cm=%.6f,sd_cm=%.6f,R_cm2=%.6f,errors=%lu\n",
    (unsigned long)runId,(unsigned long)sampleN,meanCm,sd,sd*sd,(unsigned long)errors);
  phase=IDLE; pending=false; writeReg(0xF4,profiles[profile-1].sleepCtrl);
}
void startRun(uint8_t number) {
  if (!sensorOK) { Serial.println("#ERROR,sensor_not_initialized; reset board"); return; }
  if (phase!=IDLE) Serial.printf("#CANCEL,%lu\n",(unsigned long)runId);
  phase=IDLE; pending=false;
  if (!configure(number)) { fail("configuration_or_readback"); return; }
  ++runId; activeStep=stepTest; referenceN=sampleN=errors=markN=0;
  referenceSum=referenceCm=meanCm=m2Cm=0;
  phase=SETTLE; phaseStart=millis();
  Serial.printf("#RUN,%lu,profile=%u,kind=%s,settle_ms=%lu,reference_ms=%lu,test_ms=%lu\n",
    (unsigned long)runId,profile,activeStep?"step":"static",(unsigned long)SETTLE_MS,
    (unsigned long)REFERENCE_MS,(unsigned long)TEST_MS);
  Serial.println("#PHASE,settle; hold sensor still"); trigger();
}
void consume(double pressure,double temperature,double altitude,uint32_t readyUs,uint8_t status) {
  uint32_t now=millis();
  if (phase==SETTLE && uint32_t(now-phaseStart)>=SETTLE_MS) {
    phase=REFERENCE; phaseStart=now; Serial.println("#PHASE,reference; hold sensor still");
  }
  if (phase==REFERENCE) {
    referenceSum+=altitude; ++referenceN;
    if (uint32_t(now-phaseStart)>=REFERENCE_MS) {
      referenceCm=referenceSum/referenceN; phase=TEST; phaseStart=now;
      originUs=readyUs; previousUs=readyUs;
      Serial.printf("#REFERENCE,%.6f,n=%lu\n",referenceCm,(unsigned long)referenceN);
      Serial.println("#PHASE,test");
      Serial.println("run,profile,kind,t_us,dt_us,conv_us,pressure_pa,temp_c,alt_cm,status");
    }
    return;
  }
  if (phase!=TEST) return;
  double h=altitude-referenceCm;
  ++sampleN; double delta=h-meanCm; meanCm+=delta/sampleN; m2Cm+=delta*(h-meanCm);
  uint32_t conv=readyUs-triggerUs,dt=readyUs-previousUs; previousUs=readyUs;
  Serial.printf("%lu,%u,%s,%lu,%lu,%lu,%.6f,%.6f,%.6f,%u\n",
    (unsigned long)runId,profile,activeStep?"step":"static",(unsigned long)(readyUs-originUs),
    (unsigned long)dt,(unsigned long)conv,pressure,temperature,h,status);
  if (uint32_t(now-phaseStart)>=TEST_MS) finish();
}
void help() {
  Serial.println("#COMMANDS: 1/2=start profile; s=static for next run; d=step for next run; m=movement marker; 0=stop; ?=help");
  Serial.println("#Warm up 2 minutes. Each run: 10s settle + 10s reference + 120s test.");
}
void setup() {
  Serial.begin(BAUD);
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(20); delay(100);
  uint8_t id;
  if (!readReg(0xD0,id) || id!=0x58) {
    Serial.println("#ERROR,BMP280 chip ID mismatch or I2C failure; check address/pins"); return;
  }
  if (!writeReg(0xE0,0xB6)) { Serial.println("#ERROR,reset_i2c"); return; }
  delay(5);
  sensorOK=waitIdle(100) && loadCalibration();
  if (!sensorOK) { Serial.println("#ERROR,calibration_or_status"); return; }
  Serial.println("#BMP280 ESP32 comparison ready; baud=460800; address=0x77"); help();
}
void loop() {
  while (Serial.available()) {
    char c=Serial.read();
    if (c=='1'||c=='2') startRun(c-'0');
    else if(c=='s'||c=='d') { stepTest=(c=='d'); Serial.printf("#NEXT_KIND,%s\n",stepTest?"step":"static"); }
    else if(c=='0') {
      if(phase!=IDLE) Serial.printf("#CANCEL,%lu\n",(unsigned long)runId);
      phase=IDLE; pending=false; writeReg(0xF4,profiles[profile-1].sleepCtrl);
    }
    else if(c=='m' && phase==TEST) Serial.printf("#MARK,%lu,%lu,%lu\n",(unsigned long)runId,
      (unsigned long)(micros()-originUs),(unsigned long)++markN);
    else if(c=='?') help();
  }
  if (phase==IDLE) { delay(1); return; }
  if (!pending) { trigger(); return; }
  uint32_t now=micros(),elapsed=now-triggerUs;
  if (elapsed>profiles[profile-1].maxUs+100000) { fail("measurement_timeout"); return; }
  if (elapsed<1000 || uint32_t(now-pollUs)<POLL_US) return;
  pollUs=now; uint8_t status;
  if (!readReg(0xF3,status)) { fail("status_i2c"); return; }
  if (status&0x08) sawBusy=true;
  // Busy->idle identifies completion. If busy was missed, wait worst-case conversion time.
  if ((status&0x09) || (!sawBusy && elapsed<profiles[profile-1].maxUs)) return;
  double pressure,temperature,altitude;
  if (!readSample(pressure,temperature,altitude)) { fail("invalid_sample_or_i2c"); return; }
  uint32_t readyUs=micros(); pending=false;
  consume(pressure,temperature,altitude,readyUs,status);
  if (phase!=IDLE) trigger();
}
