#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

const bool RELAY_ACTIVE = HIGH;
const int PIN_RELAY_LIGHT   = 14;
const int PIN_RELAY_PUMP    = 27;
const int PIN_RELAY_FAN     = 26;
const int PIN_PUMP_NUTRIENT_A = 25;
const int PIN_PUMP_NUTRIENT_B = 33;
const int PIN_PUMP_PH_UP      = 32;
const int PIN_PUMP_PH_DOWN    = 15;
const int PIN_DHT       = 4;
const int DHTTYPE       = DHT22;
const int PIN_ONEWIRE   = 5;
const int PIN_PH_ADC    = 35;
const int PIN_TDS_ADC   = 34;

DHT dht(PIN_DHT, DHTTYPE);
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature waterTemp(&oneWire);
RTC_DS3231 rtc;

unsigned long nowMs;

struct EdgeTimer {
  unsigned long last = 0;
  bool runEvery(unsigned long intervalMs) {
    if (millis() - last >= intervalMs) { last = millis(); return true; }
    return false;
  }
};

struct Config {
  uint8_t lightsOnHour  = 6;
  uint8_t lightsOnMin   = 0;
  uint8_t lightsOffHour = 22;
  uint8_t lightsOffMin  = 0;
  float fanOnTempC      = 28.0;
  float fanOffTempC     = 26.5;
  float fanOnHumidity   = 80.0;
  float fanOffHumidity  = 70.0;
  float targetPH        = 5.8;
  float phDeadband      = 0.08;
  float targetEC_mS     = 1.2;
  float ecDeadband      = 0.1;
  unsigned long dosePulseMs     = 1200;
  unsigned long mixDelayMs      = 90UL * 1000UL;
  unsigned int  maxPHpulsesDay  = 30;
  unsigned int  maxECpulsesDay  = 60;
  bool pumpContinuous = true;
  unsigned long pumpOnMs  = 15UL * 60UL * 1000UL;
  unsigned long pumpOffMs = 45UL * 60UL * 1000UL;
  uint8_t phSamples  = 20;
  uint8_t tdsSamples = 20;
  float phSlope  = -5.70;
  float phOffset =  21.34;
  float tdsKValue = 1.0;
  float adcVref = 3.3;
  uint16_t adcMax = 4095;
} CFG;

struct DoseState {
  unsigned long lastDoseEndMs = 0;
  unsigned int pulsesToday = 0;
  bool lockedOut = false;
};

DoseState dosePHUp, dosePHDown, doseECA, doseECB;

struct AlarmState {
  bool phSensorFault = false;
  bool tdsSensorFault = false;
  bool tempSensorFault = false;
} ALARM;

EdgeTimer sensorTimer, lightTimer, envTimer, dailyResetTimer, pumpTimer;

void setActuator(int pin, bool on) {
  digitalWrite(pin, on ? RELAY_ACTIVE : !RELAY_ACTIVE);
}

bool isTimeBetween(DateTime t, uint8_t h1, uint8_t m1, uint8_t h2, uint8_t m2) {
  uint16_t start = h1*60 + m1;
  uint16_t end   = h2*60 + m2;
  uint16_t cur   = t.hour()*60 + t.minute();
  if (start <= end) return (cur >= start && cur < end);
  return !(cur >= end && cur < start);
}

float readADCvoltage(int pin, uint8_t samples) {
  uint32_t acc = 0;
  for (uint8_t i=0;i<samples;i++) {
    acc += analogRead(pin);
    delay(3);
  }
  float avg = (float)acc / samples;
  return (avg / CFG.adcMax) * CFG.adcVref;
}

float ambientTempC = NAN, ambientRH = NAN;
float waterTempC   = NAN;
float phValue      = NAN;
float ec_mS        = NAN;

void readAmbient() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) return;
  ambientTempC = t;
  ambientRH = h;
}

void readWaterTemp() {
  waterTemp.requestTemperatures();
  float t = waterTemp.getTempCByIndex(0);
  if (t > -40 && t < 125) waterTempC = t;
  else ALARM.tempSensorFault = true;
}

float computePH() {
  float v = readADCvoltage(PIN_PH_ADC, CFG.phSamples);
  return CFG.phSlope * v + CFG.phOffset;
}

float computeEC_mS() {
  float voltage = readADCvoltage(PIN_TDS_ADC, CFG.tdsSamples);
  float vRatio = voltage / CFG.adcVref;
  float ec = (133.42*pow(vRatio,3) - 255.86*pow(vRatio,2) + 857.39*vRatio) * 0.001;
  float tempCoef = 1.0 + 0.02 * (waterTempC - 25.0);
  ec = ec / tempCoef;
  ec *= CFG.tdsKValue;
  if (ec < 0 || ec > 10) { ALARM.tdsSensorFault = true; }
  return ec;
}

void readChemistry() {
  float ph = computePH();
  if (ph < 3.0 || ph > 10.5) {
    ALARM.phSensorFault = true;
  } else {
    phValue = ph;
  }
  float ec = computeEC_mS();
  if (!ALARM.tdsSensorFault) ec_mS = ec;
}

bool canDose(const DoseState &ds, unsigned int maxDaily) {
  if (ds.lockedOut) return false;
  if (ds.pulsesToday >= maxDaily) return false;
  if (millis() - ds.lastDoseEndMs < CFG.mixDelayMs) return false;
  return true;
}

void pulsePump(int pin, unsigned long ms) {
  setActuator(pin, true);
  delay(ms);
  setActuator(pin, false);
}

void tryDosePH(float ph) {
  float err = ph - CFG.targetPH;
  if (fabs(err) <= CFG.phDeadband) return;
  if (err > 0) {
    if (canDose(dosePHDown, CFG.maxPHpulsesDay)) {
      pulsePump(PIN_PUMP_PH_DOWN, CFG.dosePulseMs);
      dosePHDown.pulsesToday++;
      dosePHDown.lastDoseEndMs = millis();
    }
  } else {
    if (canDose(dosePHUp, CFG.maxPHpulsesDay)) {
      pulsePump(PIN_PUMP_PH_UP, CFG.dosePulseMs);
      dosePHUp.pulsesToday++;
      dosePHUp.lastDoseEndMs = millis();
    }
  }
}

void tryDoseEC(float ec) {
  float err = CFG.targetEC_mS - ec;
  if (fabs(err) <= CFG.ecDeadband) return;
  static bool toggleAB = false;
  if (err > 0) {
    if (toggleAB) {
      if (canDose(doseECA, CFG.maxECpulsesDay)) {
        pulsePump(PIN_PUMP_NUTRIENT_A, CFG.dosePulseMs);
        doseECA.pulsesToday++;
        doseECA.lastDoseEndMs = millis();
      }
    } else {
      if (canDose(doseECB, CFG.maxECpulsesDay)) {
        pulsePump(PIN_PUMP_NUTRIENT_B, CFG.dosePulseMs);
        doseECB.pulsesToday++;
        doseECB.lastDoseEndMs = millis();
      }
    }
    toggleAB = !toggleAB;
  }
}

void controlLights(const DateTime &now) {
  bool on = isTimeBetween(now, CFG.lightsOnHour, CFG.lightsOnMin, CFG.lightsOffHour, CFG.lightsOffMin);
  setActuator(PIN_RELAY_LIGHT, on);
}

void controlFan() {
  static bool fanOn = false;
  bool shouldOn = false;
  if (!isnan(ambientTempC) && ambientTempC >= CFG.fanOnTempC) shouldOn = true;
  if (!isnan(ambientRH)    && ambientRH    >= CFG.fanOnHumidity) shouldOn = true;
  if (fanOn) {
    if ((!isnan(ambientTempC) && ambientTempC <= CFG.fanOffTempC) && (!isnan(ambientRH) && ambientRH <= CFG.fanOffHumidity)) {
      fanOn = false;
    }
  } else {
    if (shouldOn) fanOn = true;
  }
  setActuator(PIN_RELAY_FAN, fanOn);
}

void controlPump() {
  if (CFG.pumpContinuous) { setActuator(PIN_RELAY_PUMP, true); return; }
  static bool pumpOn = false;
  static unsigned long lastToggle = 0;
  unsigned long interval = pumpOn ? CFG.pumpOnMs : CFG.pumpOffMs;
  if (millis() - lastToggle >= interval) {
    pumpOn = !pumpOn;
    lastToggle = millis();
    setActuator(PIN_RELAY_PUMP, pumpOn);
  }
}

void dailyResetIfNeeded(const DateTime &now) {
  static int lastDay = -1;
  if (now.day() != lastDay) {
    dosePHUp.pulsesToday = dosePHDown.pulsesToday = 0;
    doseECA.pulsesToday = doseECB.pulsesToday = 0;
    dosePHUp.lockedOut = dosePHDown.lockedOut = false;
    doseECA.lockedOut = doseECB.lockedOut = false;
    lastDay = now.day();
  }
}

void setupPins() {
  pinMode(PIN_RELAY_LIGHT, OUTPUT);
  pinMode(PIN_RELAY_PUMP,  OUTPUT);
  pinMode(PIN_RELAY_FAN,   OUTPUT);
  pinMode(PIN_PUMP_NUTRIENT_A, OUTPUT);
  pinMode(PIN_PUMP_NUTRIENT_B, OUTPUT);
  pinMode(PIN_PUMP_PH_UP,      OUTPUT);
  pinMode(PIN_PUMP_PH_DOWN,    OUTPUT);
  setActuator(PIN_RELAY_LIGHT, false);
  setActuator(PIN_RELAY_PUMP,  false);
  setActuator(PIN_RELAY_FAN,   false);
  setActuator(PIN_PUMP_NUTRIENT_A, false);
  setActuator(PIN_PUMP_NUTRIENT_B, false);
  setActuator(PIN_PUMP_PH_UP,      false);
  setActuator(PIN_PUMP_PH_DOWN,    false);
  analogReadResolution(12);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  setupPins();
  dht.begin();
  waterTemp.begin();
  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("RTC not found! Check wiring.");
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println("Hydroponic Controller Started");
}

void loop() {
  nowMs = millis();
  DateTime now = rtc.now();
  if (sensorTimer.runEvery(3000)) {
    readAmbient();
    readWaterTemp();
    readChemistry();
    Serial.print("Time "); Serial.print(now.timestamp());
    Serial.print(" | Air "); Serial.print(ambientTempC); Serial.print("C "); Serial.print(ambientRH); Serial.print("%RH");
    Serial.print(" | Water "); Serial.print(waterTempC); Serial.print("C");
    Serial.print(" | pH "); Serial.print(phValue);
    Serial.print(" | EC "); Serial.print(ec_mS); Serial.println(" mS/cm");
  }
  if (lightTimer.runEvery(10*1000)) {
    controlLights(now);
    dailyResetIfNeeded(now);
  }
  if (envTimer.runEvery(10*1000)) {
    controlFan();
  }
  controlPump();
  bool sensorsOK = !(ALARM.phSensorFault || ALARM.tdsSensorFault || ALARM.tempSensorFault);
  if (sensorsOK && sensorTimer.runEvery(5000)) {
    if (!isnan(phValue)) tryDosePH(phValue);
    if (!isnan(ec_mS))  tryDoseEC(ec_mS);
  }
  static unsigned long badStart = 0;
  if (!sensorsOK) {
    if (badStart==0) badStart = millis();
    if (millis() - badStart > 5UL*60UL*1000UL) {
      dosePHUp.lockedOut = dosePHDown.lockedOut = true;
      doseECA.lockedOut = doseECB.lockedOut = true;
    }
  } else {
    badStart = 0;
  }
}
