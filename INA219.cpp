#include "INA219.h"

// INA219 Manual: https://www.sigmaelectronica.net/manuals/ADAFR-904.pdf

INA219::INA219(uint16_t capacity, uint8_t cells, float min_voltage_per_cell, float max_voltage_per_cell): previousMillis(0), interval(10), initMillis(0) {
  state.Capacity = capacity;
  state.Cells = cells;
  state.Min_voltage_per_cell = min_voltage_per_cell;
  state.Max_voltage_per_cell = max_voltage_per_cell;
}

void INA219::set_logger(HardwareSerial* serial) {
    logger = serial;
}

void INA219::init() {
  if (!ina219.begin()) { while (1) { delay(10); } }
  log("[INA219] START");
}

void INA219::loop() {
   unsigned long currentMillis = millis();
   unsigned long elapsedTime = currentMillis - previousMillis;

   if (elapsedTime >= interval) {
    previousMillis = currentMillis;

    state.ShuntVoltage = ina219.getShuntVoltage_mV();
    state.BusVoltage = ina219.getBusVoltage_V();
    state.Current_mA = ina219.getCurrent_mA();
    state.Power_mW = ina219.getPower_mW();
    state.LoadVoltage = state.BusVoltage + (state.ShuntVoltage / 1000);
    state.LoadVoltagePerCell = state.LoadVoltage / state.Cells;
    state.Drawn_mAH += (state.Current_mA * static_cast<double>(elapsedTime) / 3600000);

    log("[INA219]");
    log("Bus Voltage: " + String(state.BusVoltage) + "V");
    log("Shunt Voltage: " + String(state.ShuntVoltage) + "mV");
    log("Load Voltage: " + String(state.LoadVoltage) + "V");
    log("Load Voltage Per Cell: " + String(state.LoadVoltagePerCell) + "V");
    log("Drawn_mAH: " + String(state.Drawn_mAH) + "V");
    log("Current: " + String(state.Current_mA) + "mA");
    log("Power: " + String(state.Power_mW) + "mV");
   }
}

INA219State INA219::get_state() {
  return state;
}

void INA219::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}