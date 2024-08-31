#pragma once

#include <Wire.h>
#include <Adafruit_INA219.h>

struct INA219State {
  INA219State(float shunt_voltage = 0, float bus_voltage = 0, float current_mA = 0, float power_mW = 0, float load_voltage = 0, float load_voltage_per_cell = 0, float drawn_mAH = 0) : ShuntVoltage(shunt_voltage), BusVoltage(bus_voltage), Current_mA(current_mA), Power_mW(power_mW), LoadVoltage(load_voltage), LoadVoltagePerCell(load_voltage_per_cell), Drawn_mAH(drawn_mAH) {}

  float ShuntVoltage;
  float BusVoltage;
  float Current_mA;
  float Power_mW;
  float LoadVoltage;
  float LoadVoltagePerCell;
  double Drawn_mAH;

  uint16_t Capacity;
  uint8_t Cells;
  float Min_voltage_per_cell;
  float Max_voltage_per_cell;
};

class INA219 {
public:
  INA219(uint16_t capacity, uint8_t cells, float min_voltage_per_cell, float max_voltage_per_cell);
  void init();
  void set_logger(HardwareSerial* serial = nullptr);
  void loop();

  INA219State get_state();

private:
  Adafruit_INA219 ina219;
  INA219State state;

  unsigned long initMillis;
  unsigned long previousMillis;
  const long interval;

  HardwareSerial* logger;
  void log(String val, bool line = true);
};
