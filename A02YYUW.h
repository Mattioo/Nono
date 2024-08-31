#pragma once

#include <Arduino.h>
#include "CrsfSerial.h"

class A02YYUW {
public:
  A02YYUW(HardwareSerial& serial);
  void set_logger(HardwareSerial* serial = nullptr);
  void init();

  void receive_data();
  uint16_t get_distance();
  bool distance_received();
  bool is_backward_movement(int Move_Y);
private:
  HardwareSerial* uart;
  unsigned char data[4];
  bool distanceReceived;
  int dataIndex;
  float distance;

  void process_data();

  HardwareSerial* logger;
  void log(String val, bool line = true);
};