#pragma once

#include <Arduino.h>
#include "CrsfSerial.h"

class A02YYUW {
public:
  A02YYUW(HardwareSerial& serial);
  void set_logger(HardwareSerial* serial = nullptr);
  void init();

  void loop();
  uint16_t get_distance();
  bool is_backward_movement(int Move_Y);
private:
  HardwareSerial* uart;

  unsigned long previousMillis;
  const long interval;
  
  unsigned char data[4];
  int dataIndex;

  float distance;
  bool ledState;

  void blink();

  HardwareSerial* logger;
  void log(String val, bool line = true);
};