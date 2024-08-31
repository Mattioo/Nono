#pragma once

#define CRSF_CHANNEL_VALUE_OFF 50

#include <Arduino.h>
#include "CrsfSerial.h"

struct CrsfSerialState {
  CrsfSerialState(int move_y, int move_x, int camera_y, int camera_x, int a, int b, int c, int d, int e, int f)
  {
    Move_Y = move_y;
    Move_X = move_x;
    Camera_Y = camera_y;
    Camera_X = camera_x;

    A = a;
    B = b;
    C = c;
    D = d;
    E = e;
    F = f;
  }

  int Move_Y;
  int Move_X;
  int Camera_Y;
  int Camera_X;
  int A;
  int B;
  int C;
  int D;
  int E;
  int F;
};

class RXNANO45 {
public:
  RXNANO45(HardwareSerial& serial);
  void set_logger(HardwareSerial* serial = nullptr);
  void init();
  void loop();

  static bool arm_state();
  int get_channel(unsigned int channel);
  CrsfSerialState get_state();
  
  static bool IsInit;
  static bool IsAlive;

private:
  CrsfSerial receiver;
  static bool* ARM;

  HardwareSerial* logger;
  void log(String val, bool line = true);
};
