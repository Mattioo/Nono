#ifndef RXNANO45_H
#define RXNANO45_H

#include <Arduino.h>
#include "CrsfSerial.h"

struct State {
  State(int move_y, int move_x, int camera_y, int camera_x, int a, int b, int c, int d)
  {
    Move_Y = move_y;
    Move_X = move_x;
    Camera_Y = camera_y;
    Camera_X = camera_x;

    A = a;
    B = b;
    C = c;
    D = d;
  }

  int Move_Y;
  int Move_X;
  int Camera_Y;
  int Camera_X;
  int A;
  int B;
  int C;
  int D;
};

class RXNANO45 {
public:
  RXNANO45(HardwareSerial& serial);
  void SetLogger(HardwareSerial& logger);
  void Init();
  int GetChannel(unsigned int channel);
  State GetState();
  void Loop();
  static bool IsAlive;

private:
  CrsfSerial receiver;
  HardwareSerial* logger;

  void log(String val, bool line = true);
};

#endif // A02YYUW_H
