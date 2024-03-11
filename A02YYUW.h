#ifndef A02YYUW_H
#define A02YYUW_H

#include <Arduino.h>
#include "CrsfSerial.h"

class A02YYUW {
public:
  A02YYUW(HardwareSerial& serial);
  void SetLogger(HardwareSerial* serial = nullptr);
  void Init();
  void ReceiveData();
  float GetDistance();
  bool DistanceReceived();
  bool IsBackwardMovement(int Move_Y);
private:
  HardwareSerial* uart;
  HardwareSerial* logger;
  unsigned char data[4];
  bool distanceReceived;
  int dataIndex;
  float distance;

  void processData();
  void log(String val, bool line = true);
};

#endif // A02YYUW_H
