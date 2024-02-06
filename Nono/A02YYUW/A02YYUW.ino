#include <Arduino.h>
#include "A02YYUW.h"

A02YYUW a02yyuw(Serial1);

void setup() {
  Serial.begin(115200);
  a02yyuw.SetLogger(Serial);
  a02yyuw.Init();
}

void loop() {
  if (a02yyuw.DistanceReceived()) {
    float distance = a02yyuw.GetDistance();
  }
}

/*
  Serial1 - serialEvent1()
  Serial2 - serialEvent2()
  Serial3 - serialEvent3()
*/
void serialEvent1() {
  a02yyuw.ReceiveData();
}