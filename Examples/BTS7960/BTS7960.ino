#include "BTS7960.h"

#define CRSF_CHANNEL_VALUE_MIN 1000
#define CRSF_CHANNEL_VALUE_MID 1500
#define CRSF_CHANNEL_VALUE_MAX 2000
#define CRSF_CHANNEL_VALUE_OFF 50

BTS7960 bts7960;

void setup() {
  bts7960.SetLogger(Serial);
  bts7960.SetPins(2, 3, 4, 5);
  bts7960.SetInputRange(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MAX, CRSF_CHANNEL_VALUE_OFF);
  bts7960.SetHistorySize(800);
  bts7960.SetMode(CRSF_CHANNEL_VALUE_MID);
  bts7960.Init();
}

void loop() {
  bts7960.Movement(CRSF_CHANNEL_VALUE_MAX, CRSF_CHANNEL_VALUE_MID);
}
