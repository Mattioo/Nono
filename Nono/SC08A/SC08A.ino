#include <vector>
#include "SC08A.h"

#define CRSF_CHANNEL_VALUE_MIN 1000
#define CRSF_CHANNEL_VALUE_MAX 2000

std::vector<unsigned char> channels = {1, 2};
SC08A sc08a(Serial1);

void setup() {
  sc08a.SetChannels(channels);
  sc08a.SetLogger(Serial);
  sc08a.SetInputRange(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  sc08a.Init();
}

void loop() {
  sc08a.Set({ channels[0] }, CRSF_CHANNEL_VALUE_MIN, 50);
  sc08a.Set({ channels[1] }, CRSF_CHANNEL_VALUE_MIN);
  delay(4000);
  sc08a.Home();
  delay(2000);
  sc08a.Set({ channels[0] }, CRSF_CHANNEL_VALUE_MAX, 50);
  sc08a.Set({ channels[1] }, CRSF_CHANNEL_VALUE_MAX);
  delay(4000);
}
