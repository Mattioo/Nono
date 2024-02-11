#include "SC08A.h"

std::vector<unsigned char> channels = {1, 2};
SC08A sc08a(Serial1);

void setup() {
  sc08a.SetChannels(channels);
  sc08a.SetLogger(Serial);
  sc08a.SetInputRange(0, 1024);
  sc08a.Init();
}

void loop() {
  sc08a.Set({ channels[0] }, 0, 50);
  sc08a.Set({ channels[1] }, 0);
  delay(4000);
  sc08a.Home();
  delay(2000);
  sc08a.Set({ channels[0] }, 1024, 50);
  sc08a.Set({ channels[1] }, 1024);
  delay(4000);
}
