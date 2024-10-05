#include "OSD.h"

OSD osd(Serial3);

HardwareSerial* setup_logger(HardwareSerial* serial = nullptr) {
  if (serial != nullptr) {
    serial->begin(115200);
  }
  return serial;
}

void setup() {
  osd.set_logger(setup_logger(&Serial));
  osd.init(5000, 6500, 3, 3.3, 4.2);
  osd.set_state(true, 126, 160);
}

void loop() {
  osd.loop();
}
