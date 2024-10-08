#include "A02YYUW.h"

// A02YYUW Manual: https://wiki.dfrobot.com/_A02YYUW_Waterproof_Ultrasonic_Sensor_SKU_SEN0311

A02YYUW::A02YYUW(HardwareSerial& serial) : distance(0), dataIndex(0), previousMillis(0), interval(100), ledState(false) {
  this->uart = &serial;
}

void A02YYUW::set_logger(HardwareSerial* serial) {
    logger = serial;
}

void A02YYUW::init() {
  uart->begin(9600);
  while (!(&uart));
  delay(1000);
}

uint16_t A02YYUW::get_distance() {
  return (uint16_t)distance;
}

bool A02YYUW::is_backward_movement(int Move_Y) {
  return Move_Y <= CRSF_CHANNEL_VALUE_MID;
}

void A02YYUW::blink() {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
}

void A02YYUW::loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    do {
     for(int i=0;i<4;i++) {
       data[i] = uart->read();
     }
    } while(uart->read() == 0xff);

    uart->flush();

    if(data[0] == 0xff) {
      int sum = (data[0] + data[1] + data[2]) & 0x00FF;
      if(sum == data[3]) {
        distance = ((data[1] << 8) + data[2]) / 10.0;
        blink();
      }
    }

    log("[A02YYUW] Distance: " + String(distance));
  }
}

void A02YYUW::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}
