#include "A02YYUW.h"

// A02YYUW Manual: https://wiki.dfrobot.com/_A02YYUW_Waterproof_Ultrasonic_Sensor_SKU_SEN0311

A02YYUW::A02YYUW(HardwareSerial& serial) : distance(0), dataIndex(0), distanceReceived(false) {
  this->uart = &serial;
}

void A02YYUW::set_logger(HardwareSerial* serial) {
    logger = serial;
}

void A02YYUW::init() {
  uart->begin(9600); while (!(*uart));
}

uint16_t A02YYUW::get_distance() {
  distanceReceived = false;
  return (uint16_t)distance;
}

bool A02YYUW::distance_received() {
  return distanceReceived;
}

bool A02YYUW::is_backward_movement(int Move_Y) {
  return Move_Y <= CRSF_CHANNEL_VALUE_MID;
}

void A02YYUW::receive_data() {
  static int dataIndex = 0;

  while (uart->available()) {
    unsigned char receivedByte = uart->read();

    if (dataIndex == 0 && receivedByte == 0xFF) {
      dataIndex = 0;
    }

    data[dataIndex++] = receivedByte;

    if (dataIndex == 4) {
      dataIndex = 0;
      process_data();
    }
  }

  log("[A02YYUW] Distance: " + String(distance) + " Not Read: " + distanceReceived);
}

void A02YYUW::process_data() {
  if (data[0] == 0xFF) {
    int sum = (data[0] + data[1] + data[2]) & 0x00FF;
    if (sum == data[3]) {
      distance = ((data[1] << 8) + data[2]) / 10.0;
      distanceReceived = true;
    }
  }
}

void A02YYUW::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}
