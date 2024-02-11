#include "A02YYUW.h"

A02YYUW::A02YYUW(HardwareSerial& serial) : uart(&serial), distance(0), dataIndex(0), distanceReceived(false) {

}

void A02YYUW::SetLogger(HardwareSerial& serial) {
    logger = &serial;
}

void A02YYUW::Init() {
  uart->begin(9600);
}

float A02YYUW::GetDistance() {
  distanceReceived = false;
  return distance;
}

bool A02YYUW::DistanceReceived() {
  return distanceReceived;
}

void A02YYUW::ReceiveData() {
  static int dataIndex = 0;

  while (uart->available()) {
    unsigned char receivedByte = uart->read();

    if (dataIndex == 0 && receivedByte == 0xFF) {
      dataIndex = 0;
    }

    data[dataIndex++] = receivedByte;

    if (dataIndex == 4) {
      dataIndex = 0;
      processData();
    }
  }

  log("[A02YYUW] Distance: " + String(distance) + " Not Read: " + distanceReceived);
}

void A02YYUW::processData() {
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
