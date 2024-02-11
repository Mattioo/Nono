#include "RXNANO45.h"

bool RXNANO45::IsAlive = false;

RXNANO45::RXNANO45(HardwareSerial& serial) : receiver(serial, CRSF_BAUDRATE) {

}

void RXNANO45::SetLogger(HardwareSerial& serial) {
    logger = &serial;
}

void RXNANO45::Init() {
  if (logger != nullptr) { logger->begin(115200); }

  receiver.onPacketChannels = []() {
    IsAlive = true;
  };
  receiver.onLinkDown = []() {
    IsAlive = false;
  };

  receiver.init();
}

int RXNANO45::GetChannel(unsigned int channel) {
  int value = receiver.getChannel(channel);
  log("[RXNANO45] Channel: " + String(channel) + " Value: " + String(value));
  return value;
}

State RXNANO45::GetState() {
  State state(
    receiver.getChannel(1),
    receiver.getChannel(4),
    receiver.getChannel(3),
    receiver.getChannel(2),
    receiver.getChannel(5),
    receiver.getChannel(6),
    receiver.getChannel(7),
    receiver.getChannel(8),
    receiver.getChannel(10),
    receiver.getChannel(9)
  );

  log("[RXNANO45] MOVE_Y: " + String(state.Move_Y) + " MOVE_X: " + String(state.Move_X) + " CAMERA_Y: " + String(state.Camera_Y) + " CAMERA_X: " + String(state.Camera_X) + " A: " + String(state.A) + " B: " + String(state.B) + " C: " + String(state.C) + " D: " + String(state.D) + " E: " + String(state.E) + " F: " + String(state.F));
  return state;
}

void RXNANO45::Loop() {
  receiver.loop();
}

void RXNANO45::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}
