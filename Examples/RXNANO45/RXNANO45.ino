#include "RXNANO45.h"

RXNANO45 rxnano45(Serial1);

void setup() {
  rxnano45.SetLogger(Serial);
  rxnano45.Init();
}

void loop() {
  if (RXNANO45::IsAlive) {
    CrsfSerialState state = rxnano45.GetState();
  }
  rxnano45.Loop();
}
