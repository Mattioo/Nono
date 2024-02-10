#include "BTS7960.h"

// BTS7960 Manual: https://botland.com.pl/index.php?controller=attachment&id_attachment=843

BTS7960::BTS7960() : history(nullptr), history_size(0), history_index(0), history_row_counter(0) {

}

BTS7960::~BTS7960() {
  delete history;
  history = nullptr;
}

void BTS7960::SetLogger(HardwareSerial& serial) {
    logger = &serial;
}

void BTS7960::SetPins(unsigned int left_en, unsigned int left_pwm, unsigned int right_en, unsigned int right_pwm) {
    left_motor_en_pin = left_en;
    right_motor_en_pin = right_en;
    left_motor_pwm_pin = left_pwm;
    right_motor_pwm_pin = right_pwm;
}

void BTS7960::SetInputRange(unsigned int min, unsigned int mid, unsigned int max, unsigned int offset) {
    min_signal_value = min;
    mid_signal_value = mid;
    max_signal_value = max;
    off_signal_value = offset;
}

void BTS7960::SetHistorySize(unsigned int size) {
  history_size = size;
}

void BTS7960::SetMode(unsigned int mode) {
  movement_mode = mode;
}

void BTS7960::Init() {
  if (logger != nullptr) { logger->begin(115200); }

  if (history_size > 0) {
    history = new State[history_size];
  }
}

void BTS7960::Movement(unsigned int y, unsigned int x) {
  if (y >= min_signal_value && x >= min_signal_value && y <= max_signal_value && x <= max_signal_value)
  {
    state = scaleSignals(y, x);
    setPins();
    saveState();
  }
}

void BTS7960::Reverse() {
  if (history_size > 0)
  {
    history_index = (history_size + --history_index) % history_size;
    history_row_counter++;

    switch(history[history_index].DIRECT_LM) {
      case LOW:
        state.DIRECT_LM = HIGH;
        break;
      case HIGH:
        state.DIRECT_LM = LOW;
        break;
    }
    switch(history[history_index].DIRECT_RM) {
      case LOW:
        state.DIRECT_RM = HIGH;
        break;
      case HIGH:
        state.DIRECT_RM = LOW;
        break;
    }
    
    state.LM = history[history_index].LM;
    state.RM = history[history_index].RM;

    setPins();
  }
}

void BTS7960::Stop() {
  Movement(mid_signal_value, mid_signal_value);
}

State BTS7960::scaleSignals(unsigned int y, unsigned int x) {

  int signalY = (abs((int)(y - mid_signal_value)) >= off_signal_value) ? y : mid_signal_value;
  int signalX = (abs((int)(x - mid_signal_value)) >= off_signal_value) ? x : mid_signal_value;

  int absY = abs((int)(signalY - mid_signal_value));
  int absX = abs((int)(signalX - mid_signal_value));

  int directX = signalX > mid_signal_value ? 1 : 0;
  
  int forceY = float(absY) / (mid_signal_value - min_signal_value) * MAX_SIGNAL_MOVE;
  int forceX = float(absX) / (mid_signal_value - min_signal_value) * forceY;

  State result;

  result.DIRECT_LM = result.DIRECT_RM = (absY != 0) && signalY > mid_signal_value
    ? HIGH
    : LOW;

  // SOFT DRIVING MODE
  if (movement_mode == mid_signal_value) {
    if (directX == 1) {
      result.LM = forceY;
      result.RM = forceY - forceX;
    }
    else {
      result.LM = forceY - forceX;
      result.RM = forceY;
    }
  }
  // HARD DRIVING MODE
  else {
    if (absX > off_signal_value) {
      if (directX == 1) {
        result.DIRECT_LM = HIGH;
        result.DIRECT_RM = LOW;
      }
      else {
        result.DIRECT_LM = LOW;
        result.DIRECT_RM = HIGH;
      }
    }
    result.LM = result.RM = forceY;
  }
  return result;
}

void BTS7960::setPins()
{
  analogWrite(left_motor_pwm_pin, state.LM);
  analogWrite(right_motor_pwm_pin, state.RM);
  digitalWrite(left_motor_en_pin, state.DIRECT_LM);
  digitalWrite(right_motor_en_pin, state.DIRECT_RM);

  log("[BTS7960] DIRECT_LM: " + String(state.DIRECT_LM) + " DIRECT_RM: " + String(state.DIRECT_RM) + " LM: " + String(state.LM) + " RM: " + String(state.RM));
}

void BTS7960::saveState() {
  if (history_size > 0 && (state.LM != LOW || state.RM != LOW))
  {
    history[history_index] = state;
    history_index = ++history_index % history_size;
    history_row_counter = 0;
  }
}

void BTS7960::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}