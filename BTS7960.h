#pragma once

#define MAX_SIGNAL_MOVE 255
#define BTS7960_MIN_DISTANCE 30

#include <Arduino.h>

struct BTS7960State {
  BTS7960State() : DIRECT_LM(LOW), DIRECT_RM(LOW), LM(LOW), RM(LOW) {}

  unsigned int DIRECT_LM;
  unsigned int DIRECT_RM;
  unsigned int LM;
  unsigned int RM;
};

class BTS7960 {
public:
  BTS7960();
  ~BTS7960();

  void set_logger(HardwareSerial* serial = nullptr);
  void set_pins(unsigned int left_en, unsigned int left_pwm, unsigned int right_en, unsigned int right_pwm);
  void set_input_range(int min, int mid, int max, int offset);
  void set_history_size(unsigned int size); 
  void init();

  void set_mode(int mode);
  void movement(int y, int x);
  bool can_reverse();
  void reverse();
  void stop();

private:
  unsigned int left_motor_en_pin;
  unsigned int left_motor_pwm_pin;
  unsigned int right_motor_en_pin;
  unsigned int right_motor_pwm_pin;

  BTS7960State* history; 
  unsigned int history_index;

  static unsigned int history_size;
  static unsigned int history_counter;

  int movement_mode;

  int min_signal_value;
  int mid_signal_value;
  int max_signal_value;
  int off_signal_value;

  BTS7960State state;

  BTS7960State scale_signals(int y, int x);
  void set_pins();
  void save_state();
  bool is_hard_driving_mode();

  HardwareSerial* logger;
  void log(String val, bool line = true);
};
