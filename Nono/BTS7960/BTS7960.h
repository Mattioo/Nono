#ifndef BTS7960_H
#define BTS7960_H
#define MAX_SIGNAL_MOVE 255

#include <Arduino.h>

struct State {
  State() : DIRECT_LM(LOW), DIRECT_RM(LOW), LM(LOW), RM(LOW) {}

  unsigned int DIRECT_LM;
  unsigned int DIRECT_RM;
  unsigned int LM;
  unsigned int RM;
};

class BTS7960 {
public:
  BTS7960();
  ~BTS7960();
  void SetLogger(HardwareSerial& logger);
  void SetPins(unsigned int left_en, unsigned int left_pwm, unsigned int right_en, unsigned int right_pwm);
  void SetInputRange(unsigned int min, unsigned int mid, unsigned int max, unsigned int offset);
  void SetHistorySize(unsigned int size);
  void SetMode(unsigned int mode);

  void Init();
  void Movement(unsigned int y, unsigned int x);
  void Reverse();
  void Stop();

private:
  HardwareSerial* logger;

  unsigned int left_motor_en_pin;
  unsigned int left_motor_pwm_pin;
  unsigned int right_motor_en_pin;
  unsigned int right_motor_pwm_pin;

  State* history; 
  unsigned int history_size;
  unsigned int history_index;
  unsigned int history_row_counter;

  unsigned int movement_mode;

  unsigned int min_signal_value;
  unsigned int mid_signal_value;
  unsigned int max_signal_value;
  unsigned int off_signal_value;

  State state;

  State scaleSignals(unsigned int y, unsigned int x);
  void setPins();
  void saveState();
  void log(String val, bool line = true);
};

#endif // BTS7960_H
