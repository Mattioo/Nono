#ifndef BTS7960_H
#define BTS7960_H
#define MAX_SIGNAL_MOVE 255

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
  void SetLogger(HardwareSerial& logger);
  void SetPins(unsigned int left_en, unsigned int left_pwm, unsigned int right_en, unsigned int right_pwm);
  void SetInputRange(int min, int mid, int max, int offset);
  void SetHistorySize(unsigned int size);
  void SetMode(int mode);

  void Init();
  void Movement(int y, int x);
  void Reverse();
  void Stop();

  static unsigned int HistorySize;
  static unsigned int HistoryCounter;

private:
  HardwareSerial* logger;

  unsigned int left_motor_en_pin;
  unsigned int left_motor_pwm_pin;
  unsigned int right_motor_en_pin;
  unsigned int right_motor_pwm_pin;

  BTS7960State* history; 
  unsigned int history_index;

  int movement_mode;

  int min_signal_value;
  int mid_signal_value;
  int max_signal_value;
  int off_signal_value;

  BTS7960State state;

  BTS7960State scaleSignals(int y, int x);
  void setPins();
  void saveState();
  void log(String val, bool line = true);
};

#endif // BTS7960_H
