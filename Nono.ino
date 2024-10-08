#include "RXNANO45.h"
#include "SC08A.h"
#include "A02YYUW.h"
#include "BTS7960.h"
#include "OSD.h"

#define BUZZER_PIN 6
#define LIGHT_PIN 8

std::vector<unsigned char> servo_channels = {1, 2, 3};

A02YYUW a02yyuw(Serial);
RXNANO45 rxnano45(Serial1);
SC08A sc08a(Serial2);
OSD osd(Serial3);
BTS7960 bts7960;

HardwareSerial* setup_logger(HardwareSerial* serial = nullptr) {
  if (serial != nullptr) {
    serial->begin(115200);
  }
  return serial;
}

void setup() {
  HardwareSerial* logger = setup_logger(); // setup_logger(&Serial);
  
  setup_outputs();  
  setup_rxnano45(logger);
  setup_sc08a(logger);
  setup_a02yyuw(logger);
  setup_bts7960(logger);
  setup_osd(logger);

  pinMode(LED_BUILTIN, OUTPUT);
}

void setup_outputs() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
}

void setup_rxnano45(HardwareSerial* logger) {
  rxnano45.set_logger(logger);
  rxnano45.init();
}

void setup_sc08a(HardwareSerial* logger) {
  sc08a.set_logger(logger);
  sc08a.set_channels(servo_channels);
  sc08a.set_input_range(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  sc08a.init();

  sc08a.home();
}

void setup_a02yyuw(HardwareSerial* logger) {
  a02yyuw.set_logger(logger);
  a02yyuw.init();
}

void setup_bts7960(HardwareSerial* logger) {
  bts7960.set_logger(logger);
  bts7960.set_pins(2, 3, 4, 5);
  bts7960.set_input_range(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MAX, CRSF_CHANNEL_VALUE_OFF);
  bts7960.set_history_size(800);
  bts7960.set_mode(CRSF_CHANNEL_VALUE_MIN);
  bts7960.init();
}

void setup_osd(HardwareSerial* logger) {
  osd.set_logger(logger);
  osd.init(5000, 6500, 3, 9.9, 11.1);
}

void set_buzzer(bool isAlive) {
  digitalWrite(BUZZER_PIN, !isAlive ? LOW : HIGH);
}

void set_light(int signal) {
  digitalWrite(LIGHT_PIN, signal == CRSF_CHANNEL_VALUE_MAX ? HIGH : LOW);
}

bool set_distance_sensor(int signal) {
  return signal == CRSF_CHANNEL_VALUE_MAX;
}

float get_voltage() {
  const int numSamples = 10;
  int totalAnalogValue = 0;
  
  for (int i = 0; i < numSamples; i++) {
    totalAnalogValue += analogRead(A0);
    delay(5);
  }
  
  int averageAnalogValue = totalAnalogValue / numSamples;
  
  float measuredVoltage = (averageAnalogValue * 3.3) / 4095;
  return measuredVoltage * 5.0 * 4.2857;
}

void loop() {
  if (RXNANO45::IsAlive) {
    CrsfSerialState controller = rxnano45.get_state();

    if (RXNANO45::arm_state())
    {
      set_light(controller.A);
      bts7960.set_mode(controller.B);

      bts7960.movement(controller.Move_Y, controller.Move_X);

      bool setDistanceSensor = set_distance_sensor(controller.D);

      if (setDistanceSensor && sc08a.is_possible_movement_sensor(controller.Camera_Y)) sc08a.set_inverted({ servo_channels[2] }, controller.Camera_Y);
      else if (sc08a.is_possible_movement_Y(controller.Camera_Y)) sc08a.set({ servo_channels[0] }, controller.Camera_Y);
      if (sc08a.is_possible_movement_X(controller.Camera_X)) sc08a.set_inverted({ servo_channels[1] }, controller.Camera_X);
    }
  }
  else {
    sc08a.home();
    if (bts7960.can_reverse()) bts7960.reverse();
    else bts7960.stop();
  }

  if (!RXNANO45::IsInit)
    set_buzzer(RXNANO45::IsAlive);

  rxnano45.loop();
  a02yyuw.loop();

  osd.set_state(RXNANO45::arm_state(), get_voltage());
  osd.set_distance(a02yyuw.get_distance());
  osd.loop();
}