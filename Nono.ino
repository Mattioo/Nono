#include "RXNANO45.h"
#include "SC08A.h"
#include "A02YYUW.h"
#include "BTS7960.h"
#include "INA219.h"
#include "OSD.h"

#define BUZZER_PIN 6
#define LIGHT_PIN 8

std::vector<unsigned char> servo_channels = {1, 2};

A02YYUW a02yyuw(Serial);
RXNANO45 rxnano45(Serial1);
SC08A sc08a(Serial2);
OSD osd(Serial3, "Nono");
INA219 ina219(6500, 3, 3.3, 4.2);
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
  setup_ina219(logger);
  setup_osd(logger);
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

void setup_ina219(HardwareSerial* logger) {
  ina219.set_logger(logger);
  ina219.init();
}

void setup_osd(HardwareSerial* logger) {
  osd.set_logger(logger);
  osd.init(15000);
}

void set_buzzer(bool isAlive) {
  digitalWrite(BUZZER_PIN, !isAlive ? LOW : HIGH);
}

void set_light(int signal) {
  digitalWrite(LIGHT_PIN, signal == CRSF_CHANNEL_VALUE_MAX ? HIGH : LOW);
}

bool check_distance(int signal) {
  return signal == CRSF_CHANNEL_VALUE_MAX;
}

// EVENT
void serialEvent() {
  a02yyuw.receive_data();
}

void loop() {
  if (RXNANO45::IsAlive) {
    CrsfSerialState controller = rxnano45.get_state();
    
    if (RXNANO45::arm_state())
    {
      set_light(controller.A);
      bts7960.set_mode(controller.B);

      if (a02yyuw.distance_received()) {
        osd.set_sensor_distance(a02yyuw.get_distance());
      }

      bts7960.movement(controller.Move_Y, controller.Move_X);

      if (sc08a.is_possible_movement_Y(controller.Camera_Y)) sc08a.set({ servo_channels[0] }, controller.Camera_Y);
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
  ina219.loop();

  osd.set_arm(RXNANO45::arm_state());
  osd.set_battery_state(ina219.get_state());
  osd.loop();
}