#include "RXNANO45.h"
#include "SC08A.h"
#include "A02YYUW.h"
#include "BTS7960.h"
#include "OSD.h"

#define BUZZER_PIN 6
#define LIGHT_PIN 8

std::vector<unsigned char> servo_channels = {1, 2};

A02YYUW a02yyuw(Serial);
RXNANO45 rxnano45(Serial1);
SC08A sc08a(Serial2);
OSD osd(Serial3, 110, 13000, 3);
BTS7960 bts7960;

HardwareSerial* setup_logger(HardwareSerial* serial = nullptr) {
  if (serial != nullptr) {
    serial->begin(115200);
  }
  return serial;
}

void setup() {
  HardwareSerial* logger = nullptr; // setup_logger(&Serial);
  setup_outputs();  
  setup_rxnano45(logger);
  setup_sc08a(logger);
  setup_a02yyuw(logger);
  setup_bts7960(logger);
  setup_osd(logger);
}

void setup_outputs() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
}

void setup_rxnano45(HardwareSerial* logger) {
  rxnano45.SetLogger(logger);
  rxnano45.Init();
}

void setup_sc08a(HardwareSerial* logger) {
  sc08a.SetLogger(logger);
  sc08a.SetChannels(servo_channels);
  sc08a.SetInputRange(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  sc08a.Init();

  sc08a.Home();
}

void setup_a02yyuw(HardwareSerial* logger) {
  a02yyuw.SetLogger(logger);
  a02yyuw.Init();
}

void setup_bts7960(HardwareSerial* logger) {
  bts7960.SetLogger(logger);
  bts7960.SetPins(2, 3, 4, 5);
  bts7960.SetInputRange(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MAX, CRSF_CHANNEL_VALUE_OFF);
  bts7960.SetHistorySize(800);
  bts7960.SetMode(CRSF_CHANNEL_VALUE_MIN);
  bts7960.Init();
}

void setup_osd(HardwareSerial* logger) {
  osd.SetLogger(logger);
  osd.set_name("Nono");
  osd.Init();
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
  a02yyuw.ReceiveData();
}

void loop() {
  if (RXNANO45::IsAlive) {
    CrsfSerialState controller = rxnano45.GetState();
    
    if (RXNANO45::ArmState())
    {
      set_light(controller.A);
      bts7960.SetMode(controller.B);

      if
      (
          a02yyuw.IsBackwardMovement(controller.Move_Y) ||
          !check_distance(controller.D) ||
          (a02yyuw.DistanceReceived() && a02yyuw.GetDistance() >= BTS7960_MIN_DISTANCE)
      )
      bts7960.Movement(controller.Move_Y, controller.Move_X);

      if (sc08a.IsPossibleMovementY(controller.Camera_Y)) sc08a.Set({ servo_channels[0] }, controller.Camera_Y);
      if (sc08a.IsPossibleMovementX(controller.Camera_X)) sc08a.SetInverted({ servo_channels[1] }, controller.Camera_X);
    }
  }
  else {
    sc08a.Home();
    if (bts7960.CanReverse()) bts7960.Reverse();
    else bts7960.Stop();
  }

  if (!RXNANO45::IsInit)
    set_buzzer(RXNANO45::IsAlive);
  
  osd.set_arm(RXNANO45::ArmState());
  osd.set_battery_voltage(110, BATTERY_OK);
  
  rxnano45.Loop();
  osd.Loop();
}