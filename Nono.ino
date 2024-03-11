#include "RXNANO45.h"
#include "SC08A.h"
#include "A02YYUW.h"
#include "BTS7960.h"
#include "OSD.h"

#define BUZZER_PIN 6
#define LIGHT_PIN 8

#define CRSF_CHANNEL_VALUE_OFF 50
#define BTS7960_MIN_DISTANCE 30

uint32_t flightModeFlags = 0x00000002; // DISARM

bool isInit = true;
std::vector<unsigned char> servo_channels = {1, 2};

A02YYUW a02yyuw(Serial);
RXNANO45 rxnano45(Serial1);
SC08A sc08a(Serial2);
OSD osd(Serial3, 500);
BTS7960 bts7960;

void setup_Output() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
}

HardwareSerial* setup_Logger(HardwareSerial* serial = nullptr) {
  if (serial != nullptr) {
    serial->begin(115200);
  }
  return serial;
}

void setup() {
  setup_Output();
  HardwareSerial* logger = nullptr; // setup_Logger(&Serial);
 
  setup_RXNANO45(logger);
  setup_SC08A(logger);
  setup_A02YYUW(logger);
  setup_BTS7960(logger);
  setup_OSD(logger);
}

void setup_RXNANO45(HardwareSerial* logger) {
  rxnano45.SetLogger(logger);
  rxnano45.Init();
}

void setup_SC08A(HardwareSerial* logger) {
  sc08a.SetLogger(logger);
  sc08a.SetChannels(servo_channels);
  sc08a.SetInputRange(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  sc08a.Init();

  sc08a.Home();
}

void setup_A02YYUW(HardwareSerial* logger) {
  a02yyuw.SetLogger(logger);
  a02yyuw.Init();
}

void setup_BTS7960(HardwareSerial* logger) {
  bts7960.SetLogger(logger);
  bts7960.SetPins(2, 3, 4, 5);
  bts7960.SetInputRange(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MAX, CRSF_CHANNEL_VALUE_OFF);
  bts7960.SetHistorySize(800);
  bts7960.SetMode(CRSF_CHANNEL_VALUE_MIN);
  bts7960.Init();
}

void setup_OSD(HardwareSerial* logger) {
  osd.SetLogger(logger);
  osd.Set_Status(flightModeFlags);
  osd.Set_Status_Ex(flightModeFlags);
  osd.Set_Name("Nono");
  osd.Init();
}

void set_buzzer(bool isAlive) {
  digitalWrite(BUZZER_PIN, (!isInit && !isAlive) ? LOW : HIGH);
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
    isInit = false;
    CrsfSerialState controller = rxnano45.GetState();
    
    if (RXNANO45::ARM && *RXNANO45::ARM)
    {
      flightModeFlags = 0x00000003; // ARM

      osd.Set_Status(flightModeFlags);
      osd.Set_Status_Ex(flightModeFlags);

      set_light(controller.A);
      bts7960.SetMode(controller.B);

      if (!check_distance(controller.D) || a02yyuw.DistanceReceived() && (controller.Move_Y <= CRSF_CHANNEL_VALUE_MID || a02yyuw.GetDistance() >= BTS7960_MIN_DISTANCE)) {
        bts7960.Movement(controller.Move_Y, controller.Move_X);
      }

      if (controller.Camera_Y >= CRSF_CHANNEL_VALUE_MID && controller.Camera_Y <= (CRSF_CHANNEL_VALUE_MID + (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID)/2)) {
        sc08a.Set({ servo_channels[0] }, controller.Camera_Y);
      }
      if (controller.Camera_X >= (CRSF_CHANNEL_VALUE_MIN + (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN)/2) && controller.Camera_X <= (CRSF_CHANNEL_VALUE_MID + (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID)/2)) {
        sc08a.Set({ servo_channels[1] }, CRSF_CHANNEL_VALUE_MIN + CRSF_CHANNEL_VALUE_MAX - controller.Camera_X);
      }
    }
  }
  else {
    sc08a.Home();
    if (BTS7960::HistoryCounter < BTS7960::HistorySize) {
      bts7960.Reverse();
    }
    else {
      bts7960.Stop();
    }
    set_light(CRSF_CHANNEL_VALUE_MIN);
  }
  set_buzzer(RXNANO45::IsAlive);
  
  osd.Set_Analog(110);
  osd.Set_Battery_State(110, 13000);

  rxnano45.Loop();
  osd.Loop();
}
