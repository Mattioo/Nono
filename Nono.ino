#include "RXNANO45.h"
#include "SC08A.h"
#include "A02YYUW.h"
#include "BTS7960.h"

#define BUZZER_PIN 6
#define LIGHT_PIN 8

#define CRSF_CHANNEL_VALUE_OFF 50
#define BTS7960_MIN_DISTANCE 30

bool isInit = true;
std::vector<unsigned char> servo_channels = {1, 2};

RXNANO45 rxnano45(Serial1);
SC08A sc08a(Serial2);
A02YYUW a02yyuw(Serial3);
BTS7960 bts7960;

void setup() {
  setup_Logger();
  setup_Output();
  setup_RXNANO45();
  setup_SC08A();
  setup_A02YYUW();
  setup_BTS7960();
}

void setup_Logger() {
  Serial.begin(115200);
}

void setup_Output() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
}

void setup_RXNANO45() {
  rxnano45.SetLogger(Serial);
  rxnano45.Init();
}

void setup_SC08A() {
  sc08a.SetLogger(Serial);
  sc08a.SetChannels(servo_channels);
  sc08a.SetInputRange(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
  sc08a.Init();

  sc08a.Home();
}

void setup_A02YYUW() {
  a02yyuw.SetLogger(Serial);
  a02yyuw.Init();
}

void setup_BTS7960() {
  bts7960.SetLogger(Serial);
  bts7960.SetPins(2, 3, 4, 5);
  bts7960.SetInputRange(CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MAX, CRSF_CHANNEL_VALUE_OFF);
  bts7960.SetHistorySize(800);
  bts7960.SetMode(CRSF_CHANNEL_VALUE_MIN);
  bts7960.Init();
}

void setBuzzer(bool isAlive) {
  digitalWrite(BUZZER_PIN, (!isInit && !isAlive) ? LOW : HIGH);
}

void setLight(int signal) {
  digitalWrite(LIGHT_PIN, signal == CRSF_CHANNEL_VALUE_MAX ? HIGH : LOW);
}

bool checkDistance(int signal) {
  return signal == CRSF_CHANNEL_VALUE_MAX;
}

// EVENT
void serialEvent3() {
  a02yyuw.ReceiveData();
}

void loop() {
  if (RXNANO45::IsAlive) {
    isInit = false;
    CrsfSerialState controller = rxnano45.GetState();
    
    if (RXNANO45::ARM && *RXNANO45::ARM)
    {
      setLight(controller.A);
      bts7960.SetMode(controller.B);

      if (!checkDistance(controller.D) || a02yyuw.DistanceReceived() && (controller.Move_Y <= CRSF_CHANNEL_VALUE_MID || a02yyuw.GetDistance() >= BTS7960_MIN_DISTANCE)) {
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
    setLight(CRSF_CHANNEL_VALUE_MIN);
  }
  setBuzzer(RXNANO45::IsAlive);
  rxnano45.Loop();
}
