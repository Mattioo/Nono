#include "Servo.h"
#include "CrsfSerial.h"

#define LM_M1_EN_PIN 2
#define LM_M1_PWM_PIN 3
#define RM_M2_EN_PIN 4
#define RM_M2_PWM_PIN 5

#define BUZZER_PIN 6
#define LIGHT_PIN 7

#define SERVO_PIN_H 8
#define SERVO_PIN_V 9

#define CRSF_FRESH_TIME_US 4000
#define CRSF_SIGNAL_OFSET 50

#define HOME_HORIZONTAL 90
#define HOME_VERTICAL 90
#define HOME_VERTICAL_FIX -30

#define SERVO_ANGLE_MOVE 10
#define SERVO_ANGLE_MIN_H 0
#define SERVO_ANGLE_MIN_V HOME_VERTICAL + HOME_VERTICAL_FIX
#define SERVO_ANGLE_MAX_H 180
#define SERVO_ANGLE_MAX_V 170

#define MAX_SIGNAL_MOVE 255
#define MAX_SIGNAL_SERVO 180

#define HISTORY_SIZE 800

#define ULTRASONIC_SENSOR_DISTANCE_CM 30

/*
  MOVE-Y - CH[1]
  MOVE-X - CH[4]

  SERV-H - CH[2]
  SERV-V - CH[3]

  ARM    - CH[6] IN [CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MAX]
  ULTRASONIC_SENSOR_ENABLED    - CH[5] = CRSF_CHANNEL_VALUE_MAX
  HISTORY_REVERSE_ENABLED - CH[8]

  LIGHT MODES:

  1. OFF  CH[7] = CRSF_CHANNEL_VALUE_MIN
  2. ON   CH[7] = CRSF_CHANNEL_VALUE_MID
  3. AUTO CH[7] = CRSF_CHANNEL_VALUE_MAX

  DRIVING MODES:

  1. SOFT CH[6] = CRSF_CHANNEL_VALUE_MID
  2. HARD CH[6] = CRSF_CHANNEL_VALUE_MAX

  *******************************************************************

  PINS:

  [ARDUINO]

  5V <- 5V
  GND <- GND

  6 <- BUZZER #YELLOW
  7 <- LIGHT #BROWN

  8 <- SERV_H #ORANGE
  9 <- SERV_V #YELLOW

  19 #RX1 <- XF Nano 45 #TX RED
  18 #TX1 <- XF Nano 45 #RX BLUE

  MOTORS

  2 <- MOTOR CONTROLLER #M2_EN GREEN
  3 <- MOTOR CONTROLLER #M2_PWM BLUE
  4 <- MOTOR CONTROLLER #M1_EN VIOLET
  5 <- MOTOR CONTROLLER #M1_PWM GREY
*/

// HISTORY STRUCTURE
struct HistoryRow {
  int DIRECT_LM;
  int DIRECT_RM;
  int LM;
  int RM;
};

HistoryRow History[HISTORY_SIZE];
int HistoryIndex = 0, HistoryRowCounter = 0;

// CAMERA SERVOS
Servo CAMERA_H;
Servo CAMERA_V;

// CRSF RECEIVER
CrsfSerial crsf(Serial1, CRSF_BAUDRATE);

volatile bool INIT = true;
volatile bool HEARTBIT = false;
volatile bool ARM = false;
volatile bool ULTRASONIC_SENSOR_ENABLED = false;
volatile bool HISTORY_REVERSE_ENABLED = false;

void packetChannels() {
  HEARTBIT = true;
}

void linkDown() {
  HEARTBIT = false;
}

// ULTRASONIC SENSOR
unsigned char uart[4]={};
int DISTANCE = 0;

// OUTPUT SIGNALS
int DIRECT_LM, DIRECT_RM, LM, RM;
int SERVO_ANGLE_H = HOME_HORIZONTAL, SERVO_ANGLE_V = HOME_VERTICAL + HOME_VERTICAL_FIX;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  
  // START LISTEN CRSF SIGNAL
  crsf.onLinkDown = &linkDown;
  crsf.onPacketChannels = &packetChannels;
  crsf.init();
  
  configure();
}

void configure(){
  // SET BUZZER PIN
  pinMode(BUZZER_PIN, OUTPUT);

  // SET LIGHT PIN
  pinMode(LIGHT_PIN, OUTPUT);
  
  // SET PINS FOR MOTORS CONTROL AS OUTPUT
  pinMode(LM_M1_EN_PIN, OUTPUT);
  pinMode(LM_M1_PWM_PIN, OUTPUT);
  pinMode(RM_M2_EN_PIN, OUTPUT);
  pinMode(RM_M2_PWM_PIN, OUTPUT);

  // HOME POSITION FOR CAMERA
  CAMERA_H.attach(SERVO_PIN_H);
  CAMERA_V.attach(SERVO_PIN_V);

  setHomeForCameraPosition();

  CAMERA_H.write(SERVO_ANGLE_H);
  CAMERA_V.write(SERVO_ANGLE_V);
}

void loop() { 
  if (HEARTBIT) {
    int mode = crsf.getChannel(6);

    INIT = false;
    ARM = (mode == CRSF_CHANNEL_VALUE_MID || mode == CRSF_CHANNEL_VALUE_MAX); // SELECTED
    ULTRASONIC_SENSOR_ENABLED = crsf.getChannel(5) == CRSF_CHANNEL_VALUE_MAX; // PUSHED
    HISTORY_REVERSE_ENABLED = crsf.getChannel(8) == CRSF_CHANNEL_VALUE_MAX; // PUSHED

    if (ARM) {
      scaleSignals(
        crsf.getChannel(1), // MOVE-Y
        crsf.getChannel(4), // MOVE-X
        crsf.getChannel(2), // SERV-H
        crsf.getChannel(3), // SERV-V
        mode
      );
    }
    else {
      setHomeForCameraPosition();
      stopMotors();
    }
  }
  else if (HISTORY_REVERSE_ENABLED && HistoryRowCounter < HISTORY_SIZE) {
    Serial.println("REVERSE");
    setHomeForCameraPosition();
    reverseHistory();
  }
  else {
    Serial.println("STOP");
    setHomeForCameraPosition();
    stopMotors();
  }
    
  setMotors();
  setServos();
  setLight();
  setBuzzer();
  debug();

  crsf.loop();
}

void setBuzzer() {
  if (INIT == false && HEARTBIT == false)
    digitalWrite(BUZZER_PIN, LOW);
  else
    digitalWrite(BUZZER_PIN, HIGH);
}

void setLight() {
  digitalWrite(LIGHT_PIN, crsf.getChannel(7) == CRSF_CHANNEL_VALUE_MIN
    ? HIGH
    : LOW
  );
}

void setHomeForCameraPosition() {
  SERVO_ANGLE_H = HOME_HORIZONTAL;
  SERVO_ANGLE_V = HOME_VERTICAL + HOME_VERTICAL_FIX;
}

void setMotors(){
  if (ULTRASONIC_SENSOR_ENABLED)
  {
    checkDistance();
    if (DISTANCE <= ULTRASONIC_SENSOR_DISTANCE_CM && DIRECT_LM == HIGH && DIRECT_RM == HIGH)
    {
      Serial.println("BLOCKED");
      stopMotors();
    }
  }
  analogWrite(LM_M1_PWM_PIN, LM);
  analogWrite(RM_M2_PWM_PIN, RM);
  digitalWrite(LM_M1_EN_PIN, DIRECT_LM);
  digitalWrite(RM_M2_EN_PIN, DIRECT_RM);
}

void stopMotors() {
  DIRECT_LM = DIRECT_RM = LOW;
  LM = RM = 0;
}

void setServos(){
  CAMERA_H.write(SERVO_ANGLE_H);
  CAMERA_V.write(SERVO_ANGLE_V);
}

int rangeAngle(int angle, int servo_angle_min, int servo_angle_max){
  return min(max(angle, servo_angle_min), servo_angle_max);
}

int computeServoAngle(int currentAngle, int nextAngle){
  return abs(currentAngle - nextAngle) >= SERVO_ANGLE_MOVE ? nextAngle : currentAngle;
}

void saveInHistory(){
  if (LM > 0 || RM > 0) {
    History[HistoryIndex].DIRECT_LM = DIRECT_LM;
    History[HistoryIndex].DIRECT_RM = DIRECT_RM;
    History[HistoryIndex].LM = LM;
    History[HistoryIndex].RM = RM;
  
    HistoryIndex = ++HistoryIndex % HISTORY_SIZE;
    HistoryRowCounter = 0;
  }
}

void reverseHistory(){
  switch(History[HistoryIndex].DIRECT_LM) {
    case LOW:
      DIRECT_LM = HIGH;
      break;
     case HIGH:
      DIRECT_LM = LOW;
      break;
  }
  switch(History[HistoryIndex].DIRECT_RM) {
    case LOW:
      DIRECT_RM = HIGH;
      break;
     case HIGH:
      DIRECT_RM = LOW;
      break;
  }
  
  LM = History[HistoryIndex].LM;
  RM = History[HistoryIndex].RM;
    
  HistoryIndex = (HISTORY_SIZE + --HistoryIndex) % HISTORY_SIZE;
  HistoryRowCounter++;
}

void scaleSignals(int y, int x, int h, int v, int mode){
  
  int signalY = (abs(y - CRSF_CHANNEL_VALUE_MID) >= CRSF_SIGNAL_OFSET) ? y : CRSF_CHANNEL_VALUE_MID;
  int signalX = (abs(x - CRSF_CHANNEL_VALUE_MID) >= CRSF_SIGNAL_OFSET) ? x : CRSF_CHANNEL_VALUE_MID;
  
  int absY = abs(signalY - CRSF_CHANNEL_VALUE_MID);
  int absX = abs(signalX - CRSF_CHANNEL_VALUE_MID);

  int directX = signalX > CRSF_CHANNEL_VALUE_MID ? 1 : 0;
  
  int forceY = float(absY) / (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN) * MAX_SIGNAL_MOVE;
  int forceX = float(absX) / (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN) * forceY;

  DIRECT_LM = DIRECT_RM = (absY != 0) && signalY > CRSF_CHANNEL_VALUE_MID
    ? HIGH
    : LOW;

  // SOFT DRIVING MODE
  if (mode == CRSF_CHANNEL_VALUE_MID) {
    if (directX == 1) {
      LM = forceY;
      RM = forceY - forceX;
    }
    else {
      LM = forceY - forceX;
      RM = forceY;
    }
  }
  // HARD DRIVING MODE
  else {
    if (absX > CRSF_SIGNAL_OFSET) {
      if (directX == 1) {
        DIRECT_LM = HIGH;
        DIRECT_RM = LOW;
      }
      else {
        DIRECT_LM = LOW;
        DIRECT_RM = HIGH;
      }
    }
    LM = RM = forceY;
  }

  int servo_angle_h = 180 - float(h - CRSF_CHANNEL_VALUE_MIN) / (CRSF_CHANNEL_VALUE_MAX / 2) * MAX_SIGNAL_SERVO;
  int servo_angle_v = float(v - CRSF_CHANNEL_VALUE_MIN) / (CRSF_CHANNEL_VALUE_MAX / 2) * MAX_SIGNAL_SERVO + HOME_VERTICAL_FIX;
  
  SERVO_ANGLE_H = rangeAngle(computeServoAngle(SERVO_ANGLE_H, servo_angle_h), SERVO_ANGLE_MIN_H, SERVO_ANGLE_MAX_H);
  SERVO_ANGLE_V = rangeAngle(computeServoAngle(SERVO_ANGLE_V, servo_angle_v), SERVO_ANGLE_MIN_V, SERVO_ANGLE_MAX_V);

  saveInHistory();
}

void checkDistance(){
  delay(20);
  do{
    for(int i=0;i<4;i++)
    {
      uart[i]=Serial2.read();
    }
  }while(Serial2.read()==0xff);

  Serial2.flush();

  if(uart[0]==0xff)
  {
    int sum=(uart[0]+uart[1]+uart[2])&0x00FF;
    if(sum==uart[3])
    {
      DISTANCE = ((uart[1]<<8)+uart[2])/10;
    }
  }
}

void debug(){
  Serial.println("LM: " + String(LM) + " RM: " + String(RM) + " H: " + String(SERVO_ANGLE_H) + " V: " + String(SERVO_ANGLE_V) + " DIRECT_LM: " + String(DIRECT_LM) + " DIRECT_RM: " + String(DIRECT_RM) + " DISTANCE: " + String(DISTANCE));
}
