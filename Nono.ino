#include "PWM.hpp"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_NeoPixel.h>
#include "Servo.h"

#define PWM_PIN_X 2  // [M4: FIOLET]
#define PWM_PIN_Y 3  // [M3: POMARAŃCZ]
#define PWM_PIN_H 18 // [M5: ZIELEŃ]
#define PWM_PIN_V 19 // [M6: NIEBIESKI]

#define SERVO_PIN_H 9
#define SERVO_PIN_V 10

#define LED_PIN 6

#define PWM_MIN_VALUE 1000
#define PWM_MAX_VALUE 2000
#define PWM_OFSET 50

#define PWM_HEARTBEAT_MIN_V 1100

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

#define LED_COUNT 8
#define LED_DISTANCE_CM 30
#define LED_DISTANCE_N 0

#define HISTORY_SIZE 800

#define ULTRASONIC_SENSOR_ENABLED 0

// HISTORY STRUCTURE
struct HistoryRow {
  int Direct;
  int LM;
  int RM;
};

HistoryRow History[HISTORY_SIZE];
int HistoryIndex = 0, HistoryRowCounter = 0;

// FC PWM'S
PWM PWM_SIG_X(PWM_PIN_X);
PWM PWM_SIG_Y(PWM_PIN_Y);
PWM PWM_SIG_H(PWM_PIN_H);
PWM PWM_SIG_V(PWM_PIN_V);

// MOTOR SHIELD
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);

// LED STRIP
Adafruit_NeoPixel STRIP(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// CAMERA SERVOS
Servo CAMERA_H;
Servo CAMERA_V;

// ARM
bool ARM = false;

// HEARTBEAT
bool HEARTBEAT = true;

// ULTRASONIC SENSOR
unsigned char uart[4]={};
int DISTANCE = 0;

// OUTPUT SIGNALS
int DIRECT, LM, RM;
int SERVO_ANGLE_H = HOME_HORIZONTAL, SERVO_ANGLE_V = HOME_VERTICAL + HOME_VERTICAL_FIX;

void setup() {
  Serial.begin(115200);
  configure();
}

void configure(){
  // START READING FC PWM'S
  PWM_SIG_X.begin(true);
  PWM_SIG_Y.begin(true);
  PWM_SIG_H.begin(true);
  PWM_SIG_V.begin(true);

  // HOME POSITION FOR CAMERA
  CAMERA_H.attach(SERVO_PIN_H);
  CAMERA_V.attach(SERVO_PIN_V);

  CAMERA_H.write(SERVO_ANGLE_H);
  CAMERA_V.write(SERVO_ANGLE_V);

  // START MOTOR SHIELD
  AFMS.begin();

  // START LED STRIP
  STRIP.begin();
}

void loop() {
  
  // GET CURRENT PWM VALUES FROM FC
  int pwm_X = PWM_SIG_X.getValue();
  int pwm_Y = PWM_SIG_Y.getValue();
  int pwm_H = PWM_SIG_H.getValue();
  int pwm_V = PWM_SIG_V.getValue();

  HEARTBEAT = pwm_V >= PWM_HEARTBEAT_MIN_V;
  
  if (HEARTBEAT) {
    scaleSignals(
      rangePWM(pwm_X),
      rangePWM(pwm_Y),
      rangePWM(pwm_H),
      rangePWM(pwm_V)
    );
  }
  else if (HistoryRowCounter < HISTORY_SIZE) {
    Serial.println("REVERSE");
    reverseHistory();
  }
  else {
    Serial.println("STOP");
    stopMotors();
  }
  
  checkDistance();
  setMotors();
  setServos();
  debug();
}

void setMotors(){
  if (ULTRASONIC_SENSOR_ENABLED == 1 && DISTANCE <= LED_DISTANCE_CM && DIRECT == FORWARD)
  {
    stopMotors();
  }
  M1->setSpeed(LM);
  M2->setSpeed(RM);
  M1->run(DIRECT);
  M2->run(DIRECT);
}

void setServos(){
  CAMERA_H.write(SERVO_ANGLE_H);
  CAMERA_V.write(SERVO_ANGLE_V);
}

void changeLedColor(byte led, byte red, byte green, byte blue){
  STRIP.setPixelColor(led, red, green, blue);
  STRIP.setBrightness(255);
  STRIP.show();
}

int rangePWM(int pwm){
  return min(max(pwm, PWM_MIN_VALUE), PWM_MAX_VALUE);
}

int rangeAngle(int angle, int servo_angle_min, int servo_angle_max){
  return min(max(angle, servo_angle_min), servo_angle_max);
}

int computeServoAngle(int currentAngle, int nextAngle){
  return abs(currentAngle - nextAngle) >= SERVO_ANGLE_MOVE ? nextAngle : currentAngle;
}

void stopMotors() {
  DIRECT = RELEASE;
  LM = RM = 0;
}

void saveInHistory(){
  History[HistoryIndex].Direct = DIRECT;
  History[HistoryIndex].LM = LM;
  History[HistoryIndex].RM = RM;

  HistoryIndex = ++HistoryIndex % HISTORY_SIZE;
  HistoryRowCounter = 0;
}

void reverseHistory(){
  switch(History[HistoryIndex].Direct) {
    case FORWARD:
      DIRECT = BACKWARD;
      break;
     case BACKWARD:
      DIRECT = FORWARD;
      break;
     default:
      DIRECT = RELEASE;
      break;
  }
  
  LM = History[HistoryIndex].LM;
  RM = History[HistoryIndex].RM;
    
  HistoryIndex = (HISTORY_SIZE + --HistoryIndex) % HISTORY_SIZE;
  HistoryRowCounter++;
}

void scaleSignals(int x, int y, int h, int v){
  int center =  PWM_MIN_VALUE + (PWM_MAX_VALUE - PWM_MIN_VALUE) / 2;
 
  int signalY = (abs(y - center) >= PWM_OFSET) ? y : center;
  int signalX = (abs(x - center) >= PWM_OFSET) ? x : center;
  
  int absY = abs(signalY - center);
  int absX = abs(signalX - center);

  if (ARM) {
    
    DIRECT = (absY != 0)
    ? (signalY > center ? FORWARD : BACKWARD)
    : RELEASE;
  
    int directX = signalX > center ? 1 : 0;
    
    int forceY = float(absY) / (center - PWM_MIN_VALUE) * MAX_SIGNAL_MOVE;
    int forceX = float(absX) / (center - PWM_MIN_VALUE) * forceY;
  
    LM = directX == 1
    ? forceY
    : forceY - forceX;
    
    RM = directX == 0 
    ? forceY
    : forceY - forceX;

    int servo_angle_h = 180 - float(h - PWM_MIN_VALUE) / (PWM_MAX_VALUE / 2) * MAX_SIGNAL_SERVO;
    int servo_angle_v = float(v - PWM_MIN_VALUE) / (PWM_MAX_VALUE / 2) * MAX_SIGNAL_SERVO + HOME_VERTICAL_FIX;
    
    SERVO_ANGLE_H = rangeAngle(computeServoAngle(SERVO_ANGLE_H, servo_angle_h), SERVO_ANGLE_MIN_H, SERVO_ANGLE_MAX_H);
    SERVO_ANGLE_V = rangeAngle(computeServoAngle(SERVO_ANGLE_V, servo_angle_v), SERVO_ANGLE_MIN_V, SERVO_ANGLE_MAX_V);
  }
  else if (absY == 0){
    ARM = true;
  }

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
      setDistanceLed();
    }
  }
}

void setDistanceLed(){
  changeLedColor(LED_DISTANCE_N, (DISTANCE <= LED_DISTANCE_CM ? 255 : 0), 0, (DISTANCE > LED_DISTANCE_CM ? 255 : 0));
}

void debug(){
  Serial.println("LM: " + String(LM) + " RM: " + String(RM) + " H: " + String(SERVO_ANGLE_H) + " V: " + String(SERVO_ANGLE_V) + " D: " + String(DIRECT));
}
