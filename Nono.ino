#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <Servo.h>

//================================================================================================
// NETWORK
//================================================================================================

const char* ssid = "Nono";
const char* password = "12345678";

ESP8266WebServer server(80);

//================================================================================================
// SERVOS
//================================================================================================

Servo servo_H;
Servo servo_V;

int SERVO_H_PIN = 15;
int SERVO_V_PIN = 13;

int SERVO_H_VALUE = 90;
int SERVO_V_VALUE = 90;

//================================================================================================
// MOTORS
//================================================================================================

int MOTORS_L_IN1_PIN = 2;
int MOTORS_L_IN2_PIN = 0;
int MOTORS_L_PWM_PIN = 12;

int MOTORS_R_IN1_PIN = 4;
int MOTORS_R_IN2_PIN = 5;
int MOTORS_R_PWM_PIN = 14;

//---------------------------------------------------------------

int MOTORS_L_IN1_VAL = 0;
int MOTORS_L_IN2_VAL = 0;
int MOTORS_L_PWM_VAL = 0;

int MOTORS_R_IN1_VAL = 0;
int MOTORS_R_IN2_VAL = 0;
int MOTORS_R_PWM_VAL = 0;

//================================================================================================
// TIMER
//================================================================================================

Ticker ticker;
volatile unsigned long tim = millis();

void intervalFunc(){
  if(millis() - tim > 150) {
    stopFast();
    tim = millis();
  }
}

//================================================================================================
// SETUP
//================================================================================================

void setup() {

  pinMode(MOTORS_L_IN1_PIN, OUTPUT);
  pinMode(MOTORS_L_IN2_PIN, OUTPUT);
  pinMode(MOTORS_L_PWM_PIN, OUTPUT);
  pinMode(MOTORS_R_IN1_PIN, OUTPUT);
  pinMode(MOTORS_R_IN2_PIN, OUTPUT);
  pinMode(MOTORS_R_PWM_PIN, OUTPUT);

  //---------------------------------------------------------------

  servo_H.attach(SERVO_H_PIN);
  servo_V.attach(SERVO_V_PIN);

  updateState();

  //---------------------------------------------------------------

  server.on("/", HTTP_POST, []()
  {
    if(server.hasArg("plain"))
    {  
      DynamicJsonBuffer bufor;
      JsonObject& json = bufor.parseObject(server.arg("plain"));

      if(json.success())
      {
        if(isValid(json))
        {
          updateState();
          String json = "{\"Nono\":\"OK\"}";  
          server.send( 200, "text/json", json );
          tim = millis();
        }
        else
        {
          String json = "{\"Nono\":\"BAD\"}";  
          server.send( 404, "text/json", json );
        }
      }
    }
  });
  
  server.begin();

  //---------------------------------------------------------------

  WiFi.hostname("NonoControl");
  WiFi.begin(ssid, password);

  Serial.begin(115200);
  Serial.println("\n\nWiFi signal strength (RSSI): " + String(WiFi.RSSI()) + " dBm");

  //---------------------------------------------------------------

  ticker.attach(0.001,intervalFunc);
}

//================================================================================================
// LOOP
//================================================================================================

void loop() {
  server.handleClient();
}

//================================================================================================
// VALIDATION
//================================================================================================

bool isValid(JsonObject& json)
{
  if(json.containsKey("Nono") && json["Nono"].is<JsonArray&>())
  { 
    JsonArray& data = json["Nono"].as<JsonArray&>();
    if(data.size() == 8 && data[0].is<int>() && data[1].is<int>() && data[2].is<int>() && data[3].is<int>() 
    && data[4].is<int>() && data[5].is<int>() && data[6].is<int>() && data[7].is<int>())
    {
      SERVO_H_VALUE = data[0];
      SERVO_V_VALUE = data[1];
      MOTORS_L_IN1_VAL = data[2];
      MOTORS_L_IN2_VAL = data[3];
      MOTORS_L_PWM_VAL = data[4];
      MOTORS_R_IN1_VAL = data[5];
      MOTORS_R_IN2_VAL = data[6];
      MOTORS_R_PWM_VAL = data[7];
      return true;
    }
  }
  return false;
}

//================================================================================================
// UPDATE STATE
//================================================================================================

void updateState()
{
  digitalWrite(MOTORS_L_IN1_PIN, MOTORS_L_IN1_VAL == 0 ? LOW : HIGH);
  digitalWrite(MOTORS_L_IN2_PIN, MOTORS_L_IN2_VAL == 0 ? LOW : HIGH);
  digitalWrite(MOTORS_R_IN1_PIN, MOTORS_R_IN1_VAL == 0 ? LOW : HIGH);
  digitalWrite(MOTORS_R_IN2_PIN, MOTORS_R_IN2_VAL == 0 ? LOW : HIGH);

  analogWrite(MOTORS_L_PWM_PIN, MOTORS_L_PWM_VAL);
  analogWrite(MOTORS_R_PWM_PIN, MOTORS_R_PWM_VAL);

  servo_H.write(SERVO_H_VALUE);
  servo_V.write(SERVO_V_VALUE);
}

//================================================================================================
// FAST STOP
//================================================================================================

void stopFast()
{
  MOTORS_L_IN1_VAL = 1;
  MOTORS_L_IN2_VAL = 1;
  MOTORS_L_PWM_VAL = 1;

  MOTORS_R_IN1_VAL = 1;
  MOTORS_R_IN2_VAL = 1;
  MOTORS_R_PWM_VAL = 1;

  updateState();
}

//================================================================================================
// SOFT STOP
//================================================================================================

void stopSoft()
{
  MOTORS_L_IN1_VAL = 1;
  MOTORS_L_IN2_VAL = 1;
  MOTORS_L_PWM_VAL = 0;

  MOTORS_R_IN1_VAL = 1;
  MOTORS_R_IN2_VAL = 1;
  MOTORS_R_PWM_VAL = 0;

  updateState();
}
