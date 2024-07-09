 /*///////////////////////////////////////////////////////
------------------- DEFINE LIBRARIES --------------------
///////////////////////////////////////////////////////*/
#include <Ps3Controller.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
/*///////////////////////////////////////////////////////
------------------- DEFINE CONSTANTS --------------------
///////////////////////////////////////////////////////*/
#define SIGNAL_LED 14
#define MAC_MAKERBOT "34:94:54:5E:E6:20"
#define MAX_PWM 3276

//---- Get Balls ----//
// Rolling
#define MOTOR_CHANNEL_A1 8
#define MOTOR_CHANNEL_A2 9

//---- Drop Balls ----//
// Swop
#define MOTOR_CHANNEL_B1 10
#define MOTOR_CHANNEL_B2 11
// Switch
#define SERVO_CHANNEL_A 2
//Color Reader
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

//---- Wheels ----//
// Left Wheel
#define MOTOR_CHANNEL_C1 13
#define MOTOR_CHANNEL_C2 12
// Right Wheel
#define MOTOR_CHANNEL_D1 14
#define MOTOR_CHANNEL_D2 15

// ---- Anti Gain ---- //
int AntiG = 150;
// ---- Not used pins ---- //
// #define SERVO_CHANNEL_B 3
// #define SERVO_CHANNEL_C 4
// #define SERVO_CHANNEL_D 5

/*///////////////////////////////////////////////////////
------------------- DEFINE VARIABLES --------------------
///////////////////////////////////////////////////////*/
int motorC1_speed, motorC2_speed, motorD1_speed, motorD2_speed;
int getState = 0, dropState = 0;
unsigned long timer = 0;

// Color Reader
int previousColor = -1;
const float threshold = 500;
const float hysteresis = 50;

/*///////////////////////////////////////////////////////
----------------- MOVING MOTORS MODULE ------------------
///////////////////////////////////////////////////////*/
void movingMotors(int motorC1_speed, int motorC2_speed, int motorD1_speed, int motorD2_speed)
{
  pwm.setPWM(MOTOR_CHANNEL_C1, 0, motorC1_speed); 
  pwm.setPWM(MOTOR_CHANNEL_C2, 0, motorC2_speed);
  pwm.setPWM(MOTOR_CHANNEL_D1, 0, motorD1_speed); 
  pwm.setPWM(MOTOR_CHANNEL_D2, 0, motorD2_speed);
}

/*///////////////////////////////////////////////////////
----------------- GET BALLS MODULE ------------------
///////////////////////////////////////////////////////*/
// ---- Rulo Activate ---- //
void Rulo_Activate(int state){
  if(state) pwm.setPWM(MOTOR_CHANNEL_A1, 0, MAX_PWM);
  else pwm.setPWM(MOTOR_CHANNEL_A1, 0, 0);
}

// Main Get Ball Function
void GetBall(){
  if(getState) Rulo_Activate(1);
  else Rulo_Activate(0);
}
/*///////////////////////////////////////////////////////
----------------- GET BALLS MODULE ------------------
///////////////////////////////////////////////////////*/
// ---- Write to Servo ---- //
void WriteDeg(int pin,int deg){
  int pulse = map(deg, 0, 180, 90, 600);
  pwm.setPWM(pin, 0, pulse);
}

// ---- Read color from sensor ---- //
int readColor() {
  // Read the color data
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Calculate the brightness (light intensity)
  float brightness = (r + g + b) / 3.0;

  // Determine the color based on the brightness and hysteresis
  int color;
  if (previousColor == 1 && brightness < (threshold - hysteresis)) {
    color = 0; // Trash ball
  } else if (previousColor == 0 && brightness > (threshold + hysteresis)) {
    color = 1; // Water ball
  } else {
    color = previousColor; // Keep previous color
  }

  // Update color
  previousColor = color;

  return color;
}
// ---- Main Drop Ball Function ---- //
void DropBall(){
  if(dropState){
    pwm.setPWM(MOTOR_CHANNEL_B1, 0, MAX_PWM);
    if (readColor() == 1) WriteDeg(SERVO_CHANNEL_A, 0);
    else WriteDeg(SERVO_CHANNEL_A, 100);
  }
  else pwm.setPWM(MOTOR_CHANNEL_B1, 0, 0);
}

/*///////////////////////////////////////////////////////
----------------- PS3 CONTROLLER MODULE -----------------
///////////////////////////////////////////////////////*/
void PS3_Controller(){
  //----Restart Button----//
  if(Ps3.event.button_down.ps) ESP.restart();
  
  //----Moving Wheels----//
  // Left
  if(Ps3.event.analog_changed.button.up > AntiG){
    motorC1_speed = MAX_PWM;
    motorC2_speed = 0;
  }
  if(Ps3.event.analog_changed.button.down > AntiG){
    motorC1_speed = 0;
    motorC2_speed = MAX_PWM;
  }
  if(Ps3.event.button_up.up || Ps3.event.button_up.down){
    motorC1_speed = 0;
    motorC2_speed = 0;
  }

  // Right
  if(Ps3.event.analog_changed.button.triangle > AntiG){
    motorD1_speed = 0;
    motorD2_speed = MAX_PWM;
  }
  if(Ps3.event.analog_changed.button.cross > AntiG){
    motorD1_speed = MAX_PWM;
    motorD2_speed = 0;
  }
  if(Ps3.event.button_up.triangle || Ps3.event.button_up.cross){
    motorD1_speed = 0;
    motorD2_speed = 0;
  }

  movingMotors(motorC1_speed, motorC2_speed, motorD1_speed, motorD2_speed);

  //----Get Balls----//
  if (Ps3.event.button_down.start){
    getState = !getState;
    GetBall();
  }

  if (Ps3.event.button_down.select){
    dropState = !dropState;
    DropBall();
  }
}

void PS3_onConnect()
{
  if(!Ps3.isConnected()) digitalWrite(SIGNAL_LED, LOW);
  else digitalWrite(SIGNAL_LED, HIGH);
}
/*///////////////////////////////////////////////////////
------------------- CONFIGURE MODULES -------------------
///////////////////////////////////////////////////////*/
void setup() {
//   Serial.begin(115200);
  pinMode(SIGNAL_LED, OUTPUT);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);
  movingMotors(0,0,0,0);
  // rollingMotor(0);
  WriteDeg(SERVO_CHANNEL_A, 0);
  Ps3.attach(PS3_Controller);
  Ps3.begin(MAC_MAKERBOT);
  tcs.begin();
}

void loop() {
  PS3_onConnect();
}
