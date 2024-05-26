 /*///////////////////////////////////////////////////////
------------------- DEFINE LIBRARIES --------------------
///////////////////////////////////////////////////////*/
#include <Ps3Controller.h>
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
// Swop (360)
#define SERVO_CHANNEL_A 2
#define SERVO_CHANNEL_B 3

//---- Drop Balls ----//
// Elevator
#define MOTOR_CHANNEL_B1 10
#define MOTOR_CHANNEL_B2 11
// Drop Hydro
#define SERVO_CHANNEL_C 4
// Drop Oxygen
#define SERVO_CHANNEL_D 5

//---- Wheels ----//
// Left Wheel
#define MOTOR_CHANNEL_C1 13
#define MOTOR_CHANNEL_C2 12
// Right Wheel
#define MOTOR_CHANNEL_D1 14
#define MOTOR_CHANNEL_D2 15

/*///////////////////////////////////////////////////////
------------------- DEFINE VARIABLES --------------------
///////////////////////////////////////////////////////*/
int rotatingState = 0, rotatingReverseState = 0, shootingState = 0;
int motorC1_speed = 0, motorC2_speed = 0, motorD1_speed = 0, motorD2_speed = 0;
int getball = 0;

/*///////////////////////////////////////////////////////
----------------- MOVING MOTORS MODULE ------------------
///////////////////////////////////////////////////////*/
void movingMotors(){
  // Left
  if(Ps3.event.button_down.up){
    motorC1_speed = MAX_PWM;
    motorC2_speed = 0;
  }
  else if(Ps3.event.button_down.down){
    motorC1_speed = 0;
    motorC2_speed = MAX_PWM;
  }
  if(Ps3.event.button_up.up || Ps3.event.button_up.down){
    motorC1_speed = 0;
    motorC2_speed = 0;
  }
  // Right
  if(Ps3.event.button_down.triangle){
    motorD1_speed = 0;
    motorD2_speed = MAX_PWM;
  }
  else if(Ps3.event.button_down.cross){
    motorD1_speed = MAX_PWM;
    motorD2_speed = 0;
  }
  if(Ps3.event.button_up.triangle || Ps3.event.button_up.cross){
    motorD1_speed = 0;
    motorD2_speed = 0;
  }

  pwm.setPWM(MOTOR_CHANNEL_C1, 0, motorC1_speed); 
  pwm.setPWM(MOTOR_CHANNEL_C2, 0, motorC2_speed);
  pwm.setPWM(MOTOR_CHANNEL_D1, 0, motorD1_speed); 
  pwm.setPWM(MOTOR_CHANNEL_D2, 0, motorD2_speed);
}

/*///////////////////////////////////////////////////////
----------------- GET BALLS MODULE -----------------
///////////////////////////////////////////////////////*/
void Roll_Swop(int pin, int direction){
  //Directions
  int us = 0;
  if (direction == 1) us = 1000;
  if (direction == 2) us = 2000;
  // Convert us to pulse
  float pulseLength = 1000000.0 / 60.0;
  pulseLength /= 4096;  // 12-bit
  int pulse = us / pulseLength;

  pwm.setPWM(pin, 0, pulse);
}

void Rulo_Activate(int state){
  if(state) pwm.setPWM(MOTOR_CHANNEL_A1, 0, MAX_PWM);
  else pwm.setPWM(MOTOR_CHANNEL_A1, 0, 0);
}

void GetBall(){
  if (Ps3.event.button_down.select) getball = !getball;
  if(getball){
    Rulo_Activate(1);
    Roll_Swop(SERVO_CHANNEL_A, 1);
    Roll_Swop(SERVO_CHANNEL_B, 1);
  }
  else{
    Rulo_Activate(0);
    Roll_Swop(SERVO_CHANNEL_A, 0);
    Roll_Swop(SERVO_CHANNEL_B, 0);
  }
}
/*///////////////////////////////////////////////////////
----------------- DROP BALLS MODULE ------------------
///////////////////////////////////////////////////////*/
void WriteDeg(int pin,int deg){
  int pulse = map(deg, 0, 180, 90, 600);
  pwm.setPWM(pin, 0, pulse);
}

/*///////////////////////////////////////////////////////
-----------------   PS3 CONTROLLER  ------------------
///////////////////////////////////////////////////////*/
void PS3_Controller(){
  //----Restart Button----//
  if(Ps3.event.button_down.start) ESP.restart();

  //----Moving Wheels----//
  movingMotors();

  //----Get Balls----//
  GetBall();
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
  pinMode(SIGNAL_LED, OUTPUT);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);
  Ps3.attach(PS3_Controller);
  Ps3.begin(MAC_MAKERBOT);
}

void loop() {
  PS3_onConnect();
}
