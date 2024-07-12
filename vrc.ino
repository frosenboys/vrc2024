 /*///////////////////////////////////////////////////////
------------------- DEFINE LIBRARIES --------------------
///////////////////////////////////////////////////////*/
#include <PS2X_lib.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
/*///////////////////////////////////////////////////////
------------------- DEFINE CONSTANTS --------------------
///////////////////////////////////////////////////////*/
#define SIGNAL_LED 14
// #define MAC_MAKERBOT "34:94:54:5E:E6:20"
#define MAX_PWM 3276
// PS2 Pins
#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SLK
// PS2 Modes :
#define pressures false // read analog values from buttons
#define rumble false // vibration

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

//---- Wheels ----//
// Left Wheel
#define MOTOR_CHANNEL_C1 13
#define MOTOR_CHANNEL_C2 12
// Right Wheel
#define MOTOR_CHANNEL_D1 14
#define MOTOR_CHANNEL_D2 15

// ---- Not used pins ---- //
// #define SERVO_CHANNEL_B 3
// #define SERVO_CHANNEL_C 4
// #define SERVO_CHANNEL_D 5

/*///////////////////////////////////////////////////////
------------------- DEFINE VARIABLES --------------------
///////////////////////////////////////////////////////*/
PS2X ps2x; // init PS2x class
int nJoyR, nJoyL ,motorC1_speed, motorC2_speed, motorD1_speed, motorD2_speed;
int getState = 0, dropState = 0;
int ps2Error = -1;
unsigned long timer = 0;

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

// Main Get Ball Function (Roll the rulo)
void GetBall(){
  if(getState) pwm.setPWM(MOTOR_CHANNEL_A1, 0, MAX_PWM);
  else pwm.setPWM(MOTOR_CHANNEL_A1, 0, 0);
}
/*///////////////////////////////////////////////////////
----------------- GET BALLS MODULE ------------------
///////////////////////////////////////////////////////*/
// ---- Write to Servo ---- //
void WriteDeg(int pin,int deg){
  int pulse = map(deg, 0, 180, 90, 600);
  pwm.setPWM(pin, 0, pulse);
}

// ---- Main Drop Ball Function ---- //
void DropBall(){
  if(dropState){
    pwm.setPWM(MOTOR_CHANNEL_B1, 0, MAX_PWM);
    // if (readColor() == 1) WriteDeg(SERVO_CHANNEL_A, 0);
    WriteDeg(SERVO_CHANNEL_A, 100);
  }
  else pwm.setPWM(MOTOR_CHANNEL_B1, 0, 0);
}

/*///////////////////////////////////////////////////////
----------------- PS3 CONTROLLER MODULE -----------------
///////////////////////////////////////////////////////*/
void PS2_Controller(){
  //----Restart Button----//
  if(ps2x.ButtonPressed(PSB_START)) ESP.restart();
  
  //----Moving Wheels----//
  // Left
  nJoyL = ps2x.Analog(PSS_LY);
  if(nJoyL > 134)
  {
    motorC1_speed = map(nJoyL, 134, 255, 0, MAX_PWM);
    motorC2_speed = 0;  
  }
  if(nJoyL < 120)
  {
    motorC1_speed = 0;
    motorC2_speed = map(nJoyL, 120, 0, 0, MAX_PWM);
  }
  if(nJoyL >= 120 && nJoyL <= 134)
  {
    motorC1_speed = 0;
    motorC2_speed = 0;
  }

  // Right
  nJoyR = ps2x.Analog(PSS_RY);
  if(nJoyR > 134)
  {
    motorD1_speed = 0;
    motorD2_speed = map(nJoyR, 134, 255, 0, MAX_PWM);
  }
  if(nJoyR < 120)
  {
    motorD1_speed = map(nJoyR, 120, 0, 0, MAX_PWM);
    motorD2_speed = 0;
  }
  if(nJoyR >= 120 && nJoyR <= 134)
  {
    motorD1_speed = 0;
    motorD2_speed = 0;
  }

  movingMotors(motorC1_speed, motorC2_speed, motorD1_speed, motorD2_speed);

  //----Get Balls----//
  if (ps2x.ButtonPressed(PSB_L1)){
    getState = !getState;
    GetBall();
  }

  if (ps2x.ButtonPressed(PSB_R1)){
    dropState = !dropState;
    DropBall();
  }
}
/*///////////////////////////////////////////////////////
------------------- CONFIGURE MODULES -------------------
///////////////////////////////////////////////////////*/
void setup() {
//   Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);
  movingMotors(0,0,0,0);
  while (ps2Error != 0){
    delay(1000);
    ps2Error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  }
}

void loop() {
  ps2x.read_gamepad(pressures, rumble);
  PS2_Controller();
}