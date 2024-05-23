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

#define BTS_CHANNEL_A 12
#define BTS_CHANNEL_B 13

#define SHOOTING_PWM 243
#define COLOR_SHOOTING 220

//---- Get Balls ----//
// Rolling
#define MOTOR_CHANNEL_A1 8
#define MOTOR_CHANNEL_A2 9
// Rotate (180)
#define SERVO_CHANNEL_A 2
#define SERVO_CHANNEL_B 3
// Front (360)
#define SERVO_CHANNEL_C 4

//---- Drop Balls ----//
#define MOTOR_CHANNEL_B1 10
#define MOTOR_CHANNEL_B2 11
#define SERVO_CHANNEL_D 5

//---- Moving Wheel ----//
#define MOTOR_CHANNEL_C1 13
#define MOTOR_CHANNEL_C2 12
#define MOTOR_CHANNEL_D1 14
#define MOTOR_CHANNEL_D2 15

/*///////////////////////////////////////////////////////
------------------- DEFINE VARIABLES --------------------
///////////////////////////////////////////////////////*/
int rotatingState = 0, rotatingReverseState = 0, shootingState = 0;
int rotatingValue, shootingValue;
int rotatingMode = 0;
int motorC1_speed, motorC2_speed, motorD1_speed, motorD2_speed;
int BTS_ShootingSpeed = 70;
int getball = 0;
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
----------------- SHOOTING MOTOR MODULE -----------------
///////////////////////////////////////////////////////*/
void shootingMotor(int shootingValue, int BTS_ShootingSpeed)
{
  pwm.setPWM(MOTOR_CHANNEL_B2, 0, shootingValue);
  analogWrite(BTS_CHANNEL_A, BTS_ShootingSpeed);
}

/*///////////////////////////////////////////////////////
----------------- ROTATING SERVO MODULE -----------------
///////////////////////////////////////////////////////*/
void rotatingServo(int rotatingValue)
{
  pwm.setPWM(SERVO_CHANNEL_A, 0, rotatingValue);
}

/*///////////////////////////////////////////////////////
----------------- GET BALLS MODULE -----------------
///////////////////////////////////////////////////////*/
void GetBall(int getball){
    if (getball){
        // Roll motor
        pwm.setPWM(MOTOR_CHANNEL_A1, 0, MAX_PWM);
        pwm.setPWM(MOTOR_CHANNEL_A2, 0, 0);
        // Rotate
        pwm.setPWM(SERVO_CHANNEL_A, 0, 60);
    }
    else{
        pwm.setPWM(MOTOR_CHANNEL_A1, 0, 0);
        pwm.setPWM(MOTOR_CHANNEL_A2, 0, 0);

        pwm.setPWM(SERVO_CHANNEL_A, 0, 0);
    }
}
/*///////////////////////////////////////////////////////
----------------- PS3 CONTROLLER MODULE -----------------
///////////////////////////////////////////////////////*/
void PS3_Controller()
{
  //----Moving Wheels----//
  // Left
  if(Ps3.event.button_down.up){
    motorC1_speed = MAX_PWM;
    motorC2_speed = 0;
  }
  else if(Ps3.event.button_down.down){
    motorC1_speed = 0;
    motorC2_speed = MAX_PWM;
  }
  else{
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
  else{
    motorD1_speed = 0;
    motorD2_speed = 0;
  }
  movingMotors(motorC1_speed, motorC2_speed, motorD1_speed, motorD2_speed);

  //----Get Balls----//
  if (Ps3.event.button_down.select){
    if (getball == 1){
      getball = 0;
    }
    else{
      getball = 1;
    }
  }
  
//   shootingMotor(shootingValue, BTS_ShootingSpeed);
  GetBall(getball);
  rotatingServo(rotatingValue);
}

void PS3_onConnect()
{
  if(!Ps3.isConnected())
  {
    digitalWrite(SIGNAL_LED, LOW);
  }
  else
  {
    digitalWrite(SIGNAL_LED, HIGH);
  }
}
/*///////////////////////////////////////////////////////
------------------- CONDITIONS MODULE -------------------
///////////////////////////////////////////////////////*/
void PS3_Condition()
{
  if(Ps3.event.button_down.start)
  {
    ESP.restart();
  }

  if(Ps3.event.button_down.r2)
  {
    shootingState++;
    delay(60);
  }

  if(Ps3.event.button_down.circle)
  {
    shootingState--;
    delay(60);
  }

  if(shootingState > 1 || shootingState < -1)
  {
    shootingState = 0;
  }

  switch(shootingState)
  {
    case 0:
      shootingValue = 0;
      BTS_ShootingSpeed = 0;
      break;
    case 1:
      shootingValue = 3072;
      if(BTS_ShootingSpeed <= SHOOTING_PWM)
      {
        BTS_ShootingSpeed++;
        delay(15);
      }
      break;
    case -1:
      shootingValue = 3072;
      if(BTS_ShootingSpeed <= COLOR_SHOOTING)
      {
        BTS_ShootingSpeed++;
        delay(15);
      }
      break;
  }

  if(Ps3.event.button_down.l1)
  {
    rotatingState++;
    delay(60);
  }

  if(Ps3.event.button_down.l2)
  {
    rotatingState--;
    delay(60);
  }

  if(rotatingState > 1 || rotatingState < -1)
  {
    rotatingState = 0;
  }

  switch(rotatingState)
  {
    case 0:
      rotatingValue = 0;
      break;
    case 1:
      rotatingValue = 105;
      break;
    case -1:
      rotatingValue = 410;
      break;
  }
}
/*///////////////////////////////////////////////////////
------------------- CONFIGURE MODULES -------------------
///////////////////////////////////////////////////////*/
void setup() {
//   Serial.begin(115200);
  pinMode(SIGNAL_LED, OUTPUT);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  movingMotors(0,0,0,0);
  rollingMotor(0);
  shootingMotor(0, 0);
  rotatingServo(0);
  Ps3.attach(PS3_Controller);
  Ps3.begin(MAC_MAKERBOT);
}

void loop() {
  PS3_onConnect();
  PS3_Condition();
}