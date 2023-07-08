#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#include <Ps3Controller.h>
// More info on the controller library: https://github.com/jvpernis/esp32-ps3

#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2600
#define FREQUENCY 50

int player = 0;
int battery = 0;
/*
// NodeMCU ESP32-S (Jeff & Aaron)
int ENA = 13;
int ENB = 12;
int IN1 = 5;
int IN2 = 4;
int IN3 = 15;
int IN4 = 2;
*/


// WROOM32 (Everyone else)
int ENA = 2;
int ENB = 4;
int IN1 = 18;
int IN2 = 5;
int IN3 = 17;
int IN4 = 16;

int deadzone = 5;

int dirA = 0;
int dirB = 0;

const int frequency = 500;
const int pwm_channel_A = 0;
const int pwm_channel_B = 0;
const int resolution = 8;

const int minSpeed = 50;
const int maxSpeed = 255;

void onConnect(){
    Serial.println("Connected.");
}

void setup() {

  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(LED_BUILTIN);
  
  Serial.begin(115200);

  //Ps3.attach(notifySkidSteer);
  Ps3.attach(notifyTankDrive);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin();
  
  String address = Ps3.getAddress();
  Serial.print("To use a controller with this ESP32, pair it to: ");
  Serial.println(address);
 
  /*
   
  Serial monitor will show: "To use a controller with this ESP32, pair it to: b4:e6:2d:b7:50:37"
  
  You ***MUST*** plug the PS3 controller into a computer and use 
  SixaxisPairTool or sixaxispairer to save the ESP32's Hardware MAC 
  address into the controller's NVRAM to make it think that this
  ESP32 is the PS3 it's paired with!
  
  Windows, (closed source) SixaxisPairTool: https://dancingpixelstudios.com/sixaxis-controller/sixaxispairtool/
  Linux, Mac, Windows (open source) sixaxispairer: https://github.com/user-none/sixaxispairer
  
  */
  
  // Set up built in LED to use as an indicator
  pinMode(LED_BUILTIN, OUTPUT); 

  // Set up the pins to talk to the L298N Motor Controller
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // The ESP32's built in PWM functions were originally for LEDs, but they are also 
  // compatible with the the L298N's PWM channels
  ledcSetup(pwm_channel_A, frequency, resolution);
  ledcSetup(pwm_channel_B, frequency, resolution);
  ledcAttachPin(ENA, pwm_channel_A);
  ledcAttachPin(ENB, pwm_channel_B);

  // Flash the onboard LED when we boot, useful when the battery starts dying,
  // which causes very fast reboots, which disconnects the controller!
  notifyBooted();
  
  Serial.println("Ready.");
}

void loop() {
  
  //pwm.setPWM(0, 0, pulseWidth(0));
  //delay(1000);
  //pwm.setPWM(0, 0, pulseWidth(180));
  //delay(1000);

  // Normally, most robot code goes in here, but since we're only reacting to controller
  // events, we can just give up and do nothing until the controller is connected:
  if(!Ps3.isConnected())
    return;
}



void notifySkidSteer() {
  // Convert joystick coordinates to polar coordinates
  float joystick_r = sqrt((Ps3.data.analog.stick.lx * Ps3.data.analog.stick.lx) + (Ps3.data.analog.stick.ly * Ps3.data.analog.stick.ly));
  if (joystick_r > 127) {
    joystick_r = 127;
  }
  float joystick_theta = atan2(Ps3.data.analog.stick.ly, Ps3.data.analog.stick.lx);
  
  // Calculate the sine and cosine of the joystick angle
  float sine = sin(joystick_theta);
  float cosine = cos(joystick_theta);

  // Calculate the left and ringhty motor values
  float motor_left = joystick_r * (cosine - sine);
  float motor_right = -1 * (joystick_r * (cosine + sine));
  
  

  // Prints the motor values for testing
  Serial.print(255);
  Serial.print(",");
  Serial.print(-255);
  Serial.print(",");
  Serial.print(motor_left);
  //Serial.print(joystick_r);
  Serial.print(",");
  Serial.println(motor_right);
  //Serial.println(joystick_theta * 127);
    //Tank drive: Left Stick to Left Wheels
  //int leftStickDir = map(motor_left, 127, -128, -100, 100); //<-- could be "100, -100" instead
  //setMotor(0, leftStickDir);
    
    //Tank drive: Right Stick to Right Wheels
  //int rightStickDir = map(motor_right, 127, -128, -100, 100); //<-- could be "100, -100" instead
  //setMotor(1, rightStickDir);
  
}

// Called by PS3 lib when it receives controller input
void notifyTankDrive() {

  // If a side is incorrectly going backwards, try swapping the 
  // neg/pos of the "100" values below for that side:

  //Serial.println(Ps3.data.analog.button.r2, DEC);

  int angleR = map(Ps3.data.analog.button.r2, 0, 255, 0, 180);
  pwm.setPWM(0, 0, pulseWidth(angleR));
 
  int angleL = map(Ps3.data.analog.button.l2, 0, 255, 180, 0);
  pwm.setPWM(1, 0, pulseWidth(angleL));


  //Tank Drive
    //Tank drive: Left Stick to Left Wheels
    int leftStickDir = map(Ps3.data.analog.stick.ly, 127, -128, -100, 100); //<-- could be "100, -100" instead
    setMotor(0, leftStickDir);
    
    //Tank drive: Right Stick to Right Wheels
    int rightStickDir = map(Ps3.data.analog.stick.ry, 127, -128, -100, 100); //<-- could be "100, -100" instead
    setMotor(1, rightStickDir);
  
}

// DO NOT CHANGE:
int pulseWidth(int angle) {
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

// DO NOT CHANGE:
void setMotor(int side, int percent) {
  int motorSpeed = 0;
  switch(side) {
    case 0:
      if (percent > 0) {
        setDirection(0, 1);
      } else if (percent < 0) {
        setDirection(0, -1);
      } else {
        setDirection(0, 0);
        return;
      }
      motorSpeed = map(abs(percent), 0, 100, minSpeed, maxSpeed);
      ledcWrite(pwm_channel_A,motorSpeed);
      break;
      
    case 1:
      if (percent > 0) {
        setDirection(1, 1);
      } else if (percent < 0) {
        setDirection(1, -1);
      } else {
        setDirection(1, 0); 
        return;
      }
      motorSpeed = map(abs(percent), 0, 100, minSpeed, maxSpeed);
      ledcWrite(pwm_channel_B,motorSpeed);
      break;
  }
}

// DO NOT CHANGE:
void setDirection(int side, int dir) {
  switch(side) {
    case 0:
    
      if (dirA != dir) {
        dirA = dir;
        switch(dir) {
          case -1:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            break;
          case 0:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            break;
          case 1:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            break;
          }     
      }
      break;
      
    case 1:
    
      if (dirB != dir) {
        dirB = dir;
        switch(dir) {
          case -1:
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            break;
          case 0:
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            break;
          case 1:
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            break;
          }   
      }  
      break;
      
    }
}

// DO NOT CHANGE:
void notifyBooted(){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
