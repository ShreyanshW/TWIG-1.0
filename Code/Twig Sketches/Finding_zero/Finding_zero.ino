#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVOMIN = 500;
const int SERVOMAX = 2500;

const int servoChannel = 0;      // Change to your servo's channel
const int servoOffset  = -30;    // Counterclockwise offset in degrees

uint16_t angleToPulse(int angle, int offset) {
  int rawAngle = angle + offset;
  return map(rawAngle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  
  // Move to logical 0°, adjusted by offset → actually -30°
  pwm.setPWM(servoChannel, 0, angleToPulse(0, servoOffset));
  // pwm.setPWM(3, 0, angleToPulse(0, servoOffset));
  // pwm.setPWM(7, 0, angleToPulse(0, servoOffset));
  // pwm.setPWM(11, 0, angleToPulse(0, servoOffset));
  // pwm.setPWM(15, 0, angleToPulse(0, servoOffset));    
  
  
}

void loop() {
  // Hold position
}