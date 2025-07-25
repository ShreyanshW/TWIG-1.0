#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===== Motion Control Config =====
#define SERVO_COUNT 5
#define MOVEMENT_CYCLE_MS 20      // Control loop timing (20ms = 50Hz)
#define MIN_SPEED 8               // Degrees per second (slower for smoother motion)
#define MAX_SPEED 45              // Degrees per second (reduced for smoothness)
#define ACCEL_TIME_MS 800         // Time to reach full speed (ms)
#define DECEL_DISTANCE 15         // Start decelerating when within this many degrees

const byte servoChannels[SERVO_COUNT] = {0, 3, 7, 11, 15};
const char* servoNames[SERVO_COUNT] = {"Base", "Shoulder", "Elbow", "Wrist", "Gripper"};

// {minPulse, maxPulse, centerPulse, minAngle, maxAngle, speedFactor, isReversed}
const float servoConfig[SERVO_COUNT][7] = {
  {135, 495, 135, 0, 180, 0.8, false},    // Base (normal)
  {175, 525, 175, -60, 90, 0.6, true},   // Shoulder (REVERSED)
  {175, 520, 348, 0, 180, 0.7, false}, // Elbow (normal)
  {140, 480, 310, 0, 180, 0.6, false},   // Wrist (normal) 
  {320, 530, 425, 0, 90, 0.5, false}     // Gripper (normal)
};

// ===== Motion State Tracking =====
struct ServoState {
  float currentAngle;
  float targetAngle;
  float currentSpeed;
  float targetSpeed;
  unsigned long moveStartTime;
  bool isMoving;
  float totalDistance;
  float startAngle;
} servoState[SERVO_COUNT];

Adafruit_PWMServoDriver pwm;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(100);

  for (byte i = 0; i < SERVO_COUNT; i++) {
    resetServo(i);
  }
  
  Serial.println(F("=== Servo Controller Ready ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("M <servo> <angle> - Move servo"));
  Serial.println(F("S <servo> <speed> - Set speed"));
  Serial.println(F("R - Reset all servos to center"));
  Serial.println(F("P - Print status"));
  Serial.println(F("LIMITS - Show servo limits"));
  Serial.println(F("=============================="));
}

void loop() {
  static unsigned long lastControlCycle = 0;
  if (millis() - lastControlCycle >= MOVEMENT_CYCLE_MS) {
    updateAllServos();
    lastControlCycle = millis();
  }

  if (Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }
}

float constrainAngleToSafeLimits(float angle, byte servoNum) {
  if (servoNum >= SERVO_COUNT) return angle;
  
  float minAngle = servoConfig[servoNum][3];
  float maxAngle = servoConfig[servoNum][4];
  
  float safeMinAngle = minAngle + 0.5;
  float safeMaxAngle = maxAngle - 0.5;
  
  float constrainedAngle = constrain(angle, safeMinAngle, safeMaxAngle);
  
  if (abs(constrainedAngle - angle) > 0.1) {
    Serial.print(F("WARNING: Angle "));
    Serial.print(angle, 1);
    Serial.print(F(" for "));
    Serial.print(servoNames[servoNum]);
    Serial.print(F(" constrained to "));
    Serial.println(constrainedAngle, 1);
  }
  
  return constrainedAngle;
}

int constrainPulseToSafeLimits(int pulse, byte servoNum) {
  if (servoNum >= SERVO_COUNT) return pulse;
  
  int minPulse = (int)servoConfig[servoNum][0];
  int maxPulse = (int)servoConfig[servoNum][1];
  
  int constrainedPulse = constrain(pulse, minPulse, maxPulse);
  
  if (constrainedPulse != pulse) {
    Serial.print(F("WARNING: Pulse "));
    Serial.print(pulse);
    Serial.print(F(" for "));
    Serial.print(servoNames[servoNum]);
    Serial.print(F(" constrained to "));
    Serial.println(constrainedPulse);
  }
  
  return constrainedPulse;
}

bool isAngleInSafeRange(float angle, byte servoNum) {
  if (servoNum >= SERVO_COUNT) return false;
  
  float minAngle = servoConfig[servoNum][3];
  float maxAngle = servoConfig[servoNum][4];
  
  return (angle >= minAngle && angle <= maxAngle);
}

bool isPulseInSafeRange(int pulse, byte servoNum) {
  if (servoNum >= SERVO_COUNT) return false;
  
  int minPulse = (int)servoConfig[servoNum][0];
  int maxPulse = (int)servoConfig[servoNum][1];
  
  return (pulse >= minPulse && pulse <= maxPulse);
}

void moveServoSmooth(byte servoNum, float targetAngle) {
  if (servoNum >= SERVO_COUNT) return;
  
  targetAngle = constrainAngleToSafeLimits(targetAngle, servoNum);
  
  if (!isAngleInSafeRange(targetAngle, servoNum)) {
    Serial.print(F("ERROR: Target angle "));
    Serial.print(targetAngle, 1);
    Serial.print(F(" for "));
    Serial.print(servoNames[servoNum]);
    Serial.println(F(" is not safe. Movement cancelled."));
    return;
  }

  if (abs(targetAngle - servoState[servoNum].currentAngle) < 0.5) {
    return;
  }

  servoState[servoNum].startAngle = servoState[servoNum].currentAngle;
  servoState[servoNum].targetAngle = targetAngle;
  servoState[servoNum].totalDistance = abs(targetAngle - servoState[servoNum].currentAngle);
  servoState[servoNum].moveStartTime = millis();
  servoState[servoNum].isMoving = true;
  servoState[servoNum].currentSpeed = 0;
  
  float baseSpeed = MAX_SPEED * servoConfig[servoNum][5];
  if (servoState[servoNum].totalDistance < 30) {
    baseSpeed *= 0.6;
  }
  servoState[servoNum].targetSpeed = constrain(baseSpeed, MIN_SPEED, MAX_SPEED);
  
  Serial.print(F("Moving "));
  Serial.print(servoNames[servoNum]);
  Serial.print(F(" to "));
  Serial.print(targetAngle, 1);
  Serial.println(F(" degrees"));
}

void updateServoPosition(byte servoNum) {
  if (!servoState[servoNum].isMoving) return;
  
  float currentAngle = servoState[servoNum].currentAngle;
  float targetAngle = servoState[servoNum].targetAngle;
  float remainingDistance = abs(targetAngle - currentAngle);
  
  if (remainingDistance < 0.5) {
    servoState[servoNum].currentAngle = targetAngle;
    servoState[servoNum].isMoving = false;
    servoState[servoNum].currentSpeed = 0;
    
    int pulse = angleToPulse(targetAngle, servoNum);
    pulse = constrainPulseToSafeLimits(pulse, servoNum);
    pwm.setPWM(servoChannels[servoNum], 0, pulse);
    
    Serial.print(servoNames[servoNum]);
    Serial.println(F(" done"));
    return;
  }

  float direction = (targetAngle > currentAngle) ? 1.0 : -1.0;
  unsigned long elapsedTime = millis() - servoState[servoNum].moveStartTime;
  float desiredSpeed = servoState[servoNum].targetSpeed;
  
  if (elapsedTime < ACCEL_TIME_MS) {
    float accelProgress = (float)elapsedTime / ACCEL_TIME_MS;
    desiredSpeed *= accelProgress;
  }
  
  float decelDistance = max(DECEL_DISTANCE, desiredSpeed * 0.5);
  if (remainingDistance <= decelDistance) {
    float decelFactor = remainingDistance / decelDistance;
    desiredSpeed *= decelFactor;
    
    if (desiredSpeed < MIN_SPEED * 0.5) {
      desiredSpeed = MIN_SPEED * 0.5;
    }
  }
  
  float speedDifference = desiredSpeed - servoState[servoNum].currentSpeed;
  float maxSpeedChange = 80.0 * MOVEMENT_CYCLE_MS / 1000.0;
  if (abs(speedDifference) > maxSpeedChange) {
    speedDifference = (speedDifference > 0) ? maxSpeedChange : -maxSpeedChange;
  }
  servoState[servoNum].currentSpeed += speedDifference;
  
  if (servoState[servoNum].currentSpeed < MIN_SPEED * 0.3) {
    servoState[servoNum].currentSpeed = MIN_SPEED * 0.3;
  }
  
  float stepSize = (servoState[servoNum].currentSpeed * MOVEMENT_CYCLE_MS) / 1000.0;
  
  if (stepSize >= remainingDistance) {
    stepSize = remainingDistance;
    servoState[servoNum].currentAngle = targetAngle;
    servoState[servoNum].isMoving = false;
    servoState[servoNum].currentSpeed = 0;
  } else {
    float nextAngle = servoState[servoNum].currentAngle + (stepSize * direction);
    nextAngle = constrainAngleToSafeLimits(nextAngle, servoNum);
    
    float intendedAngle = servoState[servoNum].currentAngle + (stepSize * direction);
    if (abs(nextAngle - intendedAngle) > 0.1) {
      Serial.print(F("SAFETY STOP: "));
      Serial.print(servoNames[servoNum]);
      Serial.println(F(" movement stopped at safety boundary"));
      servoState[servoNum].isMoving = false;
      servoState[servoNum].currentSpeed = 0;
      servoState[servoNum].targetAngle = nextAngle;
    }
    
    servoState[servoNum].currentAngle = nextAngle;
  }
  
  int pulse = angleToPulse(servoState[servoNum].currentAngle, servoNum);
  pulse = constrainPulseToSafeLimits(pulse, servoNum);
  
  if (isPulseInSafeRange(pulse, servoNum)) {
    pwm.setPWM(servoChannels[servoNum], 0, pulse);
  } else {
    Serial.print(F("CRITICAL ERROR: Unsafe pulse "));
    Serial.print(pulse);
    Serial.print(F(" for "));
    Serial.print(servoNames[servoNum]);
    Serial.println(F(" - movement stopped"));
    servoState[servoNum].isMoving = false;
    servoState[servoNum].currentSpeed = 0;
  }
}

void updateAllServos() {
  for (byte i = 0; i < SERVO_COUNT; i++) {
    updateServoPosition(i);
  }
}

void resetServo(byte servoNum) {
  if (servoNum >= SERVO_COUNT) return;
  
  float centerAngle = pulseToAngle(servoConfig[servoNum][2], servoNum);
  centerAngle = constrainAngleToSafeLimits(centerAngle, servoNum);
  
  servoState[servoNum].currentAngle = centerAngle;
  servoState[servoNum].targetAngle = centerAngle;
  servoState[servoNum].currentSpeed = 0;
  servoState[servoNum].targetSpeed = 0;
  servoState[servoNum].isMoving = false;
  servoState[servoNum].totalDistance = 0;
  servoState[servoNum].startAngle = centerAngle;
  
  int centerPulse = (int)servoConfig[servoNum][2];
  centerPulse = constrainPulseToSafeLimits(centerPulse, servoNum);
  pwm.setPWM(servoChannels[servoNum], 0, centerPulse);
}

void resetAllServos() {
  Serial.println(F("Resetting all servos"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    resetServo(i);
  }
}

int angleToPulse(float angle, byte servoNum) {
  if (servoNum >= SERVO_COUNT) return 0;
  
  float minPulse = servoConfig[servoNum][0];
  float maxPulse = servoConfig[servoNum][1];
  float minAngle = servoConfig[servoNum][3];
  float maxAngle = servoConfig[servoNum][4];
  bool isReversed = (bool)servoConfig[servoNum][6];
  
  angle = constrainAngleToSafeLimits(angle, servoNum);
  
  if (isReversed) {
    float pulse = maxPulse - (angle - minAngle) * (maxPulse - minPulse) / (maxAngle - minAngle);
    return constrainPulseToSafeLimits((int)pulse, servoNum);
  }
  
  float pulse = minPulse + (angle - minAngle) * (maxPulse - minPulse) / (maxAngle - minAngle);
  return constrainPulseToSafeLimits((int)pulse, servoNum);
}

float pulseToAngle(int pulse, byte servoNum) {
  if (servoNum >= SERVO_COUNT) return 0;
  
  float minPulse = servoConfig[servoNum][0];
  float maxPulse = servoConfig[servoNum][1];
  float minAngle = servoConfig[servoNum][3];
  float maxAngle = servoConfig[servoNum][4];
  bool isReversed = (bool)servoConfig[servoNum][6];
  
  pulse = constrainPulseToSafeLimits(pulse, servoNum);
  
  if (isReversed) {
    return maxAngle - (pulse - minPulse) * (maxAngle - minAngle) / (maxPulse - minPulse);
  }
  
  return minAngle + (pulse - minPulse) * (maxAngle - minAngle) / (maxPulse - minPulse);
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command.length() == 0) return;
  
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  
  String cmd = command.substring(0, firstSpace == -1 ? command.length() : firstSpace);
  String param1 = "";
  String param2 = "";
  
  if (firstSpace != -1) {
    param1 = command.substring(firstSpace + 1, secondSpace == -1 ? command.length() : secondSpace);
    if (secondSpace != -1) {
      param2 = command.substring(secondSpace + 1);
    }
  }
  
  if (cmd == "M") {
    if (param1.length() > 0 && param2.length() > 0) {
      int servoNum = param1.toInt();
      float angle = param2.toFloat();
      if (servoNum >= 0 && servoNum < SERVO_COUNT) {
        moveServoSmooth(servoNum, angle);
      } else {
        Serial.println(F("Invalid servo number"));
      }
    } else {
      Serial.println(F("Usage: M <servo> <angle>"));
    }
  }
  else if (cmd == "S") {
    if (param1.length() > 0 && param2.length() > 0) {
      int servoNum = param1.toInt();
      float speed = param2.toFloat();
      if (servoNum >= 0 && servoNum < SERVO_COUNT) {
        speed = constrain(speed, MIN_SPEED, MAX_SPEED);
        servoState[servoNum].targetSpeed = speed;
        Serial.print(F("Set speed to "));
        Serial.println(speed);
      } else {
        Serial.println(F("Invalid servo number"));
      }
    } else {
      Serial.println(F("Usage: S <servo> <speed>"));
    }
  }
  else if (cmd == "R") {
    resetAllServos();
  }
  else if (cmd == "P") {
    printStatus();
  }
  else if (cmd == "LIMITS") {
    printLimits();
  }
  else {
    Serial.println(F("Unknown command"));
    Serial.println(F("Available commands: M, S, R, P, LIMITS"));
  }
}

void printStatus() {
  Serial.println(F("Status:"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(servoNames[i]);
    Serial.print(F(" = "));
    Serial.print(servoState[i].currentAngle, 1);
    Serial.print(F("°"));
    if (servoState[i].isMoving) {
      Serial.print(F(" (→ "));
      Serial.print(servoState[i].targetAngle, 1);
      Serial.print(F("°)"));
    }
    if ((bool)servoConfig[i][6]) {
      Serial.print(F(" [REV]"));
    }
    Serial.println();
  }
}

void printLimits() {
  Serial.println(F("Servo Safety Limits:"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(servoNames[i]);
    if ((bool)servoConfig[i][6]) {
      Serial.print(F(" [REVERSED]"));
    }
    Serial.print(F(" - Angle: "));
    Serial.print(servoConfig[i][3], 1);
    Serial.print(F("° to "));
    Serial.print(servoConfig[i][4], 1);
    Serial.print(F("°, Pulse: "));
    Serial.print((int)servoConfig[i][0]);
    Serial.print(F("-"));
    Serial.print((int)servoConfig[i][1]);
    Serial.println();
  }
}
