#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===== Arm Configuration =====
#define SERVO_COUNT 5
const byte servoChannels[SERVO_COUNT] = {0, 4, 7, 11, 15};
const char* servoNames[SERVO_COUNT] = {"Base", "Shoulder", "Elbow", "Wrist", "Gripper"};

// Servo configuration: {minPulse, maxPulse, minAngle, maxAngle, speedFactor}
const float servoConfig[SERVO_COUNT][5] = {
  {135, 495, 0, 180, 0.8},    // Base
  {175, 465, -60, 90, 0.6},   // Shoulder
  {175, 520, 0, 180, 0.7},    // Elbow  
  {90, 350, 0, 180, 0.6},     // Wrist
  {320, 530, 0, 180, 0.5}      // Gripper (adjusted max angle to 180)
};

// Motion control parameters
#define MOVEMENT_CYCLE_MS 5    // 50Hz control loop
#define SERVO_SPEED 40.0       // deg/sec
#define ACCEL_TIME_MS 250      // Time to reach full speed
#define DECEL_DISTANCE 10      // Deceleration distance (deg)
#define ANGLE_TOLERANCE 1.5    // Tolerance in degrees for target completion

Adafruit_PWMServoDriver pwm;

struct ServoState {
  float currentAngle;
  float targetAngle;
  bool isMoving;
  unsigned long moveStartTime;
} servoState[SERVO_COUNT];

// Starting positions for servos
const float startingPositions[SERVO_COUNT] = {
  90,   // Base center
  45,   // Shoulder neutral
  0,    // Elbow slight bend 
  90,   // Wrist center
  45    // Gripper half open
};

// Demo sequence steps with reduced wait times
struct DemoStep {
  byte servoNum;
  float targetAngle;
  unsigned long holdTime;
  bool waitForComplete;
};

const DemoStep demoSequence[] = {
  // Wave sequence with optimized timing
  {0, 0, 400, true},    // Turn left
  {0, 180, 400, true},
   {0, 90, 400, true},
  {1, 0, 300, true},    // Shoulder up
  {3, 150, 50, false},  // Wrist up
  {3, 0, 50, false},   // Wrist down
  {3, 90, 400, true},    // Wrist center
  
  // Return to center 
  {0, 90, 400, true},    // Base center
  {1, -30, 500, true},   // Bow
  {1, 0, 500, true},     // Stand
  
  // Final position
  {2, 90, 250, true},    // Elbow up
  {4, 0, 250, true},     // Gripper closed
  {4, 180, 250, true}    // Gripper fully open
};

const int totalSteps = sizeof(demoSequence) / sizeof(DemoStep);
int currentStep = 0;
bool demoActive = false;
unsigned long stepStartTime = 0;
bool stepInProgress = false;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);
  
  // Initialize to starting positions
  Serial.println("Moving to starting positions...");
  for (byte i = 0; i < SERVO_COUNT; i++) {
    moveServo(i, startingPositions[i]);
    servoState[i].currentAngle = startingPositions[i]; // Set current angle
    delay(250);
  }
  
  Serial.println(F("\nReady. Commands:"));
  Serial.println("DEMO - Start demo sequence");
  Serial.println("STATUS - Show current angles");
}

void loop() {
  static unsigned long lastUpdate = 0;
  
  // Handle serial commands
  processSerialInput();

  // Control loop running at 50Hz
  if (millis() - lastUpdate >= MOVEMENT_CYCLE_MS) {
    lastUpdate = millis();
    
    // Update all servos
    for (byte i = 0; i < SERVO_COUNT; i++) {
      updateServoPosition(i);
    }

    // Run demo state machine
    if (demoActive) {
      runDemoStep();
    }
  }
}

void processSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    
    if (input.equalsIgnoreCase("DEMO")) {
      startDemo();
    }
    else if (input.equalsIgnoreCase("STATUS")) {
      printServoStatus();
    }
  }
}

void startDemo() {
  if (!demoActive) {
    Serial.println("\nStarting demo sequence...");
    demoActive = true;
    currentStep = 0;
    stepInProgress = false;
  }
}

void runDemoStep() {
  // Check if we need to start a new step
  if (!stepInProgress) {
    if (currentStep >= totalSteps) {
      // Return to starting positions after demo completion
      Serial.println("\nDemo complete! Returning to starting positions...");
      for (byte i = 0; i < SERVO_COUNT; i++) {
        moveServo(i, startingPositions[i]);
      }
      demoActive = false;
      return;
    }
    
    const DemoStep* step = &demoSequence[currentStep];
    moveServo(step->servoNum, step->targetAngle);
    
    Serial.print(F("\nStep "));
    Serial.print(currentStep + 1);
    Serial.print(F("/"));
    Serial.print(totalSteps);
    Serial.print(F(": "));
    Serial.print(servoNames[step->servoNum]);
    Serial.print(F(" to "));
    Serial.print(step->targetAngle);
    Serial.println(F("°"));
    
    stepStartTime = millis();
    stepInProgress = true;
    return;
  }

  // Check if current step is complete
  const DemoStep* step = &demoSequence[currentStep];
  
  // Movement complete when servo is within tolerance AND hold time has elapsed
  bool movementComplete = fabs(servoState[step->servoNum].currentAngle - step->targetAngle) <= ANGLE_TOLERANCE;
  bool holdTimeComplete = (millis() - stepStartTime) >= step->holdTime;
  
  // Special case: Non-waiting steps can advance immediately
  if (!step->waitForComplete && movementComplete) {
    stepInProgress = false;
    currentStep++;
    return;
  }

  // Normal case: Waiting steps require both conditions
  if (step->waitForComplete && movementComplete && holdTimeComplete) {
    stepInProgress = false;
    currentStep++;
  }
}

void moveServo(byte servoNum, float angle) {
  if (servoNum >= SERVO_COUNT) return;
  
  angle = constrain(angle, servoConfig[servoNum][2], servoConfig[servoNum][3]);
  servoState[servoNum].targetAngle = angle;
  servoState[servoNum].isMoving = true;
  servoState[servoNum].moveStartTime = millis();
  
  int pulse = map(angle * 10, servoConfig[servoNum][2] * 10, servoConfig[servoNum][3] * 10, 
                 servoConfig[servoNum][0], servoConfig[servoNum][1]);
  pwm.setPWM(servoChannels[servoNum], 0, pulse);
}

void updateServoPosition(byte servoNum) {
  if (!servoState[servoNum].isMoving) return;

  float current = servoState[servoNum].currentAngle;
  float target = servoState[servoNum].targetAngle;
  float remaining = abs(target - current);

  // Movement completion check with tolerance
  if (remaining <= ANGLE_TOLERANCE) {
    servoState[servoNum].currentAngle = target;
    servoState[servoNum].isMoving = false;
    return;
  }

  // Calculate movement step with acceleration/deceleration
  float step = (SERVO_SPEED * MOVEMENT_CYCLE_MS) / 1000.0 * servoConfig[servoNum][4];
  unsigned long elapsed = millis() - servoState[servoNum].moveStartTime;

  // Acceleration phase
  if (elapsed < ACCEL_TIME_MS) {
    step *= (float)elapsed / ACCEL_TIME_MS;
  }
  // Deceleration phase
  else if (remaining <= DECEL_DISTANCE) {
    step *= remaining / DECEL_DISTANCE;
  }

  // Apply movement
  float newAngle = current + (target > current ? step : -step);
  servoState[servoNum].currentAngle = newAngle;
  
  int pulse = map(newAngle * 10, servoConfig[servoNum][2] * 10, servoConfig[servoNum][3] * 10,
                 servoConfig[servoNum][0], servoConfig[servoNum][1]);
  pwm.setPWM(servoChannels[servoNum], 0, pulse);
}

void printServoStatus() {
  Serial.println("\nCurrent Servo Angles:");
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servoNames[i]);
    Serial.print(": ");
    Serial.print(servoState[i].currentAngle);
    Serial.print("°");
    if (servoState[i].isMoving) {
      Serial.print(" (moving to ");
      Serial.print(servoState[i].targetAngle);
      Serial.print("°)");
    }
    Serial.println();
  }
}
