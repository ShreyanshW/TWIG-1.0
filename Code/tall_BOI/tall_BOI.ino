#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===== Configuration =====
#define SERVO_COUNT 5
const byte servoChannels[] = {0, 4, 7, 11, 15};
const char* servoNames[] = {"Base", "Shoulder", "Elbow", "Wrist", "Gripper"};

// Servo Config: {minPulse, maxPulse, minAngle, maxAngle, speedFactor}
const float servoConfig[SERVO_COUNT][5] = {
  {135, 495, 0, 180, 1.1},    // Base
  {175, 500, -60, 90, 0.5},   // Shoulder 
  {90, 480, 0, 270, 0.75},     // Elbow
  {215, 480, 0, 180, 1.0},    // Wrist (higher speed for flourishes)
  {300, 440, 0, 90, 0.7}      // Gripper
};

// Motion Control
#define MOVEMENT_CYCLE_MS 5    // 50Hz control
#define BASE_SPEED 30           // Reduced speed for smoother motion
#define ACCEL_TIME_MS 300       // Smoother acceleration
#define DECEL_DISTANCE 10       // Gentler braking
#define ANGLE_TOLERANCE 1.5    

Adafruit_PWMServoDriver pwm;

struct ServoState {
  float currentAngle;
  float targetAngle;
  float speedFactor; // Individual speed adjustment
  bool isMoving;
};

ServoState servoStates[SERVO_COUNT];
const float homeAngles[SERVO_COUNT] = {90, 30, 60, 90, 45}; // Updated return position

// ===== Enhanced Pick & Place Sequence =====
const struct {
  float targets[SERVO_COUNT]; // B, S, E, W, G
  unsigned int duration;       // Corrected member name
  unsigned int holdTime;
  const char* description;
} demoSteps[] = {
  // Pick Sequence (Base 0°, Shoulder 30°, Elbow 100°)
  {{  0, 90, 180, 90, 0}, 1000, 0, "I'm TAAAALLLLL"},
  {{  0, 0, 180, 45, 0}, 800,  0, "IM LOOONNNNGGGG"},
  {{ 90, 30, 30, 90,  45}, 1500, 0, "Return home"}
};

int currentStep = 0;
unsigned long stepStartTime = 0;
bool demoRunning = false;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);

  // Initialize speed factors
  for(int i=0; i<SERVO_COUNT; i++){
    servoStates[i].speedFactor = servoConfig[i][4];
  }

  Serial.println("\n=== Enhanced Pick & Place Demo ===");
  Serial.println("Key Features:");
  Serial.println("- Smooth coordinated movements");
  Serial.println("- Multiple servos moving simultaneously");
  Serial.println("- Wrist flourishes during transport");
  Serial.println("\nType 'DEMO' to start");
  
  resetAllServos();
}

void loop() {
  static unsigned long lastUpdate = 0;

  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    input.trim();
    if(input.equalsIgnoreCase("DEMO") && !demoRunning){
      startDemo();
    }
  }

  if(millis() - lastUpdate >= MOVEMENT_CYCLE_MS){
    lastUpdate = millis();
    
    if(demoRunning){
      runDemoSequence();
    }

    updateAllServos();
  }
}

void startDemo() {
  Serial.println("\n=== Starting Demo ===");
  demoRunning = true;
  currentStep = 0;
  stepStartTime = millis();
  executeStep(currentStep);
}

void runDemoSequence() {
  unsigned long elapsed = millis() - stepStartTime;
  uint16_t totalStepTime = demoSteps[currentStep].duration + demoSteps[currentStep].holdTime; // Corrected member name

  if(elapsed >= totalStepTime){
    if(checkStepCompletion()){
      currentStep++;
      if(currentStep >= sizeof(demoSteps)/sizeof(demoSteps[0])){
        endDemo();
        return;
      }
      stepStartTime = millis();
      executeStep(currentStep);
    }
  }
}

void executeStep(int step) {
  Serial.print("\nStep ");
  Serial.print(step+1);
  Serial.print(": ");
  Serial.println(demoSteps[step].description);
  
  for(int i=0; i<SERVO_COUNT; i++){
    startServoMovement(i, demoSteps[step].targets[i]);
  }
}

void startServoMovement(byte servoNum, float targetAngle) {
  targetAngle = constrain(targetAngle, servoConfig[servoNum][2], servoConfig[servoNum][3]);
  servoStates[servoNum].targetAngle = targetAngle;
  servoStates[servoNum].isMoving = true;

  int pulse = map(targetAngle, 
                servoConfig[servoNum][2], servoConfig[servoNum][3],
                servoConfig[servoNum][0], servoConfig[servoNum][1]);
  pwm.setPWM(servoChannels[servoNum], 0, pulse);
}

void updateAllServos() {
  for(byte i=0; i<SERVO_COUNT; i++){
    if(servoStates[i].isMoving){
      updateServoPosition(i);
    }
  }
}

void updateServoPosition(byte servoNum) {
  float current = servoStates[servoNum].currentAngle; // Corrected variable name
  float target = servoStates[servoNum].targetAngle;
  float remaining = abs(target - current);

  if(remaining <= ANGLE_TOLERANCE){
    servoStates[servoNum].currentAngle = target;
    servoStates[servoNum].isMoving = false;
    return;
  }

  // Enhanced movement calculation
  float effectiveSpeed = BASE_SPEED * servoStates[servoNum].speedFactor;
  float step = (effectiveSpeed * MOVEMENT_CYCLE_MS) / 1000.0;
  
  // Acceleration curve
  unsigned long moveElapsed = millis() - stepStartTime;
  if(moveElapsed < ACCEL_TIME_MS){
    step *= smoothAccelCurve((float)moveElapsed/ACCEL_TIME_MS);
  }
  
  // Deceleration curve
  if(remaining <= DECEL_DISTANCE){
    step *= smoothDecelCurve(remaining/DECEL_DISTANCE);
  }

  servoStates[servoNum].currentAngle += (target > current) ? step : -step;
  servoStates[servoNum].currentAngle = constrain(servoStates[servoNum].currentAngle,
                                               servoConfig[servoNum][2],
                                               servoConfig[servoNum][3]);

  // Update PWM
  int pulse = map(servoStates[servoNum].currentAngle,
                 servoConfig[servoNum][2], servoConfig[servoNum][3],
                 servoConfig[servoNum][0], servoConfig[servoNum][1]);
  pwm.setPWM(servoChannels[servoNum], 0, pulse);
}

// Smooth easing functions
float smoothAccelCurve(float t) {
  return t < 0.5 ? 2*t*t : 1-(2*(1-t)*(1-t)); 
}

float smoothDecelCurve(float t) {
  return 0.5*(1+sin(PI*(t-0.5)));
}

bool checkStepCompletion() {
  for(byte i=0; i<SERVO_COUNT; i++){
    if(abs(servoStates[i].currentAngle - servoStates[i].targetAngle) > ANGLE_TOLERANCE){
      return false;
    }
  }
  return true;
}

void resetAllServos() {
  Serial.println("Resetting to home position...");
  for(byte i=0; i<SERVO_COUNT; i++){
    startServoMovement(i, homeAngles[i]);
  }
}

void endDemo() {
  Serial.println("\nDemo complete!");
  demoRunning = false;
  resetAllServos();
}
