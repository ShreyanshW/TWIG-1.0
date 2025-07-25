#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ----- YOUR CALIBRATED PULSE VALUES (in PCA9685 pulses) -----
const int BASE_MIN = 70;      const int BASE_MAX = 330;     const int BASE_CENTER = 190;
const int SHOULDER_MIN = 195; const int SHOULDER_MAX = 475; const int SHOULDER_CENTER = 335;
const int ELBOW_MIN = 270;    const int ELBOW_MAX = 520;    const int ELBOW_CENTER = 395;
const int WRIST_MIN = 255;    const int WRIST_MAX = 515;    const int WRIST_CENTER = 385;
// ----------------------------------------------------------

// Servo channels (0-15 on PCA9685)
#define BASE_SERVO     0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO    2
#define WRIST_SERVO    3

// Movement parameters
const int MOVE_DELAY_MS = 20;  // Between steps
const int PULSE_STEP = 1;      // Smaller steps for smoother motion

String inputBuffer = "";        // Buffer for serial input
bool inputReady = false;        // Flag for complete input

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect (for USB)
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Standard servo PWM frequency
  
  Serial.println(F("\nROBOTIC ARM CONTROL - READY"));
  Serial.println(F("Type commands like: B90 S45 E180 W0"));
  Serial.println(F("C = Center All | H = Help"));
  
  centerAllServos();
}

void loop() {
  handleSerialInput();
  
  if (inputReady) {
    parseCommand(inputBuffer);
    inputBuffer = "";
    inputReady = false;
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      inputReady = true;
    } else {
      inputBuffer += inChar;
    }
  }
}

void parseCommand(String command) {
  command.trim(); // Remove whitespace
  command.toUpperCase();
  
  if (command.length() < 2) {
    switch(command.charAt(0)) {
      case 'C':
        centerAllServos();
        Serial.println(F("All servos centered"));
        return;
      case 'H':
        printHelp();
        return;
      default:
        Serial.println(F("Unknown command"));
        return;
    }
  }

  char servoChar = command.charAt(0);
  String angleStr = command.substring(1);
  int angle = angleStr.toInt();

  byte servoNum;
  const char* name;
  int minPulse, maxPulse;
  
  // Set parameters based on command
  switch(servoChar) {
    case 'B':
      servoNum = BASE_SERVO;
      name = "Base";
      minPulse = BASE_MIN;
      maxPulse = BASE_MAX;
      break;
    case 'S':
      servoNum = SHOULDER_SERVO;
      name = "Shoulder";
      minPulse = SHOULDER_MIN;
      maxPulse = SHOULDER_MAX;
      break;
    case 'E':
      servoNum = ELBOW_SERVO;
      name = "Elbow";
      minPulse = ELBOW_MIN;
      maxPulse = ELBOW_MAX;
      break;
    case 'W':
      servoNum = WRIST_SERVO;
      name = "Wrist";
      minPulse = WRIST_MIN;
      maxPulse = WRIST_MAX;
      break;
    default:
      Serial.println(F("Invalid servo identifier"));
      return;
  }

  // Validate angle
  if (angleStr.length() == 0 || (angle == 0 && angleStr != "0")) {
    Serial.println(F("Invalid angle value"));
    return;
  }

  angle = constrain(angle, 0, 180);
  
  // Convert angle to pulse
  int targetPulse = map(angle, 0, 180, minPulse, maxPulse);
  
  Serial.print(F("Moving "));
  Serial.print(name);
  Serial.print(F(" to "));
  Serial.print(angle);
  Serial.print(F("° (Pulse: "));
  Serial.print(targetPulse);
  Serial.println(F(")"));

  moveToPosition(servoNum, targetPulse);
}

void moveToPosition(byte servoNum, int targetPulse) {
  int currentPulse = getCurrentPulse(servoNum);
  int step = (targetPulse > currentPulse) ? PULSE_STEP : -PULSE_STEP;
  
  Serial.print(F("Current Pulse: "));
  Serial.print(currentPulse);
  Serial.print(F(", Target Pulse: "));
  Serial.println(targetPulse);
  
  while (abs(currentPulse - targetPulse) > PULSE_STEP) {
    currentPulse += step;
    pwm.setPWM(servoNum, 0, currentPulse);
    delay(MOVE_DELAY_MS);
  }
  
  pwm.setPWM(servoNum, 0, targetPulse); // Final position
  Serial.print(F("Final Pulse Set: "));
  Serial.println(targetPulse);
}

int getCurrentPulse(byte servoNum) {
  // In a real implementation, you would track current positions
  // For now, just return center positions
  switch(servoNum) {
    case BASE_SERVO: return BASE_CENTER;
    case SHOULDER_SERVO: return SHOULDER_CENTER;
    case ELBOW_SERVO: return ELBOW_CENTER;
    case WRIST_SERVO: return WRIST_CENTER;
    default: return 0;
  }
}

void centerAllServos() {
  pwm.setPWM(BASE_SERVO, 0, BASE_CENTER);
  pwm.setPWM(SHOULDER_SERVO, 0, SHOULDER_CENTER);
  pwm.setPWM(ELBOW_SERVO, 0, ELBOW_CENTER);
  pwm.setPWM(WRIST_SERVO, 0, WRIST_CENTER);
  Serial.println(F("All servos centered"));
}

void printHelp() {
  Serial.println(F("\n=== COMMAND HELP ==="));
  Serial.println(F("Command format: [servo][angle]"));
  Serial.println(F("Example: B90 (Base to 90°)"));
  Serial.println(F("         S45 (Shoulder to 45°)"));
  Serial.println(F("Servos:"));
  Serial.println(F("  B = Base (0-180°)"));
  Serial.println(F("  S = Shoulder (0-180°)"));
  Serial.println(F("  E = Elbow (0-180°)"));
  Serial.println(F("  W = Wrist (0-180°)"));
  Serial.println(F("Other commands:"));
  Serial.println(F("  C = Center all servos"));
  Serial.println(F("  H = Help"));
}
