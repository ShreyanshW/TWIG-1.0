#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_COUNT 5
#define SERVO_FREQ 50
#define PULSE_MIN 0
#define PULSE_MAX 4095
#define PULSE_CENTER 375

// Current state
byte currentServo = 0;
int currentPulse = PULSE_CENTER;

// Your servo channels on PCA9685
const byte servoChannels[SERVO_COUNT] = {0, 4, 7, 11, 15};

// Calibration storage - DS 30kg digital servo needs different range
int servoMin[SERVO_COUNT] = {150, 102, 150, 150, 150};    // DS servo min ~500us (102*4.88us≈500us)
int servoMax[SERVO_COUNT] = {600, 800, 600, 600, 600};    // DS servo max ~2500us, s1 extended to 800
int servoCenter[SERVO_COUNT] = {375, 307, 375, 375, 375}; // DS servo center ~1500us (307*4.88us≈1500us)

// Angle mapping - assumes 180 degree servos by default
float servoAngleMin[SERVO_COUNT] = {0, 0, 0, 0, 0};       // Minimum angle (degrees)
float servoAngleMax[SERVO_COUNT] = {180, 180, 180, 180, 180}; // Maximum angle (degrees)

void setup() {
  Serial.begin(9600);
  
  Serial.println(F("SERVO CALIBRATOR WITH ANGLE CONTROL"));
  Serial.println(F("==================================="));
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  Serial.println(F("Commands:"));
  Serial.println(F("s0-s4: Select servo"));
  Serial.println(F("a90: Move to 90 degrees"));
  Serial.println(F("p300: Set pulse directly"));
  Serial.println(F("+/-: Adjust pulse"));
  Serial.println(F("c: Center position"));
  Serial.println(F("v: Save as center"));
  Serial.println(F("t: Test sweep"));
  Serial.println(F("r: Show results"));
  Serial.println(F("cal: Calibrate angle range"));
  Serial.println(F("h: Help"));
  
  pwm.setPWM(servoChannels[0], 0, currentPulse);
  Serial.print(F("Servo 0, Pulse: "));
  Serial.print(currentPulse);
  Serial.print(F(", Angle: "));
  Serial.println(pulseToAngle(0, currentPulse));
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    parseCmd(cmd);
  }
}

// Convert pulse to angle for a specific servo
float pulseToAngle(byte servo, int pulse) {
  float pulseRange = servoMax[servo] - servoMin[servo];
  float angleRange = servoAngleMax[servo] - servoAngleMin[servo];
  float normalizedPulse = (float)(pulse - servoMin[servo]) / pulseRange;
  return servoAngleMin[servo] + (normalizedPulse * angleRange);
}

// Convert angle to pulse for a specific servo
int angleToPulse(byte servo, float angle) {
  // Constrain angle to valid range
  angle = constrain(angle, servoAngleMin[servo], servoAngleMax[servo]);
  
  float angleRange = servoAngleMax[servo] - servoAngleMin[servo];
  float pulseRange = servoMax[servo] - servoMin[servo];
  float normalizedAngle = (angle - servoAngleMin[servo]) / angleRange;
  return servoMin[servo] + (int)(normalizedAngle * pulseRange);
}

void setServoFrequency(byte servo) {
  // Frequency management removed - use manual freq commands if needed
}

void parseCmd(String cmd) {
  // Debug output
  Serial.print("Raw command: '");
  Serial.print(cmd);
  Serial.print("' Length: ");
  Serial.println(cmd.length());
  
  cmd.toLowerCase();
  
  Serial.print("Processed command: '");
  Serial.print(cmd);
  Serial.println("'");
  
  // Select servo (s0, s1, s2, s3, s4)
  if (cmd.startsWith("s") && cmd.length() == 2) {
    byte servo = cmd.charAt(1) - '0';
    Serial.print("Servo number parsed: ");
    Serial.println(servo);
    
    if (servo < SERVO_COUNT) {
      currentServo = servo;
      currentPulse = servoCenter[servo];
      
      Serial.print("Setting PWM on channel ");
      Serial.print(servoChannels[currentServo]);
      Serial.print(" to pulse ");
      Serial.println(currentPulse);
      
      pwm.setPWM(servoChannels[currentServo], 0, currentPulse);
      Serial.print("PWM sent to channel ");
      Serial.print(servoChannels[currentServo]);
      Serial.print(": pulse = ");
      Serial.println(currentPulse);
      delay(334); // Speeded up by 2x from 667ms to 334ms
      Serial.print(F("Servo "));
      Serial.print(servo);
      Serial.print(F(", Pulse: "));
      Serial.print(currentPulse);
      Serial.print(F(", Angle: "));
      Serial.println(pulseToAngle(servo, currentPulse));
    } else {
      Serial.print("Servo number ");
      Serial.print(servo);
      Serial.println(" is out of range!");
    }
  }
  // Set angle (a90, a45.5, a-30, etc.)
  else if (cmd.startsWith("a")) {
    float angle = cmd.substring(1).toFloat();
    Serial.print("Parsed angle: ");
    Serial.println(angle);
    
    int pulse = angleToPulse(currentServo, angle);
    if (pulse >= PULSE_MIN && pulse <= PULSE_MAX) {
      currentPulse = pulse;
      pwm.setPWM(servoChannels[currentServo], 0, currentPulse);
      delay(167); // Speeded up by 2x from 333ms to 167ms
      Serial.print(F("Angle: "));
      Serial.print(angle);
      Serial.print(F("°, Pulse: "));
      Serial.println(currentPulse);
    } else {
      Serial.print(F("Angle "));
      Serial.print(angle);
      Serial.println(F("° is out of range!"));
    }
  }
  // Set pulse (p300)
  else if (cmd.startsWith("p")) {
    int pulse = cmd.substring(1).toInt();
    if (pulse >= PULSE_MIN && pulse <= PULSE_MAX) {
      currentPulse = pulse;
      pwm.setPWM(servoChannels[currentServo], 0, currentPulse);
      delay(167); // Speeded up by 2x from 333ms to 167ms
      Serial.print(F("Pulse: "));
      Serial.print(currentPulse);
      Serial.print(F(", Angle: "));
      Serial.println(pulseToAngle(currentServo, currentPulse));
    }
  }
  // Increase pulse
  else if (cmd == "+") {
    if (currentPulse < PULSE_MAX) {
      currentPulse += 1;
      pwm.setPWM(servoChannels[currentServo], 0, currentPulse);
      delay(84); // Speeded up by 2x from 167ms to 84ms
      Serial.print(currentPulse);
      Serial.print(F(" ("));
      Serial.print(pulseToAngle(currentServo, currentPulse));
      Serial.println(F("°)"));
    }
  }
  // Decrease pulse
  else if (cmd == "-") {
    if (currentPulse > PULSE_MIN) {
      currentPulse -= 1;
      pwm.setPWM(servoChannels[currentServo], 0, currentPulse);
      delay(84); // Speeded up by 2x from 167ms to 84ms
      Serial.print(currentPulse);
      Serial.print(F(" ("));
      Serial.print(pulseToAngle(currentServo, currentPulse));
      Serial.println(F("°)"));
    }
  }
  // Go to center
  else if (cmd == "c") {
    currentPulse = servoCenter[currentServo];
    pwm.setPWM(servoChannels[currentServo], 0, currentPulse);
    delay(334); // Speeded up by 2x from 667ms to 334ms
    Serial.print(F("Center - Pulse: "));
    Serial.print(currentPulse);
    Serial.print(F(", Angle: "));
    Serial.println(pulseToAngle(currentServo, currentPulse));
  }
  // Save current as center
  else if (cmd == "v") {
    servoCenter[currentServo] = currentPulse;
    Serial.print(F("Saved center - Pulse: "));
    Serial.print(currentPulse);
    Serial.print(F(", Angle: "));
    Serial.println(pulseToAngle(currentServo, currentPulse));
  }
  // Calibrate angle range
  else if (cmd == "cal") {
    calibrateAngleRange();
  }
  // Test sweep
  else if (cmd == "t") {
    testSweep();
  }
  // Show results
  else if (cmd == "r") {
    showResults();
  }
  // Help
  else if (cmd == "h") {
    showHelp();
  }
  // Set min/max for advanced users
  else if (cmd.startsWith("min")) {
    servoMin[currentServo] = currentPulse;
    Serial.print(F("Min saved - Pulse: "));
    Serial.print(currentPulse);
    Serial.print(F(", Angle: "));
    Serial.println(pulseToAngle(currentServo, currentPulse));
  }
  else if (cmd.startsWith("max")) {
    servoMax[currentServo] = currentPulse;
    Serial.print(F("Max saved - Pulse: "));
    Serial.print(currentPulse);
    Serial.print(F(", Angle: "));
    Serial.println(pulseToAngle(currentServo, currentPulse));
  }
  // Set angle range
  else if (cmd.startsWith("amin")) {
    servoAngleMin[currentServo] = cmd.substring(4).toFloat();
    Serial.print(F("Min angle set to: "));
    Serial.println(servoAngleMin[currentServo]);
  }
  else if (cmd.startsWith("amax")) {
    servoAngleMax[currentServo] = cmd.substring(4).toFloat();
    Serial.print(F("Max angle set to: "));
    Serial.println(servoAngleMax[currentServo]);
  }
  // Debug: Test specific channels directly
  else if (cmd.startsWith("test")) {
    int channel = cmd.substring(4).toInt();
    Serial.print("Testing channel ");
    Serial.print(channel);
    Serial.println(" directly");
    pwm.setPWM(channel, 0, 200);
    delay(834); // Speeded up by 2x from 1667ms to 834ms
    pwm.setPWM(channel, 0, 500);
    delay(834); // Speeded up by 2x from 1667ms to 834ms
    pwm.setPWM(channel, 0, 375);
  }
  // Scan range for DS servo
  else if (cmd == "scan") {
    Serial.println("Scanning pulse range...");
    for (int pulse = 50; pulse <= 700; pulse += 50) {
      Serial.print("Trying pulse: ");
      Serial.print(pulse);
      Serial.print(" (");
      Serial.print(pulseToAngle(currentServo, pulse));
      Serial.println("°)");
      pwm.setPWM(servoChannels[currentServo], 0, pulse);
      delay(1667); // Speeded up by 2x from 3333ms to 1667ms
    }
    Serial.println("Scan complete");
  }
  // Fast scan to detect any response
  else if (cmd == "fastscan") {
    Serial.println("Fast scanning...");
    for (int pulse = 0; pulse <= 800; pulse += 50) {
      pwm.setPWM(servoChannels[currentServo], 0, pulse);
      delay(334); // Speeded up by 2x from 667ms to 334ms
    }
    Serial.println("Fast scan complete");
  }
  // Manual frequency control
  else if (cmd == "freq60") {
    pwm.setPWMFreq(60);
    Serial.println("Frequency set to 60Hz");
  }
  else if (cmd == "freq50") {
    pwm.setPWMFreq(50);
    Serial.println("Frequency set to 50Hz");
  }
}

void calibrateAngleRange() {
  Serial.println(F("ANGLE CALIBRATION"));
  Serial.println(F("Move servo to minimum angle position using +/- or p commands"));
  Serial.println(F("Then type 'amin[angle]' (e.g., 'amin-90' for -90 degrees)"));
  Serial.println(F("Move servo to maximum angle position"));
  Serial.println(F("Then type 'amax[angle]' (e.g., 'amax90' for 90 degrees)"));
  Serial.println(F("Current servo angle range:"));
  Serial.print(F("Min: "));
  Serial.print(servoAngleMin[currentServo]);
  Serial.print(F("°, Max: "));
  Serial.print(servoAngleMax[currentServo]);
  Serial.println(F("°"));
}

void testSweep() {
  Serial.println(F("Testing angle sweep..."));
  
  // Go to min angle
  int minPulse = angleToPulse(currentServo, servoAngleMin[currentServo]);
  pwm.setPWM(servoChannels[currentServo], 0, minPulse);
  Serial.print(F("Moving to min angle: "));
  Serial.print(servoAngleMin[currentServo]);
  Serial.println(F("°"));
  delay(3334); // Speeded up by 2x from 6667ms to 3334ms
  
  // Sweep to max angle
  float angleStep = (servoAngleMax[currentServo] - servoAngleMin[currentServo]) / 50.0; // 50 steps
  for (float angle = servoAngleMin[currentServo]; angle <= servoAngleMax[currentServo]; angle += angleStep) {
    int pulse = angleToPulse(currentServo, angle);
    pwm.setPWM(servoChannels[currentServo], 0, pulse);
    delay(167); // Speeded up by 2x from 333ms to 167ms
  }
  
  Serial.print(F("Reached max angle: "));
  Serial.print(servoAngleMax[currentServo]);
  Serial.println(F("°"));
  delay(1667); // Speeded up by 2x from 3333ms to 1667ms
  
  // Return to center by sweeping back (prevents collision)
  Serial.println(F("Returning to center via safe path..."));
  float centerAngle = pulseToAngle(currentServo, servoCenter[currentServo]);
  
  // Sweep back from max to center
  for (float angle = servoAngleMax[currentServo]; angle >= centerAngle; angle -= angleStep) {
    int pulse = angleToPulse(currentServo, angle);
    pwm.setPWM(servoChannels[currentServo], 0, pulse);
    delay(167); // Speeded up by 2x from 333ms to 167ms
  }
  
  // Final move to exact center position
  pwm.setPWM(servoChannels[currentServo], 0, servoCenter[currentServo]);
  currentPulse = servoCenter[currentServo];
  delay(834); // Speeded up by 2x from 1667ms to 834ms
  Serial.print(F("Returned to center: "));
  Serial.print(pulseToAngle(currentServo, currentPulse));
  Serial.println(F("°"));
  Serial.println(F("Done"));
}

void showResults() {
  Serial.println(F("RESULTS:"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(" Pulse - Min:"));
    Serial.print(servoMin[i]);
    Serial.print(F(" Max:"));
    Serial.print(servoMax[i]);
    Serial.print(F(" Center:"));
    Serial.println(servoCenter[i]);
    
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(" Angle - Min:"));
    Serial.print(servoAngleMin[i]);
    Serial.print(F("° Max:"));
    Serial.print(servoAngleMax[i]);
    Serial.print(F("° Center:"));
    Serial.print(pulseToAngle(i, servoCenter[i]));
    Serial.println(F("°"));
  }
  
  Serial.println(F("\nPulse Code arrays:"));
  Serial.print(F("int sMin[]={"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servoMin[i]);
    if (i < SERVO_COUNT-1) Serial.print(F(","));
  }
  Serial.println(F("};"));
  
  Serial.print(F("int sMax[]={"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servoMax[i]);
    if (i < SERVO_COUNT-1) Serial.print(F(","));
  }
  Serial.println(F("};"));
  
  Serial.print(F("int sCenter[]={"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servoCenter[i]);
    if (i < SERVO_COUNT-1) Serial.print(F(","));
  }
  Serial.println(F("};"));
  
  Serial.println(F("\nAngle Code arrays:"));
  Serial.print(F("float sAngleMin[]={"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servoAngleMin[i]);
    if (i < SERVO_COUNT-1) Serial.print(F(","));
  }
  Serial.println(F("};"));
  
  Serial.print(F("float sAngleMax[]={"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(servoAngleMax[i]);
    if (i < SERVO_COUNT-1) Serial.print(F(","));
  }
  Serial.println(F("};"));
}

void showHelp() {
  Serial.println(F("ANGLE CONTROL COMMANDS:"));
  Serial.println(F("a90: Move to 90 degrees"));
  Serial.println(F("a45.5: Move to 45.5 degrees"));
  Serial.println(F("a-30: Move to -30 degrees"));
  Serial.println(F("cal: Start angle calibration"));
  Serial.println(F("amin-90: Set min angle to -90°"));
  Serial.println(F("amax180: Set max angle to 180°"));
  Serial.println(F(""));
  Serial.println(F("PULSE CONTROL COMMANDS:"));
  Serial.println(F("s0-s4: Select servo"));
  Serial.println(F("p300: Set pulse (100-700)"));
  Serial.println(F("+: Increase pulse"));
  Serial.println(F("-: Decrease pulse"));
  Serial.println(F("c: Go to center"));
  Serial.println(F("v: Save current as center"));
  Serial.println(F("min: Save current as min"));
  Serial.println(F("max: Save current as max"));
  Serial.println(F("t: Test sweep"));
  Serial.println(F("r: Show results"));
  Serial.println(F("freq50/freq60: Manual frequency override"));
}