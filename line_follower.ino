// Line Following Robot with PID Control and Hall Effect Sensor
// Arduino Nano with TB6612FNG Motor Driver and QTR Sensor Array
// Competition Version with All Required Tasks
// Memory optimized for Arduino Nano
// Fixed ignore mechanism - 6th trigger is ignored, no complex flags needed

// Motor pins for TB6612FNG
const int AIN1 = 7;
const int AIN2 = 8;
const int PWMA = 5;
const int BIN1 = 9;
const int BIN2 = 11;
const int PWMB = 6;

// QTR sensor array pins (as per specifications)
const int sensorPins[8] = {10, A0, A1, A2, A3, A4, A5, 3};
const int numSensors = 8;

// Push button
const int buttonPin = 13;

// LED pins
const int redLED = 12;
const int yellowLED = 2;
const int greenLED = 4;

// Hall effect sensor pin
const int hallSensorPin = A6;

// Sensor calibration values
int sensorMin[numSensors];
int sensorMax[numSensors];
int sensorValues[numSensors];

// PID constants
float Kp = 0.5;
float Ki = 0.0;
float Kd = 1.0;

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

// Motor speeds
int baseSpeed = 255;
int maxSpeed = 255;
int leftSpeed, rightSpeed;

// Calibration parameters
const int motorSpeed = 150;
const unsigned long initialPhase = 500;
const unsigned long loopPhase = 1000;
const unsigned long totalRun = 5000;

// Hall effect sensor variables
int hallThreshold = 512;
bool magnetDetected = false;
bool lastMagnetState = false;

// Task sequence and index (updated for competition tasks)
int taskSequence[] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; // All 9 tasks as per competition rules
int currentTaskIndex = 0;
int magnetTriggerCount = 0; // Simple counter for magnet triggers

// LED state tracking
bool redLEDState = false;
bool yellowLEDState = false;
bool greenLEDState = false;

// Task management flags
bool isGoingBack = false;
bool secondReverseTraverse = false;

// Line crossing detection variables
bool wasOnLine = false;
bool wasCentered = false;
int lineCrossings = 0;
const int LINE_DETECTION_THRESHOLD = 500;
const int CENTER_THRESHOLD = 600;
const int ROTATION_SPEED = 120;
const unsigned long ROTATION_TIMEOUT = 15000;
const unsigned long MIN_CROSSING_INTERVAL = 200;
unsigned long lastCrossingTime = 0;

// Zigzag handling variables
float lastValidPosition = 3500;
float positionHistory[5] = {3500, 3500, 3500, 3500, 3500};
int historyIndex = 0;
unsigned long lastLineTime = 0;
const unsigned long LINE_LOST_TIMEOUT = 100;
const int ZIGZAG_SEARCH_SPEED = 100;
const int AGGRESSIVE_TURN_SPEED = 150;
bool inZigzagMode = false;
float zigzagDirection = 0;

// States
enum RobotState {
  WAITING_FOR_CALIBRATION,
  CALIBRATING,
  WAITING_FOR_LINE_FOLLOWING,
  LINE_FOLLOWING
};

RobotState currentState = WAITING_FOR_CALIBRATION;
bool lastButtonState = LOW;
bool buttonPressed = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  // Initialize button pin
  pinMode(buttonPin, INPUT);
  
  // Initialize LED pins
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  
  // Initialize hall sensor pin
  pinMode(hallSensorPin, INPUT);
  
  // Initialize sensor pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Initialize calibration arrays
  for (int i = 0; i < numSensors; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  
  // Turn off all LEDs initially
  updateLEDStates();
  
  stopMotors();
  Serial.println("Line Following Robot Ready!");
  Serial.println("Press button to start calibration...");
}

void loop() {
  // Check for button press
  bool currentButtonState = digitalRead(buttonPin);
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    buttonPressed = true;
    delay(50); // Debounce
  }
  lastButtonState = currentButtonState;
  
  switch (currentState) {
    case WAITING_FOR_CALIBRATION:
      if (buttonPressed) {
        buttonPressed = false;
        currentState = CALIBRATING;
        Serial.println("Starting calibration...");
      }
      break;
      
    case CALIBRATING:
      calibrateSensors();
      currentState = WAITING_FOR_LINE_FOLLOWING;
      Serial.println("Calibration complete! Press button again to start line following...");
      break;
      
    case WAITING_FOR_LINE_FOLLOWING:
      if (buttonPressed) {
        buttonPressed = false;
        currentState = LINE_FOLLOWING;
        Serial.println("Starting line following...");
      }
      break;
      
    case LINE_FOLLOWING:
      followLine();
      break;
  }
}

// Function to update LED states based on flags
void updateLEDStates() {
  digitalWrite(redLED, redLEDState ? HIGH : LOW);
  digitalWrite(yellowLED, yellowLEDState ? HIGH : LOW);
  digitalWrite(greenLED, greenLEDState ? HIGH : LOW);
}

bool checkMagnetDetection() {
  int hallValue = analogRead(hallSensorPin);
  bool currentMagnetState = (hallValue < hallThreshold);
  
  if (currentMagnetState && !lastMagnetState) {
    magnetDetected = true;
  } else {
    magnetDetected = false;
  }
  
  lastMagnetState = currentMagnetState;
  return magnetDetected;
}

void calibrateSensors() {
  unsigned long startTime = millis();
  
  runMotors(-motorSpeed, motorSpeed);
  
  unsigned long phaseStart = millis();
  while (millis() - phaseStart < initialPhase) {
    readSensors();
    updateCalibration();
  }
  
  bool dir = false;
  while (millis() - startTime < totalRun) {
    if (dir) {
      runMotors(-motorSpeed, motorSpeed);
    } else {
      runMotors(motorSpeed, -motorSpeed);
    }
    
    phaseStart = millis();
    while (millis() - phaseStart < loopPhase && 
           (millis() - startTime < totalRun)) {
      readSensors();
      updateCalibration();
    }
    dir = !dir;
  }
  
  stopMotors();
  
  Serial.println("Calibration complete");
}

void readSensors() {
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

void updateCalibration() {
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] < sensorMin[i]) {
      sensorMin[i] = sensorValues[i];
    }
    if (sensorValues[i] > sensorMax[i]) {
      sensorMax[i] = sensorValues[i];
    }
  }
}

void updatePositionHistory(float position) {
  positionHistory[historyIndex] = position;
  historyIndex = (historyIndex + 1) % 5;
}

bool detectZigzagPattern() {
  int leftCount = 0, rightCount = 0;
  
  for (int i = 0; i < 5; i++) {
    if (positionHistory[i] < 3000) leftCount++;
    if (positionHistory[i] > 4000) rightCount++;
  }
  
  return (leftCount >= 2 && rightCount >= 2);
}

float predictZigzagDirection() {
  float recent = positionHistory[(historyIndex + 4) % 5];
  float older = positionHistory[(historyIndex + 3) % 5];
  
  if (recent < 3500 && older > 3500) {
    return -1.0;
  } else if (recent > 3500 && older < 3500) {
    return 1.0;
  } else if (recent < 3500) {
    return -1.0;
  } else if (recent > 3500) {
    return 1.0;
  }
  
  return 0;
}

void followLine() {
  readSensors();
  
  int normalizedValues[numSensors];
  for (int i = 0; i < numSensors; i++) {
    normalizedValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    normalizedValues[i] = constrain(normalizedValues[i], 0, 1000);
  }
  
  bool onLine = false;
  for (int i = 0; i < numSensors; i++) {
    if (normalizedValues[i] > LINE_DETECTION_THRESHOLD) {
      onLine = true;
      break;
    }
  }
  
  if (onLine) {
    lastLineTime = millis();
    inZigzagMode = false;
    
    long weightedSum = 0;
    long sum = 0;
    for (int i = 0; i < numSensors; i++) {
      long value = normalizedValues[i];
      long weight = (numSensors - 1 - i) * 1000;
      weightedSum += value * weight;
      sum += value;
    }
    float position = 3500;
    if (sum > 0) {
      position = (float)weightedSum / sum;
      lastValidPosition = position;
      updatePositionHistory(position);
    }
    
    error = position - 3500;
    integral += error;
    derivative = error - lastError;
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    if (integral > 5000) integral = 5000;
    if (integral < -5000) integral = -5000;
    
    leftSpeed = baseSpeed + correction;
    rightSpeed = baseSpeed - correction;
    
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
    
    runMotors(leftSpeed, rightSpeed);
    lastError = error;
  } else {
    unsigned long timeSinceLineLost = millis() - lastLineTime;
    
    if (timeSinceLineLost < LINE_LOST_TIMEOUT) {
      runMotors(leftSpeed, rightSpeed);
    } else {
      if (!inZigzagMode) {
        if (detectZigzagPattern()) {
          inZigzagMode = true;
          zigzagDirection = predictZigzagDirection();
        }
      }
      
      if (inZigzagMode && zigzagDirection != 0) {
        if (zigzagDirection < 0) {
          runMotors(-AGGRESSIVE_TURN_SPEED, AGGRESSIVE_TURN_SPEED);
        } else {
          runMotors(AGGRESSIVE_TURN_SPEED, -AGGRESSIVE_TURN_SPEED);
        }
      } else {
        if (lastValidPosition < 3500) {
          runMotors(-ZIGZAG_SEARCH_SPEED, ZIGZAG_SEARCH_SPEED);
        } else {
          runMotors(ZIGZAG_SEARCH_SPEED, -ZIGZAG_SEARCH_SPEED);
        }
      }
      
      if (timeSinceLineLost > 2000) {
        stopMotors();
        // Search for line instead of calling taskRotate360() to avoid interfering with magnet triggers
        if (lastValidPosition < 3500) {
          runMotors(-ROTATION_SPEED, ROTATION_SPEED);
        } else {
          runMotors(ROTATION_SPEED, -ROTATION_SPEED);
        }
        inZigzagMode = false;
        zigzagDirection = 0;
      }
    }
  }
  
  // FIXED: Properly ignore the 6th magnet trigger
  if (checkMagnetDetection()) {
    magnetTriggerCount++; // Increment trigger count
    Serial.print("Magnet trigger #");
    Serial.println(magnetTriggerCount);
    
    if (magnetTriggerCount == 6) {
      // 6th trigger is completely ignored - no task execution, no index increment
      Serial.println("6th trigger ignored as per task requirements");
      // Do nothing - continue line following as if no magnet was detected
    } else if (isGoingBack && !secondReverseTraverse) {
      // Special case: during reverse traverse, handle 180 rotation
      taskRotate180();
      isGoingBack = false;
      secondReverseTraverse = true;
      currentTaskIndex++; // Move to next task after completing reverse traverse
    } else {
      // All other triggers perform their respective tasks
      stopMotors();
      performTask(currentTaskIndex);
      currentTaskIndex++;
      
      inZigzagMode = false;
      zigzagDirection = 0;
    }
  }
  
  delay(5);
}

bool isOnLine() {
  readSensors();
  
  for (int i = 0; i < numSensors; i++) {
    int normalizedValue = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    normalizedValue = constrain(normalizedValue, 0, 1000);
    
    if (normalizedValue > LINE_DETECTION_THRESHOLD) {
      return true;
    }
  }
  return false;
}

bool isCenteredOnLine() {
  readSensors();
  
  for (int i = 3; i <= 4; i++) {
    int normalizedValue = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    normalizedValue = constrain(normalizedValue, 0, 1000);
    
    if (normalizedValue > CENTER_THRESHOLD) {
      return true;
    }
  }
  return false;
}

bool detectLineCrossing() {
  bool currentlyOnLine = isOnLine();
  bool currentlyCentered = isCenteredOnLine();
  bool crossingDetected = false;
  
  if (currentlyCentered && !wasCentered) {
    if (millis() - lastCrossingTime > MIN_CROSSING_INTERVAL) {
      crossingDetected = true;
      lastCrossingTime = millis();
      lineCrossings++;
    }
  }
  
  wasOnLine = currentlyOnLine;
  wasCentered = currentlyCentered;
  return crossingDetected;
}

void resetLineCrossings() {
  lineCrossings = 0;
  wasOnLine = isOnLine();
  wasCentered = isCenteredOnLine();
  lastCrossingTime = millis();
}

// Competition Task Functions

void taskStop5Seconds() {
  Serial.println("Task: Stop for 5 seconds");
  stopMotors();
  delay(5000);
  
  // Reset PID variables after stop to prevent jerky movement
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;
  
  // Brief realignment attempt
  unsigned long alignStart = millis();
  while (millis() - alignStart < 500) {
    if (isOnLine()) break;
    
    // Gentle search for line
    if (lastValidPosition < 3500) {
      runMotors(-80, 80);
    } else {
      runMotors(80, -80);
    }
    delay(10);
  }
  stopMotors();
  delay(100);
}

void taskTurnOnRedLED() {
  Serial.println("Task: Turn on Red LED");
  redLEDState = true;
  updateLEDStates();
}

void taskYellowOnRedOff() {
  Serial.println("Task: Yellow LED on, Red LED off");
  yellowLEDState = true;
  redLEDState = false;
  updateLEDStates();
}

void taskTurnOnGreenLED() {
  Serial.println("Task: Turn on Green LED, Red LED off");
  greenLEDState = true;
  yellowLEDState = false;
  updateLEDStates();
}

void taskRotate360() {
  Serial.println("Task: 360 degree rotation");
  
  resetLineCrossings();
  runMotors(ROTATION_SPEED, -ROTATION_SPEED);
  
  unsigned long startTime = millis();
  bool rotationComplete = false;
  bool firstLinePassed = false;
  const unsigned long minRotationTime360 = 3300; // Double the 180° time (2000ms * 2)
  
  while (!rotationComplete && (millis() - startTime < ROTATION_TIMEOUT)) {
    if (millis() - startTime >= minRotationTime360) {
      if (detectLineCrossing()) {
        if (!firstLinePassed) {
          firstLinePassed = true;
        } else {
          rotationComplete = true;
        }
      }
    }
    delay(10);
  }
  
  stopMotors();
  delay(200);
  
  // FIXED: Removed the checkMagnetDetection() call that was consuming the next trigger
  delay(300);
}

void taskReverseTraverse() {
  Serial.println("Task: Reverse traverse");
  taskRotate180();
  isGoingBack = true;
}

void taskSecondReverseTraverse() {
  Serial.println("Task: Second reverse traverse trigger");
  // This is handled in the main magnet detection logic
}

void taskStopBot() {
  Serial.println("Task: Stop bot permanently");
  stopMotors();
  while(true) {
    updateLEDStates();
    delay(1000);
  }
}

void taskRotate180() {
  resetLineCrossings();
  runMotors(ROTATION_SPEED, -ROTATION_SPEED);
  
  unsigned long startTime = millis();
  bool rotationComplete = false;
  const unsigned long minRotationTime180 = 1800;
  
  while (!rotationComplete && (millis() - startTime < ROTATION_TIMEOUT)) {
    if (millis() - startTime >= minRotationTime180) {
      if (detectLineCrossing()) {
        rotationComplete = true;
      }
    }
    delay(10);
  }
  
  stopMotors();
  delay(200);
}

void performTask(int taskIndex) {
  switch (taskIndex) {
    case 0: // Task 1: Stop 5 seconds
      taskStop5Seconds();
      break;
    case 1: // Task 2: Turn on red LED
      taskTurnOnRedLED();
      break;
    case 2: // Task 3: Yellow on, red off
      taskYellowOnRedOff();
      break;
    case 3: // Task 4: Green LED on, red off
      taskTurnOnGreenLED();
      break;
    case 4: // Task 5: 360 rotation
      taskRotate360();
      break;
    case 5: // Task 6: This won't be reached due to ignored 6th trigger
      Serial.println("Task 6 - This shouldn't execute");
      break;
    case 6: // Task 7: Reverse traverse
      taskReverseTraverse();
      break;
    case 7: // Task 8: Second reverse traverse
      taskSecondReverseTraverse();
      break;
    case 8: // Task 9: Stop bot
      taskStopBot();
      break;
    default:
      Serial.println("Invalid task");
      break;
  }
}

void runMotors(int speedA, int speedB) {
  setMotor(AIN1, AIN2, PWMA, speedA);
  setMotor(BIN1, BIN2, PWMB, speedB);
}

void stopMotors() {
  runMotors(0, 0);
}

void setMotor(int in1, int in2, int pwm, int spd) {
  if (spd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, spd);
  } else if (spd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -spd);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}
