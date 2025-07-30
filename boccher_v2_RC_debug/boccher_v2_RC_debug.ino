/*
 *  SumoBot 3kg TechnoCorner 2025 - RC Control Version (FIXED & IMPROVED)
 *  Ready to Win for TechnoCorner 2025!!
 *  Boccher Team - COMPREHENSIVE DEBUG VERSION
 *  Component : Motor JGA25-370 dengan driver terintegrasi, RC Receiver 2.4GHz, ESP32 Doit Devkit V1
 *  RC Transmitter: TX4 2.4 GHz 4CH
 */

// Motor kiri
#define MOTOR_KIRI_CW  32
#define MOTOR_KIRI_CCW 33

// Motor kanan
#define MOTOR_KANAN_CW  18
#define MOTOR_KANAN_CCW 19

// RC Receiver pins - FIXED MAPPING
#define RC_CH1_PIN 4   // Steering (belok kiri-kanan) - FIXED
#define RC_CH2_PIN 5   // Throttle (maju-mundur) - FIXED
#define RC_CH3_PIN 23  // Switch untuk mode rotate (opsional)
#define RC_CH4_PIN 22  // Emergency stop

// PWM Configuration
#define PWM_FREQUENCY          1000
#define PWM_RESOLUTION         8
#define PWM_CHANNEL_KIRI_CW    0
#define PWM_CHANNEL_KIRI_CCW   1
#define PWM_CHANNEL_KANAN_CW   2
#define PWM_CHANNEL_KANAN_CCW  3

// RC signal parameters - IMPROVED VALUES
#define RC_MIN_PULSE 1100    // Adjusted for better range
#define RC_MAX_PULSE 1900    // Adjusted for better range
#define RC_MID_PULSE 1500
#define RC_DEADZONE 30       // Reduced deadzone

// Motor parameters - IMPROVED
int maxSpeed = 255;          // Full PWM range
int minSpeed = 50;           // Minimum speed untuk motor response
int speedMultiplier = 100;   // Percentage of max speed to use (adjustable)
bool isRCConnected = false;
unsigned long lastRCSignal = 0;
#define RC_TIMEOUT 1000

// LED status
#define LED_BUILTIN 2
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Debug mode - ENHANCED
bool debugMode = true;
unsigned long lastDebugTime = 0;
#define DEBUG_INTERVAL 500   // Debug setiap 500ms untuk lebih readable

// RC channel values
volatile unsigned long rc_ch1_start = 0;
volatile unsigned long rc_ch2_start = 0;
volatile unsigned long rc_ch3_start = 0;
volatile unsigned long rc_ch4_start = 0;

volatile int rc_ch1_value = RC_MID_PULSE;  // Steering
volatile int rc_ch2_value = RC_MID_PULSE;  // Throttle
volatile int rc_ch3_value = RC_MID_PULSE;  // Rotate mode
volatile int rc_ch4_value = RC_MAX_PULSE;  // Emergency stop (default HIGH)

// Motor status untuk debug
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
String currentMode = "STOP";
int mappedThrottle = 0;
int mappedSteering = 0;

// RC Channel interrupt handlers - SAME AS ORIGINAL (they're correct)
void IRAM_ATTR readRC_CH1() {
  if (digitalRead(RC_CH1_PIN) == HIGH) {
    rc_ch1_start = micros();
  } else {
    if (rc_ch1_start != 0) {
      unsigned long pulseWidth = micros() - rc_ch1_start;
      if (pulseWidth >= RC_MIN_PULSE && pulseWidth <= RC_MAX_PULSE) {
        rc_ch1_value = pulseWidth;
        lastRCSignal = millis();
      }
      rc_ch1_start = 0;
    }
  }
}

void IRAM_ATTR readRC_CH2() {
  if (digitalRead(RC_CH2_PIN) == HIGH) {
    rc_ch2_start = micros();
  } else {
    if (rc_ch2_start != 0) {
      unsigned long pulseWidth = micros() - rc_ch2_start;
      if (pulseWidth >= RC_MIN_PULSE && pulseWidth <= RC_MAX_PULSE) {
        rc_ch2_value = pulseWidth;
        lastRCSignal = millis();
      }
      rc_ch2_start = 0;
    }
  }
}

void IRAM_ATTR readRC_CH3() {
  if (digitalRead(RC_CH3_PIN) == HIGH) {
    rc_ch3_start = micros();
  } else {
    if (rc_ch3_start != 0) {
      unsigned long pulseWidth = micros() - rc_ch3_start;
      if (pulseWidth >= RC_MIN_PULSE && pulseWidth <= RC_MAX_PULSE) {
        rc_ch3_value = pulseWidth;
        lastRCSignal = millis();
      }
      rc_ch3_start = 0;
    }
  }
}

void IRAM_ATTR readRC_CH4() {
  if (digitalRead(RC_CH4_PIN) == HIGH) {
    rc_ch4_start = micros();
  } else {
    if (rc_ch4_start != 0) {
      unsigned long pulseWidth = micros() - rc_ch4_start;
      if (pulseWidth >= RC_MIN_PULSE && pulseWidth <= RC_MAX_PULSE) {
        rc_ch4_value = pulseWidth;
        lastRCSignal = millis();
      }
      rc_ch4_start = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for serial
  
  Serial.println("==========================================");
  Serial.println("  SUMOBOT RC CONTROL - FIXED VERSION");
  Serial.println("==========================================");
  Serial.println("Channel Mapping:");
  Serial.println("CH1 (Pin 4)  = STEERING (Kiri-Kanan)");
  Serial.println("CH2 (Pin 5)  = THROTTLE (Maju-Mundur)");
  Serial.println("CH3 (Pin 23) = ROTATE MODE (Optional)");
  Serial.println("CH4 (Pin 22) = EMERGENCY STOP");
  Serial.println("==========================================");
  
  // Setup motor pins sebagai output
  pinMode(MOTOR_KIRI_CW, OUTPUT);
  pinMode(MOTOR_KIRI_CCW, OUTPUT);
  pinMode(MOTOR_KANAN_CW, OUTPUT);
  pinMode(MOTOR_KANAN_CCW, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Setup PWM channels
  ledcSetup(PWM_CHANNEL_KIRI_CW, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_KIRI_CCW, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_KANAN_CW, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_KANAN_CCW, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_KIRI_CW, PWM_CHANNEL_KIRI_CW);
  ledcAttachPin(MOTOR_KIRI_CCW, PWM_CHANNEL_KIRI_CCW);
  ledcAttachPin(MOTOR_KANAN_CW, PWM_CHANNEL_KANAN_CW);
  ledcAttachPin(MOTOR_KANAN_CCW, PWM_CHANNEL_KANAN_CCW);

  // Setup RC receiver pins
  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);
  pinMode(RC_CH3_PIN, INPUT);
  pinMode(RC_CH4_PIN, INPUT);

  // Attach interrupts for RC channels
  attachInterrupt(digitalPinToInterrupt(RC_CH1_PIN), readRC_CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH2_PIN), readRC_CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH3_PIN), readRC_CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH4_PIN), readRC_CH4, CHANGE);

  stopMotors();
  
  Serial.println("Setup completed! Waiting for RC signal...");
  Serial.println("Debug info will be displayed every 500ms");
  Serial.println("==========================================");
}

void loop() {
  // Check RC connection status
  checkRCConnection();
  
  // Handle LED status
  handleLEDStatus();
  
  // Enhanced debug output
  if (debugMode) {
    printDebugInfo();
  }
  
  // Process RC commands if connected
  if (isRCConnected) {
    processRCCommands();
  } else {
    if (currentMode != "DISCONNECTED") {
      stopMotors();
      currentMode = "DISCONNECTED";
    }
  }
  
  delay(10);
}

void checkRCConnection() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastRCSignal < RC_TIMEOUT) {
    if (!isRCConnected) {
      isRCConnected = true;
      Serial.println("\n*** RC TRANSMITTER TERHUBUNG! ***");
    }
  } else {
    if (isRCConnected) {
      isRCConnected = false;
      Serial.println("\n*** RC TRANSMITTER TERPUTUS! ***");
      stopMotors();
    }
  }
}

void handleLEDStatus() {
  if (isRCConnected) {
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= 500) {
      lastBlinkTime = currentTime;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void printDebugInfo() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
    lastDebugTime = currentTime;
    
    Serial.println("\n========== DEBUG INFO ==========");
    Serial.print("RC Connected: "); Serial.println(isRCConnected ? "YES" : "NO");
    Serial.print("Current Mode: "); Serial.println(currentMode);
    
    Serial.println("--- RC Channels (Raw) ---");
    Serial.print("CH1 (Steering): "); Serial.print(rc_ch1_value); 
    Serial.print(" | Mapped: "); Serial.println(mappedSteering);
    Serial.print("CH2 (Throttle): "); Serial.print(rc_ch2_value); 
    Serial.print(" | Mapped: "); Serial.println(mappedThrottle);
    Serial.print("CH3 (Rotate): "); Serial.println(rc_ch3_value);
    Serial.print("CH4 (E-Stop): "); Serial.println(rc_ch4_value);
    
    Serial.println("--- Motor Speeds ---");
    Serial.print("Left Motor: "); Serial.println(currentLeftSpeed);
    Serial.print("Right Motor: "); Serial.println(currentRightSpeed);
    
    Serial.println("--- PWM Outputs ---");
    Serial.print("Left CW: "); Serial.print(ledcRead(PWM_CHANNEL_KIRI_CW));
    Serial.print(" | Left CCW: "); Serial.println(ledcRead(PWM_CHANNEL_KIRI_CCW));
    Serial.print("Right CW: "); Serial.print(ledcRead(PWM_CHANNEL_KANAN_CW));
    Serial.print(" | Right CCW: "); Serial.println(ledcRead(PWM_CHANNEL_KANAN_CCW));
    Serial.println("================================");
  }
}

void processRCCommands() {
  // Emergency stop check (CH4) - LOW position = emergency stop
  if (rc_ch4_value < 1300) {  
    if (currentMode != "EMERGENCY_STOP") {
      stopMotors();
      currentMode = "EMERGENCY_STOP";
      Serial.println("*** EMERGENCY STOP ACTIVATED! ***");
    }
    return;
  }
  
  // FIXED MAPPING: CH1 = Steering, CH2 = Throttle
  mappedSteering = map(rc_ch1_value, RC_MIN_PULSE, RC_MAX_PULSE, -maxSpeed, maxSpeed);
  mappedThrottle = map(rc_ch2_value, RC_MIN_PULSE, RC_MAX_PULSE, -maxSpeed, maxSpeed);
  
  // Apply speed multiplier (adjustable power)
  mappedSteering = (mappedSteering * speedMultiplier) / 100;
  mappedThrottle = (mappedThrottle * speedMultiplier) / 100;
  
  // Apply deadzone - IMPROVED LOGIC
  if (abs(mappedThrottle) < RC_DEADZONE) mappedThrottle = 0;
  if (abs(mappedSteering) < RC_DEADZONE) mappedSteering = 0;
  
  // Check for rotate mode (CH3) - HIGH position = rotate mode
  if (rc_ch3_value > 1700) {  
    if (abs(mappedSteering) > minSpeed) {
      if (mappedSteering > 0) {
        rotateRight(abs(mappedSteering));
      } else {
        rotateLeft(abs(mappedSteering));
      }
    } else {
      stopMotors();
      currentMode = "ROTATE_IDLE";
    }
    return;
  }
  
  // Normal movement mode - IMPROVED TANK DRIVE
  if (abs(mappedThrottle) > 0 || abs(mappedSteering) > 0) {
    moveRobot(mappedThrottle, mappedSteering);
    currentMode = "NORMAL_DRIVE";
  } else {
    stopMotors();
    currentMode = "IDLE";
  }
}

void moveRobot(int throttle, int steering) {
  // Improved tank drive mixing
  int leftMotorSpeed  = throttle + (steering * 0.95);  // Steering influence 95%
  int rightMotorSpeed = throttle - (steering * 0.95);

  // Constrain speeds
  leftMotorSpeed  = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
  
  // Apply minimum speed threshold
  if (abs(leftSpeed) > 0 && abs(leftSpeed) < minSpeed) {
    leftSpeed = (leftSpeed > 0) ? minSpeed : -minSpeed;
  }
  if (abs(rightSpeed) > 0 && abs(rightSpeed) < minSpeed) {
    rightSpeed = (rightSpeed > 0) ? minSpeed : -minSpeed;
  }
  
  // Motor kiri - FIXED LOGIC
  if (leftSpeed > 0) {
    // Maju - Motor kiri CW
    ledcWrite(PWM_CHANNEL_KIRI_CW, abs(leftSpeed));
    ledcWrite(PWM_CHANNEL_KIRI_CCW, 0);
  } else if (leftSpeed < 0) {
    // Mundur - Motor kiri CCW
    ledcWrite(PWM_CHANNEL_KIRI_CW, 0);
    ledcWrite(PWM_CHANNEL_KIRI_CCW, abs(leftSpeed));
  } else {
    // Stop
    ledcWrite(PWM_CHANNEL_KIRI_CW, 0);
    ledcWrite(PWM_CHANNEL_KIRI_CCW, 0);
  }

  // Motor kanan - FIXED LOGIC
  if (rightSpeed > 0) {
    // Maju - Motor kanan CCW (sesuai dengan original code)
    ledcWrite(PWM_CHANNEL_KANAN_CW, 0);
    ledcWrite(PWM_CHANNEL_KANAN_CCW, abs(rightSpeed));
  } else if (rightSpeed < 0) {
    // Mundur - Motor kanan CW
    ledcWrite(PWM_CHANNEL_KANAN_CW, abs(rightSpeed));
    ledcWrite(PWM_CHANNEL_KANAN_CCW, 0);
  } else {
    // Stop
    ledcWrite(PWM_CHANNEL_KANAN_CW, 0);
    ledcWrite(PWM_CHANNEL_KANAN_CCW, 0);
  }
}

void rotateLeft(int speed) {
  // Rotate left: motor kiri mundur, motor kanan maju
  currentLeftSpeed = -speed;
  currentRightSpeed = speed;
  currentMode = "ROTATE_LEFT";
  
  speed = constrain(speed, minSpeed, maxSpeed);
  
  ledcWrite(PWM_CHANNEL_KIRI_CW, 0);
  ledcWrite(PWM_CHANNEL_KIRI_CCW, speed);    // Kiri mundur
  ledcWrite(PWM_CHANNEL_KANAN_CW, 0);       
  ledcWrite(PWM_CHANNEL_KANAN_CCW, speed);   // Kanan maju
}

void rotateRight(int speed) {
  // Rotate right: motor kiri maju, motor kanan mundur
  currentLeftSpeed = speed;
  currentRightSpeed = -speed;
  currentMode = "ROTATE_RIGHT";
  
  speed = constrain(speed, minSpeed, maxSpeed);
  
  ledcWrite(PWM_CHANNEL_KIRI_CW, speed);     // Kiri maju
  ledcWrite(PWM_CHANNEL_KIRI_CCW, 0);
  ledcWrite(PWM_CHANNEL_KANAN_CW, speed);    // Kanan mundur
  ledcWrite(PWM_CHANNEL_KANAN_CCW, 0);
}

void stopMotors() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  if (currentMode != "DISCONNECTED" && currentMode != "EMERGENCY_STOP") {
    currentMode = "STOP";
  }
  
  ledcWrite(PWM_CHANNEL_KIRI_CW, 0);
  ledcWrite(PWM_CHANNEL_KIRI_CCW, 0);
  ledcWrite(PWM_CHANNEL_KANAN_CW, 0);
  ledcWrite(PWM_CHANNEL_KANAN_CCW, 0);
}

// Test function - panggil dari setup() jika perlu test motor
void testMotors() {
  Serial.println("\n*** MOTOR TEST MODE ***");
  
  Serial.println("Testing Left Motor Forward (CW)...");
  ledcWrite(PWM_CHANNEL_KIRI_CW, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Testing Left Motor Backward (CCW)...");
  ledcWrite(PWM_CHANNEL_KIRI_CCW, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Testing Right Motor Forward (CCW)...");
  ledcWrite(PWM_CHANNEL_KANAN_CCW, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Testing Right Motor Backward (CW)...");
  ledcWrite(PWM_CHANNEL_KANAN_CW, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Motor test complete!");
}

// Fungsi untuk kalibrasi RC (panggil dari setup jika perlu)
void calibrateRC() {
  Serial.println("\n*** RC CALIBRATION MODE ***");
  Serial.println("Move all sticks to extreme positions...");
  
  int minCH1 = 2000, maxCH1 = 1000;
  int minCH2 = 2000, maxCH2 = 1000;
  
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // 10 seconds
    if (rc_ch1_value < minCH1) minCH1 = rc_ch1_value;
    if (rc_ch1_value > maxCH1) maxCH1 = rc_ch1_value;
    if (rc_ch2_value < minCH2) minCH2 = rc_ch2_value;
    if (rc_ch2_value > maxCH2) maxCH2 = rc_ch2_value;
    
    Serial.print("CH1: "); Serial.print(rc_ch1_value);
    Serial.print(" | CH2: "); Serial.println(rc_ch2_value);
    delay(100);
  }
  
  Serial.println("\nCalibration Results:");
  Serial.print("CH1 Range: "); Serial.print(minCH1); Serial.print(" - "); Serial.println(maxCH1);
  Serial.print("CH2 Range: "); Serial.print(minCH2); Serial.print(" - "); Serial.println(maxCH2);
}