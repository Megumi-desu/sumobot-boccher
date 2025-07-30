/*
 *  SumoBot 3kg TechnoCorner 2025 - RC Control Version dengan L298N Driver
 *  Ready to Win for TechnoCorner 2025!!
 *  Boccher Team - L298N VERSION
 *  Component : Motor JGA25-370 dengan driver L298N, RC Receiver 2.4GHz, ESP32 Doit Devkit V1
 *  RC Transmitter: TX4 2.4 GHz 4CH
 */

// L298N Driver Motor Kanan
#define MOTOR_KANAN_IN1 32   
#define MOTOR_KANAN_IN2 33     
#define MOTOR_KANAN_ENA 25   

// L298N Driver Motor Kiri
#define MOTOR_KIRI_IN3 27    
#define MOTOR_KIRI_IN4 14    
#define MOTOR_KIRI_ENB 12    

// RC Receiver pins
#define RC_CH1_PIN 5   // Steering
#define RC_CH2_PIN 17  // Throttle
#define RC_CH3_PIN 16  // Switch untuk mode rotate
#define RC_CH4_PIN 4   // Emergency stop

// PWM Configuration untuk L298N Enable pins
#define PWM_FREQUENCY          1000
#define PWM_RESOLUTION         8
#define PWM_CHANNEL_KANAN      0    // Channel PWM untuk ENA (motor kanan)
#define PWM_CHANNEL_KIRI       1    // Channel PWM untuk ENB (motor kiri)

// RC signal parameters - SAMA SEPERTI SEBELUMNYA
#define RC_MIN_PULSE 1100    
#define RC_MAX_PULSE 1900    
#define RC_MID_PULSE 1500
#define RC_DEADZONE 50

// #define THROTTLE_MAX_POINT 1320  // 3/5 dari range (1500-1100)*0.6 + 1100
// #define THROTTLE_MIN_POINT 1680  // 3/5 dari range (1900-1500)*0.6 + 1500
// #define STEERING_MAX_POINT 1320  // 3/5 dari range
// #define STEERING_MIN_POINT 1680  // 3/5 dari range

#define RC_CH1_CENTER 1500
#define RC_CH2_CENTER 1500

// Motor parameters
int maxSpeed = 255;          
int minSpeed = 20;           
int speedMultiplier = 100;   
bool isRCConnected = false;
unsigned long lastRCSignal = 0;
#define RC_TIMEOUT 1000

// LED status
#define LED_BUILTIN 2
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Debug mode
bool debugMode = true;
unsigned long lastDebugTime = 0;
#define DEBUG_INTERVAL 500   

// RC channel values
volatile unsigned long rc_ch1_start = 0;
volatile unsigned long rc_ch2_start = 0;
volatile unsigned long rc_ch3_start = 0;
volatile unsigned long rc_ch4_start = 0;

volatile int rc_ch1_value = RC_CH1_CENTER;  // Steering
volatile int rc_ch2_value = RC_CH2_CENTER;  // Throttle
volatile int rc_ch3_value = RC_MID_PULSE;   // Rotate mode
volatile int rc_ch4_value = RC_MAX_PULSE;   // Emergency stop (default HIGH)

// Motor status untuk debug
int currentFixRightSpeed = 0;
int currentFixLeftSpeed = 0;
String currentMode = "STOP";
int mappedThrottle = 0;
int mappedSteering = 0;

// RC Channel interrupt handlers - SAMA SEPERTI SEBELUMNYA
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
  delay(2000);  
  
  Serial.println("==========================================");
  Serial.println("  SUMOBOT RC CONTROL - L298N VERSION");
  Serial.println("==========================================");
  Serial.println("Motor Driver: L298N Dual H-Bridge");
  Serial.println("Channel Mapping:");
  Serial.println("CH1 = STEERING (Kiri-Kanan)");
  Serial.println("CH2 = THROTTLE (Maju-Mundur)");
  Serial.println("CH3 = ROTATE MODE (Optional)");
  Serial.println("CH4 = EMERGENCY STOP");
  Serial.println("==========================================");
  Serial.println("L298N Pin Connections:");
  Serial.println("Motor Kiri: IN3=27, IN4=14, ENB=12");
  Serial.println("Motor Kanan: IN1=32, IN2=33, ENA=25");
  Serial.println("==========================================");
  
  // Setup motor direction pins sebagai output
  pinMode(MOTOR_KANAN_IN1, OUTPUT);
  pinMode(MOTOR_KANAN_IN2, OUTPUT);
  pinMode(MOTOR_KIRI_IN3, OUTPUT);
  pinMode(MOTOR_KIRI_IN4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Setup PWM channels untuk Enable pins L298N
  ledcSetup(PWM_CHANNEL_KANAN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_KIRI, PWM_FREQUENCY, PWM_RESOLUTION);

  // Attach PWM ke Enable pins
  ledcAttachPin(MOTOR_KANAN_ENA, PWM_CHANNEL_KANAN);
  ledcAttachPin(MOTOR_KIRI_ENB, PWM_CHANNEL_KIRI);

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
    Serial.print("Right Motor: "); Serial.println(currentFixRightSpeed);
    Serial.print("Left Motor: "); Serial.println(currentFixLeftSpeed);
    
    Serial.println("--- L298N Status ---");
    Serial.print("Right ENA (PWM): "); Serial.println(ledcRead(PWM_CHANNEL_KANAN));
    Serial.print("Right Direction: IN1="); Serial.print(digitalRead(MOTOR_KANAN_IN1));
    Serial.print(", IN2="); Serial.println(digitalRead(MOTOR_KANAN_IN2));
    Serial.print("Left ENB (PWM): "); Serial.println(ledcRead(PWM_CHANNEL_KIRI));
    Serial.print("Left Direction: IN3="); Serial.print(digitalRead(MOTOR_KIRI_IN3));
    Serial.print(", IN4="); Serial.println(digitalRead(MOTOR_KIRI_IN4));
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
  
  // Mapping RC input: CH1 = Steering, CH2 = Throttle
  // PERBAIKAN: Mapping RC input dengan pendekatan yang lebih sederhana
  // Mapping dari center point yang sebenarnya
  // CH1 = Steering
  if (rc_ch1_value > RC_CH1_CENTER) {
    // Steering ke kanan (nilai > center)
    mappedSteering = map(rc_ch1_value, RC_CH1_CENTER, RC_MAX_PULSE, 0, maxSpeed);
  } else {
    // Steering ke kiri (nilai < center)  
    mappedSteering = map(rc_ch1_value, RC_MIN_PULSE, RC_CH1_CENTER, -maxSpeed, 0);
  }
  // CH2 = Throttle
  if (rc_ch2_value > RC_CH2_CENTER) {
    // Throttle maju (nilai > center)
    mappedThrottle = map(rc_ch2_value, RC_CH2_CENTER, RC_MAX_PULSE, 0, maxSpeed);
  } else {
    // Throttle mundur (nilai < center)
    mappedThrottle = map(rc_ch2_value, RC_MIN_PULSE, RC_CH2_CENTER, -maxSpeed, 0);
  }

  // Apply speed multiplier
  mappedSteering = (mappedSteering * speedMultiplier) / 100;
  mappedThrottle = (mappedThrottle * speedMultiplier) / 100;
  
  // Apply deadzone
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
  
  // Normal movement mode
  if (abs(mappedThrottle) > 0 || abs(mappedSteering) > 0) {
    moveRobot(mappedThrottle, mappedSteering);
    currentMode = "NORMAL_DRIVE";
  } else {
    stopMotors();
    currentMode = "IDLE";
  }
}

void moveRobot(int throttle, int steering) {
  // Tank drive mixing
  int leftMotorSpeed  = throttle + steering;  
  int rightMotorSpeed = throttle - steering;

  // Constrain speeds
  leftMotorSpeed  = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  setMotorSpeed(rightMotorSpeed, leftMotorSpeed);
}

void setMotorSpeed(int FixRightSpeed, int FixLeftSpeed) {
  currentFixRightSpeed = FixRightSpeed;
  currentFixLeftSpeed = FixLeftSpeed;
  
  // // Apply minimum speed threshold
  // if (abs(FixRightSpeed) > 0 && abs(FixRightSpeed) < minSpeed) {
  //   FixRightSpeed = 0;
  // }
  // if (abs(FixLeftSpeed) > 0 && abs(FixLeftSpeed) < minSpeed) {
  //   FixLeftSpeed = 0;
  // }

  // Apply Maximum Speed
  if (FixRightSpeed > 150) {
    FixRightSpeed = maxSpeed;
  } else if (FixRightSpeed < -150) {
    FixRightSpeed = -maxSpeed;
  }

  if (FixLeftSpeed > 150)  {
    FixLeftSpeed  = maxSpeed;
  } else if (FixLeftSpeed < -150) {
    FixLeftSpeed  = -maxSpeed;
  }
  
  // Motor kanan dengan L298N
  if (FixRightSpeed > 0) {
    // Maju: IN1=LOW, IN2=HIGH
    digitalWrite(MOTOR_KANAN_IN1, LOW);
    digitalWrite(MOTOR_KANAN_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_KANAN, abs(FixRightSpeed));
  } else if (FixRightSpeed < 0) {
    // Mundur: IN1=HIGH, IN2=LOW
    digitalWrite(MOTOR_KANAN_IN1, HIGH);
    digitalWrite(MOTOR_KANAN_IN2, LOW);
    ledcWrite(PWM_CHANNEL_KANAN, abs(FixRightSpeed));
  } else {
    // Stop: IN1=LOW, IN2=LOW, PWM=0
    digitalWrite(MOTOR_KANAN_IN1, LOW);
    digitalWrite(MOTOR_KANAN_IN2, LOW);
    ledcWrite(PWM_CHANNEL_KANAN, 0);
  }

  // Motor kiri dengan L298N
  if (FixLeftSpeed > 0) {
    // Maju: IN3=HIGH, IN4=LOW
    digitalWrite(MOTOR_KIRI_IN3, HIGH);
    digitalWrite(MOTOR_KIRI_IN4, LOW);
    ledcWrite(PWM_CHANNEL_KIRI, abs(FixLeftSpeed));
  } else if (FixLeftSpeed < 0) {
    // Mundur: IN3=LOW, IN4=HIGH
    digitalWrite(MOTOR_KIRI_IN3, LOW);
    digitalWrite(MOTOR_KIRI_IN4, HIGH);
    ledcWrite(PWM_CHANNEL_KIRI, abs(FixLeftSpeed));
  } else {
    // Stop: IN3=LOW, IN4=LOW, PWM=0
    digitalWrite(MOTOR_KIRI_IN3, LOW);
    digitalWrite(MOTOR_KIRI_IN4, LOW);
    ledcWrite(PWM_CHANNEL_KIRI, 0);
  }
}

void rotateLeft(int speed) {
  // Rotate left: motor kiri mundur, motor kanan maju
  currentFixRightSpeed = speed;
  currentFixLeftSpeed = -speed;
  currentMode = "ROTATE_LEFT";
  
  speed = constrain(speed, minSpeed, maxSpeed);
  
  // Motor kanan maju
  digitalWrite(MOTOR_KANAN_IN1, LOW);
  digitalWrite(MOTOR_KANAN_IN2, HIGH);
  ledcWrite(PWM_CHANNEL_KANAN, speed);
  
  // Motor kiri mundur
  digitalWrite(MOTOR_KIRI_IN3, LOW);
  digitalWrite(MOTOR_KIRI_IN4, HIGH);
  ledcWrite(PWM_CHANNEL_KIRI, speed);
}

void rotateRight(int speed) {
  // Rotate right: motor kiri maju, motor kanan mundur
  currentFixRightSpeed = -speed;
  currentFixLeftSpeed = speed;
  currentMode = "ROTATE_RIGHT";
  
  speed = constrain(speed, minSpeed, maxSpeed);
  
  // Motor kanan mundur
  digitalWrite(MOTOR_KANAN_IN1, HIGH);
  digitalWrite(MOTOR_KANAN_IN2, LOW);
  ledcWrite(PWM_CHANNEL_KANAN, speed);
  
  // Motor kiri maju
  digitalWrite(MOTOR_KIRI_IN3, HIGH);
  digitalWrite(MOTOR_KIRI_IN4, LOW);
  ledcWrite(PWM_CHANNEL_KIRI, speed);
}

void stopMotors() {
  currentFixRightSpeed = 0;
  currentFixLeftSpeed = 0;
  if (currentMode != "DISCONNECTED" && currentMode != "EMERGENCY_STOP") {
    currentMode = "STOP";
  }
  
  // Stop semua motor dengan L298N
  digitalWrite(MOTOR_KANAN_IN1, LOW);
  digitalWrite(MOTOR_KANAN_IN2, LOW);
  digitalWrite(MOTOR_KIRI_IN3, LOW);
  digitalWrite(MOTOR_KIRI_IN4, LOW);
  ledcWrite(PWM_CHANNEL_KANAN, 0);
  ledcWrite(PWM_CHANNEL_KIRI, 0);
}

// Test function untuk L298N
void testMotors() {
  Serial.println("\n*** L298N MOTOR TEST MODE ***");
  
  Serial.println("Testing Right Motor Forward...");
  digitalWrite(MOTOR_KANAN_IN1, LOW);
  digitalWrite(MOTOR_KANAN_IN2, HIGH);
  ledcWrite(PWM_CHANNEL_KANAN, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Testing Right Motor Backward...");
  digitalWrite(MOTOR_KANAN_IN1, HIGH);
  digitalWrite(MOTOR_KANAN_IN2, LOW);
  ledcWrite(PWM_CHANNEL_KANAN, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Testing Left Motor Forward...");
  digitalWrite(MOTOR_KIRI_IN3, HIGH);
  digitalWrite(MOTOR_KIRI_IN4, LOW);
  ledcWrite(PWM_CHANNEL_KIRI, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Testing Left Motor Backward...");
  digitalWrite(MOTOR_KIRI_IN3, LOW);
  digitalWrite(MOTOR_KIRI_IN4, HIGH);
  ledcWrite(PWM_CHANNEL_KIRI, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("L298N Motor test complete!");
}

// Fungsi untuk kalibrasi RC - SAMA SEPERTI SEBELUMNYA
void calibrateRC() {
  Serial.println("\n*** RC CALIBRATION MODE ***");
  Serial.println("Move all sticks to extreme positions...");
  
  int minCH1 = 2000, maxCH1 = 1000;
  int minCH2 = 2000, maxCH2 = 1000;
  int centerCH1 = 0, centerCH2 = 0;
  int sampleCount = 0;
  
  unsigned long startTime = millis();
  
  // Ambil sample center value selama 3 detik pertama
  Serial.println("Keep sticks in center position for 3 seconds...");
  while (millis() - startTime < 3000) {
    centerCH1 += rc_ch1_value;
    centerCH2 += rc_ch2_value;
    sampleCount++;
    delay(50);
  }
  
  if (sampleCount > 0) {
    centerCH1 /= sampleCount;
    centerCH2 /= sampleCount;
  }
  
  Serial.println("Now move sticks to all extreme positions...");
  
  // Cari nilai min-max selama 7 detik
  while (millis() - startTime < 10000) {
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
  Serial.print("CH1 Center: "); Serial.println(centerCH1);
  Serial.print("CH2 Range: "); Serial.print(minCH2); Serial.print(" - "); Serial.println(maxCH2);
  Serial.print("CH2 Center: "); Serial.println(centerCH2);
  
  Serial.println("\nUpdate these values in your code:");
  Serial.print("#define RC_CH1_CENTER "); Serial.println(centerCH1);
  Serial.print("#define RC_CH2_CENTER "); Serial.println(centerCH2);
}