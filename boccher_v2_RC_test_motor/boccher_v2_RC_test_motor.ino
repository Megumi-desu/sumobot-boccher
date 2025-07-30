/*
 *  SumoBot 3kg TechnoCorner 2025 - RC Control Version
 *  Ready to Win for TechnoCorner 2025!!
 *  Boccher Team
 *  Component : Motor JGA25-370 dengan driver terintegrasi, RC Receiver 2.4GHz, ESP32 Doit Devkit V1
 *  RC Transmitter: TX4 2.4 GHz 4CH
 */

// Motor kiri
#define MOTOR_KIRI_CW  32
#define MOTOR_KIRI_CCW 33

// Motor kanan
#define MOTOR_KANAN_CW  18
#define MOTOR_KANAN_CCW 19

// RC Receiver pins
#define RC_CH1_PIN 4   // Throttle (maju-mundur)
#define RC_CH2_PIN 5   // Steering (belok kiri-kanan)
#define RC_CH3_PIN 23  // Switch untuk mode rotate
#define RC_CH4_PIN 22  // Emergency stop

// PWM Configuration
#define PWM_FREQUENCY          1000  // Reduced frequency for motor drivers
#define PWM_RESOLUTION         8
#define PWM_CHANNEL_KIRI_CW    0
#define PWM_CHANNEL_KIRI_CCW   1
#define PWM_CHANNEL_KANAN_CW   2
#define PWM_CHANNEL_KANAN_CCW  3

// RC signal parameters
#define RC_MIN_PULSE 1000  // Minimum pulse width (microseconds)
#define RC_MAX_PULSE 2000  // Maximum pulse width (microseconds)
#define RC_MID_PULSE 1500  // Middle/neutral position
#define RC_DEADZONE 50     // Deadzone around neutral

// Motor parameters
int maxSpeed = 255;
int minSpeed = 60;     // Increased minimum speed for better motor response
bool isRCConnected = false;
unsigned long lastRCSignal = 0;
#define RC_TIMEOUT 1000    // RC signal timeout (milliseconds)

// RC channel values
volatile unsigned long rc_ch1_start = 0;
volatile unsigned long rc_ch2_start = 0;
volatile unsigned long rc_ch3_start = 0;
volatile unsigned long rc_ch4_start = 0;

volatile int rc_ch1_value = RC_MID_PULSE;
volatile int rc_ch2_value = RC_MID_PULSE;
volatile int rc_ch3_value = RC_MID_PULSE;
volatile int rc_ch4_value = RC_MID_PULSE;

// RC Channel interrupt handlers
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
  
  Serial.println("Robot Sumo RC Control Ready!");
  Serial.println("Motor: JGA25-370 dengan driver terintegrasi");
  Serial.println("Menunggu sinyal RC...");
}

void loop() {
  // Check RC connection status
  // checkRCConnection();
  
  // Handle LED status
  handleLEDStatus();
  
  // Process RC commands if connected
  /*
  if (isRCConnected) {
    processRCCommands();
  } else {
    stopMotors();
  }
  */

  setMotorSpeed(255, 0); // mundur v lama
  delay(3000);

  setMotorSpeed(0, 255); // mundur v lama
  delay(3000);

  setMotorSpeed(-255, 0); // maju v lama
  delay(3000);

  setMotorSpeed(0, -255); // maju v lama
  delay(3000);

  // digitalWrite(MOTOR_KIRI_CW, HIGH);
  // delay(3000);
  // digitalWrite(MOTOR_KIRI_CW, LOW);
  // delay(100);
  // digitalWrite(MOTOR_KIRI_CCW, HIGH);
  // delay(3000);
  // digitalWrite(MOTOR_KIRI_CCW, LOW);
  // delay(100);
  // digitalWrite(MOTOR_KANAN_CW, HIGH);
  // delay(3000);
  // digitalWrite(MOTOR_KANAN_CW, LOW);
  // delay(100);
  // digitalWrite(MOTOR_KANAN_CCW, HIGH);
  // delay(3000);
  // digitalWrite(MOTOR_KANAN_CCW, LOW);
  // delay(100);
  
  // Debug output (comment out for competition)
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime >= 500) {
    lastDebugTime = millis();
    Serial.print("CH1: "); Serial.print(rc_ch1_value);
    Serial.print(" CH2: "); Serial.print(rc_ch2_value);
    Serial.print(" CH3: "); Serial.print(rc_ch3_value);
    Serial.print(" CH4: "); Serial.print(rc_ch4_value);
    Serial.print(" Connected: "); Serial.println(isRCConnected);
  }
  
  delay(20);
}

void checkRCConnection() {
  unsigned long currentTime = millis();
  
  // Check if we received RC signal recently
  if (currentTime - lastRCSignal < RC_TIMEOUT) {
    if (!isRCConnected) {
      isRCConnected = true;
      Serial.println("RC Transmitter terhubung!");
    }
  } else {
    if (isRCConnected) {
      isRCConnected = false;
      Serial.println("RC Transmitter terputus!");
      stopMotors();
    }
  }
}

void handleLEDStatus() {
  if (isRCConnected) {
    // Blink LED when connected
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= 500) {
      lastBlinkTime = currentTime;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
  } else {
    // LED off when disconnected
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void processRCCommands() {
  // Emergency stop check (CH4)
  if (rc_ch4_value > 1700) {  // High position = emergency stop
    stopMotors();
    return;
  }
  
  // Convert RC values to motor commands
  int throttle = map(rc_ch1_value, RC_MIN_PULSE, RC_MAX_PULSE, -maxSpeed, maxSpeed);
  int steering = map(rc_ch2_value, RC_MIN_PULSE, RC_MAX_PULSE, -maxSpeed, maxSpeed);
  
  // Apply deadzone
  if (abs(throttle) < minSpeed) throttle = 0;
  if (abs(steering) < minSpeed) steering = 0;
  
  // Check for rotate mode (CH3)
  if (rc_ch3_value > 1700) {  // High position = rotate mode
    if (steering > 0) {
      rotateRight(abs(steering));
    } else if (steering < 0) {
      rotateLeft(abs(steering));
    } else {
      stopMotors();
    }
    return;
  }
  
  // Normal movement mode
  moveRobot(throttle, steering);
}

void moveRobot(int throttle, int steering) {
  // Tank drive mixing
  int leftMotorSpeed  = throttle + steering;
  int rightMotorSpeed = throttle - steering;

  // Constrain speeds
  leftMotorSpeed  = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Motor kiri
  // Untuk maju: motor kiri harus berputar CCW
  // Untuk mundur: motor kiri harus berputar CW
  if (leftSpeed > 0) {
    // Maju - CCW
    ledcWrite(PWM_CHANNEL_KIRI_CW, 0);
    ledcWrite(PWM_CHANNEL_KIRI_CCW, leftSpeed);
  } else if (leftSpeed < 0) {
    // Mundur - CW
    ledcWrite(PWM_CHANNEL_KIRI_CW, -leftSpeed);
    ledcWrite(PWM_CHANNEL_KIRI_CCW, 0);
  } else {
    // Stop
    ledcWrite(PWM_CHANNEL_KIRI_CW, 0);
    ledcWrite(PWM_CHANNEL_KIRI_CCW, 0);
  }

  // Motor kanan
  // Untuk maju: motor kanan harus berputar CW
  // Untuk mundur: motor kanan harus berputar CCW
  if (rightSpeed > 0) {
    // Maju - CW
    ledcWrite(PWM_CHANNEL_KANAN_CW, rightSpeed);
    ledcWrite(PWM_CHANNEL_KANAN_CCW, 0);
  } else if (rightSpeed < 0) {
    // Mundur - CCW
    ledcWrite(PWM_CHANNEL_KANAN_CW, 0);
    ledcWrite(PWM_CHANNEL_KANAN_CCW, -rightSpeed);
  } else {
    // Stop
    ledcWrite(PWM_CHANNEL_KANAN_CW, 0);
    ledcWrite(PWM_CHANNEL_KANAN_CCW, 0);
  }
}

void rotateLeft(int speed) {
  // Rotate left: motor kiri mundur (CW), motor kanan maju (CW)
  ledcWrite(PWM_CHANNEL_KIRI_CW, speed);   // Motor kiri CW (mundur)
  ledcWrite(PWM_CHANNEL_KIRI_CCW, 0);
  ledcWrite(PWM_CHANNEL_KANAN_CW, speed);  // Motor kanan CW (maju)
  ledcWrite(PWM_CHANNEL_KANAN_CCW, 0);
}

void rotateRight(int speed) {
  // Rotate right: motor kiri maju (CCW), motor kanan mundur (CCW)
  ledcWrite(PWM_CHANNEL_KIRI_CW, 0);
  ledcWrite(PWM_CHANNEL_KIRI_CCW, speed);  // Motor kiri CCW (maju)
  ledcWrite(PWM_CHANNEL_KANAN_CW, 0);
  ledcWrite(PWM_CHANNEL_KANAN_CCW, speed); // Motor kanan CCW (mundur)
}

void stopMotors() {
  ledcWrite(PWM_CHANNEL_KIRI_CW, 0);
  ledcWrite(PWM_CHANNEL_KIRI_CCW, 0);
  ledcWrite(PWM_CHANNEL_KANAN_CW, 0);
  ledcWrite(PWM_CHANNEL_KANAN_CCW, 0);
}

// Fungsi alternatif menggunakan analogWrite (jika diperlukan)
/*
void setMotorSpeedAnalog(int leftSpeed, int rightSpeed) {
  // Motor kiri
  if (leftSpeed > 0) {
    analogWrite(MOTOR_KIRI_CW, 0);
    analogWrite(MOTOR_KIRI_CCW, leftSpeed);
  } else if (leftSpeed < 0) {
    analogWrite(MOTOR_KIRI_CW, -leftSpeed);
    analogWrite(MOTOR_KIRI_CCW, 0);
  } else {
    analogWrite(MOTOR_KIRI_CW, 0);
    analogWrite(MOTOR_KIRI_CCW, 0);
  }

  // Motor kanan
  if (rightSpeed > 0) {
    analogWrite(MOTOR_KANAN_CW, rightSpeed);
    analogWrite(MOTOR_KANAN_CCW, 0);
  } else if (rightSpeed < 0) {
    analogWrite(MOTOR_KANAN_CW, 0);
    analogWrite(MOTOR_KANAN_CCW, -rightSpeed);
  } else {
    analogWrite(MOTOR_KANAN_CW, 0);
    analogWrite(MOTOR_KANAN_CCW, 0);
  }
}
*/