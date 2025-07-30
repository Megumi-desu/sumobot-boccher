/*
 *  SumoBot 3kg TechnoCorner 2025 - RC Control Version
 *  Ready to Win for TechnoCorner 2025!!
 *  Boccher Team
 *  Component : Motor DC 12 V, Driver BTS7960, RC Receiver 2.4GHz, ESP32 Doit Devkit V1
 *  RC Transmitter: TX4 2.4 GHz 4CH
 */

// Motor kiri
#define RPWM_KIRI 32
#define LPWM_KIRI 33
#define R_EN_KIRI 25
#define L_EN_KIRI 26

// Motor kanan
#define RPWM_KANAN 18
#define LPWM_KANAN 19
#define R_EN_KANAN 16
#define L_EN_KANAN 17

// RC Receiver pins
#define RC_CH1_PIN 4   // Throttle (maju-mundur)
#define RC_CH2_PIN 5   // Steering (belok kiri-kanan)
#define RC_CH3_PIN 23  // Switch untuk mode rotate
#define RC_CH4_PIN 22  // Emergency stop

#define PWM_FREQUENCY          5000
#define PWM_RESOLUTION         8
#define PWM_CHANNEL_RPWM_KIRI  0
#define PWM_CHANNEL_LPWM_KIRI  1
#define PWM_CHANNEL_RPWM_KANAN 2
#define PWM_CHANNEL_LPWM_KANAN 3

// RC signal parameters
#define RC_MIN_PULSE 1000  // Minimum pulse width (microseconds)
#define RC_MAX_PULSE 2000  // Maximum pulse width (microseconds)
#define RC_MID_PULSE 1500  // Middle/neutral position
#define RC_DEADZONE 50     // Deadzone around neutral

// Motor parameters
int maxSpeed = 255;
int minSpeed = 50;
bool isRCConnected = false;
unsigned long lastRCSignal = 0;
#define RC_TIMEOUT 1000    // RC signal timeout (milliseconds)

// LED status
#define LED_BUILTIN 2
unsigned long lastBlinkTime = 0;
bool ledState = false;

// RC channel values
volatile unsigned long rc_ch1_start = 0;
volatile unsigned long rc_ch2_start = 0;
volatile unsigned long rc_ch3_start = 0;
volatile unsigned long rc_ch4_start = 0;

volatile int rc_ch1_value = RC_MID_PULSE;
volatile int rc_ch2_value = RC_MID_PULSE;
volatile int rc_ch3_value = RC_MID_PULSE;
volatile int rc_ch4_value = RC_MID_PULSE;





// RC Channel 1 interrupt (Throttle)
void IRAM_ATTR readRC_CH1() {
  if (digitalRead(RC_CH1_PIN) == HIGH) {
    rc_ch1_start = micros();
  } else {
    if (rc_ch1_start != 0) {
      rc_ch1_value = micros() - rc_ch1_start;
      rc_ch1_start = 0;
      lastRCSignal = millis();
    }
  }
}

// RC Channel 2 interrupt (Steering)
void IRAM_ATTR readRC_CH2() {
  if (digitalRead(RC_CH2_PIN) == HIGH) {
    rc_ch2_start = micros();
  } else {
    if (rc_ch2_start != 0) {
      rc_ch2_value = micros() - rc_ch2_start;
      rc_ch2_start = 0;
      lastRCSignal = millis();
    }
  }
}

// RC Channel 3 interrupt (Mode switch)
void IRAM_ATTR readRC_CH3() {
  if (digitalRead(RC_CH3_PIN) == HIGH) {
    rc_ch3_start = micros();
  } else {
    if (rc_ch3_start != 0) {
      rc_ch3_value = micros() - rc_ch3_start;
      rc_ch3_start = 0;
      lastRCSignal = millis();
    }
  }
}

// RC Channel 4 interrupt (Emergency stop)
void IRAM_ATTR readRC_CH4() {
  if (digitalRead(RC_CH4_PIN) == HIGH) {
    rc_ch4_start = micros();
  } else {
    if (rc_ch4_start != 0) {
      rc_ch4_value = micros() - rc_ch4_start;
      rc_ch4_start = 0;
      lastRCSignal = millis();
    }
  }
}






void setup() {
  Serial.begin(115200);
  
  // Setup motor pins
  pinMode(R_EN_KIRI, OUTPUT);
  pinMode(L_EN_KIRI, OUTPUT);
  pinMode(R_EN_KANAN, OUTPUT);
  pinMode(L_EN_KANAN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Enable motor drivers
  digitalWrite(R_EN_KIRI, HIGH);
  digitalWrite(L_EN_KIRI, HIGH);
  digitalWrite(R_EN_KANAN, HIGH);
  digitalWrite(L_EN_KANAN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  // Setup PWM channels
  ledcSetup(PWM_CHANNEL_RPWM_KIRI, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LPWM_KIRI, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RPWM_KANAN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LPWM_KANAN, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(RPWM_KIRI, PWM_CHANNEL_RPWM_KIRI);
  ledcAttachPin(LPWM_KIRI, PWM_CHANNEL_LPWM_KIRI);
  ledcAttachPin(RPWM_KANAN, PWM_CHANNEL_RPWM_KANAN);
  ledcAttachPin(LPWM_KANAN, PWM_CHANNEL_LPWM_KANAN);

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
  Serial.println("Menunggu sinyal RC...");
}

void loop() {
  // Check RC connection status
  checkRCConnection();
  
  // Handle LED status
  handleLEDStatus();
  
  // Process RC commands if connected
  if (isRCConnected) {
    processRCCommands();
  } else {
    stopMotors();
  }
  
  // Debug output (comment out for competition)
  if (millis() % 500 == 0) {
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
  int leftMotorSpeed  = throttle + steering;
  int rightMotorSpeed = throttle - steering;

  leftMotorSpeed  = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Motor kiri
  if (leftSpeed > 0) {
    ledcWrite(PWM_CHANNEL_RPWM_KIRI, leftSpeed);
    ledcWrite(PWM_CHANNEL_LPWM_KIRI, 0);
  } else if (leftSpeed < 0) {
    ledcWrite(PWM_CHANNEL_RPWM_KIRI, 0);
    ledcWrite(PWM_CHANNEL_LPWM_KIRI, -leftSpeed);
  } else {
    ledcWrite(PWM_CHANNEL_RPWM_KIRI, 0);
    ledcWrite(PWM_CHANNEL_LPWM_KIRI, 0);
  }

  // Motor kanan
  if (rightSpeed > 0) {
    ledcWrite(PWM_CHANNEL_RPWM_KANAN, rightSpeed);
    ledcWrite(PWM_CHANNEL_LPWM_KANAN, 0);
  } else if (rightSpeed < 0) {
    ledcWrite(PWM_CHANNEL_RPWM_KANAN, 0);
    ledcWrite(PWM_CHANNEL_LPWM_KANAN, -rightSpeed);
  } else {
    ledcWrite(PWM_CHANNEL_RPWM_KANAN, 0);
    ledcWrite(PWM_CHANNEL_LPWM_KANAN, 0);
  }
}

void rotateLeft(int speed) {
  ledcWrite(PWM_CHANNEL_RPWM_KIRI, 0);
  ledcWrite(PWM_CHANNEL_LPWM_KIRI, speed);
  ledcWrite(PWM_CHANNEL_RPWM_KANAN, speed);
  ledcWrite(PWM_CHANNEL_LPWM_KANAN, 0);
}

void rotateRight(int speed) {
  ledcWrite(PWM_CHANNEL_RPWM_KIRI, speed);
  ledcWrite(PWM_CHANNEL_LPWM_KIRI, 0);
  ledcWrite(PWM_CHANNEL_RPWM_KANAN, 0);
  ledcWrite(PWM_CHANNEL_LPWM_KANAN, speed);
}

void stopMotors() {
  ledcWrite(PWM_CHANNEL_RPWM_KIRI, 0);
  ledcWrite(PWM_CHANNEL_LPWM_KIRI, 0);
  ledcWrite(PWM_CHANNEL_RPWM_KANAN, 0);
  ledcWrite(PWM_CHANNEL_LPWM_KANAN, 0);
}