/*
 *  SumoBot 3kg TechnoCorner 2025 - PS3 Controller Version
 *  Ready to Win for TechnoCorner 2025!!
 *  Boccher Team
 *  Component : Motor JGA25-370 dengan driver terintegrasi, PS3 Controller, ESP32 Doit Devkit V1
 *  
 *  Library yang diperlukan:
 *  - PS3Controller by Jeffery Pernis (install via Library Manager)
 *  
 *  Cara pairing PS3 Controller:
 *  1. Upload kode ini ke ESP32
 *  2. Buka Serial Monitor
 *  3. Hubungkan PS3 controller ke komputer via USB
 *  4. Tekan tombol PS pada controller
 *  5. Controller akan otomatis terhubung ke ESP32
 */

#include <Ps3Controller.h>

// Motor kiri
#define MOTOR_KIRI_CW  32
#define MOTOR_KIRI_CCW 33

// Motor kanan
#define MOTOR_KANAN_CW  18
#define MOTOR_KANAN_CCW 19

// PWM Configuration
#define PWM_FREQUENCY          1000  // Reduced frequency for motor drivers
#define PWM_RESOLUTION         8
#define PWM_CHANNEL_KIRI_CW    0
#define PWM_CHANNEL_KIRI_CCW   1
#define PWM_CHANNEL_KANAN_CW   2
#define PWM_CHANNEL_KANAN_CCW  3

// Control parameters
#define STICK_DEADZONE 15      // Deadzone untuk analog stick (0-128)
#define MAX_STICK_VALUE 128    // Maximum value dari analog stick
int maxSpeed = 255;
int minSpeed = 60;            // Minimum speed untuk motor response yang baik

// Connection status
bool isPS3Connected = false;
unsigned long lastPS3Signal = 0;
#define PS3_TIMEOUT 3000      // PS3 signal timeout (milliseconds)

// LED status
#define LED_BUILTIN 2
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Control mode
enum ControlMode {
  NORMAL_MODE,
  ROTATE_MODE,
  EMERGENCY_STOP
};
ControlMode currentMode = NORMAL_MODE;

// PS3 Controller callback functions
void onConnect() {
  Serial.println("PS3 Controller terhubung!");
  isPS3Connected = true;
  lastPS3Signal = millis();
  
  // Set LED warna untuk indikasi koneksi (opsional)
  Ps3.setLed(1, 0, 0); // LED Player 1 merah
}

void onDisconnect() {
  Serial.println("PS3 Controller terputus!");
  isPS3Connected = false;
  stopMotors();
}

void notify() {
  // Update timestamp setiap kali ada input
  lastPS3Signal = millis();
  
  // Emergency stop check (tombol SELECT)
  if (Ps3.data.button.select) {
    currentMode = EMERGENCY_STOP;
    stopMotors();
    Serial.println("EMERGENCY STOP!");
    return;
  }
  
  // Mode switching dengan tombol START
  if (Ps3.data.button.start) {
    static unsigned long lastModeSwitch = 0;
    if (millis() - lastModeSwitch > 500) { // Debounce 500ms
      if (currentMode == NORMAL_MODE) {
        currentMode = ROTATE_MODE;
        Serial.println("Mode: ROTATE");
        Ps3.setLed(0, 1, 0); // LED hijau untuk rotate mode
      } else {
        currentMode = NORMAL_MODE;
        Serial.println("Mode: NORMAL");
        Ps3.setLed(1, 0, 0); // LED merah untuk normal mode
      }
      lastModeSwitch = millis();
    }
  }
  
  // Reset emergency stop dengan tombol PS
  if (Ps3.data.button.ps && currentMode == EMERGENCY_STOP) {
    currentMode = NORMAL_MODE;
    Serial.println("Emergency stop direset");
    Ps3.setLed(1, 0, 0); // LED merah untuk normal mode
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

  // Initialize PS3 Controller
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Ps3.begin("01:02:03:04:05:06"); // MAC address ESP32 (akan otomatis terdeteksi)
  
  stopMotors();
  
  Serial.println("Robot Sumo PS3 Control Ready!");
  Serial.println("Motor: JGA25-370 dengan driver terintegrasi");
  Serial.println("Menunggu koneksi PS3 Controller...");
  Serial.println("");
  Serial.println("Kontrol:");
  Serial.println("- Left Stick (Y): Maju/mundur");
  Serial.println("- Right Stick (X): Belok kiri/kanan");
  Serial.println("- START: Toggle Normal/Rotate mode");
  Serial.println("  * Normal: Maju/mundur + belok bersamaan");
  Serial.println("  * Rotate: Hanya rotate tanpa maju/mundur");
  Serial.println("- SELECT: Emergency stop");
  Serial.println("- PS Button: Reset emergency stop");
  Serial.println("- L1/R1: Turbo speed");
  Serial.println("");
}

void loop() {
  // Check PS3 connection status
  checkPS3Connection();
  
  // Handle LED status
  handleLEDStatus();
  
  // Process PS3 commands if connected and not in emergency stop
  if (isPS3Connected && currentMode != EMERGENCY_STOP) {
    processPS3Commands();
  } else if (currentMode == EMERGENCY_STOP) {
    stopMotors(); // Pastikan motor berhenti saat emergency stop
  }
  
  // Debug output (comment out untuk kompetisi)
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime >= 1000) {
    lastDebugTime = millis();
    if (isPS3Connected) {
      Serial.print("LX: "); Serial.print(Ps3.data.analog.stick.lx);
      Serial.print(" LY: "); Serial.print(Ps3.data.analog.stick.ly);
      Serial.print(" RX: "); Serial.print(Ps3.data.analog.stick.rx);
      Serial.print(" RY: "); Serial.print(Ps3.data.analog.stick.ry);
      Serial.print(" Mode: ");
      if (currentMode == NORMAL_MODE) Serial.print("NORMAL");
      else if (currentMode == ROTATE_MODE) Serial.print("ROTATE");
      else Serial.print("EMERGENCY");
      Serial.println();
    }
  }
  
  delay(20);
}

void checkPS3Connection() {
  unsigned long currentTime = millis();
  
  // Check timeout untuk koneksi PS3
  if (isPS3Connected && (currentTime - lastPS3Signal > PS3_TIMEOUT)) {
    Serial.println("PS3 Controller timeout!");
    isPS3Connected = false;
    stopMotors();
  }
}

void handleLEDStatus() {
  if (isPS3Connected) {
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

void processPS3Commands() {
  // Check untuk turbo speed (L1 atau R1)
  int speedMultiplier = 1;
  if (Ps3.data.button.l1 || Ps3.data.button.r1) {
    speedMultiplier = 2; // Turbo mode
  }
  
  // LEFT STICK: Kontrol maju/mundur saja (Y-axis)
  int throttle = -Ps3.data.analog.stick.ly; // Invert Y axis (up = positive)
  
  // RIGHT STICK: Kontrol belok/rotate (X-axis)
  int steering = Ps3.data.analog.stick.rx;  // Left = negative, Right = positive
  
  // Apply deadzone
  if (abs(throttle) < STICK_DEADZONE) throttle = 0;
  if (abs(steering) < STICK_DEADZONE) steering = 0;
  
  // Convert dari stick value (-128 to 127) ke motor speed
  throttle = map(throttle, -MAX_STICK_VALUE, MAX_STICK_VALUE, -maxSpeed, maxSpeed);
  steering = map(steering, -MAX_STICK_VALUE, MAX_STICK_VALUE, -maxSpeed, maxSpeed);
  
  // Apply speed multiplier
  throttle *= speedMultiplier;
  steering *= speedMultiplier;
  
  // Constrain values
  throttle = constrain(throttle, -maxSpeed, maxSpeed);
  steering = constrain(steering, -maxSpeed, maxSpeed);
  
  if (currentMode == NORMAL_MODE) {
    // Normal mode: Kombinasi maju/mundur + belok
    moveRobot(throttle, steering);
    
  } else if (currentMode == ROTATE_MODE) {
    // Rotate mode: Hanya rotate tanpa maju/mundur
    if (abs(steering) > 0) {
      if (steering > 0) {
        rotateRight(abs(steering));
      } else {
        rotateLeft(abs(steering));
      }
    } else {
      stopMotors();
    }
  }
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