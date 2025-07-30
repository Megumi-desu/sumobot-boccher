/*
 *  SumoBot 3kg TechnoCorner 2025 
 *  Ready to Win for TechnoCorner 2025!!
 *  Boccher Team
 *  Component : Motor DC 12 V, Driver BTS7960, Controller PS3, ESP32
 */

#include <Ps3Controller.h>

// motor kiri
#define RPWM_KIRI 32
#define LPWM_KIRI 33
#define R_EN_KIRI 25
#define L_EN_KIRI 26

// motor kanan
#define RPWM_KANAN 18
#define LPWM_KANAN 19
#define R_EN_KANAN 16
#define L_EN_KANAN 17

#define PWM_FREQUENCY          5000
#define PWM_RESOLUTION         8
#define PWM_CHANNEL_RPWM_KIRI  0
#define PWM_CHANNEL_LPWM_KIRI  1
#define PWM_CHANNEL_RPWM_KANAN 2
#define PWM_CHANNEL_LPWM_KANAN 3

int motorSpeed = 0;
int turnSpeed = 0;
int maxSpeed = 255;
int minSpeed = 50;
bool isConnected = false;

#define LED_BUILTIN 2
unsigned long lastBlinkTime = 0;
bool ledState = false;

void setup() {
  Serial.begin(115200);

  pinMode(R_EN_KIRI, OUTPUT);
  pinMode(L_EN_KIRI, OUTPUT);
  pinMode(R_EN_KANAN, OUTPUT);
  pinMode(L_EN_KANAN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(R_EN_KIRI, HIGH);
  digitalWrite(L_EN_KIRI, HIGH);
  digitalWrite(R_EN_KANAN, HIGH);
  digitalWrite(L_EN_KANAN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  ledcSetup(PWM_CHANNEL_RPWM_KIRI, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LPWM_KIRI, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RPWM_KANAN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LPWM_KANAN, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(RPWM_KIRI, PWM_CHANNEL_RPWM_KIRI);
  ledcAttachPin(LPWM_KIRI, PWM_CHANNEL_LPWM_KIRI);
  ledcAttachPin(RPWM_KANAN, PWM_CHANNEL_RPWM_KANAN);
  ledcAttachPin(LPWM_KANAN, PWM_CHANNEL_LPWM_KANAN);

  stopMotors();

  Ps3.begin("01:02:03:04:05:06"); // nanti ganti address nya
  Serial.println("Menunggu koneksi Controller PS3...");

  Ps3.attach(processJoystick);
  Ps3.attachOnConnect(onConnect);
}

void loop() {
  if (isConnected) {
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= 500) {
      lastBlinkTime = currentTime;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(50);
}

void onConnect() {
  isConnected = true;
  Serial.println("Controller PS3 terhubung!");
  digitalWrite(LED_BUILTIN, HIGH);
}

void onDisconnect() {
  isConnected = false;
  Serial.println("Controller PS3 terputus!");
  digitalWrite(LED_BUILTIN, LOW);
  stopMotors();
}

void processJoystick() {
  int leftY  = Ps3.data.analog.stick.ly;
  int rightX = Ps3.data.analog.stick.rx;

  int throttle = -leftY;
  int steering = rightX;

  throttle = map(throttle, -128, 127, -maxSpeed, maxSpeed);
  steering = map(steering, -128, 127, -maxSpeed, maxSpeed);

  if (Ps3.data.analog.button.r2 > 100) {
    throttle = maxSpeed;
  }

  if (Ps3.data.analog.button.l2 > 100) {
    throttle = -maxSpeed;
  }

  if (Ps3.data.button.r1) {
    rotateRight(maxSpeed);
    return;
  } else if (Ps3.data.button.l1) {
    rotateLeft(maxSpeed);
    return;
  }

  if (Ps3.data.button.triangle) {
    stopMotors();
    return;
  }

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
  // motor kiri
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

  // motor kanan
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


