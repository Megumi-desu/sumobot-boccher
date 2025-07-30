// Motor kiri
#define MOTOR_KIRI_CW  32
#define MOTOR_KIRI_CCW 33

// Motor kanan
#define MOTOR_KANAN_CW  18
#define MOTOR_KANAN_CCW 19

// LED status
#define LED_BUILTIN 2
unsigned long lastBlinkTime = 0;
bool ledState = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Setup motor pins sebagai output
  pinMode(MOTOR_KIRI_CW, OUTPUT);
  pinMode(MOTOR_KIRI_CCW, OUTPUT);
  pinMode(MOTOR_KANAN_CW, OUTPUT);
  pinMode(MOTOR_KANAN_CCW, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(MOTOR_KIRI_CW, HIGH);
  delay(3000);
  digitalWrite(MOTOR_KIRI_CW, LOW);
  delay(1000);
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
}
