
#define LIN_ACT_1 5
#define LIN_ACT_2 4
#define LIN_ACT_PWM 3
#include <Servo.h>

#define SERVO_PIN 6          // PWM pin for Parallax CR servo

Servo driveServo;

void setServoRPM(int rpm) {
  // Parallax CR-servo: ≈50 RPM @ 2 ms, ‑50 RPM @ 1 ms, stop @ 1.5 ms
  rpm = constrain(rpm, 50, -50);
  int pulse = 1500 + rpm * 10;   // 10 µs per RPM (500 µs / 50 RPM)
  driveServo.writeMicroseconds(pulse);
}

void gprPower() {
  analogWrite(LIN_ACT_PWM, 150);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(150);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(3500);
  analogWrite(LIN_ACT_PWM, 150);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(200);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, LOW);
}

void linestart() {
  analogWrite(LIN_ACT_PWM, 150);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(200);
  analogWrite(LIN_ACT_PWM, 150);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(200);
  analogWrite(LIN_ACT_PWM, 150);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(200);
  analogWrite(LIN_ACT_PWM, 150);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(200);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, LOW);
}

void linestop() {
  analogWrite(LIN_ACT_PWM, 150);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(200);
  analogWrite(LIN_ACT_PWM, 150);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(200);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, LOW);
}

void setup() {
  Serial.begin(9600);
  pinMode(LIN_ACT_1, OUTPUT);
  pinMode(LIN_ACT_2, OUTPUT); 
  pinMode(LIN_ACT_PWM, OUTPUT); 

  // Attach continuous-rotation servo
  driveServo.attach(SERVO_PIN);
  setServoRPM(0);   // ensure stopped

  // Power on the GPR at startup
  gprPower();
}

void loop() {
  if (Serial.available() > 0) {
    char inputChar = Serial.read();

    if (inputChar == 'L') {
      linestart();
      // servo speed will be updated via subsequent S commands

    } else if (inputChar == 'K') {
      linestop();
      setServoRPM(0);   // stop servo when scanning stops

    } else if (inputChar == 'S') {
      // Expect ASCII signed integer RPM following 'S', terminated by \n
      int rpm = Serial.parseInt();
      setServoRPM(rpm);
    }
  }
}
