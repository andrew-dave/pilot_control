// === Pin Definitions ===
#define BRAKE         4

#define PIN_DIR_LEFT  7
#define PIN_PWM_LEFT  3

#define PIN_DIR_RIGHT 8
#define PIN_PWM_RIGHT 11

// === Variables ===
int leftPWM = 0;
int rightPWM = 0;

void setup() {
  Serial.begin(9600);

  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, LOW);  // disengage brake

  pinMode(PIN_DIR_LEFT, OUTPUT);
  pinMode(PIN_PWM_LEFT, OUTPUT);

  pinMode(PIN_DIR_RIGHT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();

    int commaIndex = inputString.indexOf(',');
    if (commaIndex != -1) {
      String leftString = inputString.substring(0, commaIndex);
      String rightString = inputString.substring(commaIndex + 1);

      leftPWM = constrain(leftString.toInt(), -255, 255);
      rightPWM = constrain(rightString.toInt(), -255, 255);

      controlLeftMotor(leftPWM);
      controlRightMotor(rightPWM);
    }
  }
}

void controlLeftMotor(int pwm) {
  if (pwm < 0) {
    digitalWrite(PIN_DIR_LEFT, LOW);
    analogWrite(PIN_PWM_LEFT, -pwm);
  } else {
    digitalWrite(PIN_DIR_LEFT, HIGH);
    analogWrite(PIN_PWM_LEFT, pwm);
  }
}

void controlRightMotor(int pwm) {
  if (pwm < 0) {
    digitalWrite(PIN_DIR_RIGHT, HIGH);
    analogWrite(PIN_PWM_RIGHT, -pwm);
  } else {
    digitalWrite(PIN_DIR_RIGHT, LOW);
    analogWrite(PIN_PWM_RIGHT, pwm);
  }
}
