
#define LIN_ACT_1 5
#define LIN_ACT_2 4
#define LIN_ACT_PWM 3

void gprPower() {
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(3500);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(200);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, LOW);
}

void linestart() {
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(200);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(200);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(200);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(200);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, LOW);
}

void linestop() {
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(200);
  analogWrite(LIN_ACT_PWM, 255);
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

  // Power on the GPR at startup
  gprPower();
}

void loop() {
  if (Serial.available() > 0) {
    char inputChar = Serial.read();
    if (inputChar == 'L') {
      linestart();
    } else if (inputChar == 'K') {
      linestop();
    }
    // Ignore other characters or add more commands as needed
  }
}
