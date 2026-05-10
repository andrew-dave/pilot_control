
#define LIN_ACT_1 11
#define LIN_ACT_2 12
#define LIN_ACT_PWM 10

int ft = 900,ss= 250;

void gprPower() {
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(ft);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(3500);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(ft);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, LOW);
}

void linestart() {
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(ft);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(ss);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(ss);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(ft);
  analogWrite(LIN_ACT_PWM, 0);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, LOW);
}

void linestop() {
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, HIGH);
  digitalWrite(LIN_ACT_2, LOW);
  delay(ft);
  analogWrite(LIN_ACT_PWM, 255);
  digitalWrite(LIN_ACT_1, LOW);
  digitalWrite(LIN_ACT_2, HIGH);
  delay(ft);
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
//  driveServo.attach(SERVO_PIN);
//  setServoRPM(0);   // ensure stopped

  // Power on the GPR at startup
  // gprPower();
  // linestart();
  linestop();
}

void loop() {
  if (Serial.available() > 0) {
    char inputChar = Serial.read();
    
    if (inputChar == 'O') {
      gprPower();
      // power off gpr
    }      

    if (inputChar == 'L') {
      linestop();
      // servo speed will be updated via subsequent S commands

    } else if (inputChar == 'K') {
      linestop();
//      setServoRPM(0);   // stop servo when scanning stops

//    } else if (inputChar == 'S') {
//      // Expect ASCII signed integer RPM following 'S', terminated by \n
//      int rpm = Serial.parseInt();
//      setServoRPM(rpm);
    }
  }
}
