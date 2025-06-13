// === Pin Definitions ===
#define BRAKE         4

#define PIN_DIR_LEFT  7
#define PIN_PWM_LEFT  3

#define PIN_DIR_RIGHT 8
#define PIN_PWM_RIGHT 11

void setup() {
  // Set up pins
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, LOW);  // Disable brake

  pinMode(PIN_DIR_LEFT, OUTPUT);
  pinMode(PIN_PWM_LEFT, OUTPUT);

  pinMode(PIN_DIR_RIGHT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);

  // Begin Serial just in case
  Serial.begin(9600);

  // Forward
  Serial.println("Motors forward");
  digitalWrite(PIN_DIR_LEFT, HIGH);
  analogWrite(PIN_PWM_LEFT, 150);

  digitalWrite(PIN_DIR_RIGHT, HIGH);
  analogWrite(PIN_PWM_RIGHT, 150);
  delay(2000);

  // Reverse
  Serial.println("Motors reverse");
  digitalWrite(PIN_DIR_LEFT, LOW);
  analogWrite(PIN_PWM_LEFT, 150);

  digitalWrite(PIN_DIR_RIGHT, LOW);
  analogWrite(PIN_PWM_RIGHT, 150);
  delay(2000);

  // Stop
  Serial.println("Motors stop");
  analogWrite(PIN_PWM_LEFT, 0);
  analogWrite(PIN_PWM_RIGHT, 0);
}

void loop() {
  // do nothing
}
