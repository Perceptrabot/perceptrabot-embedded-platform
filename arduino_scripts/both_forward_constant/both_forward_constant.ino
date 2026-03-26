// ===== PIN MAP (MATCHES YOUR WIRING) =====

#include <Encoder.h>

Encoder enc1(10, 11);
Encoder enc2(12, 13);

// RIGHT motor (Channel A)
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5;

// LEFT motor (Channel B)
const int ENB = 9;
const int IN3 = 6;
const int IN4 = 7;

const int TEST_PWM = 55;  // safe for 6V motors @ ~7.7V supply
const int DELAY = 1000;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
  Serial.println("=== L298N MOTOR WIRING TEST ===");

  stopAll();
}

void loop() {
  // -------------------------------

  rightForward(50);
  leftForward(50);

  // stopAll();
  // delay(3000);
  
}

// ===== MOTOR HELPERS =====
// working
void rightForward(int pwm) {
  Serial.println("right motor forward");  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, pwm);
}

void leftForward(int pwm) {
  Serial.println("left motor forward");  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, pwm);
}

void rightBackward(int pwm) {
  Serial.println("right motor backward");  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwm);
}

void leftBackward(int pwm) {
  Serial.println("left motor backward");  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwm);
}

void stopAll() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
