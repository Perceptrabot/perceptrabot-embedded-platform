#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

//timing variables
unsigned long now; 
unsigned long lastIMUPrint = 0;
unsigned long lastBatteryPrint = 0;

// ===== L298N PIN MAP (UPDATED WIRING) =====

// RIGHT motor (Channel A)
const int ENA = 10;
const int IN1 = 4;
const int IN2 = 5;

// LEFT motor (Channel B)
const int ENB = 9;
const int IN3 = 6;
const int IN4 = 7;

// ===== Encoder pins (updated wiring) =====
const int ENC_R_A = 2;
const int ENC_R_B = 11;
const int ENC_L_A = 3;
const int ENC_L_B = 12;

// ===== PID =====

float integralLeft  = 0.0;
float integralRight = 0.0;
float prevErrorLeft  = 0.0;
float prevErrorRight = 0.0;


// Tuning — start with just Kp, set Ki/Kd to 0
const float Kp = 8.0;
const float Ki = 1.0;
const float Kd = 1.0;


const int PWM_MIN = 50;   // below this motor doesn't move
const int PWM_MAX = 255;



// ===== Motor state =====
float cmdLeft  = 0;  // -255..255
float cmdRight = 0;  // -255..255

// ===== Encoder counters (interrupt-based) =====
volatile long encRight = 0;
volatile long encLeft = 0;

// speed in m/s = (encoder counts per second) * (wheel circumference) / (counts per revolution)
const float WHEEL_CIRCUMFERENCE   = 0.0628; // meters
const int   COUNTS_PER_REVOLUTION = 12; // encoder counts per wheel revolution
const float METERS_PER_TICK_LEFT  = 1.0 / 6022.0;   // 0.00016606...
const float METERS_PER_TICK_RIGHT = 1.0 / 5871.0;  // 0.00017035...
const float MAX_SPEED = 0.217;


float speedRight = 0.0;
float speedLeft = 0.0;
unsigned long dtSpeed = 0;
unsigned long lastSpeedRead = 0;
float encRightPrev = 0;
float encLeftPrev = 0;


// Battery Percentage Circuit
const int PIN = A3;
const float R1 = 10000.0, R2 = 10000.0;
const float VREF = 5.0;


// 2S LiPo: 6.0V empty, 8.4V full
const float V_MIN = 6.0, V_MAX = 8.4;


// ===== Serial parsing buffer =====
static const uint8_t BUF_LEN = 64;
char buf[BUF_LEN];
uint8_t buf_i = 0;

void setup() {

  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  // Count encoder A-channel transitions using hardware interrupts.
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, CHANGE);

  Wire.begin();


  if (!gyro.init()) {
    Serial.println("Failed to detect L3GD20 gyro");
    while (1) {}
  }
  gyro.enableDefault();

  if (!compass.init()) {
    Serial.println("Failed to detect LSM303 accel/mag");
    while (1) {}
  }
  compass.enableDefault();

  Serial.println("IMU initialized.");

  stopAll();

  Serial.println("READY");
  Serial.println("Commands: L <spd>, R <spd>, LR <L> <R>, S, E?, Z");
}

void loop() {
  // 1) Read serial lines
  readSerialLines();

  // 2) Apply latest motor commands continuously
  unsigned long lastControlLoop = 0;

  now = millis();
  if (now - lastControlLoop >= 20) {  // 50 Hz
    lastControlLoop = now;
    correct_speed_read_encoders(cmdLeft, cmdRight);  // see point 2
  }
  // setLeft(cmdLeft);
  // setRight(cmdRight);

  // 3) Read sensors
  gyro.read();
  compass.read();

  // readSpeed();

  // Print at 50 Hz (every 20 ms)
  now = millis();
  if (now - lastIMUPrint >= 500) {
    lastIMUPrint = now;
    readIMU();
  }
  
  now = millis();
  if (now - lastBatteryPrint >= 5000) {
    lastBatteryPrint = now;
    readBattery();
  }
  
}

int speedToPWM(float mps) {
  if (mps == 0.0) return 0;
  float pwm = PWM_MIN + (abs(mps) / MAX_SPEED) * (120 - PWM_MIN);
  pwm = constrain(pwm, PWM_MIN, 120);
  return (mps > 0) ? (int)pwm : -(int)pwm;
}

void correct_speed_read_encoders(float cmdLeft, float cmdRight) {
  
  readSpeed();
  float dt = dtSpeed / 1000.0;
  if (dt <= 0) return;

  // --- errors ---
  float errorLeft  = cmdLeft  - speedLeft;
  float errorRight = cmdRight - speedRight;

  // --- integrals ---
  integralLeft  += errorLeft  * dt;
  integralRight += errorRight * dt;
  integralLeft   = constrain(integralLeft,  -1.0, 1.0);
  integralRight  = constrain(integralRight, -1.0, 1.0);

  // --- derivatives ---
  float derivLeft  = (errorLeft  - prevErrorLeft)  / dt;
  float derivRight = (errorRight - prevErrorRight) / dt;

  // --- save for next iteration ---
  prevErrorLeft  = errorLeft;
  prevErrorRight = errorRight;

  // --- feedforward + PID correction ---
  int ffLeft  = speedToPWM(cmdLeft);
  int ffRight = speedToPWM(cmdRight);

  int pwmLeft  = ffLeft  + (int)(Kp * errorLeft  + Ki * integralLeft  + Kd * derivLeft);
  int pwmRight = ffRight + (int)(Kp * errorRight + Ki * integralRight + Kd * derivRight);

  // --- apply ---
  // if (cmdLeft == 0.0)  { applyPWMLeft(0);  integralLeft  = 0; prevErrorLeft  = 0; }
  // else                 { applyPWMLeft(constrain(pwmLeft,   -180, 180)); }

  // if (cmdRight == 0.0) { applyPWMRight(0); integralRight = 0; prevErrorRight = 0; }
  // else                 { applyPWMRight(constrain(pwmRight, -180, 180)); }
  const float STOP_THRESHOLD = 0.12;  // m/s — below this, consider "stopped"
  const int   BRAKE_PWM      = 80;    // tune this — strength of brake pulse



  if (cmdLeft == 0.0) {
    integralLeft = 0; prevErrorLeft = 0;
    if (abs(speedLeft) > STOP_THRESHOLD) {
      applyPWMLeft(speedLeft > 0 ? -BRAKE_PWM : BRAKE_PWM);
    } else {
      applyPWMLeft(0);
    }
  } else {
    applyPWMLeft(constrain(pwmLeft, -180, 180));
  }

  if (cmdRight == 0.0) {
    integralRight = 0; prevErrorRight = 0;
    if (abs(speedRight) > STOP_THRESHOLD) {
      applyPWMRight(speedRight > 0 ? -BRAKE_PWM : BRAKE_PWM);
    } else {
      applyPWMRight(0);
    }
  } else {
    applyPWMRight(constrain(pwmRight, -180, 180));
  }


}



void applyPWMRight(int pwm) {  // pwm: -255..255
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0)       { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else if (pwm < 0)  { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else               { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
  analogWrite(ENA, abs(pwm));
}

void applyPWMLeft(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0)       { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else if (pwm < 0)  { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else               { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
  analogWrite(ENB, abs(pwm));
}


void readSpeed() {
  dtSpeed = millis() - lastSpeedRead;
  lastSpeedRead = millis();

  noInterrupts();
  long encRightDelta = encRight - encRightPrev;
  long encLeftDelta  = encLeft  - encLeftPrev;
  encRightPrev = encRight;   // move these INSIDE
  encLeftPrev  = encLeft;
  interrupts();

  speedRight = encRightDelta * METERS_PER_TICK_RIGHT / dtSpeed * 1000.0;
  speedLeft  = encLeftDelta  * METERS_PER_TICK_LEFT  / dtSpeed * 1000.0;
}


void readIMU() {
  Serial.print("G,");
  Serial.print(gyro.g.x); Serial.print(",");
  Serial.print(gyro.g.y); Serial.print(",");
  Serial.println(gyro.g.z);

  Serial.print("A,");
  Serial.print(compass.a.x); Serial.print(",");
  Serial.print(compass.a.y); Serial.print(",");
  Serial.println(compass.a.z);

  Serial.print("M,");
  Serial.print(compass.m.x); Serial.print(",");
  Serial.print(compass.m.y); Serial.print(",");
  Serial.println(compass.m.z);

  // Serial.println();
}

void readBattery() {
  int raw = analogRead(PIN);
  float vMid = (raw / 1023.0) * VREF;
  float vBat = vMid * (R1 + R2) / R2;
  int pct = constrain((vBat - V_MIN) / (V_MAX - V_MIN) * 100, 0, 100);

  Serial.print("V,"); Serial.println(vBat);
  Serial.print(" PCT,"); Serial.println(pct);
}


// ------------------- Motor control (unchanged logic) -------------------
// rightForward: IN1 LOW, IN2 HIGH
// rightBackward: IN1 HIGH, IN2 LOW
// leftForward:  IN3 LOW, IN4 HIGH
// leftBackward: IN3 HIGH, IN4 LOW

// void setRight(float spd) { // -255..255
//   spd = constrain(spd, -MAX_SPEED, MAX_SPEED);
//   int pwm = abs(spd);

//   if (spd > 0) {        // forward
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//   } else if (spd < 0) { // backward
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//   } else {
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, LOW);
//   }
//   analogWrite(ENA, pwm);
// }

// void setLeft(float spd) { // -255..255
//   spd = constrain(spd, -MAX_SPEED, MAX_SPEED);
//   int pwm = abs(spd);

//   if (spd > 0) {        // forward
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//   } else if (spd < 0) { // backward
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//   } else {
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, LOW);
//   }
//   analogWrite(ENB, pwm);
// }

void stopAll() {
  cmdLeft = 0;
  cmdRight = 0;
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ------------------- Serial line reader -------------------

void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      buf[buf_i] = '\0';
      handleCommand(buf);
      buf_i = 0;
    } else {
      if (buf_i < BUF_LEN - 1) buf[buf_i++] = c;
      else buf_i = 0;
    }
  }
}

void handleCommand(const char* line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;

  if (strcmp(line, "S") == 0) {
    stopAll();
    Serial.println("OK S");
    return;
  }

  if (strcmp(line, "E?") == 0) {
    long left, right;
    noInterrupts();
    left = encLeft;
    right = encRight;
    interrupts();

    Serial.print("E,");
    Serial.print(left);
    Serial.print(",");
    Serial.println(right);
    return;
  }

  if (strcmp(line, "Z") == 0) {
    noInterrupts();
    encLeft = 0;
    encRight = 0;
    interrupts();
    Serial.println("OK Z");
    return;
  }

  // Parse: L <int>
  if (line[0] == 'L' && line[1] == ' ') {
    float v = atof(line + 2);
    cmdLeft = constrain(v, -MAX_SPEED, MAX_SPEED);

    // Stay silent for zero commands to reduce serial traffic.
    if (cmdLeft != 0) {
      Serial.print("OK L ");
      Serial.println(cmdLeft);
    }
    return;
  }

  // Parse: R <int>
  if (line[0] == 'R' && line[1] == ' ') {
    float v = atof(line + 2);
    cmdRight = constrain(v, -MAX_SPEED, MAX_SPEED);

    // Stay silent for zero commands to reduce serial traffic.
    if (cmdRight != 0) {
      Serial.print("OK R ");
      Serial.println(cmdRight);
    }
    return;
  }

  // Parse: LR <int> <int>
  if (line[0] == 'L' && line[1] == 'R' && line[2] == ' ') {
    const char* p = line + 3;
    cmdLeft  = constrain(atof(p), -MAX_SPEED, MAX_SPEED);
    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;
    cmdRight = constrain(atof(p), -MAX_SPEED, MAX_SPEED);

    // Stay silent if both commands are zero to reduce serial traffic.
    if (!(cmdLeft == 0 && cmdRight == 0)) {
      Serial.print("OK LR ");
      Serial.print(cmdLeft);
      Serial.print(" ");
      Serial.println(cmdRight);
    }
    return;
  }

  Serial.print("ERR ");
  Serial.println(line);
}

// ------------------- Encoder ISRs -------------------
// Count on A-channel changes, use B-channel to infer direction.
// If a wheel counts backward when moving forward, swap ++ and -- in that ISR.

void isrRight() {
  int a = digitalRead(ENC_R_A);
  int b = digitalRead(ENC_R_B);

  if (a == b) encRight++;
  else encRight--;
}

void isrLeft() {
  int a = digitalRead(ENC_L_A);
  int b = digitalRead(ENC_L_B);

  if (a == b) encLeft++;
  else encLeft--;
}
