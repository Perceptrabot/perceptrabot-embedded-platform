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

// ===== Motor state =====
int cmdLeft = 0;   // -255..255
int cmdRight = 0;  // -255..255

// ===== Encoder counters (interrupt-based) =====
volatile long encRight = 0;
volatile long encLeft = 0;

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

  Serial.begin(115200);
  stopAll();

  // Count encoder A-channel transitions using hardware interrupts.
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, CHANGE);

  Serial.println("READY");
  Serial.println("Commands: L <spd>, R <spd>, LR <L> <R>, S, E?, Z");
}

void loop() {
  // 1) Read serial lines
  readSerialLines();

  // 2) Apply latest motor commands continuously
  setLeft(cmdLeft);
  setRight(cmdRight);

  readBattery();
}


void readBattery() {
  int raw = analogRead(PIN);
  float vMid = (raw / 1023.0) * VREF;
  float vBat = vMid * (R1 + R2) / R2;
  int pct = constrain((vBat - V_MIN) / (V_MAX - V_MIN) * 100, 0, 100);

  Serial.print("V:"); Serial.print(vBat);
  Serial.print(" PCT:"); Serial.println(pct);
}


// ------------------- Motor control (unchanged logic) -------------------
// rightForward: IN1 LOW, IN2 HIGH
// rightBackward: IN1 HIGH, IN2 LOW
// leftForward:  IN3 LOW, IN4 HIGH
// leftBackward: IN3 HIGH, IN4 LOW

void setRight(int spd) { // -255..255
  spd = constrain(spd, -255, 255);
  int pwm = abs(spd);

  if (spd > 0) {        // forward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (spd < 0) { // backward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwm);
}

void setLeft(int spd) { // -255..255
  spd = constrain(spd, -255, 255);
  int pwm = abs(spd);

  if (spd > 0) {        // forward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (spd < 0) { // backward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, pwm);
}

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
    int v = atoi(line + 2);
    cmdLeft = constrain(v, -255, 255);

    // Stay silent for zero commands to reduce serial traffic.
    if (cmdLeft != 0) {
      Serial.print("OK L ");
      Serial.println(cmdLeft);
    }
    return;
  }

  // Parse: R <int>
  if (line[0] == 'R' && line[1] == ' ') {
    int v = atoi(line + 2);
    cmdRight = constrain(v, -255, 255);

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
    int l = atoi(p);
    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;
    int r = atoi(p);

    cmdLeft = constrain(l, -255, 255);
    cmdRight = constrain(r, -255, 255);

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
