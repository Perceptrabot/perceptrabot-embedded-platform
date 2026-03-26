// ===== L298N PIN MAP =====

// RIGHT motor (Channel A)
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5;

// LEFT motor (Channel B)
const int ENB = 9;
const int IN3 = 6;
const int IN4 = 7;

// ===== Encoder pins (your setup) =====
const int ENC_R_A = 10;
const int ENC_R_B = 11;
const int ENC_L_A = 12;
const int ENC_L_B = 13;

// ===== Motor state =====
int cmdLeft = 0;   // -255..255
int cmdRight = 0;  // -255..255

// ===== Encoder counters (simple polling; OK for now) =====
volatile long encRight = 0;
volatile long encLeft = 0;
int lastRA = HIGH, lastRB = HIGH, lastLA = HIGH, lastLB = HIGH;

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

  // initialize last states for polling
  lastRA = digitalRead(ENC_R_A);
  lastRB = digitalRead(ENC_R_B);
  lastLA = digitalRead(ENC_L_A);
  lastLB = digitalRead(ENC_L_B);

  Serial.println("READY");
  Serial.println("Commands: L <spd>, R <spd>, LR <L> <R>, S, E?");
}

void loop() {
  // 1) Read serial lines
  readSerialLines();

  // 2) Apply latest motor commands continuously
  setLeft(cmdLeft);
  setRight(cmdRight);

  // 3) Update encoder counts (simple polling)
  pollEncoders();
}

// ------------------- Motor control (matches your working logic) -------------------
// Your working logic:
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

    if (c == '\r') continue; // ignore CR

    if (c == '\n') {
      buf[buf_i] = '\0';
      handleCommand(buf);
      buf_i = 0;
    } else {
      if (buf_i < BUF_LEN - 1) buf[buf_i++] = c;
      else { // overflow: reset line
        buf_i = 0;
      }
    }
  }
}

void handleCommand(const char* line) {
  // skip empty
  while (*line == ' ') line++;
  if (*line == '\0') return;

  // Single-letter commands
  if (strcmp(line, "S") == 0) {
    stopAll();
    Serial.println("OK S");
    return;
  }
  if (strcmp(line, "E?") == 0) {
    Serial.print("E ");
    Serial.print(encLeft);
    Serial.print(" ");
    Serial.println(encRight);
    return;
  }

  // Parse: L <int>
  if (line[0] == 'L' && line[1] == ' ') {
    int v = atoi(line + 2);
    cmdLeft = constrain(v, -255, 255);
    Serial.print("OK L ");
    Serial.println(cmdLeft);
    return;
  }

  // Parse: R <int>
  if (line[0] == 'R' && line[1] == ' ') {
    int v = atoi(line + 2);
    cmdRight = constrain(v, -255, 255);
    Serial.print("OK R ");
    Serial.println(cmdRight);
    return;
  }

  // Parse: LR <int> <int>
  if (line[0] == 'L' && line[1] == 'R' && line[2] == ' ') {
    // crude parse
    // line = "LR <L> <R>"
    const char* p = line + 3;
    int l = atoi(p);
    // move p to next space
    while (*p && *p != ' ') p++;
    while (*p == ' ') p++;
    int r = atoi(p);

    cmdLeft = constrain(l, -255, 255);
    cmdRight = constrain(r, -255, 255);

    Serial.print("OK LR ");
    Serial.print(cmdLeft);
    Serial.print(" ");
    Serial.println(cmdRight);
    return;
  }

  Serial.print("ERR ");
  Serial.println(line);
}

// ------------------- Encoder polling (simple) -------------------
// NOTE: This is not as accurate as interrupts, but matches your current pin usage.
// For best results later: move A-channels to D2/D3 interrupts.

void pollEncoders() {
  int ra = digitalRead(ENC_R_A);
  int rb = digitalRead(ENC_R_B);
  int la = digitalRead(ENC_L_A);
  int lb = digitalRead(ENC_L_B);

  // Right: count on A transitions, direction from B
  if (ra != lastRA) {
    if (ra == rb) encRight++; else encRight--;
    lastRA = ra;
  }
  // Left
  if (la != lastLA) {
    if (la == lb) encLeft++; else encLeft--;
    lastLA = la;
  }

  lastRB = rb;
  lastLB = lb;
}
