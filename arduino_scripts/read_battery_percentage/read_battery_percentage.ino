const int PIN = A3;
const float R1 = 10000.0, R2 = 10000.0;
const float VREF = 5.0;

// 2S LiPo: 6.0V empty, 8.4V full
const float V_MIN = 6.0, V_MAX = 8.4;

void setup() { Serial.begin(9600); }

void loop() {
  int raw = analogRead(PIN);
  float vMid = (raw / 1023.0) * VREF;
  float vBat = vMid * (R1 + R2) / R2;
  int pct = constrain((vBat - V_MIN) / (V_MAX - V_MIN) * 100, 0, 100);

  Serial.print("V:"); Serial.print(vBat);
  Serial.print(" PCT:"); Serial.println(pct);
  delay(2000);
}