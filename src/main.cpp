#include <Arduino.h>
#include <Encoder.h>

const int enPin = 22;      // Global Enable-pin.
const bool enLow = true;   // true = aktiv når lav

bool motorsEnabled = false;

const uint32_t motionInterval_US = 500; // 0,5 ms = 2 kHz (ikke brukt ennå)

struct JointConfig {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t encA;
  uint8_t encB;
  bool dirInvert;
  float stepsPerDeg;
  float limPosDeg;
  float limNegDeg;
  float encMult; 
};

JointConfig jointCfg[6] = {
  // stepPin, dirPin, encA, encB, invert, stepsPerDeg, limPos, limNeg, encMult

  // Joint 1 — NEMA17, 1000 PPR (4000 CPR), 10:1
  {0, 0, 0, 0, false, 44.44f, 170.0f, -170.0f, 111.11f},

  // Joint 2 — NEMA23, 1000 PPR (4000 CPR), 50:1
  {0, 0, 0, 0, true,  222.22f,  90.0f,  -42.0f, 555.56f},

  // Joint 3 — NEMA17, 1000 PPR (4000 CPR), 20:1
  {0, 0, 0, 0, false, 88.89f,  52.0f,  -89.0f, 222.22f},

  // Joint 4 — NEMA14, 300 PPR (1200 CPR), 20:1
  {0, 0, 0, 0, false, 88.89f, 180.0f, -180.0f, 66.67f},

  // Joint 5 — NEMA14, 300 PPR (1200 CPR), 20:1
  {0, 0, 0, 0, false, 88.89f, 105.0f, -105.0f, 66.67f},

  // Joint 6 — NEMA14, 1000 PPR (4000 CPR), 20:1
  {0, 0, 0, 0, false, 88.89f, 180.0f, -180.0f, 222.22f}
};

// Encoder-objekter (bruker encA/encB fra jointCfg)
Encoder enc[6] = {
  Encoder(jointCfg[0].encA, jointCfg[0].encB),
  Encoder(jointCfg[1].encA, jointCfg[1].encB),
  Encoder(jointCfg[2].encA, jointCfg[2].encB),
  Encoder(jointCfg[3].encA, jointCfg[3].encB),
  Encoder(jointCfg[4].encA, jointCfg[4].encB),
  Encoder(jointCfg[5].encA, jointCfg[5].encB)
};

void enableMotors() {
  motorsEnabled = true;
  if (enPin >= 0) {
    digitalWrite(enPin, enLow ? LOW : HIGH);
  }
}

void disableMotors() {
  motorsEnabled = false;
  if (enPin >= 0) {
    digitalWrite(enPin, enLow ? HIGH : LOW);
  }
}

// encoder counts -> grader for joint j
float encCountsToDeg(int j, long counts) {
  return counts / jointCfg[j].encMult;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Enable-pin
  if (enPin >= 0) {
    pinMode(enPin, OUTPUT);
    disableMotors();
  }

  // Encoder-pins – sett pullup
  for (int j = 0; j < 6; j++) {
    pinMode(jointCfg[j].encA, INPUT_PULLUP);
    pinMode(jointCfg[j].encB, INPUT_PULLUP);
  }

  Serial.println("Encoder test klar. Roter ledd og se counts/deg.");
}

void loop() {
  Serial.print("ENC: ");

  for (int j = 0; j < 6; j++) {
    long c = enc[j].read();
    float deg = encCountsToDeg(j, c);

    Serial.print("J");
    Serial.print(j + 1);
    Serial.print("=");
    Serial.print(c);
    Serial.print(" (");
    Serial.print(deg, 2);
    Serial.print(" deg)");

    if (j < 5) Serial.print(" | ");
  }

  Serial.println();
  delay(200);
}
