#include <Arduino.h>
#include <Encoder.h>

/*
  Encoder test for all 6 joints.
  - Driver enable set HIGH (drivers disabled)
  - Read A/B on all joints
  - Read Z where available
  - Print counts and degrees on change
*/

// ---------- PIN-OPPSETT ----------

// Stegdriver enable-pin (juster denne til din faktiske)
const int enPin = 12;   // settes HIGH = disable (for denne testen)

// Encoder-pinner
const int encAPin[6] = {14, 16, 18, 20, 22, 24};
const int encBPin[6] = {15, 17, 19, 21, 23, 25};


// Encoder / gir-spesifikasjon
const int CPR[6]       = {4000, 4000, 4000, 1200, 1200, 4000};
const int gearRatio[6] = {10,   50,   20,   20,   40,   20};
float encMult[6];   // counts per degree

// Encoder-objekter (A,B)
Encoder enc[6] = {
  Encoder(encAPin[0], encBPin[0]),
  Encoder(encAPin[1], encBPin[1]),
  Encoder(encAPin[2], encBPin[2]),
  Encoder(encAPin[3], encBPin[3]),
  Encoder(encAPin[4], encBPin[4]),
  Encoder(encAPin[5], encBPin[5])
};

// For å spore endringer
long lastCount[6] = {0, 0, 0, 0, 0, 0};
int  lastZState[6] = {0, 0, 0, 0, 0, 0};

unsigned long lastPrintTime = 0;
const unsigned long printIntervalMs = 100; // maks print hvert 100 ms

void initEncoders() {
  // Sett opp pinnemodus for A/B/Z
  for (int i = 0; i < 6; i++) {
    if (encAPin[i] >= 0) {
      pinMode(encAPin[i], INPUT); // bruk INPUT; ikke PULLUP hvis du har push-pull encodere
    }
    if (encBPin[i] >= 0) {
      pinMode(encBPin[i], INPUT);
    }


    // beregn counts per degree
    encMult[i] = (float)(CPR[i] * gearRatio[i]) / 360.0f;

    // nullstill teller
    enc[i].write(0);
    lastCount[i] = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Disable drivere (EN = HIGH i testen)
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);

  initEncoders();

  Serial.println("=== ENCODER TEST STARTET ===");
  Serial.println("Driver enable-pin satt HIGH (disablet).");
  Serial.println("Vri på leddene for å se counts/deg på J1–J6.");
  Serial.println("---------------");
}

void printEncoderStatus() {
  Serial.println("Encoders:");

  for (int i = 0; i < 6; i++) {
    long c = enc[i].read();
    float deg = 0.0f;
    if (encMult[i] != 0.0f) {
      deg = (float)c / encMult[i];
    }

    Serial.print("J");
    Serial.print(i + 1);
    Serial.print(": counts=");
    Serial.print(c);
    Serial.print("  deg~=");
    Serial.print(deg, 2);


    Serial.println();
  }

  Serial.println("---------------");
}

void loop() {
  bool changed = false;

  // sjekk om noen counts har endret seg
  for (int i = 0; i < 6; i++) {
    long c = enc[i].read();
    if (c != lastCount[i]) {
      lastCount[i] = c;
      changed = true;
    }

  }

  unsigned long now = millis();
  if (changed && (now - lastPrintTime >= printIntervalMs)) {
    lastPrintTime = now;
    printEncoderStatus();
  }

  // liten pause
  delay(5);
}
