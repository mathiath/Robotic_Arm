/* 
    Inspired by AR4 Robot Control Software
    Copyright (c) 2024, Chris Annin
    All rights reserved.

*/

#include <Arduino.h>
#include <math.h>

const int n_J = 6; // Number of joints

// pins
const int stepPin[n_J] = {0, 2, 4, 6, 8, 10};
const int dirPin[n_J] = {1, 3, 5, 7, 9, 11};
const int enPin = 12;

const int encAPin[n_J] = {14, 16, 18, 20, 22, 24};
const int encBPin[n_J] = {15, 17, 19, 21, 23, 25};

const int estopPin = 26;


// motor + encoder spec
const int CPR[n_J] = {4000, 4000, 4000, 1200, 1200, 4000};
const int gearRatio[n_J] = {10, 50, 20, 20, 40, 20};
const int SPR = 1600;

float encMult[n_J];
float stepDeg[n_J];

const int invDir[n_J] = {0, 0, 0, 0, 0, 0};

float jointLimPos[n_J] = {180.0f, 90.0f, 90.0f, 180.0f, 90.0f, 90.0f};
float jointLimNeg[n_J] = {-180.0f, -90.0f, -90.0f, -180.0f, -90.0f, -90.0f};

const float maxDegPerSec[n_J] = {180.0f, 90.0f, 180.0f, 270.0f, 180.0f, 270.0f};

bool motorsEnabled = false;

long stepCount[n_J];
float jointDeg[n_J];

bool estopActive() {
    return digitalRead(estopPin) == LOW;
}

void initPins() {
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, LOW); // motor disabled during init
    
    pinMode(estopPin, INPUT_PULLUP);

    for (int i = 0; i < n_J; i++) {
        pinMode(stepPin[i], OUTPUT);
        pinMode(dirPin[i], OUTPUT);
        digitalWrite(stepPin[i], LOW);
        digitalWrite(dirPin[i], LOW);
        pinMode(encAPin[i], INPUT);
        pinMode(encBPin[i], INPUT);
    }


}

void initParams() {
    for (int i=0; i<n_J; i++) {
        encMult[i] = (float)(CPR[i] * gearRatio[i]) / 360.0f;
        stepDeg[i] = (float)(SPR * gearRatio[i]) / 360.0f;

        stepCount[i] = 0;
        jointDeg[i] = 0.0f;
    }
}

void enableMotors(bool on) {
    motorsEnabled = on;
    digitalWrite(enPin, on ? HIGH : LOW);
}

long degToSteps(int joint, float deg) {
    if (joint < 0 || joint >= n_J) return 0;
    if (deg > jointLimPos[joint]) deg = jointLimPos[joint];
    if (deg < jointLimNeg[joint]) deg = jointLimNeg[joint];

    return lroundf(deg * stepDeg[joint]);
}

float stepsToDeg(int joint, long steps) {
    if (joint < 0 || joint >= n_J) return 0.0f;
    return (float)steps / stepDeg[joint];
}

void setup() {
    Serial.begin(115200);
    initPins();
    initParams();
    
}

void loop() {


}