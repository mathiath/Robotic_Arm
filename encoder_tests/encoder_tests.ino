/* 
    Inspired by AR4 Robot Control Software
    Copyright (c) 2024, Chris Annin
    All rights reserved.

*/

#include <Arduino.h>
#include <math.h>
#include <Encoder.h>

const int n_J = 6; // Number of joints

// pins
const int stepPin[n_J] = {0, 2, 4, 6, 8, 10};
const int dirPin[n_J] = {1, 3, 5, 7, 9, 11};

const int encAPin[n_J] = {14, 16, 18, 20, 22, 24};
const int encBPin[n_J] = {15, 17, 19, 21, 23, 25};

const int enPin = 12;
const int estopPin = 26;


// motor + encoder + driver spec
const int SPR = 1600;
const int CPR[n_J] = {4000, 4000, 4000, 1200, 1200, 4000};
const int gearRatio[n_J] = {10, 50, 20, 20, 40, 20};

Encoder enc[n_J] = {
    Encoder(encAPin[0], encBPin[0]),
    Encoder(encAPin[1], encBPin[1]),
    Encoder(encAPin[2], encBPin[2]),
    Encoder(encAPin[3], encBPin[3]),
    Encoder(encAPin[4], encBPin[4]),
    Encoder(encAPin[5], encBPin[5]),
};

const int invDir[n_J] = {0, 0, 0, 0, 0, 0};

float limPos[n_J] = {180.0f, 90.0f, 90.0f, 180.0f, 90.0f, 90.0f};
float limNeg[n_J] = {-180.0f, -90.0f, -90.0f, -180.0f, -90.0f, -90.0f};

const float maxSpeed[n_J] = {180.0f, 90.0f, 180.0f, 270.0f, 180.0f, 270.0f}; // deg/s

float encMult[n_J];
float stepDeg[n_J];

long encRaw[n_J];
float encDeg[n_J];

long stepCount[n_J];
float jointDeg[n_J];

bool motorsEnabled = false;

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
    }
}

void initParams() {
    for (int i=0; i<n_J; i++) {
        encMult[i] = (float)(CPR[i] * gearRatio[i]) / 360.0f;
        stepDeg[i] = (float)(SPR * gearRatio[i]) / 360.0f;

        encRaw[i] = 0;
        encDeg[i] = 0.0f;

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
    return lroundf(deg * stepDeg[joint]);
}

float stepsToDeg(int joint, long steps) {
    if (joint < 0 || joint >= n_J) return 0.0f;
    return (float)steps / stepDeg[joint];
}

void updateEncoders() {
    static unsigned long lastUpdate = 0;
    unsigned long now = micros();
    
    // 1000 us = 1ms
    if (now - lastUpdate >= 1000) {
        lastUpdate = now;

        for (int j=0; j<n_J; j++) {
            encRaw[j] = enc[j].read();
            encDeg[j] = (float)encRaw[j] / encMult[j];
        }
    } 
}

void printEncoderDebug() {
    static unsigned long lastPrint = 0;
    unsigned long now = millis();

    if (now - lastPrint < 200) return;
    lastPrint = now;


    for (int j = 0; j < n_J; j++) {
        Serial.print("J");
        Serial.print(j+1);
        Serial.print(": counts=");
        Serial.print(encRaw[j]);
        Serial.print("  deg=");
        Serial.print(encDeg[j], 3);
        Serial.println();
    }
    Serial.println();
}

void home() {
    if (estopActive()) return;

    for (int j=0; j<n_J; j++) {
        enc[j].write(0);
        encRaw[j] = 0;
        encDeg[j] = 0.0f;
        stepCount[j] = 0;
        jointDeg[j] = 0.0f;
    }
}

void setup() {
    Serial.begin(115200);
    initPins();
    initParams();
    home();
    Serial.println("Setup done");

    
}

void loop() {
    updateEncoders();
    printEncoderDebug();


}