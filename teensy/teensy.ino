/* 
    Inspired by AR4 Robot Control Software
    Copyright (c) 2024, Chris Annin
    All rights reserved.

*/

#include <Arduino.h>
#include <math.h>
#include <Encoder.h>

const int n_J = 6; // Number of joints

extern float Jlist[n_J];
extern int robot_speed;

// pins
const int stepPin[n_J] = {0, 2, 4, 6, 8, 10};
const int dirPin[n_J] = {1, 3, 5, 7, 9, 11};

const int encAPin[n_J] = {14, 16, 18, 20, 22, 24};
const int encBPin[n_J] = {15, 17, 19, 21, 23, 25};

const int enPin = 12;
const int estopPin = 26;
const int freeMovePin = 27;
const int homePin = 28;


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
bool freeMove = false;


bool estopActive() {
    return digitalRead(estopPin) == LOW;
}

void initPins() {
    pinMode(enPin, OUTPUT);
    pinMode(estopPin, INPUT_PULLUP);
    pinMode(freeMovePin, INPUT_PULLUP);
    pinMode(homePin, INPUT_PULLUP);

    digitalWrite(enPin, LOW); // motor disabled during init
    
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

void moveJointsToDeg(const float targetDeg[n_J], float speed = 30.0f) {
    if (!motorsEnabled) return;
    if (estopActive())  return;

    long remSteps[n_J];
    int direction[n_J];

    unsigned long stepIntervalUs[n_J];
    unsigned long lastStepUs[n_J];
    bool stepLevel[n_J];

    bool hasActiveJoint = false;
    unsigned long nowUs = micros();

    for (int j = 0; j < n_J; j++) {

        // Local variable copies that can be modified
        float deg = targetDeg[j];

        // Bruk samme speed for alle ledd
        float jointSpeed = speed;

        // Clamp position
        if (deg > limPos[j]) deg = limPos[j];
        if (deg < limNeg[j]) deg = limNeg[j];

        // Clamp speed
        if (jointSpeed > maxSpeed[j]) jointSpeed = maxSpeed[j];
        if (jointSpeed < 0.0f)        jointSpeed = 0.0f;

        // Step count to target angle
        long delta = degToSteps(j, deg) - stepCount[j];
        remSteps[j] = labs(delta);

        // No movement
        if (remSteps[j] == 0 || jointSpeed == 0.0f) {
            stepIntervalUs[j] = 0;
            continue;
        }

        hasActiveJoint = true;

        // Direction-logic
        direction[j] = (delta >= 0) ? 1 : -1;

        bool dirPinState = (direction[j] > 0);
        if (invDir[j]) dirPinState = !dirPinState;
        digitalWrite(dirPin[j], dirPinState ? HIGH : LOW);

        // Speed: deg/s to steps/s to interval in us
        float stepsPerSec = jointSpeed * stepDeg[j];
        if (stepsPerSec < 1.0f) stepsPerSec = 1.0f;

        stepIntervalUs[j] = (unsigned long)(1000000.0f / stepsPerSec);
        lastStepUs[j] = nowUs;

        stepLevel[j] = LOW;
        digitalWrite(stepPin[j], LOW);
    }

    if (!hasActiveJoint) return;

    while (true) {
        if (estopActive()) break;

        unsigned long now = micros();
        bool moving = false;

        for (int j = 0; j < n_J; j++) {
            if (remSteps[j] == 0 || stepIntervalUs[j] == 0) continue;

            if (now - lastStepUs[j] >= stepIntervalUs[j]) {
                lastStepUs[j] += stepIntervalUs[j];

                stepLevel[j] = !stepLevel[j];
                digitalWrite(stepPin[j], stepLevel[j]);

                if (stepLevel[j] == LOW) {
                    remSteps[j]--;
                    stepCount[j] += direction[j];
                    jointDeg[j] = stepsToDeg(j, stepCount[j]);
                }
            }

            if (remSteps[j] > 0) {
                moving = true;
            }
        }

        if (!moving) break;
    }
}

void home(float speed = 30.0f) {
    float targetDeg[n_J] = {0, 0, 0, 0, 0, 0};

    moveJointsToDeg(targetDeg, speed);
}

void setup() {
    Serial.begin(115200);
    initPins();
    initParams();
    home();
    enableMotors(true);

    
}

void loop() {
    updateEncoders();

    if (Serial.available() > 0) {
        handleSerialInput();
        moveJointsToDeg(Jlist, (float)robot_speed);

        Serial.println("[" + String(Jlist[0]) + " " + String(Jlist[1]) + " " + String(Jlist[2]) + " " + String(Jlist[3]) + " " + String(Jlist[4]) + " " + String(Jlist[5]) + "]");
        update_encoder_valus();

        Serial.println("end");
  }

}