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
const int eStopPin = 26;
const int freeMovePin = 27;
const int homePin = 28;


// motor + encoder + driver specs
const int SPR = 1600;
const int CPR[n_J] = {4000, 4000, 4000, 1200, 1200, 4000};
const int gearRatio[n_J] = {10, 50, 20, 20, 40, 20};

const int invDir[n_J] = {0, 0, 0, 0, 0, 0};

Encoder enc[n_J] = {
    Encoder(encAPin[0], encBPin[0]),
    Encoder(encAPin[1], encBPin[1]),
    Encoder(encAPin[2], encBPin[2]),
    Encoder(encAPin[3], encBPin[3]),
    Encoder(encAPin[4], encBPin[4]),
    Encoder(encAPin[5], encBPin[5]),
};

// Joint specs
float limPos[n_J] = {180.0f, 90.0f, 90.0f, 180.0f, 90.0f, 90.0f}; // 0 degrees = straight up/out
float limNeg[n_J] = {-180.0f, -90.0f, -90.0f, -180.0f, -90.0f, -90.0f}; // 0 degrees = straight up/out
const float maxSpeed[n_J] = {180.0f, 90.0f, 180.0f, 270.0f, 180.0f, 270.0f}; // deg/s

// Vars
float encMult[n_J];
float stepDeg[n_J];

long encRaw[n_J];
float encDeg[n_J];

long stepCount[n_J];
float jointDeg[n_J];

// Func. flags
bool motorsEnabled = false;
bool eStopEnabled = false;
bool freeMoveEnabled = false;

void initPins() {
    pinMode(enPin, OUTPUT);
    pinMode(eStopPin, INPUT_PULLUP);
    pinMode(freeMovePin, INPUT_PULLUP);
    pinMode(homePin, INPUT_PULLUP);

    digitalWrite(enPin, LOW); // motor disabled during init
    
    // Initializes pins for all joints
    for (int j = 0; j < n_J; j++) {
        pinMode(stepPin[j], OUTPUT);
        pinMode(dirPin[j], OUTPUT);
        digitalWrite(stepPin[j], LOW);
        digitalWrite(dirPin[j], LOW);
    }
}

void initParams() {
    for (int j = 0; j < n_J; j++) {
        encMult[j] = (float)(CPR[j] * gearRatio[j]) / 360.0f;
        stepDeg[j] = (float)(SPR * gearRatio[j]) / 360.0f;
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

        for (int j = 0; j < n_J; j++) {
            encRaw[j] = enc[j].read();
            encDeg[j] = (float)encRaw[j] / encMult[j];
        }
    } 
}

void moveJointsToDeg(const float targetDeg[n_J], float speed = 30.0f) {
    if (!motorsEnabled || eStopEnabled || freeMoveEnabled) return; // Aborts if motors are disabled, or eStop or FreeMove is enabled

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

        // Clamp joint position
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
        handleEStop();
        handleFreeMove();

        // Aborts movement if eStop or freeMove is enabled
        if (eStopEnabled || freeMoveEnabled) {
            break;
        }
            
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

void setHome() {
    if (eStopEnabled) return;
    for (int j=0; j<n_J; j++) {
        enc[j].write(0);
        encRaw[j] = 0;
        encDeg[j] = 0.0f;
        stepCount[j] = 0;
        jointDeg[j] = 0.0f;
    }
}

void moveHome(float speed = 30.0f) {
    float targetDeg[n_J] = {0, 0, 0, 0, 0, 0};
    moveJointsToDeg(targetDeg, speed);
}

void handleEStop() {
    static bool lastRaw = HIGH;
    static bool lastStable = HIGH;
    static unsigned long lastDebounce = 0;
    const unsigned long debounceDelay = 50; // ms

    bool raw = digitalRead(eStopPin);
    unsigned long now = millis();

    if (raw != lastRaw) {
        lastDebounce = now;
        lastRaw = raw;
    }

    if ((now - lastDebounce) > debounceDelay) {
        if (raw != lastStable) {
            lastStable = raw;

            if (raw == LOW) {
                eStopEnabled = true;
                enableMotors(false);
                Serial.println("ESTOP ENABLED");
            }
        }     
    }
}

void handleFreeMove() {
    static bool lastRaw = HIGH;
    static bool lastStable = HIGH;
    static unsigned long lastDebounce = 0;
    const unsigned long debounceDelay = 50; // ms    

    bool raw = digitalRead(freeMovePin);
    unsigned long now = millis();

    if (raw != lastRaw) {
        lastDebounce = now;
        lastRaw = raw;
    }

    if ((now - lastDebounce) > debounceDelay) {
        if (raw != lastStable) {
            lastStable = raw;

            if (raw == LOW) {
                freeMoveEnabled = !freeMoveEnabled;
                enableMotors(!freeMoveEnabled);
                Serial.println(freeMoveEnabled ? "FREEMOVE ON" : "FREEMOVE OFF");
            }
        }
    }
}

void handleHome() { 
    static bool lastVal = HIGH;
    static unsigned long lastTime = 0;
    const unsigned long debounceDelay = 50; // ms

    bool raw = digitalRead(homePin);
    unsigned long now = millis();

    if(now - lastTime > debounceDelay) {
        if(raw == LOW && lastVal == HIGH) {
            if (!eStopEnabled && !freeMoveEnabled) {
                moveHome((float)robot_speed);
            }
        }
        lastVal = raw;
        lastTime = now;
    }
}

void setup() {
    Serial.begin(115200);
    initPins();
    initParams();
    setHome();
    enableMotors(true);
}

void loop() {
    updateEncoders();
    handleEStop();
    handleFreeMove();
    handleHome();

    if (eStopEnabled) return;

    if (Serial.available() > 0) {
        handleSerialInput();
        moveJointsToDeg(Jlist, (float)robot_speed);
        Serial.println("[" + String(Jlist[0]) + " " + String(Jlist[1]) + " " + String(Jlist[2]) + " " + String(Jlist[3]) + " " + String(Jlist[4]) + " " + String(Jlist[5]) + "]");
        update_encoder_valus();
        Serial.println("end");
    }

}