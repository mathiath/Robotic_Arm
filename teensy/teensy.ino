/* 
    Inspired by AR4 Robot Control Software
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    Specs:

    J1: Nema 17, 10:1, 1000 PPR (4000 CPR)
    J2: Nema 23, 50:1, 1000 PPR (4000 CPR)
    J3: Nema 17, 20:1, 1000 PPR (4000 CPR)
    J4: Nema 14, 20:1, 300 PPR (1200 CPR)
    J5: Nema 14, 20:1 + 2:1 = 40:1, 300 PPR (1200 CPR)
    J6: Nema 14, 20:1, 1000 PPR (1200 CPR)

    Encoder Multiplier: (CPR * GRatio / 360)

    J1: 4000 * 10 / 360 = 111.1111
    J2: 4000 * 50 / 360 = 555.5556
    J3: 4000 * 20 / 360 = 222.2222
    J4: 1200 * 20 / 360 = 66.6667
    J5: 1200 * 40 / 360 = 133.3333
    J6: 4000 * 20 / 360 = 222.2222

    JOINT AND MOTOR SPEEDS:

    Motor maxDegPerSec:
    Hardware cap: 2000 rpm pre gearbox. Choosing absolute motor cap at 1500 rpm for noise and safety reasons.

    maxDegPerSec = motorMaxRPM * 6 / GRatio

    Absolute motor cap / joint cap:
    J1: 1500 rpm * 6 / 10 = 900 deg/s
    J2: 1500 rpm * 6 / 50 = 180 deg/s
    J3: 1500 rpm * 6 / 20 = 450 deg/s
    J4: 1500 rpm * 6 / 20 = 450 deg/s
    J5: 1500 rpm * 6 / 40 = 225 deg/s
    J6: 1500 rpm * 6 / 20 = 450 deg/s

    Chosen motor cap / joint cap:
    J1: 300 rpm * 6 / 10 = 180 deg/s
    J2: 750 rpm * 6 / 50 = 90 deg/s
    J3: 600 rpm * 6 / 20 = 180 deg/s
    J4: 900 rpm * 6 / 20 = 270 deg/s
    J5: 1200 rpm * 6 / 40 = 180 deg/s
    J6: 900 rpm * 6 / 20 = 270 deg/s

*/

#include <Arduino.h>

// pins
const int stepPin[6] = {0, 2, 4, 6, 8, 10};
const int dirPin[6] = {1, 3, 5, 7, 9, 11};
const int encAPin[6] = {14, 16, 18, 20, 22, 24};
const int encBPin[6] = {15, 17, 19, 21, 23, 25};

const int enPin = 12;
//const int estopPin = 0;



// initParams
const int CPR[6] = {4000, 4000, 4000, 1200, 1200, 4000};
const int gearRatio[6] = {10, 50, 20, 20, 40, 20};
const int SPR = 1600;
const float encMult[6];
const float stepDeg[6];

const int motorDir[6] = {1, 1, 1, 1, 1, 1};
float axisLimPos[6] = {0, 0, 0, 0, 0, 0};
float axisLimNeg[6] = {0, 0, 0, 0, 0, 0};
const float maxDegPerSec[6] = {180, 90, 180, 270, 180, 270}; // 

bool motorsEnabled = false;

float axisLim[6];
float stepLim[6];
int zeroStep[6];
int stepM[6]; // master step count

float jointDeg[6]; // nåværende vinkel i grader

int collisionTrue[6] = {0, 0, 0, 0, 0, 0};
int totalCollision = 0;
int KinematicError = 0;

unsigned long debounceTime[6] = {0, 0, 0, 0, 0, 0};
unsigned long debounceDelay = 50;



/*
bool estopActive() {
    return digitalRead(estopPin) == LOW;
}
*/

void initPins() {

    for (int i = 0; i < 6; i++) {
        pinMode(stepPin[i], OUTPUT);
        pinMode(dirPin[i], OUTPUT);
        digitalWrite(stepPin[i], LOW);
        digitalWrite(dirPin[i], LOW);
    }
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, HIGH); // motor disabled during init
    pinMode(estopPin, INPUT_PULLUP);
}

void initParams() {
    for (int i=0; i<6; i++) {
        axisLim[i] = axisLimPos[i] + axisLimNeg[i];
        stepLim[i] = axisLim[i] * stepDeg[i];
        zeroStep[i] = axisLimNeg[i] * stepDeg[i];
        stepM[i] = zeroStep[i];
        jointDeg[i] = 0.0f;
        encMult[i] = (float)(CPR[i] * gearRatio[i]) / 360.0;
        stepDeg[i] = (float)(SPR * gearRatio[i]) / 360.0;

    }
}

long degToSteps(int axis, float deg) {
    if (deg > axisLimPos[axis]) deg = axisLimPos[axis];
    if (deg < -axisLimNeg[axis]) deg = -axisLimNeg[axis];

    float stepsF = (deg + axisLimNeg[axis]) * stepDeg[axis];
    return round(stepsF);
}

float stepsToDeg(int axis, long steps) {
    float deg = ((float)steps / stepDeg[axis]) - axisLimNeg[axis];
    return deg;
}

// NEW: flytt ett ledd til vinkel (deg) med ønsket hastighet (deg/s)
void moveJointToDeg(int axis, float targetDeg, float speedDegPerSec) {
    if (axis < 0 || axis >= 6) return;
    if (!motorsEnabled) return;       // ikke kjør hvis motorer er disabled
    if (estopActive()) return;        // stopp hvis e-stop

    // clamp mot maks-hastighet for dette leddet
    if (speedDegPerSec > maxDegPerSec[axis]) {
        speedDegPerSec = maxDegPerSec[axis];
    }
    if (speedDegPerSec <= 0.0f) {
        speedDegPerSec = maxDegPerSec[axis] * 0.1f; // fallback
    }

    long targetStep  = degToSteps(axis, targetDeg);
    long currentStep = stepM[axis];
    long delta       = targetStep - currentStep;

    if (delta == 0) return;

    int dir = (delta > 0) ? 1 : -1;
    long stepsToMove = labs(delta);

    // sett retning (inkluderer motorDir)
    int hwDir = (dir * motorDir[axis] > 0) ? HIGH : LOW;
    digitalWrite(dirPin[axis], hwDir);

    // stepFreq = speedDegPerSec * stepsPerDeg
    float stepFreq = speedDegPerSec * stepDeg[axis];   // steps/s
    if (stepFreq < 1.0f) stepFreq = 1.0f;
    unsigned long delayUs = (unsigned long)(1000000.0f / stepFreq);

    for (long s = 0; s < stepsToMove; s++) {
        if (estopActive() || !motorsEnabled) {
            return;
        }

        digitalWrite(stepPin[axis], HIGH);
        delayMicroseconds(2); // pulsbredden, kan justeres
        digitalWrite(stepPin[axis], LOW);
        delayMicroseconds(delayUs);

        stepM[axis] += dir;
    }

    jointDeg[axis] = stepsToDeg(axis, stepM[axis]);
}

void moveToJointAngles(float target[6], float speedDegPerSec) {
    for (int i = 0; i < 6; i++) {
        if (estopActive() || !motorsEnabled) return;
        moveJointToDeg(i, target[i], speedDegPerSec);
    }
}

void setup() {
    Serial.begin(115200);
    initPins();
    initParams();
    

}

void loop() {



}