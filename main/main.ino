/* 
    Inspired by AR4 Robot Control Software
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    This revision of the code has a shared speed for all joints during movement, due to the current RoboDK configuration only delivering one speed variable.

    To do: update gear ratios
*/

#include <Arduino.h>
#include <math.h>
#include <Encoder.h>

const int n_J = 6; // Number of joints

// external variables from RoboDK interface
extern float Jlist[6];
extern int robot_speed; // % of joint max deg/s

// pins
const int stepPin[n_J] = {0, 2, 4, 6, 8, 10};
const int dirPin[n_J]  = {1, 3, 5, 7, 9, 11};

const int encAPin[n_J] = {14, 16, 18, 20, 22, 24};
const int encBPin[n_J] = {15, 17, 19, 21, 23, 25};

const int enPin      = 12;
const int eStopPin   = 26; // latching button
const int freeMovePin= 27; // momentary button, yellow
const int homePin    = 28; // momentary button, green

// motor + encoder + driver specs
const int SPR = 1600; // motor steps per rev
const int CPR[n_J]       = {4000, 4000, 4000, 1200, 1200, 4000}; // encoder counts per rev
const float gearRatio[n_J] = {71,   50,   20,   20,   33.333,   20};   // gearbox ratio

const int invDir[n_J] = {0, 0, 0, 0, 0, 0}; // invert direction for motors

// encoder instances (one per joint)
Encoder enc[n_J] = {
    Encoder(encAPin[0], encBPin[0]),
    Encoder(encAPin[1], encBPin[1]),
    Encoder(encAPin[2], encBPin[2]),
    Encoder(encAPin[3], encBPin[3]),
    Encoder(encAPin[4], encBPin[4]),
    Encoder(encAPin[5], encBPin[5]),
};

// Joint limits (deg)
float limPos[n_J] = {180.0f,  90.0f,  135.0f, 360.0f,  90.0f,  360.0f};
float limNeg[n_J] = {-180.0f, -90.0f, -135.0f, -360.0f, -90.0f, -360.0f};

// Maximum velocity per joint (deg/s at 100% speed)
const float maxSpeed[n_J] = {30.0f, 20.0f, 30.0f, 40.0f, 30.0f, 180.0f};

const float accelK = 0.2f; // Acceleration duration (% of movement)
const float accelMinSpeed = 0.2f; // Percentage of speed at the acceleration's slowest point.

// internal vars
float encMult[n_J];  // encoder counts per degree (includes gearbox)
float stepDeg[n_J];  // motor steps per degree (includes gearbox)
long  encRaw[n_J];   // raw encoder counts
float encDeg[n_J];   // encoder deg position

long  stepCount[n_J]; // accumulated step count
float jointDeg[n_J];  // current joint angle (deg) from step count
long totalSteps[n_J]; // total steps for current move (for accel/decel)
long stepsDone[n_J];  // steps already taken in current move

// function state flags
bool motorsEnabled   = false;
bool eStopEnabled    = false;
bool freeMoveEnabled = false;

// init step/dir pins
void initPins() {
    pinMode(enPin, OUTPUT);
    pinMode(eStopPin, INPUT_PULLUP);
    pinMode(freeMovePin, INPUT_PULLUP);
    pinMode(homePin, INPUT_PULLUP);

    digitalWrite(enPin, LOW); // disable motors at startup
    
    // init all motor pins
    for (int j = 0; j < n_J; j++) {
        pinMode(stepPin[j], OUTPUT);
        pinMode(dirPin[j],  OUTPUT);
        digitalWrite(stepPin[j], LOW);
        digitalWrite(dirPin[j],  LOW);
    }
}

// init conversion parameters
void initParams() {
    for (int j = 0; j < n_J; j++) {
        encMult[j] = (float)(CPR[j] * gearRatio[j]) / 360.0f;
        stepDeg[j] = (float)(SPR   * gearRatio[j]) / 360.0f;
    }
}

// enable / disable motor drivers
void enableMotors(bool on) {
    motorsEnabled = on;
    digitalWrite(enPin, on ? HIGH : LOW);
}

// convert degrees to motor steps
long degToSteps(int joint, float deg) {
    if (joint < 0 || joint >= n_J) return 0;
    return lroundf(deg * stepDeg[joint]);
}

// convert motor steps to degrees
float stepsToDeg(int joint, long steps) {
    if (joint < 0 || joint >= n_J) return 0.0f;
    return (float)steps / stepDeg[joint];
}

// update encoder readings (1 kHz max rate)
void updateEncoders() {
    static unsigned long lastUpdate = 0;
    const unsigned long interval = 1000;
    unsigned long now = micros();
    
    if (now - lastUpdate >= interval) {
        lastUpdate = now;

        for (int j = 0; j < n_J; j++) {
            encRaw[j] = enc[j].read();
            encDeg[j] = (float)encRaw[j] / encMult[j];
        }
    } 
}

// main joint motion function
void moveJointsToDeg(const float targetDeg[n_J], float speed = 25.0f) {
    // abort if unsafe
    if (!motorsEnabled || eStopEnabled || freeMoveEnabled) return;

    long  remSteps[n_J];
    int   direction[n_J];
    unsigned long stepIntervalUs[n_J];
    unsigned long lastStepUs[n_J];
    bool  stepLevel[n_J];

    bool hasActiveJoint = false;
    unsigned long nowUs = micros();

    // preprocess targets
    for (int j = 0; j < n_J; j++) {

        float deg = targetDeg[j];

        // limit angle to joint range
        if (deg > limPos[j]) deg = limPos[j];
        if (deg < limNeg[j]) deg = limNeg[j];

        // global speed = prosent (0–100) av maxSpeed for hvert ledd
        float speedK = speed;
        if (speedK < 0.0f)  speedK = 0.0f;
        if (speedK > 100.0f) speedK = 100.0f;

        float jointSpeed = maxSpeed[j] * (speedK / 100.0f);
        // sikkerhet: ikke negativ
        if (jointSpeed < 0.0f) jointSpeed = 0.0f;

        // calculate step difference
        long delta = degToSteps(j, deg) - stepCount[j];
        remSteps[j] = labs(delta);
        totalSteps[j] = remSteps[j];
        stepsDone[j] = 0;

        // skip if no motion needed
        if (remSteps[j] == 0 || jointSpeed == 0.0f) {
            stepIntervalUs[j] = 0;
            continue;
        }

        hasActiveJoint = true;

        // set direction pin
        direction[j] = (delta >= 0) ? 1 : -1;

        bool dirPinState = (direction[j] > 0);
        if (invDir[j]) dirPinState = !dirPinState;
        digitalWrite(dirPin[j], dirPinState ? HIGH : LOW);

        // convert deg/s to steps/s to microsecond step interval
        float stepsPerSec = jointSpeed * stepDeg[j];
        if (stepsPerSec < 1.0f) stepsPerSec = 1.0f;

        stepIntervalUs[j] = (unsigned long)(1000000.0f / (2 * stepsPerSec));
        lastStepUs[j] = nowUs;

        stepLevel[j] = LOW;
        digitalWrite(stepPin[j], LOW);
    }

    if (!hasActiveJoint) return;

    // stepping loop (blocking)
    while (true) {
        handleEStop();
        handleFreeMove();

        // Immediately stop motion if unsafe condition is active
        if (eStopEnabled || freeMoveEnabled) break;
            
        unsigned long now = micros();
        bool moving = false;

        for (int j = 0; j < n_J; j++) {
            // Skip joints that are finished or inactive
            if (remSteps[j] == 0 || stepIntervalUs[j] == 0) continue;

            // Base step interval (cruise)
            unsigned long intervalUs = stepIntervalUs[j];

            // Simple accel/decel profile based on progress (0..1)
            if (totalSteps[j] > 0) {
                float pos   = (float)stepsDone[j] / (float)totalSteps[j]; // 0..1
                float scale = 1.0f;

                if (pos < accelK) {
                    // Acceleration phase (start)
                    float t = pos / accelK; // 0 -> 1
                    scale = accelMinSpeed + (1.0f - accelMinSpeed) * t;
                } else if (pos > 1.0f - accelK) {
                    // Deceleration phase (end)
                    float t = (1.0f - pos) / accelK; // 0 -> 1 towards the end
                    scale = accelMinSpeed + (1.0f - accelMinSpeed) * t;
                } else {
                    // Cruise phase
                    scale = 1.0f;
                }

                // Higher scale = higher speed = shorter interval
                intervalUs = (unsigned long)((float)stepIntervalUs[j] / scale);
            }

            // Time to toggle step pin?
            if (now - lastStepUs[j] >= intervalUs) {
                lastStepUs[j] += intervalUs;

                // Toggle step signal
                stepLevel[j] = !stepLevel[j];
                digitalWrite(stepPin[j], stepLevel[j]);

                // Count steps only on falling edge
                if (stepLevel[j] == LOW) {
                    remSteps[j]--;
                    stepsDone[j]++;               // track progress for accel/decel
                    stepCount[j] += direction[j];
                    jointDeg[j]  = stepsToDeg(j, stepCount[j]);
                }
            }

            // If any joint still has remaining steps, keep loop running
            if (remSteps[j] > 0) moving = true;
        }

        // Exit loop when all joints are finished
        if (!moving) break;
    }


}

// reset all positions to zero
void setHome() {
    if (eStopEnabled) return;

    for (int j = 0; j < n_J; j++) {
        enc[j].write(0);
        encRaw[j]  = 0;
        encDeg[j]  = 0.0f;
        stepCount[j] = 0;
        jointDeg[j]  = 0.0f;
    }
}

// move robot to home position (0 deg all joints, straight up)
void moveHome(float speed = 30.0f) {
    float targetDeg[n_J] = {0, 0, 0, 0, 0, 0};
    moveJointsToDeg(targetDeg, speed);
}

// debounced e-stop handler
void handleEStop() {
    static bool lastRaw = HIGH;
    static bool lastStable = HIGH;
    static unsigned long lastDebounce = 0;
    const unsigned long debounceDelay = 50; // ms

    static unsigned long lastPrint = 0;
    const unsigned long printDelay = 200;

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

    if (eStopEnabled) {
        if (now - lastPrint >= printDelay) {
            lastPrint = now;
            Serial.println("ESTOP ENABLED");
        }
    }
}

// debounced free-move toggle
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
                if (eStopEnabled) {
                    Serial.println("FREEMOVE BLOCKED - ESTOP ACTIVE");
                    return;
                }

                freeMoveEnabled = !freeMoveEnabled;
                enableMotors(!freeMoveEnabled);
                Serial.println(freeMoveEnabled ? "FREEMOVE ON" : "FREEMOVE OFF");
            }
        }
    }
}

// debounced home button
void handleHome() { 
    static bool lastVal = HIGH;
    static unsigned long lastTime = 0;
    const unsigned long debounceDelay = 50; // ms

    bool raw = digitalRead(homePin);
    unsigned long now = millis();

    if (now - lastTime > debounceDelay) {
        if (raw == LOW && lastVal == HIGH) {
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

    // serial command handler
    if (Serial.available() > 0) {
        handleSerialInput();
        moveJointsToDeg(Jlist, (float)robot_speed);
        Serial.println("[" + String(Jlist[0]) + " " + String(Jlist[1]) + " " 
                           + String(Jlist[2]) + " " + String(Jlist[3]) + " "
                           + String(Jlist[4]) + " " + String(Jlist[5]) + "]");
        update_encoder_valus();
        Serial.println("end");
    }
}
