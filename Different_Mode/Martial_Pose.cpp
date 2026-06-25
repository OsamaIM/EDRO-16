
 /*  ================================================================
    main.cpp  -  Martial posing 
    ================================================================
    PURPOSE: Robot walks forward continuously (not one cycle then
             stop) using a 4-phase weight-shift gait for balance.


    Hardware : ESP32-WROOM + PCA9685 + 16x MG90S
    Lib      : Adafruit PWM Servo Driver Library ^3.0.2
               Adafruit BusIO ^1.17.0
    ================================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN   125
#define SERVOMAX   575
#define SERVO_FREQ  50

// ── Channel assignments ───────────────────────────────────────────
#define CH_HEAD_PAN          0
#define CH_L_SHOULDER_PITCH  1
#define CH_L_SHOULDER_ROLL   2
#define CH_L_ELBOW           3
#define CH_R_SHOULDER_PITCH  4
#define CH_R_SHOULDER_ROLL   5
#define CH_R_ELBOW           6
#define CH_WAIST             7
#define CH_L_HIP_PITCH       8
#define CH_L_HIP_ROLL        9
#define CH_L_KNEE           10
#define CH_L_ANKLE          11
#define CH_R_HIP_PITCH      12
#define CH_R_HIP_ROLL       13
#define CH_R_KNEE           14
#define CH_R_ANKLE          15

// ════════════════════════════════════════════════════════════════
//  TUNING ZONE 1  -  NEUTRAL ANGLES (degrees, per joint)
//  If a leg is not straight at "90", adjust its neutral here.
// ════════════════════════════════════════════════════════════════
uint8_t NEUTRAL[16] = {
    90,  // CH 0  HEAD PAN
    90,  // CH 1  L SHOULDER PITCH
    90,  // CH 2  L SHOULDER ROLL
    90,  // CH 3  L ELBOW
    90,  // CH 4  R SHOULDER PITCH
    90,  // CH 5  R SHOULDER ROLL
    90,  // CH 6  R ELBOW
    90,  // CH 7  WAIST
    90,  // CH 8  L HIP PITCH
    90,  // CH 9  L HIP ROLL
    90,  // CH 10 L KNEE
    90,  // CH 11 L ANKLE
    90,  // CH 12 R HIP PITCH
    90,  // CH 13 R HIP ROLL
    90,  // CH 14 R KNEE
    90   // CH 15 R ANKLE
};

// ════════════════════════════════════════════════════════════════
//  TUNING ZONE 2  -  DIRECTION SIGNS (+1 or -1 ONLY)
//  Flip a sign to -1 if that joint moves the WRONG physical
//  direction during the gait. Leave +1 if it's correct.
// ════════════════════════════════════════════════════════════════
int SIGN_L_HIP_PITCH = +1;   // +1 = increasing angle swings left leg forward
int SIGN_R_HIP_PITCH = +1;   // +1 = increasing angle swings right leg forward
int SIGN_L_KNEE      = +1;   // +1 = increasing angle bends knee (lifts foot)
int SIGN_R_KNEE      = +1;
int SIGN_L_HIP_ROLL  = +1;   // +1 = increasing angle shifts weight toward this leg
int SIGN_R_HIP_ROLL  = +1;
int SIGN_L_ANKLE     = +1;   // +1 = increasing angle tilts ankle to compensate lean
int SIGN_R_ANKLE     = +1;

// ════════════════════════════════════════════════════════════════
//  TUNING ZONE 3  -  GAIT AMPLITUDES (degrees from neutral)
//  Start small (5-8 deg) and increase gradually once direction
//  signs are confirmed correct.
// ════════════════════════════════════════════════════════════════
int HIP_SWING_DEG   = 8;    // how far hip pitches forward/back
int KNEE_LIFT_DEG   = 14;   // how much knee bends to lift foot
int HIP_ROLL_DEG    = 8;    // lateral weight-shift amount
int ANKLE_COMP_DEG  = 4;    // ankle compensation during weight-shift

// ════════════════════════════════════════════════════════════════
//  TUNING ZONE 4  -  TIMING (milliseconds)
//  Increase PHASE_DELAY for a slower, more stable walk.
//  Decrease for faster walking once balance is confirmed.
// ════════════════════════════════════════════════════════════════
int PHASE_STEPS  = 18;   // interpolation steps per phase (smoothness)
int PHASE_SPEED  = 12;   // ms delay between each interpolation step
int PHASE_DELAY  = 70;   // ms pause after each phase completes

// Track current servo position
uint8_t currentAngle[16];

// ════════════════════════════════════════════════════════════════
//  LOW-LEVEL SERVO CONTROL
// ════════════════════════════════════════════════════════════════

void setServo(uint8_t ch, int angle) {
    angle = constrain(angle, 0, 180);
    currentAngle[ch] = (uint8_t)angle;
    pwm.setPWM(ch, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
}

// Move several servos simultaneously, smoothly, to target angles
void moveParallel(uint8_t* channels, int* targets, uint8_t count,
                  uint8_t steps, uint8_t speedMs) {
    float cur[count], delta[count];
    for (uint8_t i = 0; i < count; i++) {
        cur[i]   = currentAngle[channels[i]];
        int tgt  = constrain(targets[i], 0, 180);
        delta[i] = ((float)tgt - cur[i]) / steps;
    }
    for (uint8_t s = 0; s < steps; s++) {
        for (uint8_t i = 0; i < count; i++) {
            cur[i] += delta[i];
            setServo(channels[i], (int)cur[i]);
        }
        delay(speedMs);
    }
}

void standNeutral() {
    uint8_t allCh[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    int targets[16];
    for (uint8_t i = 0; i < 16; i++) targets[i] = NEUTRAL[i];
    moveParallel(allCh, targets, 16, 30, 10);
    delay(300);
}

// ════════════════════════════════════════════════════════════════
//  WALK GAIT  -  4-phase weight-shift cycle, runs continuously
//
//  Phase A: shift CoM onto RIGHT foot   (lift LEFT leg next)
//  Phase B: swing LEFT leg forward, knee bends to clear ground
//  Phase C: shift CoM onto LEFT foot    (lift RIGHT leg next)
//  Phase D: swing RIGHT leg forward, knee bends to clear ground
//
//  All hip/knee/ankle targets use NEUTRAL[ch] + SIGN*amount,
//  so flipping a SIGN constant flips that joint's direction
//  without touching the gait logic itself.
// ════════════════════════════════════════════════════════════════

void gaitPhaseA_shiftRight() {
    // Weight shifts onto right foot: left hip rolls one way,
    // right hip rolls opposite, ankles compensate to keep
    // the foot flat on the ground.
    uint8_t ch[] = { CH_L_HIP_ROLL, CH_R_HIP_ROLL, CH_L_ANKLE, CH_R_ANKLE };
    int tgt[] = {
        NEUTRAL[CH_L_HIP_ROLL] + SIGN_L_HIP_ROLL * HIP_ROLL_DEG,
        NEUTRAL[CH_R_HIP_ROLL] - SIGN_R_HIP_ROLL * HIP_ROLL_DEG,
        NEUTRAL[CH_L_ANKLE]    - SIGN_L_ANKLE    * ANKLE_COMP_DEG,
        NEUTRAL[CH_R_ANKLE]    + SIGN_R_ANKLE    * ANKLE_COMP_DEG
    };
    moveParallel(ch, tgt, 4, PHASE_STEPS, PHASE_SPEED);
    delay(PHASE_DELAY);
}

void gaitPhaseB_swingLeft() {
    // Left leg swings forward (hip pitch + knee lift),
    // right leg compensates by moving slightly backward
    // to maintain stride symmetry.
    uint8_t ch[] = { CH_L_HIP_PITCH, CH_L_KNEE, CH_R_HIP_PITCH };
    int tgt[] = {
        NEUTRAL[CH_L_HIP_PITCH] + SIGN_L_HIP_PITCH * HIP_SWING_DEG,
        NEUTRAL[CH_L_KNEE]      + SIGN_L_KNEE      * KNEE_LIFT_DEG,
        NEUTRAL[CH_R_HIP_PITCH] - SIGN_R_HIP_PITCH * HIP_SWING_DEG
    };
    moveParallel(ch, tgt, 3, PHASE_STEPS, PHASE_SPEED);
    delay(PHASE_DELAY);

    // Left foot plants flat - straighten knee back to neutral
    uint8_t kch[] = { CH_L_KNEE };
    int ktgt[] = { NEUTRAL[CH_L_KNEE] };
    moveParallel(kch, ktgt, 1, 12, PHASE_SPEED);
    delay(PHASE_DELAY / 2);
}

void gaitPhaseC_shiftLeft() {
    uint8_t ch[] = { CH_L_HIP_ROLL, CH_R_HIP_ROLL, CH_L_ANKLE, CH_R_ANKLE };
    int tgt[] = {
        NEUTRAL[CH_L_HIP_ROLL] - SIGN_L_HIP_ROLL * HIP_ROLL_DEG,
        NEUTRAL[CH_R_HIP_ROLL] + SIGN_R_HIP_ROLL * HIP_ROLL_DEG,
        NEUTRAL[CH_L_ANKLE]    + SIGN_L_ANKLE    * ANKLE_COMP_DEG,
        NEUTRAL[CH_R_ANKLE]    - SIGN_R_ANKLE    * ANKLE_COMP_DEG
    };
    moveParallel(ch, tgt, 4, PHASE_STEPS, PHASE_SPEED);
    delay(PHASE_DELAY);
}

void gaitPhaseD_swingRight() {
    uint8_t ch[] = { CH_R_HIP_PITCH, CH_R_KNEE, CH_L_HIP_PITCH };
    int tgt[] = {
        NEUTRAL[CH_R_HIP_PITCH] + SIGN_R_HIP_PITCH * HIP_SWING_DEG,
        NEUTRAL[CH_R_KNEE]      + SIGN_R_KNEE      * KNEE_LIFT_DEG,
        NEUTRAL[CH_L_HIP_PITCH] - SIGN_L_HIP_PITCH * HIP_SWING_DEG
    };
    moveParallel(ch, tgt, 3, PHASE_STEPS, PHASE_SPEED);
    delay(PHASE_DELAY);

    uint8_t kch[] = { CH_R_KNEE };
    int ktgt[] = { NEUTRAL[CH_R_KNEE] };
    moveParallel(kch, ktgt, 1, 12, PHASE_SPEED);
    delay(PHASE_DELAY / 2);
}

void gaitReturnToCenter() {
    // Bring both hips/ankles back to neutral before the next
    // full cycle begins, so steps don't accumulate drift.
    uint8_t ch[] = {
        CH_L_HIP_PITCH, CH_R_HIP_PITCH,
        CH_L_HIP_ROLL,  CH_R_HIP_ROLL,
        CH_L_ANKLE,     CH_R_ANKLE
    };
    int tgt[] = {
        NEUTRAL[CH_L_HIP_PITCH], NEUTRAL[CH_R_HIP_PITCH],
        NEUTRAL[CH_L_HIP_ROLL],  NEUTRAL[CH_R_HIP_ROLL],
        NEUTRAL[CH_L_ANKLE],     NEUTRAL[CH_R_ANKLE]
    };
    moveParallel(ch, tgt, 6, 14, PHASE_SPEED);
}

// One complete 4-phase gait cycle (= one full stride, both legs)
void walkOneCycle() {
    gaitPhaseA_shiftRight();
    gaitPhaseB_swingLeft();
    gaitPhaseC_shiftLeft();
    gaitPhaseD_swingRight();
    gaitReturnToCenter();
}

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================");
    Serial.println("  CONTINUOUS WALKING - TUNING BUILD");
    Serial.println("========================================");
    Serial.println("Remember: flip SIGN_* constants if a leg");
    Serial.println("swings the wrong way, BEFORE expecting");
    Serial.println("a clean walk. See comments at top of file.\n");

    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    delay(200);

    for (uint8_t ch = 0; ch < 16; ch++) {
        currentAngle[ch] = NEUTRAL[ch];
        setServo(ch, NEUTRAL[ch]);
        delay(20);
    }
    delay(1000);

    Serial.println("Standing at neutral. Walking starts in 3s...\n");
    delay(3000);
}

// ════════════════════════════════════════════════════════════════
//  LOOP  -  walks continuously with posing forever, never stops on its own
// ════════════════════════════════════════════════════════════════
uint32_t cycleCount = 0;

void loop() {
    cycleCount++;
    Serial.printf("[WALK] Cycle %lu\n", cycleCount);
    walkOneCycle();

    // No stop condition - keeps walking indefinitely.
    // To make it stop on command later, you can check a
    // flag here (e.g. set by a button, IR sensor, or Wi-Fi
    // command) and call standNeutral() + return early.
}