#include "Encoder.h"
#include <HelperFunctions.h>

#include <util/atomic.h>

// Left Motor Encoder Pins
const int L_ENCA = 18; // INT3
const int L_ENCB = 19; // INT2

// Right Motor Encoder Pins
const int R_ENCA = 20; // INT1
const int R_ENCB = 21; // INT0

// Current Encoder States 
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

// encoder counters for relative position tracking (single definition)
volatile long countA = 0;
volatile long countB = 0;

volatile long globalCountA = 0;
volatile long globalCountB = 0;

// Predetermined table to see which way the motor is spinning
const int8_t encTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

// Distance represented by one encoder tick (mm)
double distance_per_encoder_tick = 1.0 / COUNTS_PER_MM;

static long prevLeftCount = 0;
static long prevRightCount = 0;

// Configure encoder pins and interrupts.
void Encoder_Init() {
    pinMode(L_ENCA, INPUT_PULLUP);
    pinMode(L_ENCB, INPUT_PULLUP);
    pinMode(R_ENCA, INPUT_PULLUP);
    pinMode(R_ENCB, INPUT_PULLUP);

    cli();
    EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);

    EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20) | (1 << ISC30);
    sei();
}

// Update the left encoder state.
void updateLeft() {
    uint8_t a = digitalRead(L_ENCA);
    uint8_t b = digitalRead(L_ENCB);
    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateA << 2) | newState;
    countA += encTable[index];
    globalCountA += encTable[index];
    stateA = newState;
}

// Update the right encoder state.
void updateRight() {
    uint8_t a = digitalRead(R_ENCA);
    uint8_t b = digitalRead(R_ENCB);
    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;
    countB += encTable[index];
    globalCountB += encTable[index];
    stateB = newState;
}

long Encoder_getLeftEncoderCount()
{
    long count;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        count = countA;
    }

    return count;
}

long Encoder_getRightEncoderCount()
{
    long count;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        count = countB;
    }

    return count;
}

long Encoder_getLeftGlobalEncoderCount()
{
    long count;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        count = globalCountA;
    }

    return count;
}

long Encoder_getRightGlobalEncoderCount()
{
    long count;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        count = globalCountB;
    }

    return count;
}


double Encoder_getLeftVelocity(double dt) {
    long current = Encoder_getLeftEncoderCount();
    double velocity = (current - prevLeftCount) * distance_per_encoder_tick / dt;
    prevLeftCount = current;
    return velocity; // mm/s
}

double Encoder_getRightVelocity(double dt) {
    long current = Encoder_getRightEncoderCount();
    double velocity = (current - prevRightCount) * distance_per_encoder_tick / dt;
    prevRightCount = current;
    return velocity; // mm/s
}

void Encoder_setLeftEncoderCountZero()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        countA = 0;
        prevLeftCount = 0;
    }
}

void Encoder_setRightEncoderCountZero()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        countB = 0;
        prevRightCount = 0;
    }
}

void Encoder_setLeftGlobalEncoderCountZero()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        globalCountA = 0;
    }
}

void Encoder_setRightGlobalEncoderCountZero()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        globalCountB = 0;
    }
}

// Check whether the encoder counts have settled.
bool encoderCountsChanged()
{
    static Timer encoderTimer;
    static long previousLeft = 0;
    static long previousRight = 0;
    static bool initialized = false;

    if (!initialized)
    {
        previousLeft = Encoder_getLeftEncoderCount();
        previousRight = Encoder_getRightEncoderCount();
        initialized = true;
        return true;
    }

    // Don't compare until 50 ms has passed
    if (!encoderTimer.startTimer(50))
    {
        return true;
    }

    long currentLeft = Encoder_getLeftEncoderCount();
    long currentRight = Encoder_getRightEncoderCount();

    bool changed = (currentLeft != previousLeft) || (currentRight != previousRight);

    previousLeft = currentLeft;
    previousRight = currentRight;

    return changed;
}

ISR(INT0_vect) { updateRight(); } // pin 21 = R_ENCB
ISR(INT1_vect) { updateRight(); } // pin 20 = R_ENCA
ISR(INT2_vect) { updateLeft();  } // pin 19 = L_ENCB
ISR(INT3_vect) { updateLeft();  } // pin 18 = L_ENCA