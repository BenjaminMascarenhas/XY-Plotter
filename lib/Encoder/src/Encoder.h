#ifndef ENCODER_H
#define ENCODER_H

#include <HelperFunctions.h>
#include <Arduino.h>
#include <avr/interrupt.h>

const long Y_MAX = 125*COUNTS_PER_MM;   //max distance y in counts
const long Y_MIN = -50*COUNTS_PER_MM;
const long X_MAX = 195*COUNTS_PER_MM;   //max distance x in counts
const long X_MIN = -50*COUNTS_PER_MM;
const long BOUNDS_TOLERANCE = 2*COUNTS_PER_MM; //tolerance for bounds checking

extern volatile long countA;
extern volatile long countB;

extern volatile long globalCountA;
extern volatile long globalCountB;

extern double Encoder_getLeftVelocity(double dt);
extern double Encoder_getRightVelocity(double dt);

// Left Motor Encoder Pins
extern const int L_ENCA; // INT3
extern const int L_ENCB; // INT2

// Right Motor eEncoder Pins
extern const int R_ENCA; // INT1
extern const int R_ENCB; // INT0

// Current Encoder Counts
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;

extern double distance_per_encoder_tick;

void Encoder_Init();
void Encoder_updateLeft();
void Encoder_updateRight();
long Encoder_getLeftEncoderCount();
long Encoder_getRightEncoderCount();
long Encoder_getLeftGlobalEncoderCount();
long Encoder_getRightGlobalEncoderCount();
void Encoder_setLeftEncoderCountZero();
void Encoder_setRightEncoderCountZero();
void Encoder_setLeftGlobalEncoderCountZero();
void Encoder_setRightGlobalEncoderCountZero();
bool encoderCountsChanged();

#endif