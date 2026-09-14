#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <Arduino.h>

extern const double COUNTS_PER_MM;
extern const int MIN_DRIVE_PWM; // Expose minimum drive PWM used across modules

// static feedforward helper: applies MIN_DRIVE_PWM bias and kff scaling
double staticFeedforward(double target, double kff);

long distanceToCounts(double distance_mm);
double countsToDistance(long counts);
int8_t outputToDirection(double output);
class Timer
{
private:
    unsigned long startTime;
    bool timerActive;

public:
    Timer();

    bool startTimer(unsigned long waitTime);
    void reset();
};
#endif