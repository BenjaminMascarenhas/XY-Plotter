#include "HelperFunctions.h"

const double COUNTS_PER_MM = 183; // 167.37 counts/mm

// Convert mm to encoder counts.
long distanceToCounts(double distance_mm)
{
    return (long)round(distance_mm * COUNTS_PER_MM);
}

// convert encoder counts to mm
double countsToDistance(long counts)
{
    return counts / COUNTS_PER_MM;
}

// return motor direction based on input
int8_t outputToDirection(double output)
{
    if (output > 0)
    {
        return 1;
    }
    if (output < 0)
    {
        return -1;
    }
    return 0;
}

// Lowest PWM that reliably turns the motor under load.
const int MIN_DRIVE_PWM = 45;

// add minimum drive to the motor so it overcomes static friction
double staticFeedforward(double target, double kff)
{
    if (target > 0)
        return MIN_DRIVE_PWM + kff * target;

    if (target < 0)
        return -MIN_DRIVE_PWM + kff * target;

    return 0;
}

Timer::Timer()
{
    startTime = 0;
    timerActive = false;
}

// start non blocking timer
bool Timer::startTimer(unsigned long waitTime)
{
    if (timerActive == false)
    {
        startTime = millis();
        timerActive = true;

        return false;
    }

    unsigned long currentTime = millis();

    if ((currentTime - startTime) >= waitTime)
    {
        timerActive = false;

        return true;
    }

    return false;
}

//reset timer
void Timer::reset()
{
    startTime = 0;
    timerActive = false;
}
