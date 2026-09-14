#include "Motor.h"

// Set up motor control pins.
void Motor_Init()
{
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);
}

// Drive the left motor in a given direction.
String setLeftMotor(int8_t dir, int pwm)
{
    if (dir == 0)
    {
        analogWrite(E1, 0);
        return "STOP";
    }

    digitalWrite(M1, dir > 0 ? HIGH : LOW);
    analogWrite(E1, pwm);

    return "SUCCESS";
}

// Drive the right motor in a given direction.
String setRightMotor(int8_t dir, int pwm)
{
    if (dir == 0)
    {
        analogWrite(E2, 0);
        return "STOP";
    }

    digitalWrite(M2, dir > 0 ? HIGH : LOW);
    analogWrite(E2, pwm);

    return "SUCCESS";
}

// Stop both motors.
void stopMotors()
{
    setLeftMotor(0, 0);
    setRightMotor(0, 0);
}