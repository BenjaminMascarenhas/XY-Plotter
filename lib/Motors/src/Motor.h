#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Left Motor Pins 
const int E1 = 5;
const int M1 = 4;

//Right Motor Pins
const int E2 = 6;
const int M2 = 7;

void Motor_Init();  // initialise pins for motor control
String setLeftMotor(int8_t dir, int pwm);   // returns "FAULT" for fault or "SUCCESS" for success
String setRightMotor(int8_t dir, int pwm);  // returns "FAULT" for fault or "SUCCESS" for success
void stopMotors();

#endif