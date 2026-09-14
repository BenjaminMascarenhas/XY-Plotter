#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>

enum HomingResult
{
    HOMING_RUNNING,
    HOMING_COMPLETE,
    HOMING_FAULT
};

void homingStart();

HomingResult homingUpdate();

#endif
