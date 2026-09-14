#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <Arduino.h>

void setupLimitSwitches();
void updateLimits();

bool LimitSwitch_leftPressed();
bool LimitSwitch_rightPressed();
bool LimitSwitch_topPressed();
bool LimitSwitch_bottomPressed();

#endif
