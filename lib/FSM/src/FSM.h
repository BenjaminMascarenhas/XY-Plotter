#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

// System State
enum SystemState {
    STATE_IDLE,
    STATE_PARSING,
    STATE_MOTION,
    STATE_HOMING,
    STATE_FAULT
};

// enum MotionSubstate {
//     MOTION_ACCEL,
//     MOTION_CRUISE,
//     MOTION_DECEL,
//     MOTION_COMPLETE
// };

void FSM_init();
void FSM_update();
SystemState FSM_getCurrentState();

void FSM_triggerFault();

#endif