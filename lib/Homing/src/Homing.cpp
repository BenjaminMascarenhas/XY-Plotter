#include "Homing.h"
#include <Motor.h>
#include <LimitSwitch.h>
#include <PID_FSM.h>
#include <Encoder.h>


// Homing movement settings
static const int homingFeedrate = 1000;
static const int slowHomingFeedrate = 300;
static const int homingTravel = 10000;
static const int homingBackoffmm = 10;

// Enums in order to keep track of current homing state
enum HomingState {
    HOMING_DOWN_FAST,
    HOMING_BACKOFF_UP_1,
    HOMING_DOWN_SLOW,
    HOMING_BACKOFF_UP_2,

    HOMING_LEFT_FAST,
    HOMING_BACKOFF_RIGHT_1,
    HOMING_LEFT_SLOW,
    HOMING_BACKOFF_RIGHT_2,

    HOMING_DONE
};

// Initial homing state
static HomingState homingState = HOMING_DOWN_FAST;

static bool waiting = false;

// Straight line until desired limit switch is hit
static HomingResult moveToLimit(int x, int y, int feedrate, bool targetSwitchPressed, bool invalidSwitchPressed, HomingState nextState) {

        // If a move is finished and we are waiting for encoders to stop moving slightly
        if (waiting) {
            stopMotors();       // Halts all movement

            // Encoders are still moving slightly so return without changing 'waiting' variable
            if (encoderCountsChanged()) {
                return HOMING_RUNNING;
            }

            // Encoders are not moving so switch to next state in homing
            waiting = false;
            homingState = nextState;

            return HOMING_RUNNING;
        }

        // If an unexpected limit switch is press
        // For example if we are moving down we only expected bottom limit switch to be hit
        // So if any of the three others are hit, then we return HOMING_FAULT to FSM, and in the 
        // FSM it changes states from homing to FAULT
        if (invalidSwitchPressed) {
            stopMotors();

            moveActive = false;
            moveStarted = false;

            return HOMING_FAULT;
        }

        // If the switch we were aiming for is pressed, set waiting to true and stop moving
        if (targetSwitchPressed) {
            moveActive = false;
            moveStarted = false;

            stopMotors();

            waiting = true;

            return HOMING_RUNNING;
        }

        // If it is still currently moving towards the target limit switch
        move_FSM(x, y, feedrate);
        return HOMING_RUNNING;
    }

// Hits the target limit switch and now has to back off it
static HomingResult backoffMove(int x, int y, int feedrate, bool invalidSwitchPressed, HomingState nextState) {
        
        // If a move is finished and we are waiting for encoders to stop moving slightly
        if (waiting) {
            stopMotors();           // Halts all movement

            // Encoders are still moving slightly so return without changing 'waiting' variable
            if (encoderCountsChanged()) {
                return HOMING_RUNNING;
            }

            waiting = false;
            homingState = nextState;

            // If encoder count is not changing and there are no more homing states 
            // Return HOMING_COMPLETE to FSM to switch states to IDLE
            if (nextState == HOMING_DONE) {
                return HOMING_COMPLETE;
            }

            return HOMING_RUNNING;
        }

        // If an unexpected limit switch is press
        // For example if we are moving down we only expected bottom limit switch to be hit
        // So if any of the three others are hit, then we return HOMING_FAULT to FSM, and in the 
        // FSM it changes states from homing to FAULT
        if (invalidSwitchPressed) {
            stopMotors();

            moveActive = false;
            moveStarted = false;

            return HOMING_FAULT;
        }

        // If it is still currently moving away from limit switch
        move_FSM(x, y, feedrate);

        // If the movement is finished then switch variable to waiting which will halt movement 
        // until motors settle
        if (moveFinished) {
            waiting = true;
        }

        return HOMING_RUNNING;
    }

// Enters from FSM, Parsing --> Homing
// Sets the initial homing state to HOMING_DOWN_FAST and all variables to false
void homingStart() {
    stopMotors();

    homingState = HOMING_DOWN_FAST;

    waiting = false;

    moveActive = false;
    moveStarted = false;
    moveFinished = false;
}

// Called from FSM, Homing --> Homing
// Runs through all possible states and calls their respective movement functions
HomingResult homingUpdate() {
    switch (homingState)
    {

    case HOMING_DOWN_FAST:
    {
        return moveToLimit(
            0,
            -homingTravel,
            homingFeedrate,

            LimitSwitch_bottomPressed(),

            LimitSwitch_leftPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_topPressed(),

            HOMING_BACKOFF_UP_1);
    }

    // First move off bottom limit switch
    case HOMING_BACKOFF_UP_1:
    {
        return backoffMove(
            0,
            homingBackoffmm,
            homingFeedrate,

            LimitSwitch_leftPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_topPressed(),

            HOMING_DOWN_SLOW);
    }

    // Secondary downwards movement towards bottom limit switch (slow)
    case HOMING_DOWN_SLOW:
    {
        return moveToLimit(
            0,
            -homingTravel,
            slowHomingFeedrate,

            LimitSwitch_bottomPressed(),

            LimitSwitch_leftPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_topPressed(),

            HOMING_BACKOFF_UP_2);
    }

    // Second move up from bottom limit switch (slow)
    case HOMING_BACKOFF_UP_2:
    {
        return backoffMove(
            0,
            homingBackoffmm,
            slowHomingFeedrate,

            LimitSwitch_leftPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_topPressed(),

            HOMING_LEFT_FAST);
    }

    case HOMING_LEFT_FAST:
    {
        return moveToLimit(
            -homingTravel,
            0,
            homingFeedrate,

            LimitSwitch_leftPressed(),

            LimitSwitch_topPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_bottomPressed(),

            HOMING_BACKOFF_RIGHT_1);
    }

    // First move off left limit switch
    case HOMING_BACKOFF_RIGHT_1:
    {
        return backoffMove(
            homingBackoffmm,
            0,
            homingFeedrate,

            LimitSwitch_topPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_bottomPressed(),

            HOMING_LEFT_SLOW);
    }

    // Second move towards left limit switch (slow)
    case HOMING_LEFT_SLOW:
    {
        return moveToLimit(
            -homingTravel,
            0,
            slowHomingFeedrate,

            LimitSwitch_leftPressed(),

            LimitSwitch_topPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_bottomPressed(),

            HOMING_BACKOFF_RIGHT_2);
    }

    // Second move away from left limit switch (slow)
    case HOMING_BACKOFF_RIGHT_2:
    {
        return backoffMove(
            homingBackoffmm,
            0,
            slowHomingFeedrate,

            LimitSwitch_topPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_bottomPressed(),

            HOMING_DONE);
    }

        // If the homing state is set to HOMING_DONE after the last homing move away from 
        // the left switch, and the encoder count has not changed, the homing movment is done.
        // Will return HOMING_COMPLETE to the FSM and then the FSM will switch to idle
        case HOMING_DONE:
        {
            stopMotors();
            return HOMING_COMPLETE;
        }
    }
    stopMotors();

    moveActive = false;
    moveStarted = false;
    moveFinished = true;

    // Wrong limit switch has been hit.
    // Return HOMING_FAULT to FSM, then FSM will change state to FAULT state
    return HOMING_FAULT;
}