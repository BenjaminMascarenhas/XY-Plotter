#include "FSM.h"
#include <Encoder.h>
#include <HelperFunctions.h>
#include <Homing.h>
#include <Parsing.h>
#include <GcodeToken.h>
#include <LimitSwitch.h>
#include <PID_FSM.h>
#include <Motor.h>
#include <Graph.h>
static volatile SystemState currentState = STATE_IDLE; // System begins in IDLE state
// static MotionSubstate motionState = MOTION_ACCEL;   // Motion_state begins in Acceloration

static SystemState lastPrintedState = (SystemState)-1;

// elapsed time tracking for moves (millis at move start). 0 when no move running.
static unsigned long elapsed_from_move_start = 0;
static bool _prevMoveStarted = false;

// Initialises current state to idle.
void FSM_init()
{
    currentState = STATE_IDLE;
}

// Idle can only go to parsing or stay in idle
// Idle --> Parsing. Event: Serial buffer recieves input
static void handleIdle()
{
    if (Serial.available() > 0)
    {
        currentState = STATE_PARSING;
    }
}

// Parsing can go to idle, homing or motion, or stay in idle
// Parsing --> Parsing. Event: User has not yet pressed enter '\n'
// Parsing --> Idle. Event: After user presses enter, command is invalid
// Parsing --> Homing. Event: If command is valid and G value is 28
// Parsing --> Motion. Event: If all above do not occur
static void handleParsing()
{
    int err = readLine();
    if (err == 2)
    {
        // Stay in Parsing
        return;
    }

    // Check if command was invalid
    if (err == 1)
    {
        resetTokenArray();
        currentState = STATE_IDLE;
        return;
    }

    int value = (Parsing_getToken(G)).GetValue(); // returns value of G token

    if (value == 28)
    {
        currentState = STATE_HOMING;
        homingStart();
        return;
    }

    // other value G01 therfore, motion
    currentState = STATE_MOTION;
    return;
}

// Motion can go to either fault or idle, or stay in motion
// Motion --> Fault. Event: If it hits a limit switch it goes to fault state
// Motion --> Idle. Event: If the move is completed go to idle
// Motion --> Motion. Event: If the moveFinished state is still false (plotter still moving)
static void handleMotion()
{
    long x = Parsing_getToken(X).GetValue();
    long y = Parsing_getToken(Y).GetValue();
    long vf = Parsing_getToken(F_token).GetValue();

    // start timer when moveStarted transitions from false -> true
    if (moveStarted && !_prevMoveStarted)
    {
        elapsed_from_move_start = millis();
    }
    _prevMoveStarted = moveStarted;

    // Compute safe elapsed (0 if move hasn't started)
    unsigned long elapsed_ms = (elapsed_from_move_start != 0) ? (millis() - elapsed_from_move_start) : 0UL;

    moveCurrentLeftCount = Encoder_getLeftEncoderCount();
    moveCurrentRightCount = Encoder_getRightEncoderCount();

    currentLeftMM = countsToDistance(moveCurrentLeftCount);
    currentRightMM = countsToDistance(moveCurrentRightCount);

    currentX = (currentLeftMM + currentRightMM) / 2.0;
    currentY = (currentLeftMM - currentRightMM) / 2.0;

    static unsigned long lastGraphTime = 0;

    double actualPathVelocity =
        sqrt(
            pow((velocity_left + velocity_right) / 2.0, 2) +
            pow((velocity_left - velocity_right) / 2.0, 2));

    if (elapsed_ms - lastGraphTime >= 50)
    {
        lastGraphTime = elapsed_ms;
        //addDataPoint(x - currentX, y - currentY, elapsed_ms);
        addDataPoint(actualPathVelocity, V, millis() - elapsed_from_move_start);
    }

    if (moveFinished == false)
    {
        move_FSM(x, y, vf);
    }

    if (currentState == STATE_FAULT)
    {
        moveStarted = false;
        moveActive = false;
        // clear elapsed tracking on fault
        elapsed_from_move_start = 0;
        _prevMoveStarted = false;
        return;
    }

    if (moveActive == false)
    {
        stopMotors();

        // Waits for the encoder counts to settle and not change
        if (encoderCountsChanged() && elapsed_from_move_start < 200000)
        {
            return;
        }

        // Serial.println("Total horizontal distance travelled: " + String(currentX) + " mm");
        // Serial.println("Total vertical distance travelled: " + String(currentY) + " mm");

        exportData();
        clearData();

        resetTokenArray();
        moveStarted = false;
        moveFinished = false;

        // clear elapsed tracking when move completes
        elapsed_from_move_start = 0;
        _prevMoveStarted = false;

        currentState = STATE_IDLE;
        lastGraphTime = 0;
        // exportData();
        // clearData();
        return;
    }
}

// Homing can go to either fault or idle, or stay in motion
// Homing --> Fault. Event: If it hits a limit switch that it is not meant to hit at that time
// Homing --> Idle. Event: If the homing is completed
static void handleHoming()
{
    HomingResult result = homingUpdate();

    if (result == HOMING_FAULT)
    {
        currentState = STATE_FAULT;
        return;
    }

    if (result == HOMING_COMPLETE)
    {
        if (encoderCountsChanged())
        {
            return;
        }

        Encoder_setLeftGlobalEncoderCountZero();
        Encoder_setRightGlobalEncoderCountZero();

        moveActive = false;
        moveStarted = false;
        moveFinished = false;

        currentState = STATE_IDLE;
        resetTokenArray();

        return;
    }
}

// Fault can go to either fault or idle, or stay in motion
// Fault --> Fault. Event: If user is still typing a command, the command after entered is invalid
//                         or it is a valid M999 but limit switch is still being pressed
// Fault --> Idle. Event: M999 entered and not touching a limit switch
static void handleFault()
{
    moveActive = false;
    moveStarted = false;
    stopMotors();

    if (Serial.available() > 0)
    {

        int err = readLine();

        if (err == 2)
        {
            // Still reading command (stay in fault)
            currentState = STATE_FAULT;
            return;
        }
        else if (err == 1)
        {
            // Command invalid (stay in fault)
            resetTokenArray();
            currentState = STATE_FAULT;
            return;
        }

        GcodeToken token = Parsing_getToken(M);
        if (token.GetLetter() == 'M' && token.GetValue() == 999)
        {
            // M999 entered but still on limit switch
            if (LimitSwitch_leftPressed() || LimitSwitch_rightPressed() || LimitSwitch_topPressed() || LimitSwitch_bottomPressed())
            {
                resetTokenArray();
                Serial.println("FAULT");
                return;
            }

            resetTokenArray();
            currentState = STATE_IDLE; // M999 entered and not touching a limit switch
            return;
        }
    }
}

static void printStateIfChanged()
{
    return;
    if (currentState == lastPrintedState)
    {
        return;
    }

    lastPrintedState = currentState;

    switch (currentState)
    {
    case STATE_IDLE:

        Serial.println("IDLE");
        // Serial.println((globalCountA + globalCountB)/2);
        // Serial.println((globalCountA - globalCountB)/2);

        /*
        Serial.println(" ");
        int dx = (countsToDistance(countA) + countsToDistance(countB))/2;
        int dy = (countsToDistance(countA) - countsToDistance(countB))/2;
        Serial.print("X = ");
        Serial.println(dx);
        Serial.print("Y = ");
        Serial.println(dy);*/
        break;

    case STATE_PARSING:
        Serial.println("PARSING");
        break;

    case STATE_MOTION:
        Serial.println("MOTION");
        break;

    case STATE_HOMING:
        Serial.println("HOMING");
        break;

    case STATE_FAULT:
        Serial.println("FAULT");
        break;
    }
}

void FSM_update()
{
    updateLimits();

    // Call the handler for the current state (non-blocking)
    switch (currentState)
    {
    case STATE_IDLE:
        handleIdle();
        break;
    case STATE_PARSING:
        handleParsing();
        break;
    case STATE_MOTION:
        handleMotion();
        break;
    case STATE_HOMING:
        handleHoming();
        break;
    case STATE_FAULT:
        handleFault();
        break;
    }

    printStateIfChanged();
}

SystemState FSM_getCurrentState()
{
    return currentState;
}

void FSM_triggerFault()
{
    currentState = STATE_FAULT;
}
