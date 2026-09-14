#include "LimitSwitch.h"
#include <FSM.h>
#include <avr/interrupt.h>

// Pin layout
static const int LEFT_LIMIT_PIN = 12;
static const int RIGHT_LIMIT_PIN = 13;
static const int TOP_LIMIT_PIN = 11;
static const int BOTTOM_LIMIT_PIN = 10;

// Limit switch debounce time
static const unsigned long DEBOUNCE_TIME_MS = 10;

static volatile bool leftState = LOW;
static volatile bool rightState = LOW;
static volatile bool topState = LOW;
static volatile bool bottomState = LOW;

// Previous ege change of each limit switch
static volatile unsigned long leftLastEdgeTime = 0;
static volatile unsigned long rightLastEdgeTime = 0;
static volatile unsigned long topLastEdgeTime = 0;
static volatile unsigned long bottomLastEdgeTime = 0;

static volatile bool limitSwitchTriggered = false;

// Checks which limit switch has been pressed
static void checkLimit(int pin, volatile bool &switchState, volatile unsigned long &lastEdgeTime) {
    // Gets the current time
    unsigned long now = millis();

    // If the current time gap is not greater then the last time the switch changed edge then, exit
    if ((now - lastEdgeTime) < DEBOUNCE_TIME_MS) {
        return;
    }

    // Read the pin connected to each limit switch
    bool newState = digitalRead(pin);

    // If the state is still the same, exit
    if (newState == switchState) {
        return;
    }

    // Limit switch has been pressed, reset time for debouncing and change the state of 
    // the specific pin which has called the function, it using a reference, so we know which 
    // switch has been pressed
    switchState = newState;
    lastEdgeTime = now;

    // If the current state is has been changed to high and not in homing then enter FAULT STATE
    if (switchState == HIGH) {
        if (FSM_getCurrentState() != STATE_HOMING) {
            FSM_triggerFault();
        }
    }
}

// 
void updateLimits() {
    // If none of the limit switch's are pressed, exit
    if (!limitSwitchTriggered) {
        return;
    }

    // Toggle variable to false
    limitSwitchTriggered = false;

    // Calls functions to check which limit switch has been pressed
    checkLimit(LEFT_LIMIT_PIN, leftState, leftLastEdgeTime);
    checkLimit(RIGHT_LIMIT_PIN, rightState, rightLastEdgeTime);
    checkLimit(TOP_LIMIT_PIN, topState, topLastEdgeTime);
    checkLimit(BOTTOM_LIMIT_PIN, bottomState, bottomLastEdgeTime);
}

// Limit switch setup
void setupLimitSwitches() {
    pinMode(LEFT_LIMIT_PIN, INPUT);
    pinMode(RIGHT_LIMIT_PIN, INPUT);
    pinMode(TOP_LIMIT_PIN, INPUT);
    pinMode(BOTTOM_LIMIT_PIN, INPUT);

    leftState = digitalRead(LEFT_LIMIT_PIN);
    rightState = digitalRead(RIGHT_LIMIT_PIN);
    topState = digitalRead(TOP_LIMIT_PIN);
    bottomState = digitalRead(BOTTOM_LIMIT_PIN);

    if (leftState || rightState || topState || bottomState) {
        FSM_triggerFault();
    }

    unsigned long now = millis();

    leftLastEdgeTime = now - DEBOUNCE_TIME_MS;
    rightLastEdgeTime = now - DEBOUNCE_TIME_MS;
    topLastEdgeTime = now - DEBOUNCE_TIME_MS;
    bottomLastEdgeTime = now - DEBOUNCE_TIME_MS;

    cli();

    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);
    PCMSK0 |= (1 << PCINT6);
    PCMSK0 |= (1 << PCINT7);

    PCIFR |= (1 << PCIF0);
    PCICR |= (1 << PCIE0);

    sei();
}

bool LimitSwitch_leftPressed() {
    return leftState;
}

bool LimitSwitch_rightPressed() {
    return rightState;
}

bool LimitSwitch_topPressed() {
    return topState;
}

bool LimitSwitch_bottomPressed() {
    return bottomState;
}

// ISR for all the limit switch's as they are all connected to the same ISR
ISR(PCINT0_vect) {
    limitSwitchTriggered = true;
}
