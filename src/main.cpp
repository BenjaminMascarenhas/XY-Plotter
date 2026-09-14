#include <HelperFunctions.h>
#include <LimitSwitch.h>
#include <Motor.h>
#include <Parsing.h>
#include <Encoder.h>
#include <PID_FSM.h>
#include <Arduino.h>
#include <FSM.h>

void setup()
{
    Serial.begin(115200); // no other file should contain this line

    /* insert initialisation functions below  */
    Encoder_Init();
    FSM_init();
    setupLimitSwitches();
    Motor_Init();
    initialiseTokenArray();

}

void loop()
{
    FSM_update();
}

// 1 is cw, 0 is stop, -1 is ccw