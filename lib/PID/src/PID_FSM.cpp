#include <Arduino.h>
#include <avr/interrupt.h>
#include <LimitSwitch.h>

#include <Encoder.h>
#include "PID_FSM.h"
#include <Motor.h>
#include <HelperFunctions.h>
#include <Graph.h>

#include <FSM.h>

// Time Constant Variables
double T1 = 0.2;
double T2 = 0.4;
double TA = T1 + T2 + T1;

// Movement Variables
double moveJ = 0;  // Jerk required for movement based on T1, T2 and final velocity
double moveVf = 0; // Final movement speed
double moveT4 = 0; // Time at final velocity
double moveUnitX = 0;
double moveUnitY = 0;
double moveStartTime = 0;

double currentX = 0;
double currentY = 0;

double currentLeftMM = 0;
double currentRightMM = 0;

// Movement Active Variables
bool moveActive = false;
bool moveStarted = false;

// If T4 is negative it becomes a triangle profile as it doesnt have the time to accelerate to final velocity
bool triangleProfile = false;

// Left Motor PID Variables
double kp_left = 30, /*18*/ ki_left = 10, /*2*/ kd_left = 0, kff_left = 10; // Revert kff to 5.5 if doesnt work
double integral_left = 0, lastError_left = 0;

// Right Motor PID Variables
double kp_right = 30, /*18*/ ki_right = 10, /*2*/ kd_right = 0, kff_right = 10; // Revert kff to 5.5 if doesnt work
double integral_right = 0, lastError_right = 0;

// Sync PID Variables
double kp_sync = 0, ki_sync = 0, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;

// Time Control Variables
uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 20000;        // 50Hz
const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0; // Seconds per control loop tick

// Motor Direction Variables
int8_t leftDir;
int8_t rightDir;

// Encoder tracking
long moveCurrentLeftCount = 0;
long moveCurrentRightCount = 0;

long moveTargetLeftCount = 0;
long moveTargetRightCount = 0;

const double tolerance_mm = 0.5;           // Stops when it reaches within 0.2mm of target
unsigned long elapsed_from_move_start = 0; // Tracks how long since movement started

int integral_maxPWM = 100; // Anti-integral windup term to keep integral from accumulating

// NEEDS TUNING
// MIN_DRIVE_PWM moved to HelperFunctions.h / HelperFunctions.cpp
// (previous local definition removed)

bool xTargetReached = false; // If X-axis has reached target
bool yTargetReached = false; // If Y-axis has reached target

bool moveFinished = false;

// Target velocities for left and right motors
double targetLeft;
double targetRight;

double velocity_left = 0;
double velocity_right = 0;

Timer recordTimer;

double V = 0; // Target velocity at any given time
// Returns the target velocity at different stages in the movement, using an s-curve profile.
double velocityProfile_FSM(double J, double Vf, double t4, double t)
{
    double V1 = 0.5 * J * T1 * T1; // Velocity after stage 1
    double V2 = J * T1 * T2;       // Velocity after stage 2

    if (t < T1)
    { // Stage 1
        return 0.5 * J * t * t;
    }
    else if (t < T1 + T2)
    { // Stage 2
        double tau = t - T1;
        return V1 + (J * T1) * tau;
    }
    else if (t < TA)
    { // Stage 3
        double tau = t - (T1 + T2);
        return V1 + V2 + (J * T1) * tau - 0.5 * J * tau * tau;
    }
    else if (t < TA + t4)
    { // Stage 4
        return Vf;
    }
    else if (t < TA + t4 + T1)
    { // Stage 5
        double tau = t - (TA + t4);
        return Vf - 0.5 * J * tau * tau;
    }
    else if (t < TA + t4 + T1 + T2)
    { // Stage 6
        double tau = t - (TA + t4 + T1);
        return Vf - V1 - (J * T1) * tau;
    }
    else if (t < TA + t4 + TA)
    { // Stage 7
        double tau = t - (TA + t4 + T1 + T2);
        return Vf - V1 - V2 - (J * T1) * tau + 0.5 * J * tau * tau;
    }
    else
    {
        // Move finished
        return 0;
    }
}

// Initial speed plan to find distance, time at constant velocity and if it is a triangular profile.
void plan_FSM(double x, double y, double vf_target)
{
    vf_target = vf_target / 60.0;   // Converts the mm/min to mm/s
    double S = sqrt(x * x + y * y); // Finds straight line distance

    // Resets encoder counts to zero
    Encoder_setLeftEncoderCountZero();
    Encoder_setRightEncoderCountZero();

    // If distance is zero return
    if (S <= 0.0)
    {
        moveActive = false;
        moveStarted = false;
        return;
    }

    // Direction of travel (x and y as fractions of total distance, magnitude removed)
    // Unit vectors
    moveUnitX = x / S;
    moveUnitY = y / S;

    // Distance covered in one ramp time
    double k = T1 + T2 / 2;

    double vf;
    double t4;

    // If distance of ramp up and down is greater then distance
    // This means it will never reach cruise velocity and need to implement a triangular profile
    if (S < 2.0 * k * vf_target)
    {
        vf = S / (2.0 * k); // New final velocity
        t4 = 0.0;           // No time cruising at final velocity
        triangleProfile = true;
    }
    else
    {
        vf = vf_target;
        t4 = (S - 2.0 * k * vf) / vf; // Time at final velocity calculation
        triangleProfile = false;
    }

    // Main variables to calculate target speed at any time
    moveVf = vf;
    moveJ = moveVf / (T1 * (T1 + T2));
    moveT4 = t4;

    // Movement start time set
    moveStartTime = millis() / 1000.0;
    moveActive = true;
}

// Main PID loop.
void move_FSM(int x, int y, int vf)
{
    // If movement has not started yet we need to reset all variables and call movement plan
    if (moveStarted == false)
    {
        elapsed_from_move_start = millis(); // Reset the elapsed time from the start of the movement
        moveStarted = true;
        moveFinished = false;

        // Ensuring variables are zeroed
        integral_left = 0;
        lastError_left = 0;

        integral_right = 0;
        lastError_right = 0;

        integral_sync = 0;
        lastError_sync = 0;

        // Call movement plan to find jerk and t4
        plan_FSM(x, y, vf);
        if (!moveActive)
        {
            return;
        }

        // Sets last loop time to the present
        lastControlLoopMicros = micros();

        // Zeros the encoders
        moveCurrentLeftCount = Encoder_getLeftEncoderCount();
        moveCurrentRightCount = Encoder_getRightEncoderCount();

        // Finds the target encoder count for both motors
        moveTargetLeftCount = moveCurrentLeftCount + distanceToCounts(x) + distanceToCounts(y);

        moveTargetRightCount = moveCurrentRightCount + distanceToCounts(x) - distanceToCounts(y);
    }

    // Gets the current encoder count
    moveCurrentLeftCount = Encoder_getLeftEncoderCount();
    moveCurrentRightCount = Encoder_getRightEncoderCount();

    // Ensures there is a constant frequency of 50Hz which we are using for the PID loop as to not
    // call the motors too frequently.
    uint32_t nowMicros = micros();

    if (nowMicros - lastControlLoopMicros < CONTROL_LOOP_INTERVAL_US)
    {
        return;
    }

    lastControlLoopMicros += CONTROL_LOOP_INTERVAL_US; // Sets new last loop time

    // Gets both motor velocities
    velocity_left = Encoder_getLeftVelocity(dt);
    velocity_right = Encoder_getRightVelocity(dt);

    // Gets current time in reference to start time
    double t = (millis() / 1000.0) - moveStartTime;

    // Sees what the velocity should be at the moment
    V = velocityProfile_FSM(moveJ, moveVf, moveT4, t);

    // Breaks total veloity into velocity needed from each motor
    targetLeft = V * (moveUnitX + moveUnitY);
    targetRight = V * (moveUnitX - moveUnitY);

    // Converts current encoder counts to mm
    double currentLeftMM = countsToDistance(moveCurrentLeftCount);
    double currentRightMM = countsToDistance(moveCurrentRightCount);

    // Converts mm moved by each motor to the actual x and y movement
    currentX = (currentLeftMM + currentRightMM) / 2.0;
    currentY = (currentLeftMM - currentRightMM) / 2.0;

    // Calculates the distance error in terms of x and y coordinates
    double xPositionError_mm = x - currentX;
    double yPositionError_mm = y - currentY;

    // Checks to see if both the x and y coordinates have moved the desired amount
    if (!xTargetReached)
    {
        xTargetReached = abs(xPositionError_mm) <= tolerance_mm;
    }
    if (!yTargetReached)
    {
        yTargetReached = abs(yPositionError_mm) <= tolerance_mm;
    }

    // Has reached target before time
    if (xTargetReached && yTargetReached)
    {
        // Sets all variables to show movement is finished
        moveActive = false;
        moveStarted = false;
        moveFinished = true;

        // Stops motors
        stopMotors();

        // Serial.println("Vf: " + String(moveVf));

        // Sets target reached to false so it does not interfere with next movement
        yTargetReached = false;
        xTargetReached = false;

        return;
    }

    // Timeout when movement stops if the total projected run time has elapsed
    if (t >= TA + moveT4 + TA)
    {
        // Sets all variables to show movement is finished
        moveActive = false;
        moveStarted = false;
        moveFinished = true;

        // Stops motors
        stopMotors();

        // Serial.println("Move timed out, stopping motors.");
        // Serial.println("Total horizontal distance travelled: " + String(currentX) + " mm");
        // Serial.println("TotSal vertical distance travelled: " + String(currentY) + " mm");
        // Serial.println("Target Velocity: " + String(moveVf));
        // Serial.println("Target Distance: " + String(sqrt(x*x + y*y)));

        // Sets target reached to false so it does not interfere with next movement
        yTargetReached = false;
        xTargetReached = false;

        return;
    }

    // left velocity PID
    double error_left = targetLeft - velocity_left;

    if (abs((integral_left + error_left * dt) * ki_left) < integral_maxPWM)
    {
        integral_left += error_left * dt;
    }

    double derivative_left = (error_left - lastError_left) / dt;
    double outputLeft = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
    lastError_left = error_left;
    // Serial.println("Left Integral Term: " + String(integral_left*ki_left));
    // Serial.println("Left Proportional Term: " + String(error_left*kp_left));

    // right velocity PID
    double error_right = targetRight - velocity_right;

    if (abs((integral_right + error_right * dt) * ki_right) < integral_maxPWM)
    {
        integral_right += error_right * dt;
    }

    double derivative_right = (error_right - lastError_right) / dt;
    double outputRight = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;
    lastError_right = error_right;
    // Serial.println("Right Integral Term: " + String(integral_right*ki_right));
    // Serial.println("Right Proportional Term: " + String(error_right*kp_right));

    // sync PID
    double ratioLeft = (targetLeft != 0.0) ? (velocity_left / targetLeft) : 1.0;
    double ratioRight = (targetRight != 0.0) ? (velocity_right / targetRight) : 1.0;
    double percentError = (ratioLeft - ratioRight) * 100.0;
    integral_sync += percentError * dt;
    double derivative_sync = (percentError - lastError_sync) / dt;
    double syncCorrection = kp_sync * percentError + ki_sync * integral_sync + kd_sync * derivative_sync;
    lastError_sync = percentError;

    // Direction the motors need to spin
    int leftTargetDir = 0;
    int rightTargetDir = 0;

    if (targetLeft > 0)
        leftTargetDir = 1;
    else if (targetLeft < 0)
        leftTargetDir = -1;

    if (targetRight > 0)
        rightTargetDir = 1;
    else if (targetRight < 0)
        rightTargetDir = -1;

    // Feedforward calculations
    double feedforwardLeft = staticFeedforward(targetLeft, kff_left);
    double feedforwardRight = staticFeedforward(targetRight, kff_right);

    // Final output
    double finalOutputLeft = outputLeft - syncCorrection * leftTargetDir + feedforwardLeft;
    double finalOutputRight = outputRight + syncCorrection * rightTargetDir + feedforwardRight;

    leftDir = outputToDirection(finalOutputLeft);
    rightDir = outputToDirection(finalOutputRight);

    // Sets the desired pwm of the motors
    int leftPWM = (int)abs(finalOutputLeft);
    int rightPWM = (int)abs(finalOutputRight);

    // Motor movement
    String leftSuccess = setLeftMotor(leftDir, leftPWM);
    String rightSuccess = setRightMotor(rightDir, rightPWM);

    double actualPathVelocity =
        sqrt(
            pow((velocity_left + velocity_right) / 2.0, 2) +
            pow((velocity_left - velocity_right) / 2.0, 2));
    SystemState state = FSM_getCurrentState();

    if (state == STATE_MOTION)
    {
        // addDataPoint(actualPathVelocity, V, millis() - elapsed_from_move_start);
    }

    // Serial.println(leftPWM);
    // Serial.println(rightPWM);

    // Prints to serial monitor the current velocity
    // double actualPathVelocity = sqrt(pow((velocity_left + velocity_right) / 2.0, 2) + pow((velocity_left - velocity_right) / 2.0, 2));
    // Serial.println("Actual Velocity: " + String(actualPathVelocity));
}