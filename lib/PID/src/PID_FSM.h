#ifndef PID_FSM_H
#define PID_FSM_H

#include <Arduino.h>

// Motion profile helpers
double velocityProfile_FSM(double J, double Vf, double t4, double t);
void plan_FSM(double x, double y, double vf_target);
void move_FSM(int x, int y, int vf);

// Shared motion state used by the control loop
extern double velocity_left;
extern double velocity_right;
extern double moveJ;
extern double moveVf;
extern double moveT4;
extern double moveUnitX;
extern double moveUnitY;
extern double moveStartTime;
extern bool moveActive;
extern bool triangleProfile;
extern bool moveStarted;

extern double kp_left;
extern double ki_left;
extern double kd_left;
extern double integral_left;
extern double lastError_left;

extern double kp_right;
extern double ki_right;
extern double kd_right;
extern double integral_right;
extern double lastError_right;

extern double kp_sync;
extern double ki_sync;
extern double kd_sync;
extern double integral_sync;
extern double lastError_sync;

extern bool moveFinished;

extern double currentX;
extern double currentY;

extern double currentX;
extern double currentY;

extern double currentLeftMM;
extern double currentRightMM;

extern long moveCurrentLeftCount;
extern long moveCurrentRightCount;

extern double velocity_left;
extern double velocity_right;

extern double V;

#endif // PID_FSM_H
