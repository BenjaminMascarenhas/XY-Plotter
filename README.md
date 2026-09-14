# CoreXY Closed-Loop Plotter

A 2-axis CoreXY plotter built around an Arduino Mega 2560, developed for MECHENG 306 (Design of Sensing and Actuating Systems). Implements G-code motion parsing, non-blocking finite state machine control, per-motor PID, and a from-scratch S-curve trajectory profiler - all original implementation, no third-party control libraries.

## Overview

The plotter accepts a subset of G-code over serial (`G28` homing, `G1` linear moves) and executes coordinated two-axis motion using CoreXY kinematics, closed-loop PID control on quadrature encoder feedback, and smooth S-curve velocity ramping.

## Key Features

- **CoreXY kinematics** - belt-driven XY motion decoupled via `dA = dX + dY`, `dB = dX − dY`
- **Non-blocking FSM** - `IDLE`, `HOMING`, `MOVING`, `FAULT` states with no `delay()` calls anywhere in the control path
- **S-curve motion profiling** - full 7-segment velocity profile, converted from time-based to distance-based using live encoder measurement. Derived from scratch: cube root solution for the initial ramp stage, the quadratic formula for the constant-jerk stage, and Cardano's formula for the final stage
- **Per-motor PID** - ratio-based synchronization comparing each motor's actual-to-target velocity ratio, keeping both axes in step during coordinated moves
- **Quadrature encoder handling** - interrupt-driven position tracking; velocity computed from encoder count deltas at a fixed 50 Hz control loop tick (deliberately outside the ISR, to keep interrupt handlers short)
- **Software limit switch debouncing** and fault detection - unexpected limit trigger during a move halts motion and drops the system into `FAULT`

## Technical Decisions

- Velocity is normalized against Euclidean path length so diagonal moves ramp consistently with axis-aligned ones
- Jerk is derived from commanded velocity and a tunable `RAMP_TIME_S`, rather than hardcoded
- Distance tracking uses an incremental dx/dy accumulator rather than absolute position, to stay robust to any single bad encoder read

## Contributions

This was a group project (MECHENG 306). Individual contribution breakdown:

- Solely implemented the PID control system, S-curve velocity profiling, and quadrature encoder logic. Also contributed significantly to debugging, and helped write the G-code parsing and FSM implementation.

## Hardware

- Arduino Mega 2560
- 2x DC servo motors with integrated quadrature encoders
- DFRobot L298P motor shield (used only for pin/direction abstraction — motor control logic is original)
- 4x mechanical limit switches

## Building / Running

1. Open `firmware/PID_FSM.cpp` in the Arduino IDE
2. Select Arduino Mega 2560 as the target board
3. Upload, then connect via Serial Monitor (or any serial terminal) at the configured baud rate
4. Send `G28` to home, then `G1 X.. Y..` commands for linear moves