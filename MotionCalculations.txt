# S-Curve Motion Profile — Derivation

This document derives the 7-stage S-curve velocity profile used in `velocityProfile_FSM()` and `plan_FSM()`, and explains where each constant in the code comes from.

## 1. Profile Shape

The move is split into 7 time stages, built from two fixed time constants:

- $T_1$ — jerk ramp time (accelerating/decelerating jerk phase)
- $T_2$ — constant-jerk-free acceleration phase (jerk = 0, acceleration constant)
- $T_A = T_1 + T_2 + T_1$ — total time to go from rest to cruise velocity (and, by symmetry, from cruise back to rest)

| Stage | Duration | Motion |
|---|---|---|
| 1 | $0 \to T_1$ | Jerk ramps up, velocity increases with $t^2$ |
| 2 | $T_1 \to T_1+T_2$ | Constant acceleration, velocity increases linearly |
| 3 | $T_1+T_2 \to T_A$ | Jerk ramps down, velocity approaches $V_f$ |
| 4 | $T_A \to T_A+t_4$ | Cruise at constant velocity $V_f$ |
| 5–7 | mirror of 1–3 | Symmetric deceleration back to 0 |

## 2. Stage 1 — Jerk Ramp Up

Jerk $J$ is constant and positive. Starting from rest:

$$a(t) = J t$$
$$v(t) = \frac{1}{2} J t^2$$

At the end of Stage 1 ($t = T_1$):

$$V_1 = \frac{1}{2} J T_1^2$$

This matches the code:
```cpp
double V1 = 0.5 * J * T1 * T1;
```

## 3. Stage 2 — Constant Acceleration

Jerk is zero here, so acceleration is held at its Stage-1 peak value, $a = J T_1$. Velocity increases linearly from $V_1$:

$$v(\tau) = V_1 + (J T_1)\tau, \quad \tau = t - T_1$$

At the end of Stage 2 ($\tau = T_2$), the additional velocity gained is:

$$V_2 = J T_1 T_2$$

Matching the code:
```cpp
double V2 = J * T1 * T2;
```

## 4. Stage 3 — Jerk Ramp Down

Jerk is now negative ($-J$), decelerating the acceleration back to zero as velocity approaches $V_f$:

$$v(\tau) = V_1 + V_2 + (J T_1)\tau - \frac{1}{2} J \tau^2, \quad \tau = t - (T_1 + T_2)$$

At $\tau = T_1$, this should equal $V_f$ exactly — this is what defines the relationship between $J$, $V_f$, $T_1$, and $T_2$ used in `plan_FSM()`.

## 5. Solving for Jerk Given a Target Velocity

Setting the Stage 3 endpoint equal to $V_f$:

$$V_f = V_1 + V_2 + (J T_1) T_1 - \frac{1}{2} J T_1^2$$

$$V_f = \frac{1}{2} J T_1^2 + J T_1 T_2 + J T_1^2 - \frac{1}{2} J T_1^2$$

$$V_f = J T_1^2 + J T_1 T_2$$

$$V_f = J T_1 (T_1 + T_2)$$

Solving for $J$:

$$J = \frac{V_f}{T_1 (T_1 + T_2)}$$

This is exactly:
```cpp
moveJ = moveVf / (T1 * (T1 + T2));
```

## 6. Stages 4–7

- **Stage 4**: velocity holds at $V_f$ for duration $t_4$ (the cruise phase)
- **Stages 5–7**: mirror image of Stages 1–3, decelerating back to zero. Each stage subtracts the same terms that were added on the way up, which is why the code reuses $V_1$, $V_2$, and $J T_1$ with sign flips.

## 7. Distance Covered During Ramp (Triangle Profile Check)

Before running a move, `plan_FSM()` needs to know whether the plotter has enough distance to actually reach cruise velocity $v_{f,\text{target}}$, or whether it must decelerate before getting there (a "triangle" profile with no Stage 4 cruise).

The distance covered during just the ramp-up (Stages 1–3) can be shown to be:

$$d_{\text{ramp}} = k V_f, \quad k = T_1 + \frac{T_2}{2}$$

By symmetry, ramp-down covers the same distance, so total ramp distance for a full trapezoidal move is $2k V_f$.

**Triangle condition:** if the total path distance $S$ is less than the distance needed to ramp up and back down at the target velocity:

$$S < 2k \cdot v_{f,\text{target}}$$

...there's no room for a cruise phase. In that case, solve for the *peak* velocity actually reachable over distance $S$:

$$v_f = \frac{S}{2k}, \quad t_4 = 0$$

This matches the code:
```cpp
if (S < 2.0 * k * vf_target) {
    vf = S / (2.0 * k);
    t4 = 0.0;
    triangleProfile = true;
}
```

Otherwise, the plotter reaches $v_{f,\text{target}}$, cruises, and the cruise duration is whatever's left after subtracting the ramp distance:

$$t_4 = \frac{S - 2k v_f}{v_f}$$

## 8. CoreXY Decoupling

Once the path-relative target velocity $V(t)$ is known from the profile, it's split into per-motor velocities using the CoreXY transform (unit vector components $\hat{x}$, $\hat{y}$):

$$\text{targetLeft} = V \cdot (\hat{x} + \hat{y})$$
$$\text{targetRight} = V \cdot (\hat{x} - \hat{y})$$

Each motor then has its own velocity-based PID loop closing on this target, with a separate low-gain sync term correcting for any left/right drift using the ratio of actual to target velocity per motor.

## Known Limitation

The triangle-profile branch computes a reduced peak velocity $v_f$, but the current implementation does not yet re-derive `moveJ` distinctly for the triangle case in all downstream calculations — this is the open issue referenced in the README under "Known Issues."