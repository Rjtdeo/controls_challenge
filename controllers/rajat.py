"""
Rajat Deo - steering controller for the comma.ai Controls Challenge.

Architecture (all terms add up to one steer command):

    steer = FEEDFORWARD + ROLL COMPENSATION + PID FEEDBACK

1. Feedforward with preview
   The simulator needs ~2-4 steps (0.2-0.4 s) before a steer change shows up in
   lateral acceleration, and ~1.6 units of lateral acceleration result from each
   unit of steer (measured with open-loop steps, see experiments/sysid_openloop.py).
   So we steer *now* for the lateral acceleration the plan wants in the next 5
   steps:   u_ff = FF_GAIN * mean(next 5 target lataccel) / PLANT_GAIN.
   FF_GAIN < 1 on purpose: a partial feedforward plus feedback tracks better than
   trying to invert the plant exactly (full inversion overshoots and adds jerk).

2. Roll compensation
   Lateral acceleration due to road roll pushes the car sideways with a measured
   coefficient of ~0.8. We cancel it in advance instead of waiting for the
   integrator:   u_roll = -ROLL_GAIN * roll_lataccel / PLANT_GAIN.

3. PID feedback on the current error (same gains as the baseline PID; the grid
   search showed they are already near optimal once feedforward is present).

4. Smooth conditional integration (anti-windup)
   During large manoeuvres the simulator rate-limits lateral acceleration, which
   adds delay; a freely integrating I-term then winds up and the loop falls into
   a limit cycle. The integrator therefore slows down smoothly as the error grows:
   it integrates  w * error  with  w = 1 / (1 + (error / 0.7)^2)  (w = 1 for a
   zero error, 0.5 at 0.7 m/s^2, 0.2 at 1.4 m/s^2). Big errors are handled by
   feedforward and P; the integrator only removes small steady offsets.
   (A hard freeze at 0.5 m/s^2 was version 1; a plain clamp on the integral hurts.)

5. Gentler feedback in large manoeuvres
   When the plan asks for more than 2 m/s^2 of lateral acceleration the loop is
   working in the rate-limited region, where the fixed PID gains are too aggressive.
   The feedback (P, I, D together) is scaled down linearly from 100% at 2 m/s^2 to
   50% at 3 m/s^2 and above. Feedforward and roll compensation are unchanged.

Results (mean total cost; lower is better):
                                   baseline PID   v1 (freeze)   v2 (this file)
    DEV      (segs 0-99)               84.85         46.75          46.76
    VAL      (500 unseen segs)        110.52         55.67          50.56
    VAL2     (1000 unseen segs)       108.56         54.68          51.92
    OFFICIAL (eval.py, 5000 segs)     110.26         57.12          52.06
"""
from . import BaseController
import numpy as np


class Controller(BaseController):
  # ---- plant model (identified, not tuned) ----
  PLANT_GAIN = 1.6        # steady-state lataccel per unit steer
  ROLL_GAIN = 0.8         # lataccel produced per unit of roll lataccel
  # ---- feedforward ----
  FF_GAIN = 0.7           # fraction of the plant inverse used as feedforward
  PREVIEW_STEPS = 5       # average the next 5 planned targets (0.1 .. 0.5 s ahead)
  # ---- feedback ----
  KP = 0.195
  KI = 0.100
  KD = -0.053
  I_SOFT_WIDTH = 0.7      # integration rate halves at this error (m/s^2); see update()
  BIG_TARGET = 2.0        # above this planned lataccel (m/s^2) feedback starts to be reduced ...
  BIG_FEEDBACK = 0.5      # ... down to this fraction one m/s^2 later

  def __init__(self):
    self.error_integral = 0.0
    self.prev_error = 0.0

  def _previewed_target(self, target_lataccel, future_plan):
    """Mean of the next PREVIEW_STEPS planned targets; pads with the last known value near the end of a route."""
    future = list(future_plan.lataccel[:self.PREVIEW_STEPS]) if future_plan is not None else []
    if not future:
      return target_lataccel
    if len(future) < self.PREVIEW_STEPS:
      future += [future[-1]] * (self.PREVIEW_STEPS - len(future))
    return float(np.mean(future))

  def update(self, target_lataccel, current_lataccel, state, future_plan):
    # 1. feedforward: steer now for where the plan is going
    planned = self._previewed_target(target_lataccel, future_plan)
    u_ff = self.FF_GAIN * planned / self.PLANT_GAIN

    # 2. cancel the road-roll disturbance
    u_roll = -self.ROLL_GAIN * state.roll_lataccel / self.PLANT_GAIN

    # 3. PID feedback on the current error
    error = target_lataccel - current_lataccel
    # 4. smooth conditional integration: integrate slower the larger the error
    integration_weight = 1.0 / (1.0 + (error / self.I_SOFT_WIDTH) ** 2)
    self.error_integral += integration_weight * error
    error_diff = error - self.prev_error
    self.prev_error = error
    u_fb = self.KP * error + self.KI * self.error_integral + self.KD * error_diff

    # 5. gentler feedback while the plan asks for a large manoeuvre
    big = min(max(abs(planned) - self.BIG_TARGET, 0.0), 1.0)      # 0 below BIG_TARGET, 1 one m/s^2 above it
    feedback_scale = 1.0 - big * (1.0 - self.BIG_FEEDBACK)

    return float(np.clip(u_ff + u_roll + feedback_scale * u_fb, -2.0, 2.0))
