# Steering controller for the comma.ai Controls Challenge

Controller: `controllers/rajat.py`. Experiments, scripts and results: `experiments/`.

## Results

| route set | baseline PID | v1 (hard integrator freeze) | **v2 (this controller)** |
|---|---|---|---|
| first 100 segments (tuning set) | 84.85 | 46.75 | 46.76 |
| 500 held-out segments (10000–10499) | 110.52 | 55.67 | **50.56** |
| 1000 held-out segments (15000–15999) | 108.56 | 54.68 | **51.92** |
| official 5000-segment `eval.py` run | 110.26 | 57.12 | **52.06** |

Total cost = 50 × lateral-acceleration cost + jerk cost, averaged over segments. Lower is better.

## Architecture

One steer command made of three parts:

```
steer = u_ff + u_roll + u_pid

u_ff   = 0.7 * mean(target_lataccel[t+1 .. t+5]) / 1.6     # feedforward with 5-step preview
u_roll = -0.8 * roll_lataccel[t] / 1.6                       # road-roll compensation
u_pid  = 0.195 * e + 0.100 * I + (-0.053) * (e - e_prev)   # PID on e = target - current
I     += e / (1 + (e / 0.7)^2)                               # smooth conditional integration
scale  = 1 - 0.5 * clip(|planned| - 2.0, 0, 1)               # 100% feedback below 2 m/s², 50% above 3 m/s²
steer  = u_ff + u_roll + scale * u_pid
```

`1.6` is the measured steady-state gain of the simulator (lateral acceleration per unit of
steer). `0.8` is the measured effect of road-roll lateral acceleration on the car.

### Why each part exists

1. **Feedforward with preview.** The simulator needs 2–4 steps (0.2–0.4 s) before a steer
   change shows up in lateral acceleration. A feedback-only PID can only react after an
   error has appeared, so it lagged the target by about 4 steps. The feedforward steers for
   where the plan is going over the next 0.5 s, which removes the lag. Using only 70% of the
   plant inverse, with feedback doing the rest, tracked better than a full inverse (which
   overshoots and adds jerk). Preview beyond 5 steps steers too early.
2. **Roll compensation.** Road roll pushes the car sideways. The PID eventually integrates it
   away, but on low-speed roads where the roll steps suddenly it produced large transients.
   Cancelling it directly cut the worst tuning segment from 358 to 155.
3. **PID feedback.** Handles model mismatch and the simulator's random disturbances. A 48-point
   gain grid showed the comma gains are already near-optimal once feedforward is present.
4. **Smooth conditional integration.** In large manoeuvres the simulator rate-limits lateral
   acceleration (0.5 m/s² per step), which adds delay. A freely integrating I-term winds up
   and the loop falls into a limit cycle (lateral acceleration swinging between −1 and −5).
   The integrator therefore integrates `e / (1 + (e/0.7)²)`: full speed for small errors,
   half speed at 0.7 m/s², one fifth at 1.4 m/s². Version 1 used a hard freeze at 0.5 m/s²;
   the smooth form scored the same on ordinary routes and 14% better on the hardest ones.
   A fixed clamp on the integral was tested and hurts, because the integral is genuinely
   needed for steady offsets.
5. **Gentler feedback in large manoeuvres.** When the plan asks for more than 2 m/s², the
   loop is in the rate-limited region where the fixed PID gains are too aggressive. The
   feedback is scaled down linearly to 50% between 2 and 3 m/s² of planned acceleration.
   Feedforward and roll compensation are not scaled. Together with 4 this cut the hardest
   segments by 42% and the worst held-out segment from 1035 to 358, with no change on
   ordinary routes.

### What was tested and rejected

Speed-scheduled plant gain (gain only rises mildly with speed), integral clamps and leaks,
derivative filtering, steer rate limits, feedback on the previewed target, previewed roll,
impulse-shaped preview weights, a lead (target-derivative) term, measurement low-pass,
speed-scheduled feedback gain, back-calculation anti-windup, error clipping, a linear-ramp
integration weight, a hard integrator freeze (v1). Details and numbers in `experiments/REPORT.md`.

## How it was tuned

* `experiments/sysid_openloop.py`: open-loop random steer excitation on 100 non-tuning
  segments, linear model fit → delay, gain, roll coefficient.
* `experiments/analyze_baseline.py`: where the PID loses (lag, oscillation, roll, noise).
* `experiments/exp01…exp08_*.py`: one feature at a time on the first 100 segments; each
  feature kept only if it improved the total cost. The anti-limit-cycle fix was tuned on 30
  hard segments from a separate block (05000–05999).
* `experiments/validate_final.py`: confirmation on 1500 segments never used for tuning.

## Limitations

* About half the remaining jerk is the simulator's own random per-step noise; it cannot be
  removed by any controller, only avoided.
* Large, fast manoeuvres at low and mid speed still oscillate somewhat because of the
  simulator's lateral-acceleration rate clip.
* The plant gain and roll coefficient are fixed constants; a real car would need them
  scheduled with speed and tyre conditions.
* The controller assumes the future plan is reliable for 0.5 s; a noisy or jumpy plan would
  feed straight into the steer command.

## Reproduce

```bash
# 100-segment quick check (expect total ≈ 46.76)
/opt/anaconda3/envs/comma/bin/python tinyphysics.py --model_path ./models/tinyphysics.onnx --data_path ./data --num_segs 100 --controller rajat
```

```bash
# official report (report.html)
/opt/anaconda3/envs/comma/bin/python eval.py --model_path ./models/tinyphysics.onnx --data_path ./data --num_segs 5000 --test_controller rajat --baseline_controller pid
```

```bash
# held-out validation used in the report
/opt/anaconda3/envs/comma/bin/python experiments/validate_final.py
```

Note: the simulator seeds its random generator from the data-file path string, so scores only
match when the data path is given as `./data` (relative), as in the commands above.
