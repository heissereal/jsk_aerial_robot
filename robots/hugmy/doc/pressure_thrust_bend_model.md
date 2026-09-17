# Hugmy pressure/thrust bend model

## Data

The model combines the two Neuron 4 / motor 3 identification runs:

- `arm4_bend_20260822_204404_raw.csv`: 495 conditions, 0.5 s windows
- `arm4_bend_20260822_212525_raw_addtest.csv`: 238 conditions, 1.0 s windows

There are 733 measured conditions in total. The model uses the measured mean
pressure, not the requested pressure.

## Angle and thrust definitions

`bend_deg` in the CSV is an unsigned change from the initial gravity vector.
It cannot distinguish pressure bending from thrust straightening. The model
therefore reconstructs the absolute total three-joint angle from the condition
mean Neuron acceleration:

```text
theta = clamp(atan2(-mean(acc_x), mean(acc_z)), 0 deg, 170 deg)
```

Angles crossing 360 degrees are unwrapped before clamping. This convention is
consistent with the local `-Y` joint axes in the Hugmy URDF: zero is straight,
pressure increases the bend angle, and thrust decreases it.

PWM is converted to thrust using Hugmy's existing calibration:

```text
pwm = -0.000679 T^2 + 0.044878 T + 0.5
```

## Model structure

Four surfaces represent the measured hysteresis:

```text
pressure increasing/decreasing x thrust increasing/decreasing
```

Each surface is fitted with monotonic gradient boosting and sampled onto a
regular table:

- pressure: 0--50 kPa in 5 kPa steps
- thrust: 0--9 N in 0.5 N steps
- angle: 0--170 degrees

The fitted surfaces are constrained so pressure cannot reduce the predicted
bend and thrust cannot increase it. Runtime evaluation is bilinear and has no
machine-learning dependency. If direction history is unavailable, the API
returns the average of all four hysteresis surfaces.

## Validation

Validation leaves out each of the five complete experimental repetitions and
fits the other four. Metrics include the error introduced by sampling the fit
onto the runtime lookup grid.

| Held-out repetition | MAE | RMSE |
|---|---:|---:|
| coarse 1 | 6.92 deg | 9.47 deg |
| coarse 2 | 7.68 deg | 10.63 deg |
| coarse 3 | 6.18 deg | 8.35 deg |
| fine 1 | 6.65 deg | 8.85 deg |
| fine 2 | 8.54 deg | 12.77 deg |
| all held-out samples | 7.14 deg | 10.02 deg |

The absolute-error 90th percentile is 16.47 degrees. The largest errors occur
near the rapid straightening transition, where the physical hysteresis and
rotor vibration are greatest. The table is suitable as a feed-forward geometry
model, but Neuron IMU feedback should still determine whether an arm has
actually reached its requested geometry.

## Regeneration

```bash
python3 robots/hugmy/script/fit_pressure_thrust_bend_model.py \
  /path/to/arm4_bend_20260822_204404_raw.csv \
  /path/to/arm4_bend_20260822_212525_raw_addtest.csv \
  --output robots/hugmy/include/hugmy/model/pressure_thrust_bend_model_data.h
```

The generator requires pandas and scikit-learn only when regenerating the
table. They are not Hugmy runtime dependencies.
