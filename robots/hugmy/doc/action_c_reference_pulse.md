# Action C: timed-pulse search and current roll recovery

> Historical implementation, preserved in commit `0a2db4956`. Its execution
> path and launch switches have been removed. The text below is archival, not
> current startup guidance; use [predictive locomotion](hybrid_locomotion.md).

**Archived experimental mode.** At that time, normal Action C used
[the restored optimizer baseline](action_c_optimizer_baseline.md).
The sequence and launch examples below describe explicit timed-mode experiments;
pressure pull and amplitude search are disabled by default. Do not use these
examples to reproduce the supplied successful optimizer diff.

This is a bounded pulse-search integration, not a validated real-machine gait or an
exact restoration of an old revision. The supplied historical excerpts do
not contain a complete controller snapshot. Keep the current roll supervisor
and isolate the compliant front-pair pulse from later adaptive controllers.

## Experimental sequence

| Phase | Front pressure | Rear pressure | Front thrust | Bottom |
| --- | --- | --- | --- | --- |
| Prepare | 5 kPa | base grasp (30 kPa) | 0 | 30 kPa |
| Rear release | 5 kPa | 0 kPa; verify measured <=8 kPa | 0; wait at most 3 s | hold 30 kPa |
| ON/extension | 5 kPa | 0 kPa, monitor <=8 kPa | equal pair, start at 3 N each | hold 30 kPa |
| Pressure-driven flexion | base grasp (30 kPa) | keep 0 kPa | residual 35% of target, max 1.5 N each | hold 30 kPa |
| OFF/regrip | base grasp | base grasp | 0 | hold 30 kPa |
| Goal/cycle limit or safety stop | base grasp | base grasp | 0 | exhaust to configured low pressure |

The base ON is 0.6 s **including** the 8 N/s rise. Search extends it to at least
`target thrust / ramp rate + 0.15 s`, bounded by 1 s. Thus 3 N uses 0.6 s and
6 N uses 0.9 s instead of cutting before reaching 6 N. With pressure pull
enabled, both arms reaching 8 degrees of relative extension also ends ON.
On that edge the front pressure target rises to base grasp. Positive residual
thrust stays on while pressure supplies flexion torque; it never exceeds
the actual ON output. It tapers individually over the last half of the
6-degree rebound target and cannot re-increase within that pull. Rear pressure
stays commanded to zero until front pressure has recovered and both arms
have rebent, or the bounded 1.5-s pull finishes. Failure to build front pressure
by that deadline stops the task; a missing rebound is logged, not falsely
reported as angle success. Regrip then stops thrust, restores rear pressure,
waits at least 1 s and confirms pressure recovery. The existing anchor timeout
bounds this unpowered wait. Rear rotors always stay at zero thrust.

`action_c_pressure_pull_enabled` must be explicitly enabled in the controller YAML.
Residual thrust is configured by `action_c_pressure_pull_ratio` (0.35) and
`action_c_pressure_pull_max_n` (1.5). The powered pull is independently bounded
by `action_c_pressure_pull_timeout_sec` (1.5); configuration is capped at 2 s.
Rear release is checked before ON and throughout ON/pull, using a pressure
message no older than 0.5 s. Roll recovery remains available while waiting
for exhaust. No pressure reading is inferred from an exhaust command. This is a control sequence, not proof
of unloading, contact-point advance or retained body motion on hardware.

For +x the front pair is 0/3; for -x it is 1/2. Angle trim, roll differential
within the pulse, individual thrust learning and online pressure adaptation
are disabled in timed mode. No 90-degree pressure-chasing override is
allowed. An unsafe initial bend pauses preparation instead of silently
raising front pressure above the requested 5 kPa.

In this experimental mode, enable amplitude search explicitly. After
complete OFF/regrip without a pulse guard or posture recovery, advance under
2 mm raises the next pulse by 0.5 N: 3 -> 3.5 -> 4 -> ... -> at most 6 N.
At least 2 mm retained advance holds the amplitude. Missing the 8-degree
angle target alone does not block this search. Retained backward motion ALSO
earns a 0.5-N increase after a complete, otherwise safe regrip. Recovery after a pulse lowers the
next amplitude by a step (not below the configured minimum); a guard stops
automatic repetition and existing safety ceilings still apply. No-pulse
recovery and incomplete data cannot raise amplitude. Bottom remains 30 kPa.
The existing objective is logged, but this mode selects amplitude using the
bounded per-pulse rule, not the legacy three-variable coordinate search.

The motion supervisor still uses Spinal roll only: >8 degrees continuously
for 0.3 s requests correction; active correction ends within 5 degrees.
Existing angular-rate, pressure, stale-sensor, bend, extension, mismatch,
relative body-tilt guards remain. In pressure-pull mode, the old 5-mm ON
backward threshold is diagnostic only and cannot latch the ceiling to 3 N.
The new 30-mm retreat limit, relative to pulse start, remains a hard stop
during ON and powered flexion; it is not permission to increase after an
unsafe trial. Disabling pressure pull restores the old 5-mm hard ON limit.
Posture recovery can
still run while in GRASP after the single locomotion pulse has ended.

## Start and inspect

First use a secured non-human fixture or MuJoCo. Do not run two attitude
controllers at once. Keep the existing bridge/mocap/pressure setup.
For the real-machine workflow start the allocator separately from bringup:

```bash
roslaunch hugmy attitude_pressure_controller.launch \
  inchworm_action_c:=true \
  action_c_reference_pulse_mode:=true \
  action_c_reference_single_pulse:=false \
  action_c_pulse_search_enabled:=true \
  action_c_reference_thrust_n:=3.0
```

For MuJoCo the same arguments are forwarded by:

```bash
roslaunch hugmy bringup.launch \
  simulation:=true real_machine:=false mujoco:=true headless:=false \
  start_attitude_allocator:=true inchworm_action_c:=true \
  action_c_reference_single_pulse:=false action_c_pulse_search_enabled:=true \
  action_c_reference_thrust_n:=3.0
```

Launch alone is not a motion request. Use the existing target-attitude,
enable and inchworm-command procedure after confirming grasp and sensors.
For an explicit diagnostic set `action_c_reference_single_pulse:=true`.
In that mode, **one accepted motion command produces at most one pulse**,
even if the distance goal has not been reached. The log `Action C REFERENCE
done` reports retained mocap x, angle-extension detection, guard and regrip.
It does not certify a successful gait merely because a timed pulse finished.
In search mode, `Action C PULSE SEARCH` reports retained advance, safety state
and the next amplitude. A normal trial missing its distance goal continues
until the overall goal or the existing cycle limit (default 10) is reached.

`action_c_reference_thrust_n` is the initial per-front-rotor amplitude, read
at startup and bounded by the configured minimum/maximum and safety ceilings.
`action_c_pulse_search_step_n` defaults to 0.5 N and is capped at 0.5 N.
`action_c_pulse_search_progress_m` defaults to 0.002 m. To retain the previous
fixed-amplitude diagnostic use `action_c_pulse_search_enabled:=false`.
That diagnostic retains the original fixed 0.6 s ON, which cannot reach a
peak above 4.8 N at an 8 N/s ramp. Never bypass guards to force an increase.

`action_c_reference_single_pulse:=false` allows repetition with goal/cycle limits and roll
recovery. A pulse guard or incomplete OFF/regrip still stops repetition.
Bottom stays inflated between successful repeated pulses, but is exhausted
during posture recovery or at the end of the task.

## Angle diagnostics and limitations

Record these alongside Neuron IMU, Spinal IMU, PWM commands, pressure targets,
measured pressures and mocap:

- `/quadrotor/attitude_pressure_controller/arm_bend_angle_deg`: existing
  accelerometer-derived estimate (can be corrupted by pulse acceleration).
- `/quadrotor/attitude_pressure_controller/action_c_pulse_bend_angle_deg`:
  ON and powered-flexion short-horizon gyro estimate; non-front entries are NaN.

The pulse estimate starts from a settled acceleration-based bend, then
integrates Neuron local-Y gyro minus body gyro projected into that arm's
local Y. The old extra minus sign was incorrect: a +Y sensor rotation q
expresses gravity as [-sin(q), 0, cos(q)], so the accelerometer convention
gives measuredArmAngle=-q and bend=q. Positive relative gyro is flexion;
negative is extension. A geometric finite-difference regression checks this
independently of the integrator. It is not an encoder reading.
The static seed can still contain mounting bias; axes and sign must be
checked with slow, known extension/flexion before enabling angle tracking.
Invalid/out-of-order/missing integration samples stop the pulse. MuJoCo uses
the exact rotor-level joint bend when available.

Individual angle tracking and the three-variable optimizer remain disabled
in timed mode; verify angle sign and retained motion before restoring them.
The legacy path remains available via `action_c_reference_pulse_mode:=false`,
but it is not the recommended next real-machine test.

## Offline checks

```bash
python3 robots/hugmy/test/test_reference_pulse.py
python3 robots/hugmy/test/test_action_c_control.py
python3 robots/hugmy/test/test_pressure_pull.py
python3 robots/hugmy/test/test_posture_confirmation.py
python3 robots/hugmy/test/test_roll_only_correction.py
```

These tests emit no ROS actuator commands. They do not establish real-machine
stability, safe human contact, or successful locomotion.
