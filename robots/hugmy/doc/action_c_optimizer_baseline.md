# Action C: restored optimizer baseline (2026-09-28)

> Historical implementation, preserved in commit `0a2db4956`. Its execution
> path and settings have been removed. The text below is archival, not current
> startup guidance; use [predictive locomotion](hybrid_locomotion.md).

This is the normal Action C configuration again. It restores the semantics
identified by the user's supplied `+63/-14` excerpt, not a byte-for-byte
checkout: the attachment is a partial diff, not a full historical snapshot.
Do not replace this baseline when adding another gait experiment.

The default now adds bounded residual coordination and a recovery-inclusive
speed objective, documented in [action_c_online_coordination.md](action_c_online_coordination.md).
The waveform and coordinate-search mechanism below remain; the static table
preload was subsequently corrected after the 14:01 bag showed failed extension.
the old distance-score weights describe the explicit legacy A/B mode.

## What was restored

- Release front pressure to 0 kPa before pulsing (arrival within +/-5 kPa).
  The previous restoration incorrectly used the static pressure/thrust/bend
  table and held about 24 kPa at 3 N, opposing extension. That table is now
  only available with `action_c_front_release_enabled: false` for comparisons.
- Rear exhaust starts at pulse ON, without waiting for measured rear pressure.
- Front pulse: 3--5 N candidate, 8 N/s rise, up to 0.6 s, earlier angle/guard
  termination. Existing small reduction-only angle and roll corrections remain.
- OFF: ramp thrust to zero at 30 N/s; front pressure boost and rear regrip
  begin together. No residual-thrust pressure-pull phase or fixed 1-s OFF gate.
- Keep the supplied diff's 15-degree relative-extension mismatch guard,
  active only once one arm has extended by 8 degrees; pressure adaptation OFF;
  recovery cost 0.030; predicted next-roll is diagnostic, not a recovery trigger.
- Restore objective-based coordinate search over thrust, bottom pressure and
  burst count. Candidates are centered on the best eligible measured score.
  Bounds: bottom 20--30 kPa (start 30), burst 1--5, thrust 3--5 N.
  Before an eligible successful trial exists, bounded baseline thrust discovery
  remains at 30 kPa and one pulse. Unsafe or incomplete trials cannot earn
  positive motion credit. Safety ceilings remain active.

Later experiments are disabled by default: `action_c_reference_pulse_mode`,
`action_c_pulse_search_enabled`, `action_c_pressure_pull_enabled`,
`action_c_individual_extension_enabled` are false; the later pair-sync gain
is zero. The original per-arm angle trim is NOT zeroed. The corrected gyro
sign is retained for the optional gyro-observer path; baseline locomotion
keeps the existing measured-bend path (rotor bend in sim, Neuron accelerometer
relative to Spinal on hardware).

## Recovery kept separate

Spinal roll-only recovery and its established pair allocation are preserved.
The whole-body-motion confirmation still requires 0.15 s and 1 degree of
actual improvement. Existing anti-overshoot behavior is NOT rolled back:
10-degree slowdown band, 0.20-s gyro prediction, 6 N/s thrust reduction,
5-degree stop band and 3-N correction ceiling. The 8-degree/0.30-s entry
confirmation and stalled-motion protection remain. No new recovery gains are
being guessed from the partial gait diff.

## Phase-aware recovery (2026-09-28)

Normal C cycles use the selected bottom pressure during PREPARE/ON, subtract
3 kPa at OFF through recoil/regrip/SETTLE, and restore it at the next PREPARE.
The candidate itself remains unchanged; the subtraction does not accumulate.
The old recovery timer, however, counted roll excursions during inflation and
pulsing and could interrupt recoil as soon as thrust reached zero.

Ordinary recovery now assesses **residual roll** only in PREPARE/SETTLE,
after measured arm/bottom targets and roll rate have been stable for
`action_c_posture_settle_sec` (0.5 s). Above 8 degrees, recovery additionally
requires 1 degree growth in |roll error| over 0.5 s agreeing with outward
Spinal gyro >=1 deg/s, confirmed for 0.3 s. Each state change resets history. SETTLE
cannot launch the next pulse before this assessment, and its wait has the
existing prepare-timeout bound. Stable/improving offsets below 15 degrees
are accepted on either side of the fixed roll target; 15 degrees is a hard
recovery threshold regardless of trend. Rate guards remain independent.

Recovery requests zero bottom pressure and waits for measured deflation,
regrasp and settling before either correcting or resuming. The existing
5-degree active correction target remains, while settled, non-worsening
regrasp offsets below 15 degrees are accepted;
on resumption the bottom is refilled and assessed afresh. A bounded timeout
cancels continuation if deflation/settling never completes. Normal final goal
completion, operator stop and faults still return to the ordinary GRASP
pressure path; this does not hold bottom pressure indefinitely after stopping.

Rate/extension/relative-tilt guards remain separate. This change does not
raise motor thrust limits or modify the arm-angle estimator.
The same ordinary-recovery policy applies to the optional pressure-pull path.

## Start after restarting the controller (commands)

Use the existing bridge/pressure/mocap setup. Do not run a second controller
alongside the one started by bringup.

```bash
roslaunch hugmy attitude_pressure_controller.launch \
  inchworm_action_c:=true \
  action_c_reference_pulse_mode:=false \
  action_c_pulse_search_enabled:=false
```

For a bringup-owned controller, use the same two `false` arguments with
`start_attitude_allocator:=true inchworm_action_c:=true` on bringup instead.
Remove the old `action_c_reference_pulse_mode:=true` command-line override.
Launch is NOT a motion request; use the existing enable/command procedure.

Startup should show `Action C optimizer baseline`, not `Action C timed pulse`.
During operation, `optimizer trial ... RESULT ... J=` and `optimizer NEXT`
(or bounded `optimizer BASELINE` discovery) identify the active search.
`PULSE SEARCH`, `TRACKING FIRST`, `REAR RELEASE` and `PRESSURE PULL` must not
drive this baseline. Thrust-only discovery before an eligible sample is
distinguished from the best-score coordinate search in the logs.

## Offline verification

```bash
python3 robots/hugmy/test/test_action_c_optimizer_baseline.py
python3 robots/hugmy/test/test_action_c_control.py
python3 robots/hugmy/test/test_reference_pulse.py
python3 robots/hugmy/test/test_pressure_pull.py
python3 robots/hugmy/test/test_posture_confirmation.py
python3 robots/hugmy/test/test_action_c_phase_recovery.py
python3 robots/hugmy/test/test_roll_only_correction.py
```

Tests exercise production pressure selection and objective/selector code,
configuration/launch consistency, and recovery protection. They do not prove
that this reconstruction reproduces the historical physical trajectory.
