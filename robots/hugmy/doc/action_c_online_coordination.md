# Action C: coordinated residual feedback and online efficiency search

This extends the restored C pulse; it does not replace it with a new gait,
RL policy or a strict arm-angle trajectory. It uses the existing measured
relative bend (straight = 0, flexion positive) and Spinal posture signals.
Hardware displacement remains signed mocap x. No contact-force measurement
or friction/shape inference is claimed by this implementation.

## Ownership and execution order

The normal pulse now commands **front pressure 0 kPa before ON** and waits
for measured front pressure within +/-5 kPa. Rear arms retain normal grasp
pressure during this preparation. With pressure-driven pull enabled they
exhaust before ON and regrip after powered flexion, as described below.
The former static table selected about 24 kPa at 3 N and prevented extension
in the 2026-09-28 bag. `action_c_front_release_enabled: false` explicitly
selects that legacy table for comparisons; it is not the default gait.
The minimum starting bend, pressure timeout and posture guards still apply.
Starting bend and pulse guards are now separate: PREPARE and the actual ON
edge require each active front arm >=40 deg AND enough headroom for the
requested 8-deg extension plus 2-deg margin above an absolute 30-deg floor.
The ON-edge bend is frozen separately for each arm and each pulse. Its
in-pulse floor is `max(30, min(55, start_bend - extension_target - 2))` deg:
52 ->42 deg, 45 ->35 deg, 40 ->30 deg, 90 ->55 deg. This same floor applies
to optional pressure-driven pull; it never follows the moving current bend.
The existing maximum 20-deg relative extension, extension-rate, pair-mismatch,
posture and sensor-validity guards are retained. Thus lowering the starting
gate does not immediately trip the old shared 55-deg guard. These 40/30-deg
defaults accommodate the observed replacement-arm initial geometry; they
are NOT measured mechanical safety limits. Validate the IMU mounting and
physical clearance in restrained non-human testing before using this range.
The bend-limit separation alone does not change the waveforms. The subsequent
pressure-pull change below does change ON/OFF and regrip timing.
Logs `PREPARE bend blocked` and `bend limits arm...`
identify the failed start condition and the frozen per-pulse guard respectively.
Recovery commands bottom 0 kPa, but continuous C does not require bottom
to reach zero before correcting OR resuming. Fresh pressure and settled
posture are required. The next PREPARE owns the low waiting target.

### Pulse roll handoff and powered flexion (2026-09-29)

`action_c_pulse_roll_handoff_enabled` adds a pulse-time exception to ordinary
quiet-boundary recovery. Require |roll| >8 deg, growth >=1 deg from ON, and
outward gyro >=1 deg/s, PLUS an active front arm whose predicted remaining
bend margin is <=2 deg. Predict 0.10 s using extension rate against both
the frozen bend floor and relative extension limit. Either arm/pair and
both roll signs are supported. Stable/improving tilt does not trigger this
early path. The 15-deg roll / 0.8-rad/s hard limits interrupt immediately.
First publish zero pulse thrust, then hand pressure/bottom to the existing
shared recovery and preserve the displacement goal for automatic resumption.
No movement output is published later in that tick. Actual angle violations
are still guards; a predictive interruption alone is not credited as a
successful pulse or a newly measured physical thrust ceiling. Recovery gains,
thrust caps, and shadow-MPC status are unchanged.

`action_c_pressure_pull_enabled: true` now works in normal optimizer C,
independently of reference-pulse mode. Front pressure still releases before
ON. Rear pressure must be measured <=8 kPa before starting the pulse clock
(3-s bounded wait). Once acquired, the same release gate spans ON and powered
pull: pressures <=10 kPa remain ready; >10 through 12 kPa get at most 0.10 s
grace; >12 kPa, nonfinite rear pressure, stale/future samples or backwards
clock fail immediately. Returning to <=10 resets the excursion timer, while
loss of readiness requires <=8 again. A new cycle, regrip, recovery, sensor
fault or front/rear role change resets the latch. This specifically avoids
aborting at the recorded 7.85 ->8.08 kPa ripple without weakening acquisition.
A settled IMU
anchors the ON-edge bend; body-relative gyro integration runs continuously
through ON and powered flexion (MuJoCo may use fresh joint bend). Stale or
invalid integration is a stop, not a fall-back to accelerated gravity angles.
After unguarded extension/ON timeout, increase front pressure and retain at
most `min(0.35 * pulse_thrust, 1.5 N, actual_ON_output)` on each front rotor.
This residual cannot re-increase. It tapers with measured re-flexion, and
ends after pressure readiness plus 6-deg re-flexion, or the bounded 1.5-s
pull interval. Failure to reach pressure by timeout stops. Rear then regrips.
Roll/bend handoff also owns this powered phase; no residual is retained on
a safety stop. Recovery always cancels pending bottom inflation/powered pull.

ON retreat of 5 mm is now diagnostic in this mode, not an automatic pulse
cut and permanent 3-N ceiling. A 30-mm retreat remains a hard stop; signed
mocap-x displacement is scored after pull/regrip, including recovery time.
This is a pressure-driven flexion with limited thrust assistance, NOT a
horizontal-force allocator, contact-force model, or arm-trajectory MPC.
The attachment's Bayesian/asynchronous-pulse architecture is not implemented
by these changes. Human/contact safety and net progression are unverified.

Normal Action C now requires an explicitly received `target_attitude` before
accepting a new nonzero move. Missing target used to appear only when recovery
was needed, stranding a partially executed move. No target is inferred from
current tilt or silently set to zero. After a controller restart, re-send the
intended target before moving. Zero/stop commands and other gaits are unchanged.

Rear-release waiting may postpone thrust after bottom inflation has started;
measured rear-release safety takes priority over exact peak synchronization.
The bottom scheduler/targets themselves are unchanged. Logs `REAR RELEASE`,
`PRESSURE PULL`, and `pulse roll handoff` expose these additional phases.

With `action_c_bottom_sync_enabled: true` (normal C default), every cycle is:
SETTLE/early PREPARE **5 kPa** (`action_c_bottom_wait_kpa`, capped by candidate),
then inflate only after arm pressure, minimum bend and quiet roll are ready.
Release the unchanged thrust pulse when measured inflation predicts arrival
within +/-2 kPa of the candidate at the nominal thrust peak. Peak lead is
min(candidate thrust / thrust ramp, ON duration); an early safety/extension
cutoff is never delayed to satisfy bottom timing. Command candidate pressure
during ON; at OFF subtract 3 kPa through recoil/regrip, then return to the
low wait target in SETTLE. Example **5 ->30 (fill/ON) ->27 ->5 kPa**.

Supply delay (first 0.5-kPa rise) and rise rate (>=0.1-s measured windows)
are learned from pre-pulse inflation only, not contact-compression spikes
during thrust. Each fill requires a fresh local positive slope: an old fast
estimate cannot launch a new slow or stalled fill. ETA uses the slower of
the learned and local slopes plus a 50-ms margin. Without usable slope,
wait for actual target arrival; stale pressure cannot release a pulse.
The first fill measures its own response, rather than claiming a calibrated
model exists at startup. `Action C BOTTOM SYNC` logs rate, lag, ETA and the
previous nominal-peak pressure error. This is predictive synchronization,
NOT a guarantee of exactly minimal precharge or exact pressure at an early
guard-truncated peak. Real hardware response must be checked in the logs.
Reference-pulse mode disables this scheduler for reproducible comparisons.
The optimizer's peak-pressure candidate is not overwritten by phase targets.

Recovery and C PREPARE/SETTLE boundaries now acquire arm-pressure readiness
at the normal +/-2 kPa (released front: +/-5). Continuous C recovery GRASP /
RECOVER acquires at the configured hold band (+/-5 kPa), so regulator ripple
does not block automatic continuation. Once acquired, +/-5 kPa
regulation ripple retains readiness. Deviations up to +/-8 kPa get at most
0.10 s grace; larger deviations, stale/nonfinite data, target changes and
state changes reset the gate. Raw overpressure checks are not filtered.
Pressure ripple no longer restarts settling/confirmation on every spike.
Ordinary roll recovery now requires |target-relative roll| >8 deg AND at
least 1 deg growth over 0.5 s agreeing with outward Spinal gyro >=1 deg/s,
confirmed for 0.3 s at a settled boundary. Stable/improving offsets below
15 deg may continue on either side of zero, including after regrasp. Pulse
and inflation history is excluded. Absolute roll >=15 deg and the existing
0.8-rad/s rate guard still request recovery at thrust-off; existing in-pulse
guards remain unchanged. Pitch is still not a recovery objective.

### Continuous model / roll-rate recovery (2026-09-29)

An optional multi-scenario predictive replacement for the thrust/pressure
targets is now available; it defaults to shadow (logging only). See
[predictive recovery](recovery_predictive_control.md) for model assumptions,
activation, limits and validation. The baseline and shared guards below
remain active in shadow mode; movement pulses are unchanged.

`action_c_continuous_recovery_enabled: true` selects a continuous correction
path for Action C only. After a confirmed handoff, if roll error still exceeds
8 deg and sensors/rate are valid, it does not wait for bottom deflation, full
regrasp, another quiet window, or selected-arm pressure to reach zero. The
selected pair is commanded to 0 kPa, the other arms to grasp pressure, and
bottom to 0 kPa while **both correcting rotors ramp together** at 1.5 N/s.
Positive roll selects arms 2+3, negative roll selects 0+1 (zero-based).
Existing pulse-boundary recovery decisions and forward pulse waveforms are
unchanged; angular rate >=0.8 rad/s still prohibits correction thrust.

The correcting pair remains fixed during one recovery. Its common initial
thrust is based on measured rotor/Neuron bend (not a pressure-angle lookup),
the existing approximate contact geometry and a nominal roll restoring
moment ONLY. Full gravity compensation is deliberately excluded: the robot
is perched and the unknown contact reactions already support its weight.
Changing the legacy `gravity_compensation` flag, mass or COM does not add
gravity FF to this recovery path. Local-X rotor cant is 6.5 degrees, with URDF signs +,-,+,-.
The pair lever is the SUM of the two unit-thrust roll moments: the quotient
is per-rotor thrust, not total thrust. An uncertain/wrong-sign/small lever
uses a conservative 0.5-N feedforward fallback, never an absolute-value
division. Distal arm joints, contact migration and aerodynamic reaction
torques are not identified by this approximate model.

The active controller is now:

```
Euler roll_dot = p + tan(pitch) * (sin(roll)*q + cos(roll)*r)
lookahead = 0.2s + current_thrust / thrust_down_rate
predicted_error = signed_roll_error - helpful_rate*lookahead
desired_rate = min(0.12 rad/s, 1.0/s * signed_roll_error,
                   signed_roll_error / lookahead)
model_moment = roll_Kp * max(0, signed_roll_error - stop_angle) # NO gravity
FF = smoothed bounded(model_moment / measured_pair_lever)
# Blend with fallback near the minimum lever; hold FF baseline after confirmed motion.
F_target = bounded(FF + Kp_rate*(desired_rate - measured_rate) + I)
I_dot = bounded(Ki_rate*(desired_rate - measured_rate), +/-0.30 N/s)
if helpful_rate > desired_rate:
    F_target <= current_thrust - proportional_overspeed_decrement
if predicted_error <= 0: brake (target thrust = 0, retain I)
if actual_error <= stop_angle: finish and clear I
```

The nominal moment includes only the existing roll proportional term,
neither gravity compensation nor the old PD derivative. It is not stacked with a second angle PID.
Rate PI runs from the start, can reduce thrust before reaching the nominal
model target, and continues to adjust after motion begins. Default Kp=4
N/(rad/s), Ki=3 N/rad. Both amplitude and command-slew saturation inhibit
windup. Overspeed, using the faster of raw/filtered helpful rate, has priority
over FF and retained I: the command must decrease even if the model requests
4 N. The decrease is proportional to overspeed, capped by the existing 6-N/s
down-rate; this does not introduce a timed OFF pulse or opposite-pair thrust.
Positive integration and response learning are inhibited while overspeed
braking. Prediction includes command rundown time; it is not a guarantee of
physical stopping distance or a model of regrasp/contact transients.
Predictive braking retains I so a gyro transient does not force another
breakaway search. Actual completion, fault and reset still clear I.
The geometry feedforward blends into fallback around the minimum valid
lever and is slew-smoothed before motion. After confirmed motion its baseline
is held; the total thrust is NOT held, and rate PI / speed braking remain active.
Breakaway is still confirmed by helpful
rate >=0.05 rad/s for >=0.15 s AND error reduction >=1 deg, but captured
breakaway thrust is diagnostic only, no longer a permanent hold command.
Pitch is NOT a recovery objective; pitch/other gyro axes only convert Spinal
body rates to Euler roll rate. Near the Euler singularity recovery stops.

Separate positive/negative-pair memories estimate local (rad/s)/N response
from command/rate differences with configured 0.2-s response delay and
0.4-s windows. No new excitation pulse is injected. Learning requires
confirmed body motion, stable measured pressures (including bottom) and
all four bends, no near-target braking, and headroom below saturation.
Estimates are bounded 0.02--0.25 (rad/s)/N, updated at most 10% per accepted
window, and require three accepted windows in compatible pressure/angle
context. Only bounded, slowly changing rate Kp uses the estimate; feedforward
still uses geometry. This is NOT a physical friction/contact-force estimate
or guaranteed system identification. Untrusted/new contexts use nominal Kp;
`recovery_response_learning_enabled: false` disables this adaptation.

Pressure is a separate slow actuator. Default selected-pair target remains
0 kPa, opposite pair stays at grasp pressure, bottom target stays 0 kPa:
thrust never waits for measured zero. If a higher initial release target is
configured, a persistent stall near 80% of the thrust cap lowers it slowly
(0.5 kPa/s) toward the existing floor. At the default floor no additional
pressure change is possible; thrust feedback handles residual pressure.
Per-rotor output remains <=4 N, rise <=1.5 N/s and fall <=6 N/s. Enter regrasp inside
3 deg for Action C or if the signed error crosses the target. No timed 0.2-s correction
pulse is used, even if an arm-angle target is reached before body motion.

Sensor freshness (pressure reception <=0.25 s, existing IMU timeouts), raw
overpressure and gyro stops remain. Invalid input or excessive angular rate
stops the task and blocks automatic correction retry until a new operator
command / disable-enable cycle. With a saved locomotion goal, 10 s with no
posture progress (including initial search), or the 60-s overall correction
timeout instead stops thrust, commands regrasp and bottom zero, and retains
the goal. No additional correction thrust is tried in this waiting state.
Progress now compares mean roll error in the first and last quarters of a
rolling `recovery_progress_window_sec` (default 1 s). A reduction of at least
`correction_progress_epsilon_rad` (default 0.5 deg) renews the no-progress
timer, even if error is still larger than at recovery entry. This fixes the
c3 (2026-09-29) 15.5 -> 32 -> 26 deg return being stopped at 10 s. Stationary
or sub-threshold jitter does not renew progress. Oscillatory improvements can
renew the local timer, but never extend the independent 60-s overall deadline.
The log reports `recent_progress` and `progress_age`; thrust, pressure,
geometry validity, gyro guards and the +/-3 deg success band are unchanged.
After roll is inside +/-3 deg, roll rate <=0.05 rad/s for 1 s, fresh sensors
and settled grasp pressure are available, locomotion can resume using fresh
mocap. The optimizer trial is closed at timeout, before any later manual
repositioning. Explicit zero motion command or disable cancels the saved goal.
**This waiting mode can resume automatically after hand assistance: stop or
disable before handling the robot, and keep hands clear before re-enabling.**
These limits are not a guarantee of physical
recovery: a 4-N ceiling may still be insufficient for some contacts.
Normal completion retains the saved absolute mocap-x goal and resumes after
regrasp/settling; no new movement command is required. It waits for fresh
mocap before evaluating remaining travel. Valid pressure settling alone no
longer discards that saved goal after 20 s. Real faults, recovery-count limits,
explicit stop and goal completion still do not automatically restart motion.
Continuous recovery selects the Spinal sign pair directly, independent of
the pressure-model moment magnitude; signed-error and gyro projection tests
cover both roll directions. A/B/D and fixed-arm tests keep their legacy path.

The c14 recording (2026-09-29 14:30:58) contained no completed recovery:
progress timeouts occurred near bag-relative 27.92 s and 78.78 s. These
previously cleared the locomotion goal. Repeated prediction-driven I resets
(for example about 1.62 N to zero near 68.78 s) and abrupt model/fallback
switching were additional sources of stop/re-bend/search cycling. The changes
above address those mechanisms; hand-assisted data cannot demonstrate
one-shot autonomous recovery, which still needs a hardware test.

Offline regressions: `test_continuous_roll_recovery.py` exercises the actual
rate law, delayed/stiction synthetic plants and response-learning gates;
`test_recovery_geometry.py` compiles actual measured-angle geometry and checks
both pairs and each rotor's cant; `test_posture_confirmation.py` exercises production entry/resume
expressions. Hardware closed-loop validation is still required. Do not use
hand-assisted recovery intervals as autonomous progress measurements.
The diagnostic log prefix is `Action C model-rate recovery`: rate is actual/
target, thrust is commanded/target PER ROTOR; FF/I/breakaway/moment/lever and
model/learning validity are included. Learned memories are runtime-only,
not written to the gait optimizer or persisted to a trained policy.
`gravity_ff=0`, `speed_brake`, `predicted_brake` and `horizon` distinguish the
restoring-only model and rate-priority braking from the previous c13 behavior.

1. The optimizer fixes common thrust, bottom pressure and burst count for a
   complete trial. The fixed post-OFF bottom drop is applied to that candidate.
2. During ON, a bounded residual assists the lagging front arm (relative to
   each arm's OWN starting bend). Learned feed-forward and live feedback
   share **0.3 N total** extra across the pair, within the existing motor and
   hard safety ceiling. Default per-rotor maximum is **5 N including FB**;
   the common candidate starts at 3 N. Pair-mismatch restrictions limit the
   common candidate but leave this small lagging-arm headroom; physical
   tilt/overextension restrictions still limit every rotor. Live lag gain
   is 6 N/rad with a 2-degree deadband (previously 1 N/rad). The measured bend
   estimator and common pulse/ramp are unchanged. Existing
   angle/roll reductions take priority; additional thrust fades to zero by
   half the relative-tilt safety limit, or at 8-degree roll / 0.3-rad/s roll
   rate. Those last two values only remove assist; they do not stop the gait.
3. A front chamber's empirical command-to-50%-pressure-rise time is measured.
   After valid learning, its flexion-pressure command may start at most
   0.12 s early. It requires >=half the requested extension and >=0.15 s ON.
   Remaining time is estimated from the fixed pulse end OR the measured
   extension/rate. If conditions never hold, the original OFF edge applies.
   The pressure target is frozen at pulse start and reused at OFF: no double
   pressure boost. Rear release/regrip timing is unchanged.
4. No new feedback/pressure condition blocks a phase transition. Existing
   extension/timeout/recoil/regrip criteria and safety guards remain in charge.
5. Measurements are staged after a completed regrip. Learning commits only
   after a whole trial with all counted pulses successful (extension, recoil,
   retained progress and regrip), no guard, no recovery
   and a valid clock. Pressure dropouts/timeouts are censored, not converted
   into large learned delays. A zero/noisy arm response is not an instruction
   to raise both motors.
6. A commit invalidates the old best score and repeats the same T/P for one
   pulse. It does NOT also advance the optimizer. Updates are bounded to
   0.05 N and 0.03 s per trial and at most 3 eligible trials per operator task.
   Thereafter calibration freezes so search can progress. A new operator
   command reopens that finite learning budget; posture recovery does not.
   Direction/optimizer reset clears the learned values. Values are process
   memory, not a persisted context database.

The older individual-trajectory controller, pressure-sign adaptation and
reference-pulse modes are mutually exclusive with these new residuals.
Selecting one disables coordination with a startup warning. This avoids two
controllers learning/increasing the same input independently.

## Objective

Default score (m/s, with weights carrying the corresponding units):

```
J = dx_retained / total_seconds
    - 0.0001 * integral(sum_i(T_i^2) dt) / total_seconds
    - 0.001 * recovery_seconds / total_seconds
    - 0.010 * max(0, 1 - dx_retained / burst_goal)
```

Total time includes preparation, pneumatic waits, pulsing, recoil, regrip,
settling and shared posture recovery. It uses elapsed ROS time, not the
clipped dt used for slew limiting. Backward clock jumps invalidate a trial.
The T-squared integral is a thrust-use proxy, not measured electrical energy.
The burst goal is frozen before its first pulse: requested count times the
configured stroke distance, capped by remaining task distance. Zero progress
costs 0.010 m/s, a half-goal deficit costs 0.005 m/s; backward progress costs
more. This term is NOT diluted by longer waiting or early recovery. It is a
penalty, not a new abort threshold. The legacy A/B objective is unchanged.

Positive progress must originate in valid pulse-to-regrip measurements.
Backward motion during recovery is subtracted; forward recovery drift earns
no extra credit. Credit is also capped by total signed trial displacement,
so sliding backward during SETTLE cannot leave an inflated positive score.
At least one successful pulse, plus positive finite unguarded net progress,
is required to become best; safe completion alone is not enough. While no
successful best exists, unguarded completed attempts raise common thrust by
the existing bounded step instead of probing bottom or committing residual
calibration. Thrust ceilings and guards are unchanged.
Positive, finite, unguarded validated motion is required to become best;
standing still cannot win by avoiding recovery. Increasing burst count with
the same speed does not automatically improve its score.

Peak roll/rate/mismatch and the former weighted-distance objective remain
logged as diagnostics. Their old weights do not enter the new default score;
unchanged safety guards and the phase-aware recovery supervisor remain active.
The optimizer is the existing bounded coordinate search, not Bayesian search.

## Enable, compare and diagnose

Default YAML enables both `action_c_coordination_enabled` and
`action_c_efficiency_objective_enabled`. Restart the existing controller;
do not start a second publisher/controller. Existing Action-C launch and
movement commands are unchanged. Use the normal baseline, not reference or
individual-extension overrides. For an explicit baseline comparison, set
both flags false in a separate configuration before restarting.

Look for:

- `Action C pressure lead arm...`: measured-extension-qualified early pressure.
- `Action C COORDINATION commit...`: frozen next-trial calibration; T/P held.
- `Action C efficiency...`: credited distance, full time and recovery time.
- `optimizer NEXT`: optimizer-owned T/P/N change after scoring.

Config is in `config/attitude_pressure_controller.yaml`; helper logic is in
`include/hugmy/control/action_c_coordination.h`.

## Validation limits

Offline tests exercise production helper and scoring code plus integration
checks; they do not establish physical locomotion performance or human-contact
safety. Short-pulse accelerometer-derived bend can still contain dynamic
acceleration error. The small budgets and safety checks limit intervention,
but real cylinder trials/log review remain necessary. No new actuator
commands are sent by these tests.

```
python3 robots/hugmy/test/test_action_c_coordination.py
python3 robots/hugmy/test/test_action_c_optimizer_baseline.py
python3 robots/hugmy/test/test_action_c_phase_recovery.py
```
