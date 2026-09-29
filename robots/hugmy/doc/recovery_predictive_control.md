# Action C: uncertain-model predictive posture recovery

This adds a finite-scenario, receding-horizon controller **only to continuous
Action C roll recovery**. It is not SAC, not a new movement gait, and not a
certified robust MPC. Forward pulses, their optimizer, firmware and motor
calibration are unchanged. The default is **shadow mode**: predictions are
logged, but the existing recovery controller still commands the actuators.

## Control and model

- The existing Spinal roll sign selects the correcting pair (positive roll:
  motors 2+3; negative: 0+1, zero-based). Both get common positive thrust;
  pressure targets are selected independently for the two arms. Other arms
  retain grasp pressure and bottom is commanded to zero.
- Observations are measured rotor/Neuron bend, arm/bottom pressure, Spinal
  roll and Euler roll rate, previous pressure targets and **sent thrust
  commands**, not measured propeller thrust. Freshness/overpressure checks
  remain in the parent controller.
- The existing hysteresis-table midpoint maps P/T to equilibrium bend, with
  the existing `bend_model_thrust_offset_n: 1.8` coordinate convention.
  Table extrapolation is rejected; sensor pressure down to -5 kPa is treated
  as zero for lookup only. Commands never request negative pressure.
- Pressure and bend use first-order response models. The measured geometry,
  contact-position approximation and 6.5-degree cant give forward roll
  moment. There is no division by the current lever and no full-gravity FF.
  A currently small or wrong-sign lever does not alone discard trajectories
  that extend the arms into a useful geometry.
- Roll uses a **local incremental** moment/acceleration model anchored at the
  measured rate and current predicted moment. This is not identification of
  inertia, contact forces or friction. Nominal pressure/bend time constants
  (0.6/0.35 s), acceleration gain (8 rad/s² per Nm) and damping (1.5 /s) are
  initial hypotheses, **not calibrated values**.
- Five scenarios cover nominal, slow/weak, fast/strong and asymmetric arm
  responses. Pressure/bend time scales span 0.5--1.8, moment gains 0.5--1.5
  times nominal, with signed per-rotor lever perturbations up to 0.01 m.
  These are engineering test scenarios, **not validated error bounds**.

Every 0.1 s, simulate 2 s with 0.1-s steps. Up to 216 candidates combine two
thrust knots and two arm-pressure targets. Larger *future* thrust candidates
are included to see past the extension threshold, while the first move is
limited by confidence and the normal actuator slew. Pressure command slew
and measured-pressure response are included in prediction. Only the first
move is applied; the future knot is replanned, not played back as a pulse.

For each candidate, reject a scenario exceeding the predicted rate envelope,
more than 5 degrees additional roll error, or crossing the opposite 3-degree
band. Among survivors, minimize the worst scenario cost, which combines
roll error/rate tracking, excess bend outside the 0 +/-10-degree extension
target, thrust squared and command changes. Extension is a **soft objective**,
not a command to violate mechanical limits or override braking. This is a
weighted finite search, not a proof of minimum thrust or guaranteed progress.
No-progress and overall timeouts remain responsible for ineffective trials.

An 8-ms wall-time search budget returns the best **fully evaluated** candidate
so far. This is not a hard real-time guarantee: one candidate evaluation and
OS scheduling can overrun the budget. If none was feasible, active mode
stops correction and regrips; it does not switch to an increasing fallback
force. The normal hard-fault path requires a new operator command. The
separate progress-timeout stable-wait/resume behavior is unchanged.

## Model correction and safeguards

Bounded local updates estimate pressure time constants, settled bend bias
and roll-acceleration residual. Large prediction errors or rapid contact/
pressure changes reset the correction and heuristic confidence. Each new
recovery also resets it; no persistent model is saved. `confidence` is **not
a probability or a calibrated safety certificate**. Small manual assistance
cannot reliably be separated from contact response. Disable adaptation when
assisting by hand; do not use those intervals to validate autonomous control.

The planner replaces the nominal FF+PI thrust target in active mode; the two
are never added. The unused PI integrator is frozen. The same downstream
actual-rate braking, target-crossing prediction, 4-N cap, rise/fall limits,
+/-3-degree completion, sensor faults, 10-s recent-progress timeout and
60-s total recovery timeout still apply. Actual overspeed can override a
planner request of 4 N. Pressure shaping has one owner: the old stall-pressure
ramp is disabled only while predictive commands are active.

## Starting and switching modes

Do not start a second attitude/pressure controller beside an existing one.
Restart the existing controller launch with its usual arguments, adding:

```bash
recovery_predictive_enabled:=true recovery_predictive_shadow:=true
```

For example, when the bridge/robot is already running and no other attitude
or pressure controller is running:

```bash
roslaunch hugmy attitude_pressure_controller.launch \
  inchworm_action_c:=true \
  recovery_predictive_enabled:=true recovery_predictive_shadow:=true
```

This does **not** make the whole robot non-actuating: shadow means the new
planner does not control it; the previous recovery and gait still do.

The same two arguments are forwarded by `bringup.launch` when it starts the
simulation attitude allocator (`simulation:=true start_attitude_allocator:=true`).
All numeric defaults are in `config/attitude_pressure_controller.yaml`.
The launch booleans override their YAML counterparts and are read at startup;
changing ROS parameters while the node is running does not switch modes.

After prediction validation, on a supported non-human test fixture only,
restart that same controller with `recovery_predictive_shadow:=false` to
apply the predictive commands. `recovery_predictive_enabled:=false` restores
the prior FF+rate-PI recovery. Do not enable the new control first on a person.

## What to inspect

`Action C predictive recovery` logs show shadow/active, feasibility, first
and later thrust targets, pressure targets, worst predicted terminal roll,
cost, evaluated/rejected candidates, search budget exhaustion, heuristic
confidence, pressure/bend/rate prediction errors and accepted observations.
Rate/pressure/bend errors compare local one-step predictions with subsequent
measurements under the commands actually sent, never a hypothetical shadow
command. Long-horizon counterfactual predictions cannot be validated merely
by replaying a bag recorded under a different controller.

1. Record unassisted, fixture-supported recovery in shadow mode. Check both
   roll directions and different initial bends/pressures. Mark interventions.
2. Examine prediction errors and whether proposed increases actually predict
   arm extension and restoring moment, not only low cost. Frequent model
   resets/infeasible plans are a reason to revise the hypotheses, not to
   remove the guards or enlarge thrust limits.
3. Validate withheld trials and MuJoCo contact/response variations, then try
   active mode on the fixture. Compare total recovery time, peak roll/rate,
   arm extension, thrust and completion. Human-contact safety is not validated
   by this implementation or by its offline tests.

Offline checks:

```bash
python3 robots/hugmy/test/test_recovery_predictive_control.py
python3 robots/hugmy/test/test_recovery_geometry.py
python3 robots/hugmy/test/test_predictive_launch.py
```

The tests cover forward candidate search (including initially wrong-sign
lever), bounded first moves, invalid inputs/configuration, clock reset,
budget/infeasibility, adaptation reset, shadow isolation and downstream
fault/brake priority. The geometry test also uses the production rotor model
and P/T table for both correcting pairs. These do not establish hardware
closed-loop recovery, robust constraint satisfaction or manual-intervention
recognition.
