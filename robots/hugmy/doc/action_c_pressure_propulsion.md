# Action C: pressure-aware powered flexion (2026-09-29)

> Historical implementation, preserved in commit `0a2db4956`. Its execution
> path and settings have been removed. The text below is archival, not current
> startup guidance; use [predictive locomotion](hybrid_locomotion.md).

Normal C enables `action_c_propulsion_enabled: true`. Set it false to restore
the previous tapered fixed-ratio powered pull and coordination timing (also
restore the timeout to 1.5 s for the previous complete configuration). Experimental
reference/individual/online-pressure-sign modes disable the new learner to
avoid competing actuator ownership. Action A/D and shared recovery are unchanged.

## Motion and timing

The extension pulse, front release, rear release and all existing angle, gyro,
roll, pressure-freshness and reverse-travel guards remain active. Each arm's
pressure command edge, first measured 0.5 kPa rise, subsequent rise rate and
gyro-confirmed flexion onset (0.5 deg rebound, positive bend rate for 40 ms)
are observed. These are empirical response estimates, not pneumatic constants:
contact and changing volume affect them.

Each arm now has a latched extension -> flexion transition, independent of the
other arm. After 100 ms, reaching the 8-deg target switches that arm to inflation
and bounded residual thrust. After at least 2 deg extension, an 80-ms prediction
of target/bend-margin arrival, or reaching half the hard extension-rate limit,
can switch it earlier. The other arm continues its own phase. The previous
hard guards always take priority: a violation still cancels powered pulling.
The existing small roll trim is retained, not assumed to explain all pair
asymmetry. Once flexing, an arm cannot return to extension or re-boost within
the pulse, including when its roll trim relaxes.
Once the phases differ, pair mismatch compares each arm's attained peak
extension, not one arm's re-flexion against the other's extension. Hard
individual limits still use current angles/rates; a true peak-extension
mismatch still stops the movement.

The first trial has zero *learned* pressure lead, but the predictive transition
above is active immediately. Pressure observations continue through EVALUATE,
REGRIP and SETTLE, including zero-thrust re-inflation after an extension guard.
Identification uses measured pressure latency and rise rate to 50% of the
commanded increment; gyro-confirmed flexion onset is a separate observation.
Missing bend data does not invalidate valid pressure data or invent a flexion.
Response measurements span at most 6 s. Only fresh data are used; clock reversal
invalidates the sample. No samples are borrowed from shared posture recovery.

At a trial boundary without a recovery, valid pressure observations may update
lead even if movement was incomplete, backward or guarded. This is **not** a
successful gait sample and never authorizes a thrust increase. Lead changes by
at most 30 ms/boundary, bounded to 200 ms; it is fixed during each trial. Learned
early inflation requires 150 ms and half the requested extension. Peak extension
is remembered for each arm, so re-bending does not erase a reached target. The
full measured extension target is still required for pull-amplitude adaptation.

During powered flexion, front pressure rises and rear pressure stays low.
Residual thrust stays on after rebound (no immediate taper to zero). When both
arms have re-bent and front measured pressure is ready, rear inflation starts
while residual thrust continues. Both rear pressures must be within tolerance
for 150 ms before thrust is zeroed. The release gate applies until rear
inflation starts; it is replaced by fresh finite pressure and regrip-arrival
checks during inflation, not silently disabled throughout the movement.

The default flexion deadline is 5 s (`action_c_pressure_pull_timeout_sec`,
clamped to <=6 s); rear regrip has a separate 3-s deadline
(`action_c_powered_regrip_timeout_sec`, <=5 s). Front pressure OR bend failure
at timeout stops the movement, rather than accepting missing re-flexion.
Angle, roll/rate, relative tilt and reverse-travel protection remains active
throughout; no unlimited hold is allowed. A timeout is never a completed sample.

## Forward-force estimate

`Action C PROPULSION` logs commanded force projected onto the horizontal
initial-body-x direction, plus powered-pull forward impulse. Geometry includes
measured rotor/Neuron bend, fixed cant, body roll/pitch and Spinal yaw change.
Backward travel reverses this projection. This is **not measured contact force,
net acceleration, or proof that the robot lifted off**. The start body-x must
be aligned with the mocap x travel axis to compare this force direction with
mocap progress; marker quaternion is deliberately not assumed body-aligned.
Retained progress scoring itself still uses the existing signed mocap x.

## Adaptation ownership

At a trial boundary: pressure timing first, powered-pull strength second,
existing left/right feed-forward third, outer T/bottom/burst search last.
When any calibration changes, other inputs stay fixed for re-evaluation and
old best scores are invalidated. The old pressure-lead learner is suppressed
while this timing learner owns the phase; bounded live pair assist stays active.

Only trials where every pulse extended both arms, completed powered re-flexion
and confirmed powered rear regrip, with both front rotors producing a positive
estimated forward component for at least 0.1 s, and without guard, timeout,
recovery or excessive roll/rate may raise
pull strength. Insufficient **nonnegative** retained displacement and positive
estimated pull impulse permit +0.1 N of residual per trial. Total residual is
capped by `action_c_pressure_pull_max_n` (1.5 N) and the actual ON-end output.
The ON amplitude and powered-pull duration are not changed by this learner.
Once capped, the outer optimizer can continue its existing search. Reverse
slip, incomplete motion or hand-assisted motion cannot establish effectiveness;
there is no automatic manual-intervention detector.

Diagnostics include `action_c_pull_extra_n`, `action_c_pull_forward_impulse_ns`
and `Action C PROPULSION commit`, `armN SOFT FLEXION`, `POWERED REGRIP start`
and `PRESSURE PULL done` logs. Learning resets on a new task/direction;
it is not persisted across controller restarts.

The hard optimizer cap rule is unchanged. A real extension-rate/angle guard at
3 N sets `hard_max=min(old_max,max(3,N_trial-step))=3 N`. That is a safety latch,
not an optimum found by reward optimization. This change prevents anticipated
limits from becoming violations where possible; it does not unlock a hard cap
on the basis of failed trials. Only pair-mismatch caps have the existing safe
re-probe mechanism.

Offline tests/build do not establish forward motion on hardware. In particular,
if physical pressure response exceeds the bounded lead window, unloading and
re-flexion may still fail to overlap. Do not increase ON thrust to hide that;
inspect measured delay/rate, bend and force logs first. Existing caps are not
human-contact force limits. Use a supported non-human test fixture.
