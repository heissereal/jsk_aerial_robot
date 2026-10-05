# 2026-09-28 Action C c7 / c8 / c9: recovery analysis

> Historical bag analysis and implementation notes from commit `0a2db4956`.
> Current locomotion uses [predictive locomotion](hybrid_locomotion.md);
> shared posture recovery is retained separately.

Sources: `walk_inchworm_c_7_2026-09-28-20-49-47.bag`,
`walk_inchworm_c_8_2026-09-28-20-51-59.bag`,
`walk_inchworm_c_9_2026-09-28-20-54-46.bag` under
`/home/miyamichi/rosbag/20260928`. All times below are seconds from bag start.
Offline read only; no bag replay or actuator commands were used.

## Observed blockers

- c7: recovery requested at 45.603 s; bottom was 27.7 kPa at 45.622,
  12.2 at 46.643 and 5.0 at 52.683 s. No active correction occurred before
  locomotion resumed at 56.295 s (10.69 s waiting). Another request at
  64.163 s hit the 20-s deflation/regrasp timeout at 84.172 s.
- c8: PREPARE at 48.763, 59.163 and 69.283 s waited 8.20, 7.88 and 7.63 s
  respectively before THRUST_CORRECTION. Each correction lasted only about
  0.20--0.22 s and then regrasped. The selected pair was held at zero thrust
  until both arm pressures reached 0 +/-2 kPa.
- Across the three bags there were **no** `Spinal confirmed whole-body
  correction`, `Continuous posture correction`, or `stalled for` logs.
  Thus the old hold/stall-adaptation branch did not run in these trials.
  This is more specific than assuming a captured hold was merely too low.

The source separately gated GRASP correction entry on bottom +/-5 kPa and
quiet regrasp, then gated PREPARE thrust on selected-arm pressure +/-2 kPa.
An arm-geometry completion could route to the timed 0.2-s correction without
ever confirming body motion. These are removed only in the new C recovery
path, not in the forward locomotion pulse or in A/B/D.

## Continuous-motion candidate intervals

The following intervals start at pulse ON and end at a logged completed
regrip, with no intervening shared-recovery request. Mocap world-x differences
(interpolated at log timestamps), not whole-run travel including handling:

| Bag | ON -> regrip (s) | dx (mm) |
| --- | --- | --- |
| c8 | 118.082 ->120.046 | +1.99 |
| c8 | 122.382 ->124.142 | +0.78 |
| c8 | 126.442 ->128.142 | +0.41 |
| c9 | 210.948 ->212.709 | +1.22 |
| c9 | 215.088 ->216.768 | +4.70 |
| c9 | 219.188 ->220.988 | +2.61 |
| c9 | 243.328 ->245.249 | +7.46 |

These are **not certified hands-off intervals**: the bags contain no manual
support label. Nor should these values be summed as net whole-burst progress;
motion between pulses can undo progress. Several intervals also have large
world-y changes. No estimator or optimizer-success definition was changed
on the strength of these data. Hand-assisted recoveries are not credited as
evidence of autonomous correction.

## Patch / validation

See `action_c_online_coordination.md`, “Continuous roll recovery”. Exhaust is
parallel to a synchronous, rate-limited pair command; confirmed motion holds,
stalled motion raises delivered thrust within 3 N per rotor, and predictive
gyro braking tapers near the 5-deg stop band. Fault/no-progress stops require
operator rearm rather than producing repeated short correction pulses.
Once one correction finishes, a stable regrasp rebound below the hard limit
does not use the immediate handoff path again; ordinary trend/settling rules
still apply. Bottom deflation readiness still gates locomotion resumption.

Tests cover control law and source-wired entry/resume expressions. This is
offline verification, not proof of closed-loop hardware stability or a
counterfactual replay of what the robot would have done with different thrust.
