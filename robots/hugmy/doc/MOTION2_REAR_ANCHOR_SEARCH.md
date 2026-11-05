# Experimental Motion-2 rear-anchor search

The controller has an **opt-in** short-pulse Motion 2 for object-dependent
contact. The normal controller remains the default (`false`). No learned
friction coefficient, contact-force estimate, or extra motion mode is needed.

## Behavior

1. Vent the two front arms with the existing front-vent phase. Enter Lift
   with the rear pair at the ordinary grasp pressure (normally 30 kPa).
2. Apply the existing front-rotor extension search. Each front arm targets
   `hybrid_trajectory_min_extension_rad` of **relative** extension from its
   own grasp pose (default 15 deg), not the same absolute 45-deg angle.
3. If both front arms are powered at at least half of the configured initial
   pulse maximum but the **smaller** measured front extension does not grow
   by 1 deg for 0.6 s, reduce commanded rear pressure by 2.5 kPa. Repeat
   only if the pair remains stalled; never go below the configured floor.
   Before the configured release-start fraction, an initial zero-progress
   stall requires a second 0.6-s confirmation.
4. When **both** front arms have measured the required extension, are still
   powered, and pass the existing roll/mismatch guards, start Motion 3
   immediately. Pitch/height load relief remains diagnostic, **not** a hard
   condition of this experimental handoff. An arm that extends by itself
   cannot complete Motion 2.
5. A roll-risk indication cancels further unloading and requests the rear
   grasp pressure again. Outer roll recovery, hard roll limits and emergency
   extension retain independent control.

The Motion-3 pull, regrip, sliding probe, and bottom-pressure controller are
not modified by this option. Thus success **cannot** be inferred merely
from the new paired-extension handoff: retained axial displacement and
contact stability still need to be measured on the real target.

## Parameters

Enable via `roslaunch hugmy human_arm_locomotion.launch` arguments:

```bash
roslaunch hugmy human_arm_locomotion.launch \
  hybrid_motion2_rear_anchor_search_enabled:=true \
  hybrid_motion2_rear_min_pressure_kpa:=20.0 \
  hybrid_motion2_release_start_ratio:=0.10
```

The latter two are the primary per-object tuning parameters. The relative
front-extension target can also be changed using the **existing**
`hybrid_trajectory_min_extension_rad` argument, one variable at a time.

Do not infer a pressure preset directly from the bags: Oct 7 `pink_18`
was an approximately 100-mm elliptical, fabric-covered arm mannequin,
whereas Nov 5 `ento_1` was a 160-mm smooth cylinder. Geometry, covering and
controller all differed. Start with a physical catch/tether, test commands
at zero thrust first, then progress to supervised low-authority trials.
Compare roll, front-angle dwell, measured rear pressure,
Motion-2/3 duration and **retained** displacement before selecting a floor.

Logs contain `Hybrid Motion-2 rear-anchor search` for each pressure step and
`Hybrid Motion-2 short-pulse extension complete` for the direct handoff.
Unit regression: `python3 robots/hugmy/test/test_hybrid_motion2_anchor.py`.

Disable the option to reproduce the pre-change control path without
reverting code. An emergency or recovery behavior has not been redesigned
in this patch.
