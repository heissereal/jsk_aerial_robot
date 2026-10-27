#!/usr/bin/env python3
"""Exercise production single-pass scheduling, pressure handoff and force bounds."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
source=r'''
#include <hugmy/control/hybrid_single_pass.h>
#include <cassert>
#include <iostream>
using namespace hugmy::hybrid;
int main(){
 Config c;SinglePassConfig k;Model model;model.reset(c);
 // A real grasp may start above the nominal 150-deg model ceiling. The
 // captured pose receives only a bounded 5-deg allowance: its initial state
 // is valid, while continued pneumatic over-flexion still trips the guard.
 Pair deep_grasp{{2.05,155.*3.141592653589793/180.}};
 const double deep_limit=contactRelativeBendMaximum(
     c.bend_max,deep_grasp,c.grasp_bend_margin);
 State deep_state;deep_state.fresh=true;deep_state.bend=deep_grasp;
 deep_state.start_bend=deep_grasp;deep_state.floor={{.523599,.523599}};
 assert(deep_limit>deep_grasp[1] && deep_limit<c.bend_max+.18);
 auto deep_config=c;deep_config.bend_max=deep_limit;
 assert(Planner::valid(deep_state,deep_config));
 deep_state.bend[1]=deep_limit+.001;
 assert(!Planner::valid(deep_state,deep_config));
 BendCoordinateRate observed;
 for(int i=0;i<100;++i)observed.update(1.5-.1*i*.02,i*.02);
 assert(std::abs(observed.rate+.1)<1e-5);
 auto corrected=reduceAdverseRoll({{3.,3.}},{{.03,-.03}},-.1,0.);
 assert(corrected[0]==3. && corrected[1]<3.); // negative roll requires positive moment
 auto opposite=reduceAdverseRoll({{3.,3.}},{{-.03,.03}},-.1,0.);
 assert(opposite[0]<3. && opposite[1]==3.); // geometry sign, not fixed motor index
 // Lift preserves the common extension force and adds the helpful side.  It
 // must not starve the lagging/adverse arm unless the helpful side is capped.
 auto supported=supportExtensionAndRoll({{3.,3.}},{{.03,-.03}},-.1,0.,6.);
 assert(supported[0]>3. && supported[1]==3.);
 auto capped=supportExtensionAndRoll({{6.,6.}},{{.03,-.03}},-.1,0.,6.);
 assert(capped[0]==6. && capped[1]<6.);
 // With an asymmetric learned pair, worsening roll raises the helpful small
 // side at least to the existing pair level before requesting extra moment.
 auto sim_balanced=supportExtensionAndRoll({{2.,.5}},{{-.03,.03}},-.1,0.,6.);
 assert(sim_balanced[1]>=2. && sim_balanced[0]<=2.);
 // Hardware pair sign is independent of the contact-lever sign reversal.
 auto verified=verifiedPairRollMoments({{-.02,.03}});
 assert(verified[0]>.0 && verified[1]<.0);
 auto sim_verified=verifiedPairRollMoments({{-.02,.03}},true);
 assert(sim_verified[0]<.0 && sim_verified[1]>.0);
 auto sim_crossed_geometry=verifiedPairRollMoments({{.02,-.03}},true);
 assert(sim_crossed_geometry[0]<.0 && sim_crossed_geometry[1]>.0);
 auto positive_roll=addRestoringRollSupport({{2.,2.}},verified,.2,.1,6.);
 assert(positive_roll[0]==2. && positive_roll[1]>2.); // +roll -> arm 3/2 side
 auto negative_roll=addRestoringRollSupport({{2.,2.}},verified,-.2,-.1,6.);
 assert(negative_roll[0]>2. && negative_roll[1]==2.); // -roll -> arm 0/1 side
 auto damp_negative=dampLocomotionRoll({{2.,.5}},sim_verified,-.1,-.1,6.);
 assert(damp_negative[1]>=2. && damp_negative[1]<=2.75 &&
        damp_negative[0]<=2.75);
 // Motion 2 roll damping is never allowed to raise either released arm.  It
 // may only remove adverse thrust and is bounded per allocation update.
 auto lift_damped=dampLiftRollReductionOnly(
     {{2.,.5}},sim_verified,-.1,-.1);
 assert(lift_damped[0]<=2. && lift_damped[1]<=.5 &&
        (lift_damped[0]<2. || lift_damped[1]<.5));
 auto lift_positive=dampLiftRollReductionOnly(
     {{.5,2.}},sim_verified,.1,.1);
 assert(lift_positive[0]<=.5 && lift_positive[1]<=2. &&
        (lift_positive[0]<.5 || lift_positive[1]<2.));
 auto held=observed.rate;assert(observed.update(99.,observed.stamp)==held);
 assert(observed.update(1.,5.)==0.);assert(observed.update(NAN,6.)==0.);
 assert(k.valid());auto bad=k;bad.rear_low_seconds=0;assert(!bad.valid());
 // The shared Motion-2/3 clock is continuous, has zero endpoint velocity and
 // pauses only when the measured arm is behind the reference.  An arm which
 // moves ahead must let the reference catch up instead of inducing chatter.
 assert(minimumJerkPosition(0.)==0. && minimumJerkPosition(1.)==1.);
 assert(minimumJerkVelocity(0.)==0. && minimumJerkVelocity(1.)==0.);
 assert(minimumJerkAcceleration(.25)>0. &&
        minimumJerkAcceleration(.75)<0.);
 // Rate is observed directly; angular acceleration is only a filtered,
 // bounded short-horizon predictor and is never integrated into bend.
 BendKinematicsObserver motion;
 motion.reset(0.);
 motion.update(-1.,.02,.04,.08,20.);
 assert(motion.rate<0. && motion.acceleration<0. &&
        std::abs(motion.acceleration)<=20.);
 assert(motion.predictAngle(1.5,.08,.25)<1.5);
 BendTrajectory bend_path;
 bend_path.configure({{1.5,1.5}},{{1.0,1.0}},1.0);
 bend_path.sample(.5);
 assert(std::abs(bend_path.target[0]-1.25)<1e-9 && bend_path.rate[0]<0.);
 bend_path.advance(.1,{{1.40,1.40}},.05,.10);
 assert(bend_path.clock_scale==0. && std::abs(bend_path.elapsed-.5)<1e-9);
 bend_path.advance(.1,{{1.10,1.10}},.05,.10);
 assert(bend_path.clock_scale==1. && std::abs(bend_path.elapsed-.6)<1e-9);
 // At the same measured angle, a stationary arm is behind a moving
 // reference and must slow the path. Matching extension velocity keeps it
 // moving. This is the rate/acceleration-aided phase observer used by both
 // Motion 2 and Motion 3.
 BendTrajectory predictive;
 predictive.configure({{1.5,1.5}},{{1.0,1.0}},1.0);
 predictive.sample(.5);
 predictive.advancePredictive(.02,predictive.target,{{0.,0.}},{{0.,0.}},
     .02,.10,.08,.08,1.5,1.5,.25);
 assert(predictive.clock_scale<1.);
 BendTrajectory moving_path;
 moving_path.configure({{1.5,1.5}},{{1.0,1.0}},1.0);
 moving_path.sample(.5);
 moving_path.advancePredictive(.02,moving_path.target,moving_path.rate,
     moving_path.acceleration,.02,.10,.08,.08,1.5,1.5,.25);
 assert(moving_path.clock_scale>.9);
 bad=k;bad.slip_no_progress_seconds=.5;assert(!bad.valid());
 bad=k;bad.slip_roll_abort_angle=21.*M_PI/180.;assert(!bad.valid());
 bad=k;bad.rear_preload_kpa=31.;assert(!bad.valid());
 bad=k;bad.reused_lift_rise=8.1;assert(!bad.valid());
 bad=k;bad.reused_event_checkpoint_margin=.2;assert(!bad.valid());
 bad=k;bad.reused_event_search_rise=.01;assert(!bad.valid());
 bad=k;bad.reused_lift_roll_grace=1.1;assert(!bad.valid());
 bad=k;bad.rear_vent_lead_margin=1.1;assert(!bad.valid());
 bad=k;bad.rear_vent_start_margin=.3;assert(!bad.valid());
 bad=k;bad.slip_thrust_max=6.1;assert(!bad.valid());
 bad=k;bad.slip_pressure_max=50.1;assert(!bad.valid());
 bad=k;bad.slip_pressure_tracking_tolerance=.4;assert(!bad.valid());
 bad=k;bad.slip_extension_hold_margin=.01;assert(!bad.valid());
 bad=k;bad.slip_hard_extension_margin=.2;assert(!bad.valid());
 bad=k;bad.slip_extension_brake_fall=.1;assert(!bad.valid());
 bad=k;bad.slip_max_extension=.05;assert(!bad.valid());
 bad=k;bad.slip_pressure_drop_max=21.;assert(!bad.valid());
 bad=k;bad.identification_sync_assist_max=2.1;assert(!bad.valid());
 bad=k;bad.identification_rate_deadband=.2;assert(!bad.valid());
 bad=k;bad.identification_rate_control_gain=10.1;assert(!bad.valid());
 bad=k;bad.identification_rate_control_fall=12.1;assert(!bad.valid());
 bad=k;bad.unload_pair_memory=bad.unload_confirm-.01;assert(!bad.valid());
 bad=k;bad.unload_pair_memory=.51;assert(!bad.valid());
 bad=k;bad.reused_lift_pressure_start_ratio=.95;assert(!bad.valid());
 bad=k;bad.trajectory_prediction_horizon=.5;assert(!bad.valid());
 bad=k;bad.trajectory_clock_max_scale=.5;assert(!bad.valid());
 bad=k;bad.trajectory_windup_pitch=k.unload_pitch-.01;assert(!bad.valid());
 bad=k;bad.trajectory_windup_pitch_margin=.001;assert(!bad.valid());
 bad=k;bad.trajectory_windup_thrust_max=6.1;assert(!bad.valid());
 bad=k;bad.lift_rear_release_start_ratio=.9;assert(!bad.valid());
 bad=k;bad.lift_rear_pressure_floor=31.;assert(!bad.valid());
 bad=k;bad.pull_up_hold_flex_ratio=.1;assert(!bad.valid());
 bad=k;bad.pull_forward_start_flex_ratio=.7;
 bad.pull_full_thrust_flex_ratio=.6;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_thrust=3.1;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_duration=.05;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_duration=3.1;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_front_start=3.;
 bad.pull_rear_positive_front_full=3.;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_rise=.1;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_max_pressure=21.;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_pressure_fade=.5;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_max_pressure=18.;
 bad.pull_rear_positive_pressure_fade=13.;assert(!bad.valid());
 bad=k;bad.pull_rear_positive_flex_end=.2;assert(!bad.valid());
 bad=k;bad.pull_terminal_rear_lift_thrust=3.1;assert(!bad.valid());
 bad=k;bad.pull_terminal_rear_lift_duration=.05;assert(!bad.valid());
 bad=k;bad.pull_terminal_rear_lift_rise=.5;assert(!bad.valid());
 bad=k;bad.pull_terminal_rear_lift_fall=31.;assert(!bad.valid());
 bad=k;bad.pull_terminal_front_pressure_tolerance=.2;assert(!bad.valid());
 bad=k;bad.pull_terminal_rear_pressure_tolerance=.2;assert(!bad.valid());
 bad=k;bad.pull_terminal_anchor_wait_timeout=.2;assert(!bad.valid());
 bad=k;bad.pull_support_transfer_min_flex_ratio=.2;assert(!bad.valid());
 bad=k;bad.pull_golden_roll_limit=.05;assert(!bad.valid());
 bad=k;bad.rear_pressure_adaptation_max=31.;assert(!bad.valid());
 bad=k;bad.rear_pressure_adaptation_roll_limit=.05;assert(!bad.valid());
 bad=k;bad.rear_motion_pressure_cap=-.1;assert(!bad.valid());
 bad=k;bad.rear_motion_pressure_cap=15.1;assert(!bad.valid());
 bad=k;bad.rear_release_angle_min=.01;assert(!bad.valid());
 bad=k;bad.rear_release_angle_max=.01;assert(!bad.valid());
 bad=k;bad.rear_release_angle_min=.08;
 bad.rear_release_angle_max=.07;assert(!bad.valid());
 bad=k;bad.rear_release_angle_filter_tau=.5;assert(!bad.valid());
 bad=k;bad.rear_release_assist_deadband=.04;assert(!bad.valid());
 bad=k;bad.rear_release_assist_confirm=.01;assert(!bad.valid());
 bad=k;bad.rear_release_pressure_trim_max=11.;assert(!bad.valid());
 bad=k;bad.rear_release_thrust_trim_max=1.6;assert(!bad.valid());
 bad=k;bad.rear_release_mismatch_soft=.2;
 bad.rear_release_mismatch_hard=.1;assert(!bad.valid());
 bad=k;bad.rear_release_realization_ratio=.1;assert(!bad.valid());
 bad=k;bad.regrip_rear_hold_pressure=31.;assert(!bad.valid());
 bad=k;bad.pull_rear_release_start_ratio=.7;
 bad.pull_rear_release_end_ratio=.6;assert(!bad.valid());
 bad=k;bad.goal_progress_max_scale=1.51;assert(!bad.valid());
 bad=k;bad.bottom_kpa=31;assert(!bad.valid());bad=k;bad.pull_seconds=NAN;assert(!bad.valid());
 bad=k;bad.pull_min_seconds=4.1;assert(!bad.valid());
 bad=k;bad.pull_min_seconds=NAN;assert(!bad.valid());
 bad=k;bad.pull_adaptation_step=1.1;assert(!bad.valid());
 bad=k;bad.goal_conditioning_start_ratio=.99;assert(!bad.valid());
 // Most legacy scenarios below isolate their original phase-transition
 // assertions. The dedicated catch test exercises the new default-enabled
 // terminal event explicitly.
 const auto terminal_k=k;
 k.pull_terminal_rear_lift_enabled=false;
 auto geom=[](size_t,double a,double,double){Force f;f.forward=.7*std::sin(a);f.up=std::max(0.,std::cos(a));return f;};
 auto prime_powered_baseline=[](SinglePass& pass,const SinglePassInput& sample) {
   pass.onset_this_stroke={{true,true}};
   pass.powered_extension_reference=sample.s.bend;
   pass.powered_extension_reference_valid={{true,true}};
   pass.powered_start_height=sample.height;
   pass.powered_start_pitch=sample.directional_pitch;
   pass.powered_body_reference_time=pass.phase_time;
   pass.powered_body_reference_valid=true;
 };
 SinglePassInput in;in.s.fresh=true;in.s.bend={{1.57,1.57}};in.s.start_bend=in.s.bend;
 in.s.pressure={{30.,30.}};in.s.command=in.s.pressure;in.s.floor={{.7,.7}};
 in.rear_released=true;in.speed_valid=true;in.contact_available=true;in.front_contact=true;
 in.s.bottom=30.;in.s.bottom_command=30.;in.s.now=1.;
 // A support-transfer latch from the previous stroke must not override the
 // next Motion-2 rear-pressure trajectory.  Lift owns a fresh 30 -> floor
 // schedule after the all-arm regrasp boundary.
 SinglePass fresh_rear_schedule;
 fresh_rear_schedule.support_transfer_started=true;
 fresh_rear_schedule.enter(SinglePass::Lift,in);
 assert(!fresh_rear_schedule.support_transfer_started);
 // Rear-support safety regulates roll created by this stroke, not a stable
 // contact-dependent absolute bias.  The adapter retains the independent
 // absolute hard guard outside SinglePass.
 SinglePass relative_roll;auto relative_roll_input=in;
 relative_roll_input.s.roll=.15;relative_roll_input.s.roll_rate=.08;
 relative_roll.slip_tested=true;relative_roll.begin(relative_roll_input);
 relative_roll.enter(SinglePass::Lift,relative_roll_input);
 relative_roll.update(relative_roll_input,c,k,model,geom);
 assert(relative_roll.rear_support_safety_override==0.);
 relative_roll_input.s.roll=.25;relative_roll_input.s.roll_rate=.08;
 relative_roll.update(relative_roll_input,c,k,model,geom);
 assert(relative_roll.rear_support_safety_override>0.);
 relative_roll.rear_candidate_requested=true;
 relative_roll.rear_candidate_distinct=true;
 assert(!relative_roll.selectedRearCandidateIdentifiable());
 relative_roll.rear_candidate_realized=true;
 assert(relative_roll.selectedRearCandidateIdentifiable());
 // Retained-goal conditioning is enabled only with a comparable measured
 // full stride. It shortens reach but keeps a bounded unloading stroke, then
 // applies one common pull envelope to both rotors.
 SinglePass full_goal,short_goal;
 for(auto* q:{&full_goal,&short_goal}) {
   q->slip_tested=true;q->calibrated=true;q->observed_extension=k.extension_rad;
   q->trajectory.record(in.s.bend,0.,0.,.020,4.,10.,0.,true,true,true);
 }
 auto full_input=in,short_input=in;
 full_input.remaining=.020;short_input.remaining=.005;
 full_goal.begin(full_input);short_goal.begin(short_input);
 assert(!full_goal.goal_conditioned && short_goal.goal_conditioned);
 assert(std::abs(short_goal.predicted_stride-.020)<1e-9);
 SinglePass nominal_goal;auto nominal_input=in;nominal_input.remaining=.005;
 nominal_goal.begin(nominal_input);
 assert(nominal_goal.goal_conditioned &&
        !nominal_goal.stride_estimate_measured &&
        std::abs(nominal_goal.predicted_stride-.020)<1e-9 &&
        std::abs(nominal_goal.desired_stride-.005)<1e-9 &&
        std::abs(nominal_goal.goal_scale-.30)<1e-9);
 // Stable endpoint noise must not turn an essentially nominal 20-mm stroke
 // into a final-goal special case, because special cases are intentionally
 // excluded from trajectory-candidate comparison.
 SinglePass near_nominal;auto near_nominal_input=in;
 near_nominal_input.remaining=.0189;
 near_nominal.begin(near_nominal_input);
 assert(!near_nominal.goal_conditioned);
 // A 17-mm remainder also repeats the proven 20-mm stroke. Conditioning it
 // early produced an 8-mm step followed by ineffective millimetre-scale
 // retries, although a second full stroke would finish in one cycle.
 SinglePass minimum_cycle;auto minimum_cycle_input=in;
 minimum_cycle_input.remaining=.017;
 minimum_cycle.begin(minimum_cycle_input);
 assert(!minimum_cycle.goal_conditioned);
 // Distance scheduling sees the full task remainder. A proven 10-mm stroke
 // is repeated for a 50-mm task, rather than being multiplied by a fixed
 // factor on every cycle.
 SinglePass long_task;auto long_input=in;
 long_task.slip_tested=true;long_task.calibrated=true;
 long_task.trajectory.record(in.s.bend,0.,0.,.0095,4.,10.,0.,true,true,true);
 long_task.trajectory.record(in.s.bend,0.,0.,.0105,4.,10.,0.,true,true,true);
 long_input.remaining=.050;
 long_task.begin(long_input);
 assert(std::abs(long_task.predicted_stride-.010)<1e-9);
 assert(std::abs(long_task.desired_stride-.010)<1e-9);
 assert(!long_task.goal_conditioned && long_task.goal_scale==1.);
 // One safe large stroke may reduce an oversized final command, but the same
 // single observation must not strengthen a long-range command.
 SinglePass provisional_final;auto provisional_input=in;
 provisional_final.slip_tested=true;provisional_final.calibrated=true;
 provisional_final.trajectory.record(in.s.bend,0.,0.,.021,4.,10.,0.,true,true,true);
 provisional_input.remaining=.009;
 provisional_final.begin(provisional_input);
 assert(std::abs(provisional_final.predicted_stride-.021)<1e-9);
 assert(std::abs(provisional_final.desired_stride-.009)<1e-9);
 assert(provisional_final.goal_conditioned && provisional_final.goal_scale<1.);
 SinglePass provisional_long;provisional_long.slip_tested=true;
 provisional_long.calibrated=true;provisional_long.trajectory=provisional_final.trajectory;
 provisional_input.remaining=.050;
 provisional_long.begin(provisional_input);
 assert(std::abs(provisional_long.predicted_stride-.020)<1e-9);
 assert(!provisional_long.goal_conditioned);
 // A 20-mm target with an 8-mm stable stride is tiled as two 10-mm strokes,
 // not three 8+8+4-mm strokes and not a fixed multiplier on every goal.
 SinglePass final_strong;auto final_strong_input=in;
 final_strong.slip_tested=true;final_strong.calibrated=true;
 final_strong.trajectory.record(in.s.bend,0.,0.,.0078,4.,10.,0.,true,true,true);
 final_strong.trajectory.record(in.s.bend,0.,0.,.0082,4.,10.,0.,true,true,true);
 final_strong_input.remaining=.020;
 final_strong.begin(final_strong_input);
 assert(std::abs(final_strong.predicted_stride-.008)<1e-9);
 assert(std::abs(final_strong.desired_stride-.010)<1e-9);
 assert(std::abs(final_strong.goal_scale-1.25)<1e-9);
 assert(final_strong.goal_conditioned);
 // With 12 mm remaining, the same model computes 1.5 from the distance; 1.5
 // is only the cap/result here, never a blind per-cycle multiplier.
 final_strong.initialized=false;
 final_strong_input.remaining=.012;
 final_strong.begin(final_strong_input);
 assert(std::abs(final_strong.desired_stride-.012)<1e-9);
 assert(std::abs(final_strong.goal_scale-1.5)<1e-9);
 assert(final_strong.goal_conditioned);
 full_goal.enter(SinglePass::Lift,full_input);
 short_goal.enter(SinglePass::Lift,short_input);
 auto full_lift=full_goal.update(full_input,c,k,model,geom);
 auto short_lift=short_goal.update(short_input,c,k,model,geom);
 // Remaining distance changes the pressure-driven pull, not the unloading
 // evidence required to enter it.  A short terminal correction must still
 // unload the contact before applying horizontal force.
 assert(std::abs(short_goal.active_extension_target[0]-
                 full_goal.active_extension_target[0])<1e-9);
 // A previously measured per-arm event pose is reused only as a timing hint.
 // The deeper fixed angle remains a common bounded safety/search limit.
 SinglePass asymmetric_reach;auto asymmetric_input=in;
 asymmetric_reach.slip_tested=true;asymmetric_reach.calibrated=true;
 asymmetric_reach.identified_bend_target={{1.37,1.27}};
 asymmetric_reach.observed_extension_by_arm={{.10,.20}};
 asymmetric_reach.begin(asymmetric_input);
 asymmetric_reach.enter(SinglePass::Lift,asymmetric_input);
 asymmetric_reach.update(asymmetric_input,c,k,model,geom);
 assert(std::abs(asymmetric_reach.active_extension_target[0]-
                 asymmetric_reach.active_extension_target[1])<1e-9);
 assert(std::abs(asymmetric_reach.active_bend_target[0]-
                 k.identification_bend)<1e-9);
 assert(std::abs(asymmetric_reach.active_bend_target[1]-
                 k.identification_bend)<1e-9);
 assert(std::abs(asymmetric_reach.predicted_unload_bend[0]-1.47)<1e-9);
 assert(std::abs(asymmetric_reach.predicted_unload_bend[1]-1.37)<1e-9);
 // Both goal sizes use the same continuous pressure-driven return trajectory.
 // Goal conditioning may shorten the retained horizontal impulse, but it must
 // not create a discontinuous pressure or angle command at Motion 3 entry.
 full_input.s.bend={{1.0,1.0}};short_input.s.bend=full_input.s.bend;
 full_goal.grasp_reference={{1.5,1.5}};short_goal.grasp_reference={{1.5,1.5}};
 full_goal.lift_up=short_goal.lift_up=1.;
 full_goal.pull_forward=short_goal.pull_forward=2.;
 full_goal.lift_force=short_goal.lift_force=2.;
 full_goal.enter(SinglePass::Pull,full_input);
 short_goal.enter(SinglePass::Pull,short_input);
 full_input.s.bend={{1.35,1.35}};short_input.s.bend=full_input.s.bend;
 full_input.s.pressure=short_input.s.pressure={{20.,20.}};
 full_input.s.command=short_input.s.command={{30.,30.}};
 full_input.s.thrust=short_input.s.thrust={{1.,1.}};
 full_lift=full_goal.update(full_input,c,k,model,geom);
 short_lift=short_goal.update(short_input,c,k,model,geom);
 assert(full_goal.flex_trajectory.active && short_goal.flex_trajectory.active);
 assert(full_lift.angle_target[0]>=1.0 && full_lift.angle_target[0]<=1.5 &&
        short_lift.angle_target[0]>=1.0 && short_lift.angle_target[0]<=1.5);
 assert(full_lift.pressure[0]>=0. && full_lift.pressure[0]<30. &&
        short_lift.pressure[0]>=0. && short_lift.pressure[0]<30.);
 assert(short_goal.goal_pull_envelope==0.);
 // Golden-time force scheduling is bend-progress based.  At the beginning of
 // pneumatic flexion it preserves unloading support and admits almost no new
 // horizontal pull.  Past the configured early overlap point horizontal pull
 // is fully available while upward support is being removed for landing.
 SinglePass golden;auto golden_input=in;
 golden.slip_tested=true;golden.calibrated=true;
 golden.grasp_reference={{1.5,1.5}};golden.lift_up=1.5;
 golden.pull_forward=2.5;golden.lift_force=3.;
 golden.unload_event_thrust={{3.,3.}};
 golden.load_relief_detected=true;
 golden.initialized=true;
 golden_input.s.bend={{1.,1.}};golden_input.s.pressure={{15.,15.}};
 golden_input.s.command={{15.,15.}};golden_input.s.thrust={{3.,3.}};
 golden_input.rear_pressure=0.;
 // A front load-relief precursor is not evidence that the rear pair moved.
 // It must not start the rear rotors without measured rear-angle deficit.
 SinglePass precursor=golden;auto precursor_input=golden_input;
 precursor.load_relief_detected=false;
 precursor.load_relief_candidate=true;
 precursor.enter(SinglePass::Pull,precursor_input);
 auto precursor_output=precursor.update(precursor_input,c,k,model,geom);
 assert(!precursor.rear_positive_started &&
        precursor_output.rear_positive_thrust==0. &&
        !precursor.load_relief_detected);
 golden.rear_grasp_reference={{1.6,1.4}};
 golden.rear_grasp_reference_valid=true;
 golden_input.rear_kinematics_valid=true;
 golden_input.rear_bend={{1.6,1.4}};
 golden_input.rear_rate={{0.,0.}};
 golden.enter(SinglePass::Pull,golden_input);
 // The positive rear assist follows the bounded front-pair force. Its state
 // spans Motion 2/3 and remains separate from the reverse-assist search.
 SinglePassOutput golden_start;
 for(int i=0;i<20 && !golden.rear_positive_started;++i)
   golden_start=golden.update(golden_input,c,k,model,geom);
 assert(golden.golden_window_active && golden.golden_flex_progress<.1 &&
        golden.golden_desired_up>golden.golden_desired_forward);
 assert(golden.rear_positive_started &&
        golden_start.rear_positive_thrust>0. &&
        golden_start.rear_positive_thrust<=k.pull_rear_positive_thrust &&
        golden_start.rear_reverse_thrust==0.);
 const double delivered_before_pressure_gate=golden.rear_positive_elapsed;
 golden_input.rear_pressure=k.pull_rear_positive_max_pressure+
     k.pull_rear_positive_pressure_fade+1.;
 auto rear_pulse_pressure_guard=golden.update(golden_input,c,k,model,geom);
 assert(rear_pulse_pressure_guard.rear_positive_thrust==0. &&
        golden.rear_positive_pressure_envelope==0. &&
        golden.rear_positive_elapsed==delivered_before_pressure_gate);
 golden_input.rear_pressure=0.;
 golden.rear_positive_elapsed=k.pull_rear_positive_duration;
 auto rear_pulse_finished=golden.update(golden_input,c,k,model,geom);
 for(int i=0;i<20 && rear_pulse_finished.rear_positive_thrust>0.;++i) {
   golden_input.s.thrust={{rear_pulse_finished.rear_positive_thrust,
                          rear_pulse_finished.rear_positive_thrust}};
   rear_pulse_finished=golden.update(golden_input,c,k,model,geom);
 }
 assert(rear_pulse_finished.rear_positive_thrust==0.);
 golden.rear_positive_elapsed=0.;
 golden_input.s.bend={{1.38,1.38}};
 golden_input.s.rate={{1.,1.}};
 auto golden_pull=golden.update(golden_input,c,k,model,geom);
 assert(golden.golden_flex_progress>.6 &&
        golden.golden_desired_forward>golden_start.desired_forward &&
        golden.golden_desired_up<golden_start.desired_up &&
        golden_pull.rear_positive_thrust>0.);
 // Rear pressure is not physical release.  Capture unequal absolute rear
 // bends and close one common pressure/thrust loop around the less-released
 // arm. Under-release lowers pressure and adds positive assist; a large
 // pair mismatch removes only those aggressive corrections.
 SinglePass rear_angle_fb;auto rear_angle_input=golden_input;
 rear_angle_fb.initialized=true;rear_angle_fb.slip_tested=true;
 rear_angle_fb.calibrated=true;rear_angle_fb.grasp_reference={{1.5,1.5}};
 rear_angle_fb.lift_up=1.5;rear_angle_fb.pull_forward=2.5;
 rear_angle_fb.lift_force=3.;rear_angle_fb.load_relief_detected=true;
 rear_angle_fb.unload_event_thrust={{3.,3.}};
 rear_angle_fb.rear_support_prior=-1.;
 rear_angle_fb.rear_grasp_reference={{1.6,1.4}};
 rear_angle_fb.rear_grasp_reference_valid=true;
 rear_angle_fb.rear_pressure_command=10.;
 rear_angle_input.rear_kinematics_valid=true;
 rear_angle_input.rear_bend={{1.6,1.4}};
 rear_angle_input.rear_rate={{0.,0.}};
 rear_angle_input.rear_pressure=0.;
 rear_angle_input.s.bend={{1.,1.}};
 rear_angle_input.s.pressure={{15.,15.}};
 rear_angle_input.s.command={{15.,15.}};
 rear_angle_input.s.thrust={{3.,3.}};
 rear_angle_fb.enter(SinglePass::Pull,rear_angle_input);
 SinglePassOutput rear_under;
 for(int i=0;i<20 && !rear_angle_fb.rear_positive_started;++i)
   rear_under=rear_angle_fb.update(rear_angle_input,c,k,model,geom);
 assert(rear_angle_fb.rear_angle_feedback_active &&
        std::abs(rear_angle_fb.rear_release_target-
                 k.rear_release_angle_max)<1e-9 &&
        rear_angle_fb.rear_angle_pressure_trim<0. &&
        rear_angle_fb.rear_angle_thrust_trim>0. &&
        rear_under.rear_pressure<=k.rear_motion_pressure_cap &&
        rear_under.rear_positive_thrust>0.);
 // Reaching the requested 3--5-deg release with BOTH rear arms removes the
 // assist while Motion 2/3 retain the low-pressure cap.
 rear_angle_input.rear_bend={{1.6-k.rear_release_angle_max,
                              1.4-k.rear_release_angle_max}};
 for(int i=0;i<30 && rear_under.rear_positive_thrust>0.;++i)
   rear_under=rear_angle_fb.update(rear_angle_input,c,k,model,geom);
 assert(rear_angle_fb.rear_release_angle_achieved &&
        rear_under.rear_positive_thrust==0. &&
        rear_under.rear_pressure<=k.rear_motion_pressure_cap);
 rear_angle_input.rear_bend={{1.6-3.*k.rear_release_angle_max,1.4}};
 for(int i=0;i<20;++i)
   rear_under=rear_angle_fb.update(rear_angle_input,c,k,model,geom);
 assert(rear_angle_fb.rear_release_mismatch>=
            k.rear_release_mismatch_hard &&
        rear_angle_fb.rear_release_balance_envelope==0. &&
        rear_under.rear_positive_thrust==0.);
 // A release-side optimizer candidate is not an observation merely because
 // its valve or rotor command was sent. Both rear bends must realize a
 // sustained fraction of the requested relative extension.
 SinglePass rear_candidate;auto rear_candidate_input=rear_angle_input;
 rear_candidate.initialized=true;rear_candidate.slip_tested=true;
 rear_candidate.calibrated=true;
 rear_candidate.grasp_reference={{1.5,1.5}};
 rear_candidate.lift_up=1.5;rear_candidate.pull_forward=2.5;
 rear_candidate.lift_force=3.;rear_candidate.load_relief_detected=true;
 rear_candidate.unload_event_thrust={{3.,3.}};
 rear_candidate.rear_support_prior=0.;rear_candidate.trajectory.current=2;
 rear_candidate.rear_candidate_requested=true;
 rear_candidate.rear_candidate_distinct=true;
 rear_candidate.rear_grasp_reference={{1.6,1.4}};
 rear_candidate.rear_grasp_reference_valid=true;
 rear_candidate.rear_pressure_command=7.5;
 rear_candidate_input.rear_bend={{1.6,1.4}};
 rear_candidate_input.rear_rate={{0.,0.}};
 rear_candidate_input.rear_pressure=7.5;
 rear_candidate_input.s.bend={{1.,1.}};
 rear_candidate_input.s.pressure={{15.,15.}};
 rear_candidate_input.s.command={{15.,15.}};
 rear_candidate_input.s.thrust={{3.,3.}};
 rear_candidate.enter(SinglePass::Pull,rear_candidate_input);
 for(int i=0;i<10;++i)
   rear_candidate.update(rear_candidate_input,c,k,model,geom);
 assert(!rear_candidate.rear_candidate_realized);
 const double realized_release=.75*k.rear_release_angle_max;
 rear_candidate_input.rear_bend={{1.6-realized_release,
                                  1.4-realized_release}};
 for(int i=0;i<30 && !rear_candidate.rear_candidate_realized;++i)
   rear_candidate.update(rear_candidate_input,c,k,model,geom);
 assert(rear_candidate.rear_candidate_realized);
 // Regression for the real-arm Motion-3 handoff: one front arm may still be
 // extending quickly when pneumatic flexion starts.  Rate braking must not
 // copy an almost-zero target to both rotors.  Retain a bounded fraction of
 // the measured unloading event, establish horizontal traction early, and
 // delay normal rear refill until that traction interval has been released.
 SinglePass transition;auto transition_input=in;
 transition.initialized=true;transition.slip_tested=true;
 transition.calibrated=true;
 transition.identifying_straight=true;
 transition.reference={{1.5,1.5}};
 transition.grasp_reference=transition.reference;
 transition.lift_up=.20;transition.pull_forward=2.55;
 transition.lift_force=2.71;
 transition.unload_event_thrust={{1.46,2.71}};
 transition_input.s.bend={{1.0,1.0}};
 transition_input.s.rate={{-1.2,-1.5}};
 transition_input.s.pressure={{3.,3.}};
 transition_input.s.command=transition_input.s.pressure;
 transition_input.s.thrust={{1.46,2.71}};
 transition_input.rear_pressure=0.;
 transition_input.rear_released=true;
 transition_input.front_contact=true;
 transition.enter(SinglePass::Pull,transition_input);
 SinglePassOutput transition_output;
 for(int i=0;i<50;++i) {
   transition_output=transition.update(
       transition_input,c,k,model,geom);
   transition_input.s.thrust=transition_output.thrust;
   transition_input.s.command=transition_output.pressure;
 }
 const double transition_floor=.30+
     (1.46-.30)*k.pull_residual_thrust_ratio;
 assert(transition_output.thrust[0]>=transition_floor-1e-9 &&
        transition_output.thrust[1]>=transition_floor-1e-9 &&
        !transition.support_transfer_started);
 transition_input.s.rate={{0.,0.}};
 transition_input.s.bend={{1.20,1.20}}; // 40% flexion
 transition_input.s.pressure={{18.,18.}};
 transition_input.s.command=transition_input.s.pressure;
 transition_output=transition.update(transition_input,c,k,model,geom);
 assert(transition.golden_desired_forward>2.0 &&
        !transition.support_transfer_started);
 transition_input.s.bend={{1.30,1.30}}; // 60% flexion
 transition_output=transition.update(transition_input,c,k,model,geom);
 assert(!transition.support_transfer_started); // one sample is not a catch
 for(int i=0;i<120 && !transition.support_transfer_started;++i) {
   transition_output=transition.update(transition_input,c,k,model,geom);
   transition_input.s.thrust=transition_output.thrust;
   transition_input.s.command=transition_output.pressure;
 }
 assert(transition.support_transfer_started);
 // Slip/first-unload identification chooses a coarse signed rear-support
 // prior exactly once. Later support trials belong to the same outer search
 // as front trajectory trials and therefore cannot be changed independently.
 SinglePass rear_unstable;
 rear_unstable.rear_support_prior=-.5;
 rear_unstable.stroke_lifted=true;rear_unstable.load_relief_detected=true;
 rear_unstable.stroke_peak_abs_gait_roll=13.*M_PI/180.;
 assert(rear_unstable.finishRearSupportIdentification(.01,true,k)==1 &&
        std::abs(rear_unstable.rear_support_prior-.25)<1e-9 &&
        std::abs(rear_unstable.rearPressureFloor(k)-18.75)<1e-9);
 assert(rear_unstable.finishRearSupportIdentification(.02,true,k)==0);
 SinglePass rear_stable;
 rear_stable.rear_support_prior=0.;
 rear_stable.stroke_lifted=true;rear_stable.load_relief_detected=true;
 rear_stable.stroke_peak_abs_gait_roll=2.*M_PI/180.;
 // A successful first stroke validates its actual rear support; success is
 // not evidence that maximum release would also be safe.
 assert(rear_stable.finishRearSupportIdentification(.01,true,k)==0 &&
        std::abs(rear_stable.rear_support_prior)<1e-9 &&
        std::abs(rear_stable.rearPressureFloor(k)-15.)<1e-9);
 SinglePass rear_stuck;
 rear_stuck.rear_support_prior=-.5;
 rear_stuck.stroke_lifted=true;rear_stuck.load_relief_detected=true;
 rear_stuck.stroke_peak_abs_gait_roll=2.*M_PI/180.;
 assert(rear_stuck.finishRearSupportIdentification(0.,true,k)==-1 &&
        std::abs(rear_stuck.rear_support_prior+.75)<1e-9 &&
        std::abs(rear_stuck.rearPressureFloor(k)-3.75)<1e-9);
 // Positive signed rear support is an outer-search variable. Reverse thrust
 // is used only while the higher pressure target is late and remains smooth
 // and bounded, never a phase-transition step.
 full_goal.rear_support_prior=.25;
 full_goal.trajectory.current=3;
 full_goal.rear_pressure_command=30.;
 full_goal.rear_release_commanded=true;
 full_input.rear_pressure=0.;
 full_goal.rear_positive_started=true;
 full_goal.rear_positive_elapsed=k.pull_rear_positive_duration;
 full_goal.rear_positive_thrust_command=0.;
 auto reverse_assist=full_goal.update(full_input,c,k,model,geom);
 assert(reverse_assist.rear_reverse_thrust>0. &&
        reverse_assist.rear_reverse_thrust<=
            k.trajectory_windup_rear_reverse_max+1e-9 &&
        reverse_assist.rear_positive_thrust==0.);
 full_input.rear_pressure=in.base_pressure;
 auto anchored_rear=full_goal.update(full_input,c,k,model,geom);
 assert(anchored_rear.rear_reverse_thrust>=0. &&
        anchored_rear.rear_reverse_thrust<=
            k.trajectory_windup_rear_reverse_max+1e-9);
 SinglePass p;p.begin(in);bool high=false,low=false;
 for(int i=0;i<1600 && p.phase==SinglePass::SlipProbe;++i){
   const auto previous=in.s.thrust;auto o=p.update(in,c,k,model,geom);
   // The slip probe begins released for two seconds, grasps for one second,
   // then repeats. These samples stay clear of the switching boundaries.
   if(i==0 || i==80) assert(o.rear_pressure==0.);
   if(i==110 || i==140) assert(o.rear_pressure==30.);
   if(i==160) assert(o.rear_pressure==0.);
   high=high || o.rear_pressure==30.;low=low || o.rear_pressure==0.;
   for(int j=0;j<2;++j){assert(o.thrust[j]>=0 && o.thrust[j]<=k.slip_thrust_max+1e-8);assert(o.thrust[j]-previous[j]<=c.thrust_rise*in.dt+1e-8);}
   in.s.thrust=o.thrust;in.s.command=o.pressure;in.s.now+=in.dt;
 }
 assert(high && low && p.slip_entries==1 && p.lift_probe_entries==0 && p.phase==SinglePass::StopForLift);
 // A probe which reaches its timeout without a retained slide is itself the
 // stable/no-slip classification. It must not inherit the 0-kPa lift fallback.
 assert(!p.sliding_effective && p.rear_support_slip_identified &&
        std::abs(p.rear_support_prior-k.rear_support_probe_stable)<1e-9);
 assert(p.pull_forward<=k.slip_seed_pull_max+1e-9);
 // The remainder of this long fixture checks the historical maximum-release
 // Pull path. Rear-support variation itself is covered by dedicated cases.
 p.rear_support_prior=k.rear_support_probe_sliding;
 p.rear_support_nominal=p.rear_support_command=p.rear_support_prior;
 // A horizontal arm that needs extra holding pressure can rise above the
 // 30-kPa grasp value, but never above the dedicated 40-kPa slip cap.
 SinglePass pressure_probe;auto pressure_input=in;
 pressure_input.s.now=20.;pressure_input.s.bend={{1.2,1.2}};
 pressure_input.s.start_bend=pressure_input.s.bend;
 pressure_input.s.pressure={{30.,30.}};pressure_input.s.command={{30.,30.}};
 pressure_input.s.thrust={{0.,0.}};pressure_input.rear_released=true;
 pressure_probe.begin(pressure_input);SinglePassOutput pressure_output;
 for(int i=0;i<250;++i) {
   pressure_output=pressure_probe.update(pressure_input,c,k,model,geom);
   pressure_input.s.command=pressure_output.pressure;
   pressure_input.s.thrust=pressure_output.thrust;
   pressure_input.s.now+=pressure_input.dt;
 }
 assert(pressure_output.pressure[0]>30. && pressure_output.pressure[0]<=40.);
 assert(std::abs(pressure_output.pressure[0]-k.slip_pressure_max)<1e-6);
 // A high-bend real grasp must NOT be driven all the way to 90 deg during the
 // diagnostic. Each arm gets its own start-relative target, and most grasp
 // pressure is retained. Larger release belongs to motion 2.
 SinglePass high_bend_slide;auto high_bend_input=in;
 high_bend_input.s.bend={{95.*M_PI/180.,127.*M_PI/180.}};
 high_bend_input.s.start_bend=high_bend_input.s.bend;
 high_bend_input.s.pressure={{30.,30.}};high_bend_input.s.command={{30.,30.}};
 high_bend_input.s.thrust={{2.,2.}};high_bend_input.rear_released=true;
 high_bend_slide.begin(high_bend_input);high_bend_slide.ramp=2.;
 auto absolute_slide=high_bend_slide.update(high_bend_input,c,k,model,geom);
 assert(std::abs(absolute_slide.angle_target[0]-90.*M_PI/180.)<1e-6);
 assert(std::abs(absolute_slide.angle_target[1]-117.*M_PI/180.)<1e-6);
 assert(absolute_slide.pressure[0]>=20. && absolute_slide.pressure[1]>=20.);
 // Near the 10-deg request, the common thrust is held while pressure catches
 // up; it is not alternately cut on each arm. Once pressure has caught up and
 // the bend returns inside the final two-degree corridor, both rotors resume
 // the same slow common ramp.
 SinglePass smooth_slide;auto smooth_input=high_bend_input;
 smooth_input.s.bend={{95.*M_PI/180.,127.*M_PI/180.}};
 smooth_input.s.start_bend=smooth_input.s.bend;
 smooth_input.s.command={{20.,20.}};smooth_input.s.pressure={{20.,20.}};
 smooth_input.s.thrust={{2.,2.}};
 smooth_slide.begin(smooth_input);smooth_slide.ramp=2.;
 smooth_input.s.bend={{86.*M_PI/180.,118.*M_PI/180.}};
 smooth_input.s.pressure={{25.,25.}};
 auto pressure_wait=smooth_slide.update(smooth_input,c,k,model,geom);
 assert(smooth_slide.slip_waiting_for_pressure);
 assert(std::abs(pressure_wait.thrust[0]-2.)<1e-9 &&
        std::abs(pressure_wait.thrust[1]-2.)<1e-9);
 smooth_input.s.pressure=smooth_input.s.command;
 smooth_input.s.thrust=pressure_wait.thrust;
 auto caught_up=smooth_slide.update(smooth_input,c,k,model,geom);
 assert(!smooth_slide.slip_waiting_for_pressure);
 assert(caught_up.thrust[0]>pressure_wait.thrust[0] &&
        std::abs(caught_up.thrust[0]-caught_up.thrust[1])<1e-9);
 assert(caught_up.thrust[0]-pressure_wait.thrust[0]<=
        k.probe_rise*smooth_input.dt+1e-9);
 // Only a genuine overrun beyond the separate 3-deg hard margin restores
 // grasp pressure and removes thrust.
 high_bend_input.s.bend={{81.*M_PI/180.,113.*M_PI/180.}};
 high_bend_input.s.command={{20.,20.}};high_bend_input.s.pressure={{20.,20.}};
 absolute_slide=high_bend_slide.update(high_bend_input,c,k,model,geom);
 assert(absolute_slide.pressure[0]>20. && absolute_slide.pressure[1]>20.);
 assert(absolute_slide.thrust[0]<2. && absolute_slide.thrust[1]<2.);
 // High slip thrust is removed, then the controller goes directly to the
 // coordinated rear-grasp/front-vent transition.  It must not insert an
 // all-30-kPa pose after a contact-conditioned recovery handoff.
 for(int i=0;i<60;++i){auto o=p.update(in,c,k,model,geom);assert(o.rear_pressure==30.);in.s.thrust=o.thrust;in.s.command=o.pressure;}
 assert(p.phase==SinglePass::VentFront && in.s.thrust[0]==0.);
 in.rear_grasped=true;
 // A stable contact offset above the soft gait threshold must not bounce
 // between gait and no-op recovery. Only a confirmed outward trend hands
 // over before front vent; both roll signs use the same rule.
 for(double sign:{-1.,1.}) {
   auto r=in;r.s.roll=sign*.16;r.s.roll_rate=sign*.08;
   SinglePass pre_vent;
   pre_vent.initialized=true;
   pre_vent.grasp_reference=r.s.bend;
   pre_vent.enter(SinglePass::StopForLift,r);
   SinglePassOutput pre_vent_recovery;
   const int recovery_samples=static_cast<int>(
       std::ceil((k.soft_roll_confirm+k.stop_roll_recovery_confirm)/r.dt))+2;
   for(int i=0;i<recovery_samples;++i)
     pre_vent_recovery=pre_vent.update(r,c,k,model,geom);
   assert(pre_vent_recovery.recover_roll &&
          pre_vent.phase==SinglePass::StopForLift);
 }
 // Static and inward-moving offsets continue to the front-vent phase. The
 // adapter still owns the independent absolute hard-roll boundary.
 auto stable_offset=in;stable_offset.s.roll=.16;stable_offset.s.roll_rate=0.;
 SinglePass stable_pre_vent;
 stable_pre_vent.initialized=true;
 stable_pre_vent.grasp_reference=stable_offset.s.bend;
 stable_pre_vent.enter(SinglePass::StopForLift,stable_offset);
 stable_pre_vent.update(stable_offset,c,k,model,geom);
 assert(stable_pre_vent.phase==SinglePass::VentFront);
 // Below the absolute recovery boundary, front exhaust remains a zero-thrust
 // preparation.  Differential authority begins in powered Motion 2; it must
 // not keep the vent-completion gate false inside this phase.
 auto active_vent=in;active_vent.s.command={{12.,12.}};
 active_vent.s.pressure={{12.,12.}};active_vent.s.thrust={{0.,0.}};
 active_vent.s.roll=.10;active_vent.s.roll_rate=.05;
 SinglePass vent_balance;vent_balance.initialized=true;
 vent_balance.grasp_reference=active_vent.s.bend;
 vent_balance.enter(SinglePass::VentFront,active_vent);
 auto vent_balance_output=vent_balance.update(active_vent,c,k,model,geom);
 assert(!vent_balance_output.recover_roll &&
        vent_balance.phase==SinglePass::VentFront &&
        vent_balance_output.thrust[0]==0. &&
        vent_balance_output.thrust[1]==0.);
 // Once support removal has started, a sustained outward trend must hand
 // control to recovery before entering powered Lift. A stationary 10--15-deg
 // offset on a tapered/elliptic support is allowed; the adapter retains the
 // independent absolute hard-angle guard.
 for(double sign:{-1.,1.}) {
   auto r=in;r.s.command={{12.,12.}};r.s.pressure={{12.,12.}};
   r.s.roll=sign*.19;r.s.roll_rate=sign*.12;
   SinglePass vent_worsening;
   vent_worsening.initialized=true;
   vent_worsening.grasp_reference=r.s.bend;
   vent_worsening.enter(SinglePass::VentFront,r);
   SinglePassOutput first;
   const int confirm_samples=static_cast<int>(std::ceil(
       k.soft_roll_confirm/r.dt))+1;
   for(int i=0;i<confirm_samples;++i)
     first=vent_worsening.update(r,c,k,model,geom);
   assert(first.recover_roll &&
          vent_worsening.phase==SinglePass::VentFront &&
          first.thrust[0]==0. && first.thrust[1]==0. &&
          first.pressure[0]>0. && first.pressure[1]>0.);
 }
 in.s.roll=0.;
 in.s.roll_rate=0.;
 for(int i=0;i<50;++i){auto o=p.update(in,c,k,model,geom);assert(o.thrust[0]==0.);in.s.command=o.pressure;}
 assert(p.phase==SinglePass::VentFront && p.powered_time==0.);
 const auto led_rear=p.update(in,c,k,model,geom);
 // Front-vent preparation leads the rear exhaust directly into the bounded
 // Motion-2/3 compliant corridor.  The outer regrip alone restores 30 kPa.
 assert(led_rear.rear_pressure<=k.rear_motion_pressure_cap &&
        led_rear.rear_pressure>=k.rear_low_kpa);
 assert(in.s.command[0]>=0.); // command may already be zero; measured pressure still gates Lift
 // VentFront is a zero-thrust preparation boundary with one common pressure
 // target; relative extension is owned by the following powered phase.
 SinglePass vent_sync;auto vent_sync_in=in;
 vent_sync_in.s.command={{30.,30.}};vent_sync_in.s.pressure={{30.,30.}};
 vent_sync.calibrated=true;vent_sync.slip_tested=true;
 vent_sync.begin(vent_sync_in);vent_sync.enter(SinglePass::VentFront,vent_sync_in);
 vent_sync_in.s.bend[0]-=6.*M_PI/180.;
 auto vent_sync_out=vent_sync.update(vent_sync_in,c,k,model,geom);
 assert(vent_sync.pressure_target[0]==vent_sync.pressure_target[1] &&
        vent_sync.pressure_target[0]<30. &&
        vent_sync_out.thrust[0]==0. && vent_sync_out.thrust[1]==0. &&
        vent_sync_out.pressure[0]<30. && vent_sync_out.pressure[1]<30.);
 assert(vent_sync.phase==SinglePass::VentFront);
 // An asymmetric event-completion pair is not a valid breakaway pair.  It
 // cannot leak into VentFront, and without two measured onsets the next Lift
 // uses the smaller event force as a common safe approach.
 SinglePass vent_lead;auto vent_lead_in=vent_sync_in;
 vent_lead.calibrated=true;vent_lead.slip_tested=true;
 // A valid unloading event records both arm forces even if passive exhaust
 // hides one arm's exact breakaway sample.
 vent_lead.onset_identified=false;
 vent_lead.identified_extension_thrust={{2.2,.8}};
 vent_lead.begin(vent_lead_in);vent_lead.enter(SinglePass::VentFront,vent_lead_in);
 vent_lead_in.s.pressure={{9.,9.}};
 SinglePassOutput vent_lead_out;
 for(int i=0;i<20;++i) {
   vent_lead_out=vent_lead.update(vent_lead_in,c,k,model,geom);
   vent_lead_in.s.thrust=vent_lead_out.thrust;
 }
 assert(vent_lead.phase==SinglePass::VentFront &&
        vent_lead_out.thrust[0]==0. && vent_lead_out.thrust[1]==0.);
 vent_lead_in.s.pressure={{0.,0.}};
 vent_lead.vent_common_pressure=0.;
 for(int i=0;i<15 && vent_lead.phase==SinglePass::VentFront;++i) {
   vent_lead_out=vent_lead.update(vent_lead_in,c,k,model,geom);
   vent_lead_in.s.thrust=vent_lead_out.thrust;
 }
 assert(vent_lead.phase==SinglePass::Lift);
 for(int i=0;i<10;++i) {
   vent_lead_out=vent_lead.update(vent_lead_in,c,k,model,geom);
   vent_lead_in.s.thrust=vent_lead_out.thrust;
 }
 assert(std::abs(vent_lead_out.thrust[0]-vent_lead_out.thrust[1])<1e-9);
 in.s.pressure={{0.,0.}};in.rear_pressure=30.;
 for(int i=0;i<11;++i){auto o=p.update(in,c,k,model,geom);in.s.command=o.pressure;in.s.thrust=o.thrust;}
 assert(p.phase==SinglePass::Lift && p.lift_probe_entries==1);
 // Motion 2 preserves the pre-vent command instead of returning the rear
 // pair to 30 kPa for one sample and losing the pneumatic lead.
 const auto preserved_rear=p.update(in,c,k,model,geom);
 assert(preserved_rear.rear_pressure<=k.regrip_rear_hold_pressure+1e-9 &&
        preserved_rear.rear_pressure>=p.rearPressureFloor(k)-1e-9);
 // Above 45 deg a physically extreme outward-rate spike is considered by
 // pausing the search for this sample. It must not erase the identified force,
 // and the chamber remains at zero until the measured target is reached.
 in.s.command={{0.,0.}};in.s.pressure={{0.,0.}};in.s.thrust={{2.,2.}};
 in.s.bend={{1.48,1.48}};in.s.rate={{-6.,-6.}};in.s.floor={{.7,.7}};
 auto guarded_lift=p.update(in,c,k,model,geom);
 assert(p.phase==SinglePass::Lift && guarded_lift.pressure[0]==0. && guarded_lift.pressure[1]==0.);
 assert(std::abs(guarded_lift.thrust[0]-in.s.thrust[0])<1e-9 &&
        std::abs(guarded_lift.thrust[1]-in.s.thrust[1])<1e-9);
 in.s.floor={{.7,.7}};in.s.rate={{0.,0.}};
 // Body lift/pitch alone must not start Pull: both arms still need a genuine,
 // powered minimum reach.  This rejects bladder rocking and passive venting.
 SinglePass pose_only;auto pose_input=in;pose_input.s.bend={{1.5,1.5}};
 pose_input.s.thrust={{2.,2.}};pose_input.height=0.;pose_input.s.pitch=0.;
 pose_only.slip_tested=true;pose_only.begin(pose_input);pose_only.enter(SinglePass::Lift,pose_input);
 prime_powered_baseline(pose_only,pose_input);
 pose_input.s.bend={{1.44,1.44}};pose_input.height=.01;pose_input.s.pitch=.08;
 auto pose_shortcut=pose_only.update(pose_input,c,k,model,geom);
 assert(pose_only.phase==SinglePass::Lift && pose_shortcut.pressure[0]==0.);
 // A short paired IMU excursion by itself must not end Motion 2.  The unified
 // trajectory requires meaningful travel and measured body/contact unloading.
 SinglePass immediate_pulse;auto immediate_input=in;
 immediate_input.s.bend={{1.50,1.50}};
 immediate_input.s.thrust={{2.,2.}};
 immediate_input.s.pressure={{0.,0.}};
 immediate_input.s.command=immediate_input.s.pressure;
 immediate_input.s.rate={{0.,0.}};
 immediate_pulse.slip_tested=true;
 immediate_pulse.begin(immediate_input);
 immediate_pulse.enter(SinglePass::Lift,immediate_input);
 prime_powered_baseline(immediate_pulse,immediate_input);
 immediate_pulse.phase_time=k.initial_pulse_min_duration;
 immediate_input.s.bend={{1.44,1.44}};
 immediate_input.s.rate={{-.5,-.5}};
 auto immediate_output=immediate_pulse.update(
     immediate_input,c,k,model,geom);
 assert(immediate_pulse.phase==SinglePass::Lift &&
        !immediate_pulse.unload_event_detected &&
        immediate_output.pressure[0]==0. &&
        immediate_output.pressure[1]==0.);
 // Once the earlier body cue has expired, even a large paired angle excursion
 // remains in Motion 2.  Arm angle alone is never an unloading event.
 pose_input.s.bend={{1.20,1.20}};pose_input.height=0.;
 pose_input.directional_pitch=0.;
 for(int i=0;i<15;++i)
   pose_only.update(pose_input,c,k,model,geom);
 assert(pose_only.phase==SinglePass::Lift && !pose_only.calibrated &&
        !pose_only.unload_event_detected);
 // Both front arms must extend. A one-arm reach cannot be promoted by body
 // motion, and an unsafe roll cannot be learned as a valid event.
 SinglePass one_arm;auto one_input=pose_input;
 one_input.s.bend={{1.5,1.5}};one_input.height=0.;
 one_arm.slip_tested=true;one_arm.begin(one_input);
 one_arm.enter(SinglePass::Lift,one_input);
 prime_powered_baseline(one_arm,one_input);
 one_input.s.bend={{1.20,1.48}};one_input.height=.01;
 for(int i=0;i<10;++i)one_arm.update(one_input,c,k,model,geom);
 assert(one_arm.phase==SinglePass::Lift && !one_arm.calibrated);
 // A previous peak on both arms is not enough if one arm has already
 // rebounded well below the powered-relative target.  This specifically
 // guards the old [3.0, 2.2] deg late handoff.
 SinglePass stale_pair;auto stale_input=pose_input;
 stale_input.s.bend={{1.5,1.5}};stale_input.s.rate={{0.,0.}};
 stale_input.s.thrust={{1.5,1.5}};stale_input.height=0.;
 stale_pair.slip_tested=true;stale_pair.begin(stale_input);
 stale_pair.enter(SinglePass::Lift,stale_input);
 prime_powered_baseline(stale_pair,stale_input);
 stale_pair.minimum={{1.44,1.44}}; // both previously exceeded 3 deg
 stale_pair.unload_evidence_time=k.unload_confirm-.5*stale_input.dt;
 stale_input.s.bend={{1.472,1.472}}; // only 1.6 deg remains now
 stale_pair.update(stale_input,c,k,model,geom);
 assert(stale_pair.phase==SinglePass::Lift &&
        !stale_pair.unload_event_detected);
 // Motion-2 angle/rate feedback reduces the leading arm and increases the
 // lagging arm without turning either rotor off. Front pressure remains zero
 // until measured unloading transfers control to pressure-driven Pull.
 SinglePass target_brake;auto brake_input=pose_input;
 brake_input.s.bend={{1.5,1.5}};brake_input.s.rate={{0.,0.}};
 brake_input.s.thrust={{1.5,1.5}};brake_input.s.pressure={{0.,0.}};
 brake_input.s.command={{0.,0.}};target_brake.slip_tested=true;
 target_brake.begin(brake_input);target_brake.enter(SinglePass::Lift,brake_input);
 prime_powered_baseline(target_brake,brake_input);
 target_brake.identification_tracking={{true,true}};
 target_brake.identification_hold_thrust={{1.5,1.5}};
 target_brake.lift_ramp={{1.5,1.5}};
 brake_input.s.bend={{1.44,1.48}};brake_input.s.rate={{-.4,0.}};
 const auto target_brake_output=
     target_brake.update(brake_input,c,k,model,geom);
 assert(target_brake_output.thrust[0]>0. &&
        target_brake_output.thrust[1]>0. &&
        target_brake_output.thrust[0]<brake_input.s.thrust[0] &&
        target_brake_output.thrust[1]>brake_input.s.thrust[1]);
 assert(target_brake_output.pressure[0]==0. &&
        target_brake_output.pressure[1]==0.);
 SinglePass unsafe_contact;auto unsafe_input=one_input;
 unsafe_input.s.bend={{1.5,1.5}};unsafe_input.height=0.;
 unsafe_contact.slip_tested=true;unsafe_contact.begin(unsafe_input);
 unsafe_contact.enter(SinglePass::Lift,unsafe_input);
 prime_powered_baseline(unsafe_contact,unsafe_input);
 // A measured unload inside the gait's hard roll boundary ends extension so
 // Pull can restore support. Only a sample outside that independent boundary
 // is rejected here.
 unsafe_input.s.bend={{1.20,1.20}};unsafe_input.s.roll=c.roll_limit+.01;
 unsafe_input.s.roll_rate=.1;unsafe_input.height=.01;
 unsafe_input.directional_pitch=k.unload_pitch+.01;
 for(int i=0;i<10;++i)unsafe_contact.update(unsafe_input,c,k,model,geom);
 assert(unsafe_contact.phase==SinglePass::Lift && !unsafe_contact.calibrated);
 // A confirmed body rise after a genuine two-arm extension is the completion
 // event. Save the measured event pose/thrust for the next stroke.
 SinglePass unload;auto unload_input=in;
 unload_input.contact_available=false; // hardware IMU/mocap proxy path
 unload_input.s.bend={{1.5,1.5}};unload_input.s.thrust={{2.,2.}};
 unload_input.height=0.;unload_input.directional_pitch=0.;
 unload.slip_tested=true;unload.begin(unload_input);
 unload.enter(SinglePass::Lift,unload_input);
 prime_powered_baseline(unload,unload_input);
 unload_input.s.bend={{1.20,1.20}};unload_input.height=.003;
 unload_input.directional_pitch=k.unload_min_pitch+.01;
 for(int i=0;i<10 && unload.phase==SinglePass::Lift;++i)
   unload.update(unload_input,c,k,model,geom);
 assert(unload.phase==SinglePass::Pull && unload.calibrated &&
        unload.unload_event_detected);
 assert(std::abs(unload.identified_bend_target[0]-1.20)<1e-9);
 assert(std::abs(unload.identified_bend_target[1]-1.20)<1e-9);
 assert(std::isfinite(unload.identified_extension_thrust[0]));
 // Production deep-windup mode separates the first measured load-relief cue
 // from the pressure-flexion handoff. Height/pitch noise cannot end Motion 2;
 // the front chambers remain released while thrust builds pitch, then full
 // bounded pressure starts the predicted flexion handoff.
 auto deep_k=k;deep_k.trajectory_golden_handoff=false;
 deep_k.trajectory_windup_pitch=15.*M_PI/180.;
 SinglePass deep;auto deep_input=in;
 deep_input.contact_available=false;
 deep_input.s.bend={{1.5,1.5}};deep_input.s.thrust={{2.,2.}};
 deep_input.s.pressure={{0.,0.}};deep_input.s.command={{0.,0.}};
 deep_input.height=0.;deep_input.directional_pitch=0.;
 deep.slip_tested=true;deep.begin(deep_input);
 deep.enter(SinglePass::Lift,deep_input);
 prime_powered_baseline(deep,deep_input);
 deep_input.s.bend={{1.20,1.20}};deep_input.height=.003;
 deep_input.directional_pitch=deep_k.unload_min_pitch+.01;
 for(int i=0;i<10;++i)deep.update(deep_input,c,deep_k,model,geom);
 assert(deep.load_relief_detected && !deep.unload_event_detected &&
        deep.phase==SinglePass::Lift &&
        deep.pressure_target[0]<=1e-9);
 deep_input.directional_pitch=14.*M_PI/180.;
 deep_input.pitch_rate=.30; // 80-ms prediction crosses the 15-deg target
 for(int i=0;i<5 && deep.phase==SinglePass::Lift;++i)
   deep.update(deep_input,c,deep_k,model,geom);
 assert(deep.unload_event_detected && deep.phase==SinglePass::Pull &&
        deep.windup_pitch_progress>=deep.windup_pitch_target &&
        deep.windup_pose_captured);
 // Bottom/front-pressure settling before powered breakaway must not become
 // apparent Motion-2 lift.  Latch angle, height and pitch from immediately
 // before the two measured powered onsets, then require fresh relative arm
 // travel and a fresh body/contact load-relief cue from that baseline.
 SinglePass powered_baseline;auto baseline_input=in;
 baseline_input.contact_available=false;
 baseline_input.s.bend={{1.50,1.50}};baseline_input.s.rate={{0.,0.}};
 baseline_input.s.thrust={{0.,0.}};baseline_input.s.pressure={{0.,0.}};
 baseline_input.s.command={{0.,0.}};baseline_input.height=0.;
 baseline_input.directional_pitch=0.;
 powered_baseline.slip_tested=true;powered_baseline.begin(baseline_input);
 powered_baseline.enter(SinglePass::Lift,baseline_input);
 // Pneumatic settling changes pose before either rotor produces extension.
 baseline_input.s.bend={{1.46,1.46}};baseline_input.height=.004;
 baseline_input.directional_pitch=.03;
 for(int i=0;i<5;++i)
   powered_baseline.update(baseline_input,c,k,model,geom);
 assert(!powered_baseline.unload_event_detected);
 // A short powered burst identifies breakaway but cannot complete Motion 2.
 // It must be followed by meaningful paired extension and a fresh body cue.
 baseline_input.s.thrust={{2.,2.}};
 for(int i=0;i<6 && powered_baseline.phase==SinglePass::Lift;++i) {
   baseline_input.s.bend[0]-=.012;
   baseline_input.s.bend[1]-=.012;
   baseline_input.s.rate={{-.6,-.6}};
   powered_baseline.update(baseline_input,c,k,model,geom);
 }
 baseline_input.s.rate={{0.,0.}};
 for(int i=0;i<5 && powered_baseline.phase==SinglePass::Lift;++i)
   powered_baseline.update(baseline_input,c,k,model,geom);
 assert(powered_baseline.powered_extension_reference_valid[0] &&
        powered_baseline.powered_extension_reference_valid[1] &&
        powered_baseline.powered_body_reference_valid &&
        powered_baseline.phase==SinglePass::Lift &&
        !powered_baseline.unload_event_detected);
 // Continue the same predicted reach past the 15-deg production minimum and
 // provide unloading relative to the powered-onset body reference.
 baseline_input.s.bend={{1.16,1.16}};
 baseline_input.height=.007;
 baseline_input.directional_pitch=.08;
 for(int i=0;i<10 && powered_baseline.phase==SinglePass::Lift;++i)
   powered_baseline.update(baseline_input,c,k,model,geom);
 assert(powered_baseline.powered_extension_reference[0]<1.49 &&
        std::abs(powered_baseline.powered_start_height-.004)<1e-9 &&
        powered_baseline.unload_event_detected);
 assert(powered_baseline.phase==SinglePass::Pull &&
        powered_baseline.unload_event_detected &&
        !powered_baseline.unload_from_arm_trajectory);
 // If the two arms break away at different times, body height/pitch are
 // zeroed at the later onset. Motion before the second arm joins must not be
 // counted as paired unloading.
 SinglePass paired_body_reference;auto paired_reference_input=in;
 paired_reference_input.s.bend={{1.50,1.50}};
 paired_reference_input.s.thrust={{1.,1.}};
 paired_reference_input.s.pressure={{0.,0.}};
 paired_reference_input.s.command={{0.,0.}};
 paired_reference_input.height=.030;
 paired_reference_input.directional_pitch=.20;
 paired_body_reference.slip_tested=true;
 paired_body_reference.begin(paired_reference_input);
 paired_body_reference.enter(SinglePass::Lift,paired_reference_input);
 paired_body_reference.onset_this_stroke={{true,true}};
 paired_body_reference.powered_extension_reference={{1.50,1.50}};
 paired_body_reference.powered_extension_reference_valid={{true,true}};
 paired_body_reference.onset_baseline_candidate_valid={{true,true}};
 paired_body_reference.onset_baseline_candidate[0].time=.10;
 paired_body_reference.onset_baseline_candidate[0].height=.010;
 paired_body_reference.onset_baseline_candidate[0].pitch=.05;
 paired_body_reference.onset_baseline_candidate[1].time=.30;
 paired_body_reference.onset_baseline_candidate[1].height=.030;
 paired_body_reference.onset_baseline_candidate[1].pitch=.20;
 paired_body_reference.update(
     paired_reference_input,c,k,model,geom);
 assert(paired_body_reference.powered_body_reference_valid &&
        std::abs(paired_body_reference.powered_body_reference_time-.30)<1e-9 &&
        std::abs(paired_body_reference.powered_start_height-.030)<1e-9 &&
        std::abs(paired_body_reference.powered_start_pitch-.20)<1e-9 &&
        !paired_body_reference.unload_event_detected);
 // The body cue and the second arm need not peak in the same sensor sample.
 // Preserve one confirmed cue briefly, then complete the event when both
 // powered relative extensions become available.
 SinglePass fused_unload;auto fused_input=in;
 fused_input.contact_available=false;
 fused_input.s.bend={{1.50,1.50}};fused_input.s.rate={{0.,0.}};
 fused_input.s.thrust={{2.,2.}};fused_input.s.pressure={{0.,0.}};
 fused_input.s.command={{0.,0.}};fused_input.height=0.;
 fused_input.directional_pitch=0.;
 fused_unload.slip_tested=true;fused_unload.begin(fused_input);
 fused_unload.enter(SinglePass::Lift,fused_input);
 prime_powered_baseline(fused_unload,fused_input);
 fused_input.s.bend={{1.43,1.49}};fused_input.height=.003;
 fused_input.directional_pitch=k.unload_min_pitch+.01;
 for(int i=0;i<3;++i) fused_unload.update(fused_input,c,k,model,geom);
 assert(fused_unload.unload_body_memory_time>0. &&
        !fused_unload.unload_event_detected);
 fused_input.s.bend={{1.20,1.20}};fused_input.height=0.;
 for(int i=0;i<10 && fused_unload.phase==SinglePass::Lift;++i)
   fused_unload.update(fused_input,c,k,model,geom);
 assert(fused_unload.phase==SinglePass::Pull &&
        fused_unload.unload_event_detected);
 // A time-confirmed, aligned unloading event must hand control directly to
 // the pneumatic catch. Motion 2 must not add a second settling dwell while
 // the zero-pressure arms continue moving.
 SinglePass pneumatic_capture;auto capture_input=in;
 capture_input.contact_available=false;
 capture_input.s.bend={{1.5,1.5}};capture_input.s.thrust={{1.,1.}};
 capture_input.s.pressure={{0.,0.}};capture_input.s.command={{0.,0.}};
 pneumatic_capture.slip_tested=true;pneumatic_capture.begin(capture_input);
 pneumatic_capture.enter(SinglePass::Lift,capture_input);
 pneumatic_capture.unload_event_detected=true;
 pneumatic_capture.unload_event_bend={{1.34,1.34}};
 pneumatic_capture.unload_event_thrust={{1.,1.}};
 pneumatic_capture.unload_alignment_extension_target=.16;
 capture_input.s.bend={{1.34,1.34}};capture_input.s.rate={{0.,0.}};
 auto capture_output=pneumatic_capture.update(capture_input,c,k,model,geom);
 assert(pneumatic_capture.unload_capture_started &&
        pneumatic_capture.phase==SinglePass::Pull);
 // The body can unload while one compliant arm is much farther forward than
 // the other.  Remember that cue, keep rear support and align the released
 // pair in place; the asymmetric sample itself must not become the event.
 SinglePass unequal_unload;auto unequal_input=in;
 unequal_input.contact_available=false;
 unequal_input.s.bend={{1.5,1.5}};unequal_input.s.thrust={{2.,2.}};
 unequal_input.s.pressure={{0.,0.}};unequal_input.s.command={{0.,0.}};
 unequal_input.height=0.;unequal_input.directional_pitch=0.;
 unequal_unload.slip_tested=true;unequal_unload.begin(unequal_input);
 unequal_unload.enter(SinglePass::Lift,unequal_input);
 prime_powered_baseline(unequal_unload,unequal_input);
 // 19.5 deg exceeds the ordinary alignment corridor.  It remains a bounded
 // rear-anchored alignment problem rather than a forced full regrip.
 unequal_input.s.bend={{1.20,.90}};unequal_input.height=.003;
 unequal_input.directional_pitch=k.unload_min_pitch+.01;
 SinglePassOutput unequal_output;
 for(int i=0;i<10;++i) {
   unequal_output=unequal_unload.update(unequal_input,c,k,model,geom);
   unequal_input.s.command=unequal_output.pressure;
 }
 assert(!unequal_unload.unload_event_detected &&
        unequal_unload.unload_body_memory_time>0. &&
        unequal_unload.phase==SinglePass::Lift &&
        !unequal_unload.front_target_latched &&
        !unequal_unload.calibrated && !unequal_output.calibration_regrip);
 const double aligned_extension=.30;
 unequal_input.s.bend={{1.5-aligned_extension,1.5-aligned_extension}};
 unequal_input.s.rate={{0.,0.}};
 for(int i=0;i<10 && unequal_unload.phase==SinglePass::Lift;++i) {
   unequal_output=unequal_unload.update(unequal_input,c,k,model,geom);
   unequal_input.s.command=unequal_output.pressure;
 }
 assert(unequal_unload.phase==SinglePass::Pull &&
        unequal_unload.front_target_latched && unequal_unload.calibrated &&
        !unequal_output.calibration_regrip);
 assert(std::abs(unequal_unload.observed_extension_by_arm[0]-
                 unequal_unload.observed_extension_by_arm[1])<1e-9);
 // Opposite-direction pitch is never a valid unloading event, even when both
 // arms have extended. A subsequent commanded-direction pitch/height event
 // completes the same rear-anchored Motion-2 reach.
 SinglePass negative_pitch;auto negative_input=in;
 negative_input.contact_available=false; // hardware IMU proxy, opposite mounting
 negative_input.s.bend={{1.5,1.5}};negative_input.s.thrust={{2.,2.}};
 negative_input.height=0.;negative_input.directional_pitch=0.;
 negative_pitch.slip_tested=true;negative_pitch.begin(negative_input);
 negative_pitch.enter(SinglePass::Lift,negative_input);
 prime_powered_baseline(negative_pitch,negative_input);
 negative_input.s.bend={{1.20,1.20}};
 negative_input.directional_pitch=-k.unload_pitch-.01;
 for(int i=0;i<10 && negative_pitch.phase==SinglePass::Lift;++i)
   negative_pitch.update(negative_input,c,k,model,geom);
 assert(negative_pitch.phase==SinglePass::Lift &&
        !negative_pitch.unload_event_detected);
 negative_input.directional_pitch=k.unload_pitch+.01;
 negative_input.height=k.unload_height;
 for(int i=0;i<10 && negative_pitch.phase==SinglePass::Lift;++i)
   negative_pitch.update(negative_input,c,k,model,geom);
 assert(negative_pitch.phase==SinglePass::Pull &&
        negative_pitch.unload_event_detected);
 // On a reused stroke, the short extension pulse also remains at zero front
 // pressure.  Pressure-driven flexion starts only after the paired pulse
 // event; a predicted pneumatic pilot must not suppress extension.
 unload_input.s.bend={{1.5,1.5}};unload_input.s.start_bend=unload_input.s.bend;
 unload_input.s.thrust={{2.,2.}};unload_input.s.rate={{-.2,-.2}};
 unload_input.s.pressure={{0.,0.}};unload_input.s.command={{0.,0.}};
 unload_input.height=0.;unload_input.directional_pitch=0.;
 unload.initialized=false;unload.begin(unload_input);
 unload.enter(SinglePass::Lift,unload_input);
 unload_input.s.bend={{1.41,1.40}};
 auto pilot=unload.update(unload_input,c,k,model,geom);
 assert(unload.phase==SinglePass::Lift && !unload.unload_event_detected);
 assert(pilot.pressure[0]==0. && pilot.pressure[1]==0.);
 unload_input.s.bend={{1.20,1.20}};
 unload_input.height=k.unload_height;
 unload_input.directional_pitch=k.unload_min_pitch+.01;
 unload_input.s.thrust=pilot.thrust;
 for(int i=0;i<10 && unload.phase==SinglePass::Lift;++i)
   unload.update(unload_input,c,k,model,geom);
 assert(unload.phase==SinglePass::Pull && unload.unload_event_detected);
 // During straight-identification breakaway, angle synchronization must not
 // add adverse thrust when it would worsen measured roll.  The over-extended
 // initial free-extension test keeps both chambers at zero; synchronization
 // is performed only with bounded thrust while the rear remains anchored.
 SinglePass synchronized;auto sync_input=in;sync_input.s.bend={{1.5,1.5}};
 sync_input.s.thrust={{3.,3.}};sync_input.s.command={{0.,0.}};
 sync_input.s.pressure={{0.,0.}};sync_input.s.roll=0.;sync_input.s.roll_rate=0.;
 synchronized.slip_tested=true;synchronized.begin(sync_input);
 synchronized.enter(SinglePass::Lift,sync_input);
 sync_input.s.roll=-.12;sync_input.s.roll_rate=-.1;
 sync_input.s.bend={{1.2,1.45}};
 auto sync_output=synchronized.update(sync_input,c,k,model,geom);
 assert(synchronized.phase==SinglePass::Lift && !sync_output.recover_roll);
 assert(synchronized.identification_tracking[0]);
 assert(sync_output.thrust[1]<=sync_output.thrust[0]+1e-9);
 assert(std::abs(sync_output.pressure[0])<1e-9 &&
        std::abs(sync_output.pressure[1])<1e-9);
 // The real failure case was a small, stationary positive-roll bias. It must
 // not let roll allocation erase the lagging pair-1 synchronization demand.
 SinglePass stable_sync;auto stable_sync_input=in;
 stable_sync_input.s.bend={{1.5,1.5}};stable_sync_input.s.thrust={{1.,0.}};
 stable_sync_input.s.command={{0.,0.}};stable_sync_input.s.pressure={{0.,0.}};
 stable_sync_input.s.roll=.04;stable_sync_input.s.roll_rate=0.;
 stable_sync.slip_tested=true;stable_sync.begin(stable_sync_input);
 stable_sync.enter(SinglePass::Lift,stable_sync_input);
 stable_sync_input.s.bend={{1.2,1.45}};
 auto stable_sync_output=stable_sync.update(stable_sync_input,c,k,model,geom);
 assert(stable_sync_output.thrust[1]>stable_sync_input.s.thrust[1]);
 assert(stable_sync_output.thrust[0]>=stable_sync_input.s.thrust[0]);
 // Reused Motion 2 normally accepts unequal motion, but an outward roll trend
 // must not lower both rotors to zero. Hold the leading arm and add thrust to
 // the lagging arm until the pair response or the hard guard takes ownership.
 SinglePass reused_sync;auto reused_sync_input=in;
 reused_sync_input.s.bend={{1.5,1.5}};
 reused_sync_input.s.thrust={{.5,.8}};
 reused_sync_input.s.command={{0.,0.}};reused_sync_input.s.pressure={{0.,0.}};
 reused_sync_input.s.roll=0.;reused_sync_input.s.roll_rate=0.;
 reused_sync.slip_tested=true;reused_sync.calibrated=true;
 reused_sync.identified_bend_target={{1.0,1.0}};
 reused_sync.identified_extension_thrust={{1.,1.}};
 reused_sync.begin(reused_sync_input);
 reused_sync.enter(SinglePass::Lift,reused_sync_input);
 reused_sync_input.s.roll=-.12;reused_sync_input.s.roll_rate=-.1;
 reused_sync_input.s.bend={{1.48,1.20}}; // arm 1 leads by 16 deg
 SinglePassOutput reused_sync_output;
 const int reused_sync_samples=static_cast<int>(
     std::ceil(k.soft_roll_confirm/reused_sync_input.dt))+1;
 for(int i=0;i<reused_sync_samples;++i)
   reused_sync_output=reused_sync.update(reused_sync_input,c,k,model,geom);
 assert(!reused_sync_output.fault && !reused_sync_output.recover_roll &&
        reused_sync_output.thrust[0]>=0. &&
        reused_sync_output.thrust[1]>=0.);
 assert(reused_sync_output.pressure[0]==0. &&
        reused_sync_output.pressure[1]==0.);
 // If extension matching and measured roll request opposite rotors, roll
 // safety owns the differential allocation.  In MuJoCo, negative roll is
 // restored by the arm-3 side (pair index 1), even when it is already the
 // extension leader.
 SinglePass roll_priority;auto roll_priority_input=reused_sync_input;
 roll_priority_input.s.thrust={{.5,.8}};
 roll_priority_input.s.roll=0.;roll_priority_input.s.roll_rate=0.;
 roll_priority.begin(roll_priority_input);
 roll_priority.enter(SinglePass::Lift,roll_priority_input);
 roll_priority_input.s.roll=-.12;roll_priority_input.s.roll_rate=-.1;
 auto sim_roll_k=k;sim_roll_k.reverse_geometry_roll_moment=true;
 SinglePassOutput roll_priority_output;
 for(int i=0;i<reused_sync_samples;++i)
   roll_priority_output=roll_priority.update(
       roll_priority_input,c,sim_roll_k,model,geom);
 assert(!roll_priority_output.fault &&
        !roll_priority_output.recover_roll &&
        roll_priority_output.thrust[0]>=0. &&
        roll_priority_output.thrust[1]>=0.);
 // If passive venting/current contact reveals both fresh onsets below the
 // historical feed-forward forces, Motion 2 must leave fast approach and
 // continue the bounded search.  It must not deadlock at the small measured
 // forces merely because onset_this_stroke disables fast_approach.
 SinglePass onset_resume;auto onset_resume_input=in;
 onset_resume_input.s.bend={{1.5,1.5}};
 onset_resume_input.s.thrust={{.30,.35}};
 onset_resume_input.s.pressure={{0.,0.}};
 onset_resume_input.s.command={{0.,0.}};
 onset_resume.slip_tested=true;onset_resume.calibrated=true;
 onset_resume.identified_bend_target={{1.30,1.30}};
 onset_resume.identified_extension_thrust={{2.,2.}};
 onset_resume.begin(onset_resume_input);
 onset_resume.enter(SinglePass::Lift,onset_resume_input);
 onset_resume.onset_this_stroke={{true,true}};
 auto onset_resume_output=onset_resume.update(
     onset_resume_input,c,k,model,geom);
 assert(onset_resume.onset_approach_ready[0] &&
        onset_resume.onset_approach_ready[1]);
 assert(onset_resume_output.thrust[0]>onset_resume_input.s.thrust[0] &&
        onset_resume_output.thrust[1]>onset_resume_input.s.thrust[1]);
 // Releasing a chamber can pass the small angle-onset threshold before the
 // reused rotor feed-forward begins.  That passive travel and a soft roll
 // brake must not leave the next Motion-2 reach at [0,0] N indefinitely.
 SinglePass passive_reuse;auto passive_reuse_input=in;
 passive_reuse_input.s.bend={{1.5,1.5}};
 passive_reuse_input.s.thrust={{0.,0.}};
 passive_reuse_input.s.pressure={{0.,0.}};
 passive_reuse_input.s.command={{0.,0.}};
 passive_reuse.slip_tested=true;passive_reuse.calibrated=true;
 passive_reuse.identified_bend_target={{1.30,1.30}};
 passive_reuse.identified_extension_thrust={{2.,2.}};
 passive_reuse.begin(passive_reuse_input);
 passive_reuse.enter(SinglePass::Lift,passive_reuse_input);
 passive_reuse_input.s.bend={{1.42,1.42}}; // passive 4.6-deg release
 passive_reuse_input.s.rate={{-.05,-.05}};
 passive_reuse_input.s.roll=-.08;passive_reuse_input.s.roll_rate=-.05;
 auto passive_reuse_output=passive_reuse.update(
     passive_reuse_input,c,k,model,geom);
 assert(!passive_reuse_output.fault && !passive_reuse_output.recover_roll &&
        passive_reuse_output.thrust[0]+passive_reuse_output.thrust[1]>0.);
 // Different contact angles with equal relative extension are not a
 // synchronization error. Both rotors must begin the search; the lower
 // absolute angle alone must never freeze one arm at zero thrust.
 SinglePass offset_start;auto offset_input=in;
 offset_input.s.bend={{1.85,2.25}};offset_input.s.start_bend=offset_input.s.bend;
 offset_input.s.thrust={{0.,0.}};offset_input.s.command={{0.,0.}};
 offset_input.s.pressure={{0.,0.}};offset_input.s.roll=0.;offset_input.s.roll_rate=0.;
 offset_start.slip_tested=true;offset_start.begin(offset_input);
 offset_start.enter(SinglePass::Lift,offset_input);
 auto offset_output=offset_start.update(offset_input,c,k,model,geom);
 assert(offset_output.thrust[0]>0. && offset_output.thrust[1]>0.);
 // When one arm reaches its independent safety checkpoint first, it is held
 // while the other arm continues the same gentle search. Both chambers stay
 // vented until the measured body-unloading event.
 offset_input.s.thrust={{2.,2.}};
 offset_input.s.bend={{offset_start.active_bend_target[0]-.005,
                       offset_start.active_bend_target[1]+.30}};
 offset_input.s.rate={{0.,0.}};
 auto initial_first_reach=offset_start.update(offset_input,c,k,model,geom);
 assert(offset_start.phase==SinglePass::Lift);
 assert(offset_start.lift_target_reached[0] && !offset_start.lift_target_reached[1]);
 assert(std::abs(initial_first_reach.pressure[0])<1e-9 &&
        std::abs(initial_first_reach.pressure[1])<1e-9);
 assert(initial_first_reach.thrust[0]<=offset_input.s.thrust[0]+1.e-9);
 // Although arm 0 reached its lower absolute bend first, arm 1 has travelled
 // farther from its own contact pose.  The common-travel controller therefore
 // brakes arm 1 pneumatically and must not increase it further.
 assert(initial_first_reach.thrust[1]<=offset_input.s.thrust[1]+1.e-9);
 // A fast measured-rate sample above the target cannot claim an unmeasured
 // target or add pressure before Pull.
 SinglePass imminent;auto imminent_input=in;
 imminent_input.contact_available=false;
 imminent_input.s.bend={{k.identification_bend+.35,
                         k.identification_bend+.35}};
 imminent_input.s.rate={{-1.2,-1.2}};
 imminent_input.s.thrust={{3.,3.}};imminent_input.s.pressure={{0.,0.}};
 imminent_input.s.command={{0.,0.}};imminent.slip_tested=true;
 imminent.begin(imminent_input);imminent.enter(SinglePass::Lift,imminent_input);
 prime_powered_baseline(imminent,imminent_input);
 auto imminent_output=imminent.update(imminent_input,c,k,model,geom);
 assert(!imminent.lift_target_reached[0] && !imminent.lift_target_reached[1]);
 assert(imminent_output.pressure[0]==0. && imminent_output.pressure[1]==0.);
 imminent_input.s.bend={{k.identification_bend-.005,
                         k.identification_bend-.005}};
 imminent_input.s.rate={{0.,0.}};
 imminent_input.s.thrust=imminent_output.thrust;
 // Crossing the angle guard alone no longer completes the reach.
 for(int i=0;i<4;++i) imminent.update(imminent_input,c,k,model,geom);
 assert(imminent.phase==SinglePass::Lift);
 imminent_input.height=.003;
 imminent_input.directional_pitch=k.unload_min_pitch+.01;
 for(int i=0;i<10 && imminent.phase==SinglePass::Lift;++i)
   imminent.update(imminent_input,c,k,model,geom);
 assert(imminent.phase==SinglePass::Pull);
 // Before the two powered onset references have been confirmed, the first
 // Motion-2 command is the bounded simultaneous pulse rather than the former
 // gentle force sweep.
 SinglePass stalled_tracking;auto stalled_input=in;
 stalled_input.s.bend={{1.00,1.00}};stalled_input.s.rate={{0.,0.}};
 stalled_input.s.thrust={{1.5,1.5}};stalled_input.s.pressure={{0.,0.}};
 stalled_input.s.command={{0.,0.}};stalled_input.rear_pressure=30.;
 stalled_tracking.slip_tested=true;stalled_tracking.begin(stalled_input);
 stalled_tracking.enter(SinglePass::Lift,stalled_input);
 stalled_tracking.identification_tracking={{true,true}};
 stalled_tracking.identification_hold_thrust={{.5,.5}};
 stalled_tracking.lift_ramp={{1.5,1.5}};
 auto resumed_search=stalled_tracking.update(stalled_input,c,k,model,geom);
 assert(resumed_search.thrust[0]>stalled_input.s.thrust[0]);
 assert(resumed_search.thrust[0]<=
     stalled_input.s.thrust[0]+k.initial_pulse_rise*stalled_input.dt+1e-9);
 // Raw and filtered rate spikes may shape the pulse, but the actuator command
 // remains bounded by the configured pulse slew and pressure stays zero.
 SinglePass single_spike;auto spike_input=stalled_input;
 spike_input.s.bend={{1.2,1.2}};
 single_spike.slip_tested=true;single_spike.begin(spike_input);
 single_spike.enter(SinglePass::Lift,spike_input);
 single_spike.identification_tracking={{true,true}};
 single_spike.identification_hold_thrust={{1.5,1.5}};
 single_spike.lift_ramp={{1.5,1.5}};
 spike_input.s.rate={{-1.2,-1.2}};
 auto spike_output=single_spike.update(spike_input,c,k,model,geom);
 assert(spike_output.thrust[0]<=spike_input.s.thrust[0]+
     k.initial_pulse_rise*spike_input.dt+1e-9);
 assert(spike_output.pressure[0]==0. && spike_output.pressure[1]==0.);
 SinglePass rate_limited;auto rate_input=stalled_input;
 rate_input.s.bend={{1.2,1.2}};
 rate_limited.slip_tested=true;rate_limited.begin(rate_input);
 rate_limited.enter(SinglePass::Lift,rate_input);
 rate_limited.identification_tracking={{true,true}};
 rate_limited.identification_hold_thrust={{1.5,1.5}};
 rate_limited.lift_ramp={{1.5,1.5}};
 rate_limited.identification_extension_rate_filtered={{.5,.5}};
 rate_limited.identification_fast_rate_time={{
     k.identification_rate_confirm,k.identification_rate_confirm}};
 rate_input.s.rate={{-.5,-.5}};
 auto rate_output=rate_limited.update(rate_input,c,k,model,geom);
 assert(rate_output.thrust[0]<=rate_input.s.thrust[0]+
     k.initial_pulse_rise*rate_input.dt+1e-9);
 assert(rate_output.pressure[0]==0. && rate_output.pressure[1]==0.);
 // Before paired onset is confirmed, even an apparent angle mismatch must
 // retain the same pulse on both rotors. It may be passive decompression and
 // is not enough evidence for one-sided actuation.
 SinglePass continued_sync;auto continued_input=in;
 continued_input.s.bend={{1.5,1.5}};continued_input.s.rate={{0.,0.}};
 continued_input.s.thrust={{1.5,1.5}};continued_input.s.pressure={{0.,0.}};
 continued_input.s.command={{0.,0.}};continued_sync.slip_tested=true;
 continued_sync.begin(continued_input);continued_sync.enter(SinglePass::Lift,continued_input);
 continued_sync.identification_tracking={{true,true}};
 continued_sync.identification_hold_thrust={{1.5,1.5}};
 continued_sync.lift_ramp={{1.5,1.5}};
 continued_input.s.bend={{1.20,1.40}};
 auto continued_output=continued_sync.update(continued_input,c,k,model,geom);
 assert(std::abs(continued_output.thrust[0]-continued_output.thrust[1])<1e-9);
 assert(continued_output.thrust[0]>continued_input.s.thrust[0]);
 assert(continued_output.thrust[0]<=continued_input.s.thrust[0]+
     k.initial_pulse_rise*continued_input.dt+1e-9);
 assert(std::abs(continued_output.pressure[0])<1e-9 &&
        std::abs(continued_output.pressure[1])<1e-9);
 // A one-sample apparent leading rate before paired onset also cannot chop
 // one rotor out of the simultaneous pulse.
 SinglePass moving_lead;auto moving_lead_input=continued_input;
 moving_lead_input.s.bend={{1.5,1.5}};
 moving_lead_input.s.thrust={{1.5,1.5}};
 moving_lead_input.s.rate={{0.,0.}};
 moving_lead.slip_tested=true;moving_lead.begin(moving_lead_input);
 moving_lead.enter(SinglePass::Lift,moving_lead_input);
 moving_lead.identification_tracking={{true,true}};
 moving_lead.identification_hold_thrust={{1.5,1.5}};
 moving_lead.lift_ramp={{1.5,1.5}};
 moving_lead_input.s.bend={{1.20,1.40}};
 moving_lead_input.s.rate={{-.2,0.}};
 auto moving_lead_output=
     moving_lead.update(moving_lead_input,c,k,model,geom);
 assert(std::abs(moving_lead_output.thrust[0]-
                 moving_lead_output.thrust[1])<1e-9);
 assert(moving_lead_output.thrust[0]>=moving_lead_input.s.thrust[0]);
 assert(moving_lead_output.pressure[0]==0. &&
        moving_lead_output.pressure[1]==0.);
 // Reused motion 2 still requires measured extension of both front arms.
 // One extended arm plus body motion is insufficient for Pull.
 SinglePass sequential;auto sequential_input=in;sequential_input.s.bend={{1.5,1.5}};
 sequential_input.contact_available=false;
 sequential_input.s.thrust={{2.,2.}};sequential_input.s.rate={{0.,0.}};
 sequential_input.s.pressure={{0.,0.}};sequential_input.s.command={{0.,0.}};
 sequential.slip_tested=true;sequential.calibrated=true;
 sequential.observed_extension=k.extension_rad;sequential.begin(sequential_input);
 sequential.enter(SinglePass::Lift,sequential_input);
 prime_powered_baseline(sequential,sequential_input);
 sequential_input.s.bend={{1.20,1.48}};sequential_input.height=.003;
 sequential_input.directional_pitch=k.unload_min_pitch+.01;
 auto first_reach=sequential.update(sequential_input,c,k,model,geom);
 assert(sequential.phase==SinglePass::Lift && !sequential.unload_event_detected);
 sequential_input.s.thrust=first_reach.thrust;
 sequential_input.s.command=first_reach.pressure;
 // The second arm joins the reach; a brief confirmation then creates one
 // measured unloading event, without requiring exact angle equality.
 sequential_input.s.bend={{1.20,1.20}};
 auto alternating=sequential.update(sequential_input,c,k,model,geom);
 assert(sequential.phase==SinglePass::Lift);
 sequential_input.s.thrust=alternating.thrust;
 sequential_input.s.command=alternating.pressure;
 for(int i=0;i<10 && sequential.phase==SinglePass::Lift;++i)
   sequential.update(sequential_input,c,k,model,geom);
 assert(sequential.phase==SinglePass::Pull && sequential.unload_event_detected);
 // Hardware arms often reach their useful extension one or two IMU samples
 // apart. Accept that as one event when both threshold crossings occur
 // inside the short pair memory and the CURRENT mismatch is below 10 deg.
 // There is deliberately no sample in this regression where both arms are
 // simultaneously beyond the required extension.
 SinglePass staggered;auto staggered_input=in;
 staggered_input.contact_available=false;
 staggered_input.s.bend={{1.50,1.50}};
 staggered_input.s.thrust={{2.,2.}};
 staggered_input.s.rate={{0.,0.}};
 staggered_input.s.pressure={{0.,0.}};
 staggered_input.s.command={{0.,0.}};
 staggered_input.height=0.;staggered_input.directional_pitch=0.;
 staggered.slip_tested=true;staggered.calibrated=true;
 staggered.observed_extension=k.extension_rad;
 staggered.begin(staggered_input);
 staggered.enter(SinglePass::Lift,staggered_input);
 prime_powered_baseline(staggered,staggered_input);
 staggered_input.height=k.unload_height+.001;
 staggered_input.directional_pitch=k.unload_min_pitch+.01;
 staggered_input.s.bend={{1.20,1.40}}; // only arm 0 reaches first
 staggered.update(staggered_input,c,k,model,geom);
 assert(staggered.phase==SinglePass::Lift &&
        staggered.unload_extension_memory[0]>0. &&
        staggered.unload_extension_memory[1]==0.);
 staggered_input.s.bend={{1.35,1.20}}; // arm 1 reaches after arm 0 rebounds
 for(int i=0;i<10 && staggered.phase==SinglePass::Lift;++i)
   staggered.update(staggered_input,c,k,model,geom);
 assert(staggered.phase==SinglePass::Pull &&
        staggered.unload_event_detected &&
        staggered.unload_pair_mismatch_measured<k.unload_pair_mismatch);
 // Reused motion 2 uses the Action-C 8-N/s rise, not the initial 0.7-N/s
 // identification search. The rear command is already inside the compliant
 // Motion-2 corridor; measured rear angle, rather than 30-kPa anchoring,
 // determines whether the small rotor assist is needed.
 SinglePass reused;auto reused_input=in;reused_input.s.bend={{1.5,1.5}};
 reused_input.contact_available=false;
 reused_input.s.thrust={{0.,0.}};reused_input.s.rate={{0.,0.}};
 reused_input.s.pressure={{0.,0.}};reused_input.s.command={{0.,0.}};
 reused_input.rear_pressure=30.;reused.slip_tested=true;reused.calibrated=true;
 reused.observed_extension=k.extension_rad;
 reused.identified_bend_target={{1.37,1.37}};
 reused.identified_extension_thrust={{1.,1.}};reused.begin(reused_input);
 reused.enter(SinglePass::Lift,reused_input);
 auto fast_lift=reused.update(reused_input,c,k,model,geom);
 assert(fast_lift.thrust[0]>=.15 && fast_lift.thrust[1]>=.15);
 assert(fast_lift.rear_pressure<=k.rear_motion_pressure_cap);
 reused_input.s.thrust=fast_lift.thrust;
 fast_lift=reused.update(reused_input,c,k,model,geom);
 reused_input.s.thrust=fast_lift.thrust;
 fast_lift=reused.update(reused_input,c,k,model,geom);
 reused_input.s.thrust=fast_lift.thrust;
 reused_input.s.bend={{1.44,1.44}};reused_input.s.rate={{-.20,-.20}};
 auto predicted_vent=reused.update(reused_input,c,k,model,geom);
 // Once both front rotors are powered, rear exhaust is allowed to lead the
 // arm event by the measured pneumatic delay.  The command remains bounded
 // by the selected floor and cannot itself complete Motion 2.
 assert(predicted_vent.rear_pressure>=k.lift_rear_pressure_floor &&
        predicted_vent.rear_pressure<=k.rear_motion_pressure_cap);
 reused_input.s.thrust={{1.,1.}};
 reused_input.s.bend={{1.39,1.39}};
 predicted_vent=reused.update(reused_input,c,k,model,geom);
 assert(predicted_vent.rear_pressure>=k.lift_rear_pressure_floor &&
        predicted_vent.rear_pressure<=k.rear_motion_pressure_cap);
 // A learned bend sample still cannot complete Motion 2 without the paired
 // arm/body event, but the already powered pair keeps the pre-vent command
 // active so the slow rear plant is ready when that event arrives.
 reused_input.s.bend={{1.20,1.20}};reused_input.s.rate={{0.,0.}};
 predicted_vent=reused.update(reused_input,c,k,model,geom);
 assert(predicted_vent.rear_pressure>=k.lift_rear_pressure_floor &&
        predicted_vent.rear_pressure<30.);
 assert(reused.phase==SinglePass::Lift);
 reused_input.s.thrust={{2.,2.}};
 reused_input.height=k.unload_height;
 reused_input.directional_pitch=k.unload_min_pitch+.01;
 for(int i=0;i<10 && reused.phase==SinglePass::Lift;++i)
   reused.update(reused_input,c,k,model,geom);
 assert(reused.phase==SinglePass::Pull && reused.unload_event_detected);
 assert(reused.front_target_latched);
 // With explicit simulation contacts, a weak pitch/height transient and less
 // than the requested pair travel are not an unloading event.  A sustained
 // response after BOTH arms extend is accepted before the binary contacts
 // snap open; otherwise the free-extension search itself excites roll.
 SinglePass contact_unload;auto pair_contact_input=in;
 pair_contact_input.s.bend={{1.5,1.5}};pair_contact_input.s.thrust={{2.,2.}};
 pair_contact_input.s.rate={{0.,0.}};pair_contact_input.s.pressure={{0.,0.}};
 pair_contact_input.s.command={{0.,0.}};pair_contact_input.height=0.;
 pair_contact_input.front_contact=false;pair_contact_input.front_released=false;
 contact_unload.slip_tested=true;contact_unload.calibrated=true;
 contact_unload.begin(pair_contact_input);
 contact_unload.enter(SinglePass::Lift,pair_contact_input);
 prime_powered_baseline(contact_unload,pair_contact_input);
 pair_contact_input.s.bend={{1.47,1.47}};
 pair_contact_input.height=.4*k.unload_height;
 pair_contact_input.directional_pitch=.4*k.unload_pitch;
 for(int i=0;i<10;++i)
   contact_unload.update(pair_contact_input,c,k,model,geom);
 assert(contact_unload.phase==SinglePass::Lift &&
        !contact_unload.unload_event_detected);
 pair_contact_input.s.bend={{1.20,1.20}};
 pair_contact_input.height=2.1*k.unload_height;
 pair_contact_input.directional_pitch=2.1*k.unload_pitch;
 for(int i=0;i<10 && contact_unload.phase==SinglePass::Lift;++i)
   contact_unload.update(pair_contact_input,c,k,model,geom);
 assert(contact_unload.phase==SinglePass::Pull &&
        contact_unload.unload_event_detected);
 // A directly measured contact release is useful timing evidence but cannot
 // replace signed body motion. Both arms and the pitch/height event are still
 // required before Motion 3.
 SinglePass direct_contact_unload;auto direct_contact_input=pair_contact_input;
 direct_contact_input.s.bend={{1.5,1.5}};direct_contact_input.height=0.;
 direct_contact_input.directional_pitch=0.;direct_contact_input.front_contact=true;
 direct_contact_input.front_released=false;
 direct_contact_unload.slip_tested=true;direct_contact_unload.calibrated=true;
 direct_contact_unload.begin(direct_contact_input);
 direct_contact_unload.enter(SinglePass::Lift,direct_contact_input);
 prime_powered_baseline(direct_contact_unload,direct_contact_input);
 direct_contact_input.s.bend={{1.20,1.20}};
 direct_contact_input.front_contact=false;direct_contact_input.front_released=true;
 for(int i=0;i<8 && direct_contact_unload.phase==SinglePass::Lift;++i)
   direct_contact_unload.update(direct_contact_input,c,k,model,geom);
 assert(direct_contact_unload.phase==SinglePass::Lift &&
        !direct_contact_unload.unload_event_detected);
 direct_contact_input.directional_pitch=k.unload_pitch+.01;
 direct_contact_input.height=0.;
 for(int i=0;i<8 && direct_contact_unload.phase==SinglePass::Lift;++i)
   direct_contact_unload.update(direct_contact_input,c,k,model,geom);
 assert(direct_contact_unload.phase==SinglePass::Pull &&
        direct_contact_unload.unload_event_detected);
 // The preceding unloading pose remains a timing checkpoint. The new gait
 // nevertheless uses the configured bounded pulse slew on every stroke;
 // reaching the old pose must not switch one arm to a separate command.
 SinglePass checkpoint;auto checkpoint_input=in;
 checkpoint_input.contact_available=false;
 checkpoint_input.s.bend={{1.5,1.5}};checkpoint_input.s.rate={{0.,0.}};
 checkpoint_input.s.thrust={{1.,1.}};checkpoint_input.s.pressure={{0.,0.}};
 checkpoint_input.s.command={{0.,0.}};checkpoint_input.height=0.;
 checkpoint_input.directional_pitch=0.;checkpoint.slip_tested=true;
 checkpoint.calibrated=true;checkpoint.identified_bend_target={{1.37,1.37}};
 checkpoint.observed_extension_by_arm={{.13,.13}};
 checkpoint.identified_extension_thrust={{1.,1.}};
 checkpoint.begin(checkpoint_input);checkpoint.enter(SinglePass::Lift,checkpoint_input);
 checkpoint_input.s.bend={{1.43,1.43}};
 const auto checkpoint_output=checkpoint.update(checkpoint_input,c,k,model,geom);
 assert(checkpoint.phase==SinglePass::Lift &&
        checkpoint.reused_event_checkpoint_reached[0] &&
        checkpoint.reused_event_checkpoint_reached[1]);
 assert(checkpoint_output.thrust[0]<=1.+k.initial_pulse_rise*checkpoint_input.dt+1e-9 &&
        checkpoint_output.thrust[1]<=1.+k.initial_pulse_rise*checkpoint_input.dt+1e-9);
 // Reaching a historical event angle is not permission to re-flex one front
 // arm.  Motion 2 stays at zero pressure (apart from a common delay pilot)
 // until a fresh body unloading event transfers control to Pull.
 assert(checkpoint.lift_angle_pressure_command[0]==0. &&
        checkpoint.lift_angle_pressure_command[1]==0.);
 assert(checkpoint_output.pressure[0]==checkpoint_output.pressure[1] &&
        checkpoint_output.pressure[0]<=k.unload_prefill_kpa);
 // Even for a longer learned reach, pressure remains zero until the paired
 // powered pulse event.  The subsequent Pull phase, not an angle prediction,
 // starts the common pneumatic flexion.
 SinglePass late_pressure;auto late_input=in;
 late_input.s.bend={{1.50,1.50}};late_input.s.thrust={{2.,2.}};
 late_input.s.rate={{0.,0.}};late_input.s.pressure={{0.,0.}};
 late_input.s.command={{0.,0.}};late_pressure.slip_tested=true;
 late_pressure.calibrated=true;late_pressure.onset_identified=true;
 late_pressure.identified_bend_target={{1.00,1.00}};
 late_pressure.identified_extension_thrust={{2.,2.}};
 late_pressure.begin(late_input);late_pressure.enter(SinglePass::Lift,late_input);
 late_input.s.bend={{1.25,1.25}}; // 50% of the learned reach
 auto early_reused=late_pressure.update(late_input,c,k,model,geom);
 assert(early_reused.pressure[0]==0. && early_reused.pressure[1]==0.);
 late_input.s.command=early_reused.pressure;late_input.s.thrust=early_reused.thrust;
 late_input.s.bend={{1.02,1.02}}; // prediction alone cannot start pressure
 auto late_reused=late_pressure.update(late_input,c,k,model,geom);
 assert(late_reused.pressure[0]==0. && late_reused.pressure[1]==0.);
 in.s.thrust={{2.,2.}};
 in.s.bend={{k.identification_bend-.005,
             k.identification_bend-.005}};
 in.s.rate={{0.,0.}};in.s.pressure={{0.,0.}};in.height=.003;
 in.directional_pitch=k.unload_min_pitch+.01;
 in.front_contact=false;in.front_released=true;
 SinglePassOutput o;
 for(int i=0;i<10 && p.phase==SinglePass::Lift;++i)
   o=p.update(in,c,k,model,geom);
 // Pull inherits the compliant Motion-2 rear support without jumping back to
 // 30 kPa. Further exhaust follows measured front flexion.
 assert(p.phase==SinglePass::Pull && p.lift_tested &&
        std::abs(o.rear_pressure-p.rearPressureFloor(k))<1e-9);
 // Command pressure is not sufficient: without measured flexion the rear
 // pair remains at the configured Motion-2 floor rather than changing state.
 in.s.pressure={{40.,40.}};in.s.command=o.pressure;in.s.thrust=o.thrust;
 for(int i=0;i<30;++i){o=p.update(in,c,k,model,geom);assert(!o.regrip && std::abs(o.rear_pressure-p.rearPressureFloor(k))<1e-9);}
 in.s.bend={{1.58,1.58}};in.s.rate={{0.,0.}};
 in.front_contact=false;
 // Pressure-driven flexion without a measured contact must not restore the
 // old rear anchor: that would erase the newly placed front point.
 for(int i=0;i<30;++i)o=p.update(in,c,k,model,geom);
 assert(!o.regrip &&
        std::abs(o.rear_pressure-p.rearPressureFloor(k))<1e-9);
 in.front_contact=true;
 // Even after real front contact, Motion 3 retains the compliant rear pair.
 // The outer regrip state, entered after this pass returns regrip=true, owns
 // the later restoration to grasp pressure.
 for(int i=0;i<250 &&
     std::abs(o.rear_pressure-p.rearPressureFloor(k))<1e-9;++i)
   o=p.update(in,c,k,model,geom);
 assert(o.rear_pressure<=k.rear_motion_pressure_cap);
 // The first near-straight envelope stays valid until adapter regrip.  A
 // small settling drift in FrontCatch must not restore the ordinary floor.
 assert(p.identifying_straight);
 p.stroke_lifted=true;p.pull_forward=1.;
 assert(p.adaptPullAfterMeasuredStroke(-.003,k));
 assert(std::abs(p.pull_forward-1.5)<1e-9);
 assert(!p.adaptPullAfterMeasuredStroke(-.004,k)); // comparison still pending
 assert(p.finishPullCalibration(-.004)==-1);
 assert(std::abs(p.pull_forward-1.)<1e-9);
 assert(p.pull_calibration_complete);
 assert(!p.adaptPullAfterMeasuredStroke(.003,k));
 // An improving A/B comparison also closes inner calibration; it must not
 // chain another +dF change before the outer mix/force optimizer runs.
 SinglePass one_comparison;one_comparison.stroke_lifted=true;
 one_comparison.pull_forward=1.;
 assert(one_comparison.adaptPullAfterMeasuredStroke(-.003,k));
 assert(one_comparison.finishPullCalibration(-.001)==1);
 assert(one_comparison.pull_calibration_complete &&
        std::abs(one_comparison.pull_forward-1.5)<1e-9);
 assert(!one_comparison.adaptPullAfterMeasuredStroke(-.0005,k));
 p.pull_forward=k.pull_adaptation_max;
 assert(!p.adaptPullAfterMeasuredStroke(0.,k));
 // A longer minimum pull is configurable, but never bypasses contact/pressure readiness.
 auto long_pull=k;long_pull.pull_min_seconds=2.;
 p.enter(SinglePass::Pull,in);
 for(int i=0;i<99;++i){o=p.update(in,c,long_pull,model,geom);assert(!o.regrip);}
 o=p.update(in,c,long_pull,model,geom);assert(o.regrip);
 in.front_contact=false;o=p.update(in,c,long_pull,model,geom);assert(!o.regrip);
 in.front_contact=true;
 p.initialized=false;p.begin(in);
 assert(p.phase==SinglePass::StopForLift && p.slip_entries==1 && p.lift_probe_entries==1);
 // A failed first lift must not lower the next target to a passive rear-release excursion.
 SinglePass failed;SinglePassInput stuck=in;stuck.s.bend={{1.5,1.5}};
 stuck.s.thrust={{0.,0.}};stuck.s.command={{30.,30.}};stuck.s.pressure=stuck.s.command;
 failed.slip_tested=true;failed.begin(stuck);failed.enter(SinglePass::Lift,stuck);
 failed.powered_time=k.lift_seconds;auto failure=failed.update(stuck,c,k,model,geom);
 assert(failure.fault && failed.phase==SinglePass::Lift && !failed.calibrated && failed.observed_extension==0.);
 failed.begin(stuck);failed.enter(SinglePass::Lift,stuck);stuck.s.bend={{1.46,1.46}};
 auto retry=failed.update(stuck,c,k,model,geom);
 assert(failed.phase==SinglePass::Lift && retry.angle_target[0]<1.37);
 SinglePass small;auto small_k=k;small_k.extension_rad=.035;
 stuck.s.bend={{1.5,1.5}};stuck.s.thrust={{2.,2.}};
 small.slip_tested=true;small.calibrated=true;small.observed_extension=.035;
 small.begin(stuck);small.enter(SinglePass::Lift,stuck);stuck.s.bend={{1.46,1.46}};
 small.update(stuck,c,small_k,model,geom);
 assert(small.phase==SinglePass::Lift && !small.unload_event_detected);
 // A nearby geometric limit must not turn a short arm excursion into a valid
 // unloading event.  Without the meaningful trajectory travel, stay in Lift
 // even if the body-height signal changes.
 SinglePass bounded;auto near=in;near.s.bend={{1.125,1.125}};
 near.s.floor={{.95,.95}};near.s.thrust={{2.,2.}};near.s.rate={{0.,0.}};
 bounded.slip_tested=true;bounded.calibrated=true;bounded.observed_extension=k.extension_rad;
 bounded.begin(near);bounded.enter(SinglePass::Lift,near);
 prime_powered_baseline(bounded,near);
 near.s.bend={{.985,.985}};near.height+=.003;
 SinglePassOutput caught_before_guard;
 for(int i=0;i<10 && bounded.phase==SinglePass::Lift;++i)
   caught_before_guard=bounded.update(near,c,k,model,geom);
 assert(bounded.phase==SinglePass::Lift && bounded.calibrated &&
        !bounded.unload_event_detected && !caught_before_guard.fault);
 assert(near.s.floor[0]==.95);
 // Pressure/angle never arrives: bounded catch wait and explicit fault.
 p.enter(SinglePass::FrontCatch,in);in.s.pressure={{0.,0.}};
 for(int i=0;i<500 && !o.fault;++i)o=p.update(in,c,k,model,geom);
 assert(o.fault);
 // A pressure-stable contact within the configured four-degree real-contact
 // tolerance is a completed catch, even if it cannot reproduce the exact
 // pre-lift bend against the new contact point.
 SinglePass catch_ok;auto caught=in;caught.s.bend={{1.4,1.4}};
 caught.s.pressure={{30.,30.}};caught.s.command=caught.s.pressure;
 caught.s.rate={{0.,0.}};catch_ok.begin(caught);
 caught.s.bend={{1.4-k.front_catch_tolerance+.001,
                 1.4-k.front_catch_tolerance+.001}};
 catch_ok.enter(SinglePass::FrontCatch,caught);
 SinglePassOutput catch_output;
 for(int i=0;i<35 && !catch_output.regrip;++i)
   catch_output=catch_ok.update(caught,c,k,model,geom);
 assert(catch_output.regrip && !catch_output.fault);
 // Terminal rear lift is deliberately not emitted by SinglePass.  The outer
 // regrip owns it because only that layer has all four measured arm angles and
 // can choose the bidirectional rotor sign which actually points upward.
 SinglePass terminal_catch;auto terminal_input=caught;
 terminal_input.s.bend={{1.4,1.4}};
 terminal_input.s.start_bend=terminal_input.s.bend;
 terminal_input.s.pressure={{30.,30.}};
 terminal_input.s.command=terminal_input.s.pressure;
 terminal_input.s.thrust={{.5,.5}};
 terminal_input.s.rate={{0.,0.}};
 terminal_input.rear_pressure=0.;
 terminal_input.rear_released=true;
 terminal_input.front_contact=true;
 terminal_catch.begin(terminal_input);
 terminal_input.s.bend={{1.4-terminal_k.front_catch_tolerance+.001,
                        1.4-terminal_k.front_catch_tolerance+.001}};
 terminal_catch.enter(SinglePass::FrontCatch,terminal_input);
 SinglePassOutput terminal_output;bool saw_terminal_lift=false;
 for(int i=0;i<120 && !terminal_output.regrip;++i) {
   terminal_output=terminal_catch.update(
       terminal_input,c,terminal_k,model,geom);
   if(terminal_output.rear_positive_thrust>.05)
     saw_terminal_lift=true;
   terminal_input.s.thrust=terminal_output.thrust;
   terminal_input.s.command=terminal_output.pressure;
 }
 assert(!saw_terminal_lift && !terminal_catch.terminal_rear_lift_started &&
        terminal_output.regrip && !terminal_output.fault &&
        terminal_output.rear_positive_thrust==0.);
 SinglePass no_measured_anchor;auto no_anchor_input=terminal_input;
 no_anchor_input.s.pressure={{27.,27.}};
 no_anchor_input.s.command={{30.,30.}};
 no_measured_anchor.begin(no_anchor_input);
 no_measured_anchor.enter(SinglePass::FrontCatch,no_anchor_input);
 SinglePassOutput no_anchor_output;
 for(int i=0;i<40 && !no_anchor_output.regrip;++i)
   no_anchor_output=no_measured_anchor.update(
       no_anchor_input,c,terminal_k,model,geom);
 assert(no_anchor_output.regrip &&
        !no_measured_anchor.terminal_rear_lift_started &&
        no_anchor_output.rear_positive_thrust==0.);
 // Physical contact can stop one arm short of its pre-lift free-space angle.
 // High catch pressure, a safe stationary bend and stable roll confirm that
 // contact. Preserve the residual pull at the handoff: the outer regrip
 // controller removes it only after the rear pressure is load bearing.
 SinglePass contact_catch;auto contact_input=in;
 contact_input.s.bend={{1.20,1.20}};contact_input.s.start_bend=contact_input.s.bend;
 contact_input.s.floor={{.95,.95}};contact_input.s.pressure={{40.,40.}};
 contact_input.s.command=contact_input.s.pressure;
 contact_input.s.thrust={{.5,.5}};contact_input.s.rate={{0.,0.}};
 contact_input.s.roll=-.14;contact_input.s.roll_rate=0.;
 contact_catch.begin(contact_input);
 contact_input.s.bend={{1.03,1.04}}; // safely above floor, but >4 deg short of target
 contact_catch.enter(SinglePass::FrontCatch,contact_input);
 SinglePassOutput contact_output;
 for(int i=0;i<170 && !contact_output.regrip;++i)
   contact_output=contact_catch.update(contact_input,c,k,model,geom);
 assert(contact_output.regrip && contact_output.contact_limited_regrip);
 assert(contact_output.thrust[0]>0. && contact_output.thrust[0]<=.5);
 assert(contact_output.thrust[1]>0. && contact_output.thrust[1]<=.5);
 // Pressure below the measured support-transfer threshold cannot masquerade
 // as contact-limited regrasp.
 SinglePass low_pressure_catch;contact_input.s.bend={{1.20,1.20}};
 contact_input.s.pressure={{2.,2.}};
 low_pressure_catch.begin(contact_input);contact_input.s.bend={{1.03,1.04}};
 low_pressure_catch.enter(SinglePass::FrontCatch,contact_input);
 for(int i=0;i<100;++i) contact_output=low_pressure_catch.update(contact_input,c,k,model,geom);
 assert(!contact_output.regrip);
 // A load-bearing newly placed contact should end Pull before the old grasp
 // angle is reproduced. This is the normal Motion-3 completion path.  The
 // loaded contact remains powered for the configured three-second traction
 // overlap; ending at the former 1.5 s recovered only the Motion-2 setback.
 SinglePass early_contact;auto early_input=in;
 early_input.s.bend={{1.57,1.57}};early_input.s.pressure={{30.,30.}};
 early_input.s.command=early_input.s.pressure;early_input.s.rate={{0.,0.}};
 early_input.front_contact=true;early_contact.begin(early_input);
 early_input.s.bend={{1.30,1.30}};
 early_contact.enter(SinglePass::Pull,early_input);
 early_contact.lift_force=2.;early_contact.lift_up=.5;
 early_contact.pull_forward=2.;early_input.s.bend={{1.45,1.45}};
 // A pressure inside the controller tolerance below the nominal 18-kPa
 // contact threshold is load bearing; exact floating-point equality is not
 // required.  The separate 5-kPa test below remains insufficient.
 early_input.s.pressure={{15.1,15.1}};early_input.s.command=early_input.s.pressure;
 SinglePassOutput early_contact_output;
 for(int i=0;i<149;++i) {
   early_contact_output=early_contact.update(early_input,c,k,model,geom);
   assert(!early_contact_output.regrip);
 }
 for(int i=0;i<20 && !early_contact_output.regrip;++i)
   early_contact_output=early_contact.update(early_input,c,k,model,geom);
 assert(early_contact_output.regrip &&
     early_contact_output.contact_limited_regrip);

 // Overshooting the pre-lift bend is not an exact-angle catch.  It must use
 // the load-bearing/contact-limited path and therefore respect the longer
 // traction dwell instead of ending Motion 3 at pull_min_seconds.
 SinglePass overflexed;auto overflexed_input=early_input;
 overflexed_input.s.bend={{1.57,1.57}};
 overflexed_input.s.pressure={{30.,30.}};
 overflexed_input.s.command=overflexed_input.s.pressure;
 overflexed_input.s.rate={{0.,0.}};
 overflexed_input.front_contact=true;
 overflexed.begin(overflexed_input);
 overflexed_input.s.bend={{1.30,1.30}};
 overflexed.enter(SinglePass::Pull,overflexed_input);
 overflexed_input.s.bend={{1.80,1.80}};
 overflexed.lift_force=2.;overflexed.lift_up=.5;
 overflexed.pull_forward=2.;
 SinglePassOutput overflexed_output;
 const int short_dwell_steps=int(k.pull_min_seconds/overflexed_input.dt)+10;
 for(int i=0;i<short_dwell_steps;++i) {
   overflexed_output=overflexed.update(overflexed_input,c,k,model,geom);
   assert(!overflexed_output.regrip);
 }
 for(int i=short_dwell_steps;
     i<int(k.pull_loaded_catch_min_seconds/overflexed_input.dt)+20 &&
         !overflexed_output.regrip;++i)
   overflexed_output=overflexed.update(overflexed_input,c,k,model,geom);
 assert(overflexed_output.regrip &&
     overflexed_output.contact_limited_regrip);
 // If roll is confirmed to be running outward after both arms have reversed
 // under a load-bearing synchronized pressure, restore the rear support even
 // before an explicit front-contact bit arrives. This is a safety transfer,
 // not a normal contact-success declaration.
 SinglePass guarded_catch;auto guarded_input=contact_input;
 guarded_input.front_contact=false;
 guarded_input.s.bend={{1.32,1.32}};
 // Rear refill may start from a small front pressure because it is a slow
 // overlapping support transfer, not a declaration that full grasp is done.
 guarded_input.s.pressure={{6.,6.}};
 guarded_input.s.command=guarded_input.s.pressure;
 guarded_input.s.roll=0.;guarded_input.s.roll_rate=0.;
 guarded_catch.begin(guarded_input);
 guarded_input.s.bend={{1.38,1.38}};
 guarded_catch.enter(SinglePass::Pull,guarded_input);
 guarded_input.s.roll=-.20;guarded_input.s.roll_rate=-.1;
 guarded_catch.pull_start={{1.32,1.32}};
 guarded_catch.grasp_reference={{1.50,1.50}};
 guarded_catch.lift_force=2.;guarded_catch.lift_up=.5;
 guarded_catch.pull_forward=2.;
 SinglePassOutput guarded_output;
 for(int i=0;i<50 && !guarded_output.regrip;++i)
   guarded_output=guarded_catch.update(guarded_input,c,k,model,geom);
 assert(guarded_output.regrip && guarded_output.contact_limited_regrip);
 // If that guard cancels rear exhaust before the measured rear-release
 // threshold, the guard itself may clear while all four chambers refill.
 // A stable, pressure-loaded four-arm boundary must still complete the
 // stroke; requiring rear_release_observed here deadlocks FrontCatch.
 SinglePass cancelled_release;auto cancelled_input=contact_input;
 cancelled_input.front_contact=true;cancelled_input.rear_released=false;
 cancelled_input.rear_grasped=false;
 cancelled_input.s.bend={{1.32,1.32}};
 cancelled_input.s.pressure={{18.,18.}};
 cancelled_input.s.command=cancelled_input.s.pressure;
 cancelled_input.s.roll=0.;cancelled_input.s.roll_rate=0.;
 cancelled_release.begin(cancelled_input);
 cancelled_release.enter(SinglePass::Pull,cancelled_input);
 cancelled_release.pull_start={{1.32,1.32}};
 cancelled_release.grasp_reference={{1.50,1.50}};
 cancelled_release.lift_force=2.;cancelled_release.lift_up=.5;
 cancelled_release.pull_forward=2.;
 cancelled_release.rear_release_commanded=true;
 cancelled_release.rear_pressure_command=0.;
 cancelled_release.rear_release_pressure_threshold=3.;
 cancelled_input.s.bend={{1.40,1.40}};
 cancelled_input.s.roll=-.20;cancelled_input.s.roll_rate=-.1;
 SinglePassOutput cancelled_output;
 for(int i=0;i<20 && !cancelled_release.support_transfer_started;++i)
   cancelled_output=cancelled_release.update(
       cancelled_input,c,k,model,geom);
 assert(cancelled_release.support_transfer_started &&
        !cancelled_release.rear_release_observed);
 cancelled_input.s.roll=0.;cancelled_input.s.roll_rate=0.;
 cancelled_input.s.pressure={{30.,30.}};
 cancelled_input.s.command=cancelled_input.s.pressure;
 cancelled_input.rear_grasped=true;
 for(int i=0;i<200 && !cancelled_output.regrip;++i)
   cancelled_output=cancelled_release.update(
       cancelled_input,c,k,model,geom);
 assert(cancelled_output.regrip &&
        cancelled_output.contact_limited_regrip &&
        !cancelled_release.rear_release_observed);
 // A stable biased roll must not abort a reused lift on gyro noise alone.
 SinglePass biased;biased.slip_tested=true;biased.lift_tested=true;biased.calibrated=true;biased.observed_extension=.1;
 in.s.roll=-.18;in.s.roll_rate=-.001;in.s.pressure={{30.,30.}};in.s.command=in.s.pressure;
 biased.begin(in);biased.enter(SinglePass::Lift,in);o=biased.update(in,c,k,model,geom);assert(biased.phase==SinglePass::Lift);
 in.s.roll=0.;in.s.roll_rate=0.;
 // Bottom/extension-induced pitch is part of the gait and must not erase the
 // short pull window. Hard pitch/rate protection remains in Planner.
 auto pitch_geom=[](size_t,double,double,double){Force f;f.forward=.7;f.up=.5;f.pitch_moment=.1;return f;};
 SinglePass pitched;auto pitched_in=in;pitched_in.s.bend={{1.5,1.5}};
 pitched_in.s.thrust={{1.,1.}};pitched_in.s.pitch=0.;pitched_in.pitch_rate=0.;
 pitched.slip_tested=true;pitched.begin(pitched_in);pitched.enter(SinglePass::Pull,pitched_in);
 pitched.lift_force=1.;pitched.lift_up=1.;pitched.pull_forward=2.;
 auto level_in=pitched_in;
 pitched_in.s.pitch=.13;pitched_in.pitch_rate=.1;
 auto pitch_pause=pitched.update(pitched_in,c,k,model,pitch_geom);
 SinglePass level;level.slip_tested=true;level.begin(level_in);
 level.enter(SinglePass::Pull,level_in);level.lift_force=1.;
 level.lift_up=1.;level.pull_forward=2.;
 auto level_pull=level.update(level_in,c,k,model,pitch_geom);
 assert(!pitch_pause.fault && !pitch_pause.recover_roll &&
        std::abs(pitch_pause.thrust[0]-level_pull.thrust[0])<1e-8);
 // Once pressure-driven Pull has begun, worsening posture uses the normal
 // fast thrust rundown; free-extension Lift keeps its gentler brake.
 SinglePass pull_brake;auto braking_in=in;braking_in.s.bend={{1.4,1.4}};
 braking_in.s.thrust={{3.,3.}};braking_in.s.roll=-.13;braking_in.s.roll_rate=-.1;
 braking_in.s.rate={{0.,0.}};pull_brake.slip_tested=true;
 pull_brake.begin(braking_in);pull_brake.enter(SinglePass::Pull,braking_in);
 pull_brake.lift_force=3.;pull_brake.lift_up=1.;pull_brake.pull_forward=4.;
 SinglePassOutput pull_braked;
 for(int i=0;i<6;++i) {
   pull_braked=pull_brake.update(braking_in,c,k,model,geom);
   braking_in.s.thrust=pull_braked.thrust;
 }
 assert(pull_braked.thrust[0]<=3.-c.thrust_fall*braking_in.dt+1e-8);
 // Either sign of outward roll stays inside Motion 2 after the short,
 // calibrated feed-forward grace. The inner allocator owns this transient;
 // the adapter's independent measured hard-roll/rate guard owns recovery.
 for(double sign:{-1.,1.}) {
   auto r=in;r.s.bend={{1.5,1.5}};r.s.thrust={{1.,1.}};
   r.s.roll=sign*.13;r.s.roll_rate=sign*.1;r.s.rate={{0.,0.}};
   SinglePass recover;recover.slip_tested=true;recover.calibrated=true;
   recover.observed_extension=k.extension_rad;recover.begin(r);recover.enter(SinglePass::Lift,r);
   auto handoff=recover.update(r,c,k,model,geom);
   assert(!handoff.recover_roll); // one angular-rate sample is not enough
   for(int i=0;i<30;++i)handoff=recover.update(r,c,k,model,geom);
   assert(!handoff.recover_roll && !handoff.fault && recover.calibrated);
   r.s.roll_rate=-sign*.1;
   for(int i=0;i<3;++i)handoff=recover.update(r,c,k,model,geom);
   assert(!handoff.recover_roll);
 }
 // Effective slip holds thrust and bypasses lift identification.
 SinglePass slide;in.s.pressure={{30.,30.}};in.s.command=in.s.pressure;in.s.thrust={{2.,2.}};
 in.s.speed=.01;in.x=0.;slide.begin(in);
 slide.update(in,c,k,model,geom); // actual rear release establishes the origin
 in.x=.05;
 for(int i=0;i<10;++i)o=slide.update(in,c,k,model,geom);
 assert(slide.phase==SinglePass::SlipProbe); // 0.2 s is shorter than confirmation
 for(int i=0;i<10;++i)o=slide.update(in,c,k,model,geom);
 assert(slide.phase==SinglePass::SlipSettle && !slide.slip_tested);
 in.s.thrust={{0.,0.}};in.s.speed=0.;in.s.pressure={{30.,30.}};
 in.rear_grasped=true;in.rear_released=false;
 for(int i=0;i<30 && slide.phase==SinglePass::SlipSettle;++i)
   o=slide.update(in,c,k,model,geom);
 assert(slide.phase==SinglePass::Slide && slide.slip_tested &&
     slide.sliding_effective && slide.lift_probe_entries==0 &&
     std::abs(slide.rear_support_prior-k.rear_support_probe_sliding)<1e-9);
 // A detected slide is useful only while it advances the retained position.
 // Alternating positive velocity samples at essentially the same x must hand
 // control to motion 2 after the three-second no-progress window.
 for(int i=0;i<180 && slide.phase==SinglePass::Slide;++i) {
   in.x=.05+(i%2?2.e-4:0.);in.s.speed=(i%2)?.01:-.01;
   o=slide.update(in,c,k,model,geom);
 }
 assert((slide.phase==SinglePass::StopForLift ||
         slide.phase==SinglePass::VentFront) && !slide.sliding_effective &&
     std::abs(slide.rear_support_prior-k.rear_support_probe_stable)<1e-9);
 // Genuine, even slow, monotonic progress refreshes that watchdog and keeps
 // the low-friction sliding strategy active.
 SinglePass progressing_slide;auto progressing=in;
 progressing.x=0.;progressing.s.speed=.01;progressing.s.thrust={{2.,2.}};
 progressing.rear_released=true;progressing.rear_grasped=false;
 progressing_slide.begin(progressing);
 progressing_slide.update(progressing,c,k,model,geom);
 progressing.x=.05;
 for(int i=0;i<20;++i)progressing_slide.update(progressing,c,k,model,geom);
 assert(progressing_slide.phase==SinglePass::SlipSettle);
 progressing.s.thrust={{0.,0.}};progressing.s.speed=0.;
 progressing.s.pressure={{30.,30.}};progressing.rear_released=false;
 progressing.rear_grasped=true;
 for(int i=0;i<30 && progressing_slide.phase==SinglePass::SlipSettle;++i)
   progressing_slide.update(progressing,c,k,model,geom);
 assert(progressing_slide.phase==SinglePass::Slide);
 progressing.s.thrust={{2.,2.}};progressing.s.speed=.0015;
 for(int i=0;i<400;++i) {
   progressing.x=.05+.0011*static_cast<double>(i/100);
   progressing.s.speed=.0015;
   progressing_slide.update(progressing,c,k,model,geom);
 }
 assert(progressing_slide.phase==SinglePass::Slide &&
     progressing_slide.sliding_effective);
 // The real-arm failure mode: rear/bottom settling moved about 3 mm while
 // thrust was only 0.26 N.  It must neither pass immediately nor be banked as
 // displacement for a later deliberate-thrust sample.
 SinglePass passive_motion;auto passive=in;passive.x=0.;passive.s.speed=.01;
 passive.rear_released=true;passive.rear_grasped=false;
 passive.s.thrust={{.26,.26}};passive_motion.begin(passive);
 passive_motion.update(passive,c,k,model,geom);
 passive.x=.003;
 for(int i=0;i<30;++i)passive_motion.update(passive,c,k,model,geom);
 assert(passive_motion.phase==SinglePass::SlipProbe &&
     !passive_motion.slip_measurement_started);
 passive.s.thrust={{.80,.80}};
 passive_motion.update(passive,c,k,model,geom); // new deliberate-force origin
 passive.x=.006;
 for(int i=0;i<20;++i)passive_motion.update(passive,c,k,model,geom);
 assert(passive_motion.phase==SinglePass::SlipSettle);
 passive.s.thrust={{0.,0.}};passive.s.speed=0.;
 passive.s.pressure={{30.,30.}};passive.rear_released=false;
 passive.rear_grasped=true;
 for(int i=0;i<30 && passive_motion.phase==SinglePass::SlipSettle;++i)
   passive_motion.update(passive,c,k,model,geom);
 assert(passive_motion.phase==SinglePass::Slide);
 // The same peak displacement is rejected when compliant contact springs
 // back after force removal: this is deformation, not reusable sliding.
 SinglePass deformation;auto deform=in;deform.x=0.;deform.s.speed=.01;
 deform.s.thrust={{2.,2.}};deform.rear_released=true;deform.rear_grasped=false;
 deformation.begin(deform);deformation.update(deform,c,k,model,geom);
 deform.x=.004;
 for(int i=0;i<20;++i)deformation.update(deform,c,k,model,geom);
 assert(deformation.phase==SinglePass::SlipSettle);
 deform.x=.0002;deform.s.speed=0.;deform.s.thrust={{0.,0.}};
 deform.s.pressure={{30.,30.}};deform.rear_released=false;
 deform.rear_grasped=true;
 for(int i=0;i<30 && deformation.phase==SinglePass::SlipSettle;++i)
   deformation.update(deform,c,k,model,geom);
 assert((deformation.phase==SinglePass::StopForLift ||
         deformation.phase==SinglePass::VentFront) &&
     !deformation.sliding_effective);
 // Neither mocap velocity without net travel nor a displaced but stationary
 // trace may pass the two-part slip test.
 SinglePass noisy_speed;auto noisy=in;noisy.x=0.;noisy.s.speed=.01;
 noisy_speed.begin(noisy);noisy.x=.0005;
 for(int i=0;i<100;++i)noisy_speed.update(noisy,c,k,model,geom);
 assert(noisy_speed.phase==SinglePass::SlipProbe && !noisy_speed.sliding_effective);
 SinglePass displaced_static;auto static_in=in;static_in.x=0.;static_in.s.speed=0.;
 displaced_static.begin(static_in);static_in.x=.05;
 for(int i=0;i<100;++i)displaced_static.update(static_in,c,k,model,geom);
 assert(displaced_static.phase==SinglePass::SlipProbe &&
     !displaced_static.sliding_effective);
 // A sustained soft roll brake rejects pure sliding after a zero-thrust
 // endpoint check.  One transient sample remains harmless because the roll
 // condition has its own confirmation time.
 SinglePass soft_brake;auto soft=in;soft.x=0.;soft.s.speed=0.;
 soft.s.roll=0.;soft.s.roll_rate=0.;soft_brake.begin(soft);
 soft.s.roll=k.slip_roll_abort_angle+.02;soft.s.roll_rate=.1;
 for(int i=0;i<4;++i)soft_brake.update(soft,c,k,model,geom);
 assert(soft_brake.phase==SinglePass::SlipProbe && !soft_brake.slip_tested);
 for(int i=0;i<6 && soft_brake.phase==SinglePass::SlipProbe;++i)
   soft_brake.update(soft,c,k,model,geom);
 assert(soft_brake.phase==SinglePass::SlipSettle &&
     soft_brake.slip_probe_unstable && !soft_brake.slip_tested);
 soft.s.thrust={{0.,0.}};soft.s.pressure=soft_brake.slip_settle_pressure;
 soft.s.command=soft.s.pressure;soft.rear_grasped=true;
 soft.speed_valid=true;soft.s.speed=0.;
 for(int i=0;i<40 && soft_brake.phase==SinglePass::SlipSettle;++i)
   soft_brake.update(soft,c,k,model,geom);
 assert(soft_brake.slip_tested && !soft_brake.sliding_effective &&
     (soft_brake.phase==SinglePass::StopForLift ||
      soft_brake.phase==SinglePass::VentFront));
 // pink_17 regression: crossing the diagnostic extension wall must apply one
 // slow COMMON brake.  It must not chop the leading arm by several newtons,
 // then chase the unchanged common ramp and repeat on the other arm.
 SinglePass extension_brake;auto extended=in;
 extended.s.thrust={{3.,3.}};extended.s.pressure={{25.,25.}};
 extended.s.command=extended.s.pressure;extended.rear_released=true;
 extension_brake.begin(extended);
 extended.s.bend[0]-=k.slip_max_extension+k.slip_hard_extension_margin+.01;
 auto softly_braked=extension_brake.update(extended,c,k,model,geom);
 assert(extension_brake.slip_extension_braking);
 assert(std::abs(softly_braked.thrust[0]-softly_braked.thrust[1])<1e-12);
 assert(softly_braked.thrust[0]<3. &&
     softly_braked.thrust[0]>=3.-k.slip_extension_brake_fall*extended.dt-1e-9);
 assert(softly_braked.pressure[0]>=extended.s.command[0]);
 // Actual flexion permits increasing pull thrust; pressure alone and a
 // predicted rate cannot lift that measured-angle allowance.
 SinglePass pull;SinglePassInput pulling=in;
 pulling.s.bend={{1.57,1.57}};pulling.s.thrust={{.3,.3}};pulling.s.rate={{0.,0.}};
 pull.begin(pulling);pull.calibrated=true;
 pull.identified_bend_target={{1.30,1.30}};
 pulling.s.bend={{1.30,1.30}};pull.enter(SinglePass::Pull,pulling);
 pull.reference={{1.35,1.35}};pull.grasp_reference=pull.reference;
 // Use a pull request which is actually above the residual event-hold force
 // after projection through this synthetic rotor geometry.  The assertion is
 // about measured flexion releasing traction, not about the projection
 // saturating at the hold floor.
 pull.lift_force=2.;pull.lift_up=1.;pull.pull_forward=8.;
 auto early=pull.update(pulling,c,k,model,geom);
 assert(early.thrust[0]<=2. &&
        std::abs(early.rear_pressure-k.lift_rear_pressure_floor)<1e-9);
 pulling.s.thrust=early.thrust;
 pulling.s.bend={{1.34,1.34}};
 auto flexed=pull.update(pulling,c,k,model,geom);
 assert(flexed.thrust[0]>early.thrust[0]);
 assert(flexed.thrust[0]<=2.+c.thrust_rise*pulling.dt+1e-8);
 assert(!flexed.regrip && flexed.rear_pressure==0.);
 // Front flexion/pressure must not refill the old rear anchors before their
 // measured pressure has crossed the existing release hysteresis.  Otherwise
 // the old contact is never reset and regrasp pulls the body back to its
 // previous equilibrium even though Motion 2 advanced the front contact.
 SinglePass rear_release_gate;auto delayed_rear=pulling;
 delayed_rear.s.bend={{1.57,1.57}};
 delayed_rear.s.pressure={{30.,30.}};delayed_rear.s.command={{30.,30.}};
 delayed_rear.s.thrust={{2.,2.}};delayed_rear.s.rate={{0.,0.}};
 delayed_rear.rear_released=false;
 rear_release_gate.begin(delayed_rear);
 rear_release_gate.enter(SinglePass::Lift,delayed_rear);
 delayed_rear.s.bend={{1.30,1.30}};
 rear_release_gate.enter(SinglePass::Pull,delayed_rear);
 delayed_rear.s.bend={{1.50,1.50}};
 SinglePassOutput gated;
 for(int i=0;i<75;++i) {
   gated=rear_release_gate.update(delayed_rear,c,k,model,geom);
   assert(!rear_release_gate.support_transfer_started);
   assert(!gated.regrip && gated.rear_pressure==0.);
 }
 delayed_rear.rear_released=true;
 bool transferred=false,regripped=false;
 for(int i=0;i<75 && !regripped;++i) {
   gated=rear_release_gate.update(delayed_rear,c,k,model,geom);
   transferred=transferred || rear_release_gate.support_transfer_started;
   regripped=regripped || gated.regrip;
 }
 assert(rear_release_gate.rear_release_observed && transferred && regripped);
 // Pneumatic synchronization is reserved for a large measured flexion
 // mismatch which is also producing outward body roll.  This prevents noisy
 // per-arm IMU leader swaps from creating 30/10-kPa valve chatter.  When it
 // is active, the arm which flexed farther refills more slowly.
 SinglePass pressure_roll;auto pressure_roll_in=pulling;
 pressure_roll_in.s.roll=.10;pressure_roll_in.s.roll_rate=.10;
 pressure_roll_in.s.bend={{1.30,1.30}};
 pressure_roll_in.s.pressure={{18.,18.}};
 pressure_roll_in.s.command={{30.,30.}};
 pressure_roll_in.s.thrust={{1.,1.}};
 pressure_roll_in.rear_released=true;
 pressure_roll.begin(pressure_roll_in);
 pressure_roll.enter(SinglePass::Lift,pressure_roll_in);
 pressure_roll.enter(SinglePass::Pull,pressure_roll_in);
 pressure_roll.grasp_reference={{1.57,1.57}};
 pressure_roll.pull_start={{1.20,1.20}};
 pressure_roll_in.s.bend={{1.50,1.30}};
 auto sim_k=k;sim_k.reverse_geometry_roll_moment=true;
 SinglePassOutput roll_shaped;
 for(int i=0;i<30;++i) {
   roll_shaped=pressure_roll.update(pressure_roll_in,c,sim_k,model,geom);
   pressure_roll_in.s.command=roll_shaped.pressure;
 }
 assert(roll_shaped.pressure[0]<roll_shaped.pressure[1]);
 std::cout<<"PASS: one slip/lift pass, cyclic horizontal rear, rear held low through actual front catch, bounded outputs and timeout\n";
}
'''
with tempfile.TemporaryDirectory(prefix='hugmy_single_pass_') as folder:
    binary=str(Path(folder)/'test')
    subprocess.run(['g++','-std=c++14','-O2','-I'+str(root/'include'),'-x','c++','-','-o',binary],input=source,text=True,check=True)
    subprocess.run([binary],check=True)
