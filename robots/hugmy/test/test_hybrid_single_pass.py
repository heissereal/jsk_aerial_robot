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
 // Hardware pair sign is independent of the contact-lever sign reversal.
 auto verified=verifiedPairRollMoments({{-.02,.03}});
 assert(verified[0]>.0 && verified[1]<.0);
 auto positive_roll=addRestoringRollSupport({{2.,2.}},verified,.2,.1,6.);
 assert(positive_roll[0]==2. && positive_roll[1]>2.); // +roll -> arm 3/2 side
 auto negative_roll=addRestoringRollSupport({{2.,2.}},verified,-.2,-.1,6.);
 assert(negative_roll[0]>2. && negative_roll[1]==2.); // -roll -> arm 0/1 side
 auto held=observed.rate;assert(observed.update(99.,observed.stamp)==held);
 assert(observed.update(1.,5.)==0.);assert(observed.update(NAN,6.)==0.);
 assert(k.valid());auto bad=k;bad.rear_low_seconds=0;assert(!bad.valid());
 bad=k;bad.reused_lift_rise=8.1;assert(!bad.valid());
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
 bad=k;bad.reused_lift_pressure_start_ratio=.95;assert(!bad.valid());
 bad=k;bad.goal_progress_max_scale=1.51;assert(!bad.valid());
 bad=k;bad.bottom_kpa=31;assert(!bad.valid());bad=k;bad.pull_seconds=NAN;assert(!bad.valid());
 bad=k;bad.pull_min_seconds=4.1;assert(!bad.valid());
 bad=k;bad.pull_min_seconds=NAN;assert(!bad.valid());
 bad=k;bad.pull_adaptation_step=1.1;assert(!bad.valid());
 bad=k;bad.goal_conditioning_start_ratio=.99;assert(!bad.valid());
 auto geom=[](size_t,double a,double,double){Force f;f.forward=.7*std::sin(a);f.up=std::max(0.,std::cos(a));return f;};
 SinglePassInput in;in.s.fresh=true;in.s.bend={{1.57,1.57}};in.s.start_bend=in.s.bend;
 in.s.pressure={{30.,30.}};in.s.command=in.s.pressure;in.s.floor={{.7,.7}};
 in.rear_released=true;in.speed_valid=true;in.contact_available=true;in.front_contact=true;
 in.s.bottom=30.;in.s.bottom_command=30.;in.s.now=1.;
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
 assert(short_goal.active_extension_target[0]<full_goal.active_extension_target[0]);
 assert(short_goal.active_extension_target[0]>=
        k.goal_extension_min_scale*full_goal.active_extension_target[0]-1e-9);
 // Per-arm attainable ABSOLUTE bend is reused independently. A changed grasp
 // angle changes the necessary excursion, not the final rotor attitude.
 SinglePass asymmetric_reach;auto asymmetric_input=in;
 asymmetric_reach.slip_tested=true;asymmetric_reach.calibrated=true;
 asymmetric_reach.identified_bend_target={{1.37,1.27}};
 asymmetric_reach.begin(asymmetric_input);
 asymmetric_reach.enter(SinglePass::Lift,asymmetric_input);
 asymmetric_reach.update(asymmetric_input,c,k,model,geom);
 assert(asymmetric_reach.active_extension_target[0]<
        asymmetric_reach.active_extension_target[1]);
 assert(std::abs(asymmetric_reach.active_extension_target[0]-.20)<1e-9);
 assert(std::abs(asymmetric_reach.active_extension_target[1]-.30)<1e-9);
 assert(std::abs(asymmetric_reach.active_bend_target[0]-1.37)<1e-9);
 assert(std::abs(asymmetric_reach.active_bend_target[1]-1.27)<1e-9);
 // Compare the pressure-driven pull at 70% measured flexion. The short goal
 // has already spent its bounded impulse; the full stroke remains powered.
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
 assert(full_lift.thrust[0]>short_lift.thrust[0]);
 assert(short_goal.goal_pull_envelope==0.);
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
 assert(p.pull_forward<=k.slip_seed_pull_max+1e-9);
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
 // High slip thrust is removed while the rear regrips. Front stays inflated.
 for(int i=0;i<60;++i){auto o=p.update(in,c,k,model,geom);assert(o.rear_pressure==30.);in.s.thrust=o.thrust;in.s.command=o.pressure;}
 assert(p.phase==SinglePass::StopForLift && in.s.thrust[0]==0.);
 in.rear_grasped=true;
 // Recover residual roll before removing front support. Both roll signs use
 // the same handoff, and the completed slip probe is retained.
 in.s.roll=.16;auto pre_vent_recovery=p.update(in,c,k,model,geom);
 assert(pre_vent_recovery.recover_roll && p.phase==SinglePass::StopForLift);
 in.s.roll=-.16;pre_vent_recovery=p.update(in,c,k,model,geom);
 assert(pre_vent_recovery.recover_roll && p.phase==SinglePass::StopForLift);
 in.s.roll=0.;
 for(int i=0;i<50;++i){auto o=p.update(in,c,k,model,geom);assert(o.thrust[0]==0.);in.s.command=o.pressure;}
 assert(p.phase==SinglePass::VentFront && p.powered_time==0.);
 assert(p.update(in,c,k,model,geom).rear_pressure==30.); // rear anchor stays grasped
 assert(in.s.command[0]==0.); // measured pressure is still 30: no powered search
 in.s.pressure={{0.,0.}};
 for(int i=0;i<11;++i){auto o=p.update(in,c,k,model,geom);in.s.command=o.pressure;in.s.thrust=o.thrust;}
 assert(p.phase==SinglePass::Lift && p.lift_probe_entries==1);
 assert(p.update(in,c,k,model,geom).rear_pressure==30.); // motion 2 begins rear-grasped
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
 pose_input.s.bend={{1.44,1.44}};pose_input.height=.01;pose_input.s.pitch=.08;
 auto pose_shortcut=pose_only.update(pose_input,c,k,model,geom);
 assert(pose_only.phase==SinglePass::Lift && pose_shortcut.pressure[0]==0.);
 // Relative travel, body unloading and a settled angular rate are NOT
 // completion. A stall far from the absolute target remains an incomplete
 // force-identification test.
 pose_input.s.bend={{1.30,1.20}};
 for(int i=0;i<15;++i)
   pose_only.update(pose_input,c,k,model,geom);
 assert(pose_only.phase==SinglePass::Lift && !pose_only.calibrated);
 assert(!pose_only.lift_target_reached[0] &&
        !pose_only.lift_target_reached[1]);
 // Finish the same test only with measured target angles. Dynamic overshoot
 // below the target must not make the next stroke more aggressive.
 pose_input.s.bend={{k.identification_bend-.05,k.identification_bend-.03}};
 for(int i=0;i<4 && pose_only.phase==SinglePass::Lift;++i)
   pose_only.update(pose_input,c,k,model,geom);
 assert(pose_only.phase==SinglePass::Pull && pose_only.calibrated);
 assert(std::abs(pose_only.identified_bend_target[0]-k.identification_bend)<1e-9);
 assert(std::abs(pose_only.identified_bend_target[1]-k.identification_bend)<1e-9);
 assert(std::isfinite(pose_only.identified_extension_thrust[0]));
 assert(std::isfinite(pose_only.identified_extension_thrust[1]));
 pose_input.s.bend={{1.70,1.80}};pose_input.s.start_bend=pose_input.s.bend;
 pose_input.height=0.;pose_input.s.pitch=0.;pose_input.s.rate={{0.,0.}};
 pose_only.initialized=false;pose_only.begin(pose_input);
 pose_only.enter(SinglePass::Lift,pose_input);
 pose_only.update(pose_input,c,k,model,geom);
 assert(std::abs(pose_only.active_bend_target[0]-
        std::max(k.identification_bend,1.70-k.identification_extension_limit))<1e-9);
 assert(std::abs(pose_only.active_bend_target[1]-
        std::max(k.identification_bend,1.80-k.identification_extension_limit))<1e-9);
 assert(std::abs(pose_only.active_extension_target[0]-
        std::min(1.70-k.identification_bend,k.identification_extension_limit))<1e-9);
 assert(std::abs(pose_only.active_extension_target[1]-
        std::min(1.80-k.identification_bend,k.identification_extension_limit))<1e-9);
 // Without reaching the measured absolute target, a clear relative reach is
 // never automatic success, regardless of body response.
 SinglePass clear_reach;auto clear_input=in;
 clear_input.s.bend={{1.5,1.5}};clear_input.s.thrust={{2.,2.}};
 clear_input.height=0.;clear_input.s.pitch=0.;
 clear_reach.slip_tested=true;clear_reach.begin(clear_input);
 clear_reach.enter(SinglePass::Lift,clear_input);
 clear_input.s.bend={{1.20,1.20}};
 for(int i=0;i<12;++i)clear_reach.update(clear_input,c,k,model,geom);
 assert(clear_reach.phase==SinglePass::Lift && !clear_reach.calibrated);
 // One-arm reach, or a reach whose predicted roll is already outside the
 // recovery boundary, cannot become calibration.
 SinglePass one_arm;auto one_input=clear_input;
 one_input.s.bend={{1.5,1.5}};one_arm.slip_tested=true;
 one_arm.begin(one_input);one_arm.enter(SinglePass::Lift,one_input);
 one_input.s.bend={{1.20,1.42}};
 for(int i=0;i<5;++i)one_arm.update(one_input,c,k,model,geom);
 assert(one_arm.phase==SinglePass::Lift && !one_arm.calibrated);
 SinglePass unsafe_contact;auto unsafe_input=clear_input;
 unsafe_input.s.bend={{1.5,1.5}};unsafe_contact.slip_tested=true;
 unsafe_contact.begin(unsafe_input);unsafe_contact.enter(SinglePass::Lift,unsafe_input);
 unsafe_input.s.bend={{1.20,1.20}};unsafe_input.s.roll=.13;unsafe_input.s.roll_rate=.1;
 unsafe_input.height=.01;
 for(int i=0;i<12;++i)unsafe_contact.update(unsafe_input,c,k,model,geom);
 assert(unsafe_contact.phase==SinglePass::Lift && !unsafe_contact.calibrated);
 // The nominal absolute unloading route is the only first-test completion.
 SinglePass absolute_pose;auto absolute_input=in;
 absolute_input.s.bend={{1.5,1.5}};absolute_input.s.thrust={{2.,2.}};
 absolute_pose.slip_tested=true;absolute_pose.begin(absolute_input);
 absolute_pose.enter(SinglePass::Lift,absolute_input);
 absolute_input.s.bend={{k.identification_bend-.005,
                         k.identification_bend-.005}};
 for(int i=0;i<3;++i)absolute_pose.update(absolute_input,c,k,model,geom);
 assert(absolute_pose.phase==SinglePass::Pull && absolute_pose.calibrated);
 assert(std::abs(absolute_pose.identified_bend_target[0]-
        k.identification_bend)<1e-9);
 // Slope overlap is a post-identification locomotion variation. It must not
 // replace the first absolute-bend force test.
 SinglePass slope_overlap;auto slope_input=in;
 slope_input.directional_pitch=-.20;
 slope_input.s.bend={{1.40,1.40}};slope_input.s.start_bend=slope_input.s.bend;
 slope_input.s.thrust={{2.,2.}};slope_input.s.pressure={{0.,0.}};
 slope_input.s.command={{0.,0.}};slope_input.s.rate={{-.5,-.5}};
 slope_overlap.slip_tested=true;slope_overlap.calibrated=true;
 slope_overlap.identified_bend_target={{k.identification_bend,k.identification_bend}};
 slope_overlap.identified_extension_thrust={{2.,2.}};
 slope_overlap.begin(slope_input);
 slope_overlap.enter(SinglePass::Lift,slope_input);
 slope_input.s.bend={{.86,.86}};
 auto overlap_prefill=slope_overlap.update(slope_input,c,k,model,geom);
 assert(slope_overlap.phase==SinglePass::Lift && slope_overlap.slope_overlap_active);
 assert(overlap_prefill.pressure[0]>0. && overlap_prefill.pressure[1]>0.);
 assert(std::abs(slope_overlap.active_bend_target[0]-k.identification_bend)<1e-9);
 // Once the substantial reach has been observed and measured pressure has
 // arrived, hand off while still bent enough to create horizontal force.
 slope_input.s.command=overlap_prefill.pressure;
 slope_input.s.pressure={{3.,3.}};slope_input.s.bend={{.88,.88}};
 slope_input.s.rate={{.1,.1}};slope_input.s.thrust=overlap_prefill.thrust;
 for(int i=0;i<4 && slope_overlap.phase==SinglePass::Lift;++i) {
   overlap_prefill=slope_overlap.update(slope_input,c,k,model,geom);
   slope_input.s.command=overlap_prefill.pressure;
   slope_input.s.thrust=overlap_prefill.thrust;
 }
 assert(slope_overlap.phase==SinglePass::Pull);
 assert(slope_input.s.bend[0]>k.identification_bend);
 // If the pilot pressure physically reverses both adequately extended arms
 // before the fixed handoff angle, retain their measured peaks and begin the
 // pressure-driven pull instead of timing out in Lift.
 SinglePass slope_turn;auto turn_input=in;
 turn_input.directional_pitch=-.20;
 turn_input.s.bend={{1.40,1.40}};turn_input.s.start_bend=turn_input.s.bend;
 turn_input.s.thrust={{2.,2.}};turn_input.s.pressure={{0.,0.}};
 turn_input.s.command={{0.,0.}};turn_input.s.rate={{-.2,-.2}};
 slope_turn.slip_tested=true;slope_turn.calibrated=true;
 slope_turn.identified_bend_target={{k.identification_bend,k.identification_bend}};
 slope_turn.identified_extension_thrust={{2.,2.}};
 slope_turn.begin(turn_input);
 slope_turn.enter(SinglePass::Lift,turn_input);
 turn_input.s.bend={{.86,.86}};
 auto turn_prefill=slope_turn.update(turn_input,c,k,model,geom);
 assert(slope_turn.phase==SinglePass::Lift && turn_prefill.pressure[0]>0.);
 turn_input.s.command=turn_prefill.pressure;turn_input.s.pressure={{3.,3.}};
 turn_input.s.bend={{.88,.88}};turn_input.s.rate={{.1,.1}};
 turn_input.s.thrust=turn_prefill.thrust;
 slope_turn.update(turn_input,c,k,model,geom);
 assert(slope_turn.phase==SinglePass::Pull);
 // During straight-identification breakaway, a large angle mismatch raises
 // the lagging arm instead of dropping the leading arm's required hold force
 // or handing a soft-roll warning directly to recovery. Pressure stays zero.
 SinglePass synchronized;auto sync_input=in;sync_input.s.bend={{1.5,1.5}};
 sync_input.s.thrust={{3.,3.}};sync_input.s.command={{0.,0.}};
 sync_input.s.pressure={{0.,0.}};sync_input.s.roll=-.12;sync_input.s.roll_rate=-.1;
 synchronized.slip_tested=true;synchronized.begin(sync_input);
 synchronized.enter(SinglePass::Lift,sync_input);
 sync_input.s.bend={{1.2,1.45}};
 auto sync_output=synchronized.update(sync_input,c,k,model,geom);
 assert(synchronized.phase==SinglePass::Lift && !sync_output.recover_roll);
 assert(synchronized.identification_tracking[0]);
 assert(sync_output.thrust[1]>sync_input.s.thrust[1]);
 assert(sync_output.pressure[0]==0. && sync_output.pressure[1]==0.);
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
 // Initial identification also latches each absolute target independently
 // and holds the reached arm while its partner remains short.
 offset_input.s.thrust={{2.,2.}};
 offset_input.s.bend={{offset_start.active_bend_target[0]-.005,
                       offset_start.active_bend_target[1]+.30}};
 offset_input.s.rate={{0.,0.}};
 auto initial_first_reach=offset_start.update(offset_input,c,k,model,geom);
 assert(offset_start.phase==SinglePass::Lift);
 assert(offset_start.lift_target_reached[0] && !offset_start.lift_target_reached[1]);
 assert(initial_first_reach.pressure[0]==0. && initial_first_reach.pressure[1]==0.);
 assert(initial_first_reach.thrust[0]<offset_input.s.thrust[0]);
 // A fast measured-rate sample above the target cannot claim an unmeasured
 // target or add pressure before Pull.
 SinglePass imminent;auto imminent_input=in;
 imminent_input.s.bend={{k.identification_bend+.15,
                         k.identification_bend+.15}};
 imminent_input.s.rate={{-1.2,-1.2}};
 imminent_input.s.thrust={{3.,3.}};imminent_input.s.pressure={{0.,0.}};
 imminent_input.s.command={{0.,0.}};imminent.slip_tested=true;
 imminent.begin(imminent_input);imminent.enter(SinglePass::Lift,imminent_input);
 auto imminent_output=imminent.update(imminent_input,c,k,model,geom);
 assert(!imminent.lift_target_reached[0] && !imminent.lift_target_reached[1]);
 assert(imminent_output.pressure[0]==0. && imminent_output.pressure[1]==0.);
 imminent_input.s.bend={{k.identification_bend-.005,
                         k.identification_bend-.005}};
 imminent_input.s.rate={{0.,0.}};
 imminent_input.s.thrust=imminent_output.thrust;
 for(int i=0;i<4 && imminent.phase==SinglePass::Lift;++i)
   imminent.update(imminent_input,c,k,model,geom);
 assert(imminent.phase==SinglePass::Pull);
 // With both arms past onset but still short of the absolute target, the
 // bounded rate controller must continue the search. It increases only at the
 // gentle first-pass slew rather than stopping at the breakaway force.
 SinglePass stalled_tracking;auto stalled_input=in;
 stalled_input.s.bend={{.60,.60}};stalled_input.s.rate={{0.,0.}};
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
     stalled_input.s.thrust[0]+k.lift_rise*stalled_input.dt+1e-9);
 // The first motion-2 test regulates extension speed after both breakaways.
 // A stalled pair continues the gentle search. Both a single raw-rate spike
 // and confirmed filtered overspeed pause the increase without unloading the
 // arm and causing a rebound.
 SinglePass single_spike;auto spike_input=stalled_input;
 spike_input.s.bend={{1.2,1.2}};
 single_spike.slip_tested=true;single_spike.begin(spike_input);
 single_spike.enter(SinglePass::Lift,spike_input);
 single_spike.identification_tracking={{true,true}};
 single_spike.identification_hold_thrust={{1.5,1.5}};
 single_spike.lift_ramp={{1.5,1.5}};
 spike_input.s.rate={{-1.2,-1.2}};
 auto spike_output=single_spike.update(spike_input,c,k,model,geom);
 assert(std::abs(spike_output.thrust[0]-spike_input.s.thrust[0])<1e-9);
 assert(single_spike.identification_rate_command_slope[0]==0.);
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
 assert(rate_limited.identification_rate_command_slope[0]==0.);
 assert(std::abs(rate_output.thrust[0]-rate_input.s.thrust[0])<1e-9);
 // Synchronization must continue after BOTH breakaway onsets. The real-arm
 // run diverged only after that boundary because the old code disabled sync.
 // Raise the lagging arm by a slew-bounded amount; never re-accelerate the
 // leading arm or add the unbounded multi-newton mismatch correction.
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
 assert(continued_output.thrust[0]<=continued_input.s.thrust[0]+1e-9);
 assert(continued_output.thrust[1]>continued_output.thrust[0]);
 assert(continued_output.thrust[1]<=continued_input.s.thrust[1]+
     k.identification_tracking_rise*continued_input.dt+1e-9);
 // Reused motion 2 captures each arm independently.  The first arm receives
 // pressure-assisted angle holding and reduced thrust while the other keeps
 // searching; a bare historical peak outside the holding corridor is still
 // insufficient for Pull.
 SinglePass sequential;auto sequential_input=in;sequential_input.s.bend={{1.5,1.5}};
 sequential_input.s.thrust={{2.,2.}};sequential_input.s.rate={{0.,0.}};
 sequential_input.s.pressure={{0.,0.}};sequential_input.s.command={{0.,0.}};
 sequential.slip_tested=true;sequential.calibrated=true;
 sequential.observed_extension=k.extension_rad;sequential.begin(sequential_input);
 sequential.enter(SinglePass::Lift,sequential_input);
 sequential_input.s.bend={{1.34,1.48}};
 auto first_reach=sequential.update(sequential_input,c,k,model,geom);
 assert(sequential.phase==SinglePass::Lift);
 assert(sequential.lift_target_reached[0] && !sequential.lift_target_reached[1]);
 // The reached arm receives the stronger hold pressure. The lagging arm may
 // already receive a smaller delay-compensated pilot after measured motion
 // onset, but must remain free to continue extending.
 assert(first_reach.pressure[0]>first_reach.pressure[1]);
 assert(first_reach.pressure[0]<=k.reused_lift_hold_pressure_max);
 assert(first_reach.thrust[0]<sequential_input.s.thrust[0]);
 sequential_input.s.thrust=first_reach.thrust;
 sequential_input.s.command=first_reach.pressure;
 // A rebound of the first arm does not erase its measured reach. The second
 // reach completes the pair; pressure feedback owns the first arm meanwhile.
 sequential_input.s.bend={{1.48,1.34}};
 auto alternating=sequential.update(sequential_input,c,k,model,geom);
 assert(sequential.phase==SinglePass::Lift);
 assert(sequential.lift_target_reached[0] && sequential.lift_target_reached[1]);
 // No exact simultaneous equality is required after both measured events.
 sequential_input.s.thrust=alternating.thrust;
 sequential_input.s.command=alternating.pressure;
 sequential_input.s.bend={{1.39,1.34}};
 sequential.update(sequential_input,c,k,model,geom);
 assert(sequential.phase==SinglePass::Pull);
 // Reused motion 2 uses the Action-C 8-N/s rise, not the initial 0.7-N/s
 // identification search. Rear grasp remains the anchor at its edge; exhaust
 // starts from the measured target-time prediction after extension is visible.
 SinglePass reused;auto reused_input=in;reused_input.s.bend={{1.5,1.5}};
 reused_input.s.thrust={{0.,0.}};reused_input.s.rate={{0.,0.}};
 reused_input.s.pressure={{0.,0.}};reused_input.s.command={{0.,0.}};
 reused_input.rear_pressure=30.;reused.slip_tested=true;reused.calibrated=true;
 reused.observed_extension=k.extension_rad;reused.begin(reused_input);
 reused.enter(SinglePass::Lift,reused_input);
 auto fast_lift=reused.update(reused_input,c,k,model,geom);
 assert(fast_lift.thrust[0]>=.15 && fast_lift.thrust[1]>=.15);
 assert(fast_lift.rear_pressure==30.);
 reused_input.s.thrust=fast_lift.thrust;
 fast_lift=reused.update(reused_input,c,k,model,geom);
 reused_input.s.thrust=fast_lift.thrust;
 fast_lift=reused.update(reused_input,c,k,model,geom);
 reused_input.s.thrust=fast_lift.thrust;
 reused_input.s.bend={{1.44,1.44}};reused_input.s.rate={{-.20,-.20}};
 auto predicted_vent=reused.update(reused_input,c,k,model,geom);
 assert(predicted_vent.rear_pressure==30.); // still farther than the 2-deg lead
 reused_input.s.thrust={{1.,1.}};
 reused_input.s.bend={{1.39,1.39}};
 predicted_vent=reused.update(reused_input,c,k,model,geom);
 assert(predicted_vent.rear_pressure==0.);
 // Front completion owns the transition. Measured rear pressure must not keep
 // the zero-pressure, near-straight pair in Lift after the target is reached.
 reused_input.s.bend={{1.37,1.37}};reused_input.s.rate={{0.,0.}};
 reused.update(reused_input,c,k,model,geom);
 assert(reused.phase==SinglePass::Pull);
 assert(reused.front_target_latched);
 // For a longer learned absolute reach, pressure remains zero through the
 // early 90% instead of creating a far-from-target equilibrium. It appears
 // only in the terminal segment where it can brake the arm.
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
 late_input.s.bend={{1.02,1.02}}; // 96%: terminal pressure braking is enabled
 auto late_reused=late_pressure.update(late_input,c,k,model,geom);
 assert(late_reused.pressure[0]>0. && late_reused.pressure[1]>0.);
 in.s.thrust={{2.,2.}};
 in.s.bend={{k.identification_bend-.005,
             k.identification_bend-.005}};
 in.s.rate={{0.,0.}};in.s.pressure={{0.,0.}};
 SinglePassOutput o;
 for(int i=0;i<3;++i)o=p.update(in,c,k,model,geom);
 assert(p.phase==SinglePass::Pull && p.lift_tested && o.rear_pressure==0.);
 // Command pressure is not sufficient: no observed flexion => no regrasp.
 in.s.pressure={{40.,40.}};in.s.command=o.pressure;in.s.thrust=o.thrust;
 for(int i=0;i<30;++i){o=p.update(in,c,k,model,geom);assert(!o.regrip && o.rear_pressure==0.);}
 in.s.bend={{1.58,1.58}};in.s.rate={{0.,0.}};
 in.front_contact=false;
 for(int i=0;i<30;++i){o=p.update(in,c,k,model,geom);assert(!o.regrip && o.rear_pressure==0.);}
 in.front_contact=true;
 for(int i=0;i<15;++i)o=p.update(in,c,k,model,geom);
 assert(o.regrip && o.rear_pressure==0.); // adapter starts rear refill AFTER this
 // The first near-straight envelope stays valid until adapter regrip.  A
 // small settling drift in FrontCatch must not restore the ordinary floor.
 assert(p.identifying_straight);
 p.stroke_lifted=true;p.pull_forward=1.;
 assert(p.adaptPullAfterMeasuredStroke(-.003,k));
 assert(std::abs(p.pull_forward-1.5)<1e-9);
 assert(!p.adaptPullAfterMeasuredStroke(.003,k));
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
 assert(small.phase==SinglePass::Pull && small.observed_extension<=small_k.extension_rad);
 // A requested Action-C-sized extension cannot be reached behind the braking margin of
 // a nearby floor. Shorten the target, NEVER lower the floor to fit it.
 SinglePass bounded;auto near=in;near.s.bend={{1.125,1.125}};
 near.s.floor={{.95,.95}};near.s.thrust={{2.,2.}};near.s.rate={{0.,0.}};
 bounded.slip_tested=true;bounded.calibrated=true;bounded.observed_extension=k.extension_rad;
 bounded.begin(near);bounded.enter(SinglePass::Lift,near);
 near.s.bend={{.995,.995}};
 auto caught_before_guard=bounded.update(near,c,k,model,geom);
 assert(bounded.phase==SinglePass::Pull && bounded.calibrated && !caught_before_guard.fault);
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
 // Physical contact can stop one arm short of its pre-lift free-space angle.
 // High catch pressure, a safe stationary bend and stable roll confirm that
 // contact; the residual 0.5-N pull must be removed at the handoff.
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
 for(int i=0;i<50 && !contact_output.regrip;++i)
   contact_output=contact_catch.update(contact_input,c,k,model,geom);
 assert(contact_output.regrip && contact_output.contact_limited_regrip);
 assert(contact_output.thrust[0]==0. && contact_output.thrust[1]==0.);
 // Low pressure cannot masquerade as contact-limited regrasp.
 SinglePass low_pressure_catch;contact_input.s.bend={{1.20,1.20}};
 contact_input.s.pressure={{35.,40.}};
 low_pressure_catch.begin(contact_input);contact_input.s.bend={{1.03,1.04}};
 low_pressure_catch.enter(SinglePass::FrontCatch,contact_input);
 for(int i=0;i<50;++i) contact_output=low_pressure_catch.update(contact_input,c,k,model,geom);
 assert(!contact_output.regrip);
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
 pitched_in.s.pitch=.13;pitched_in.pitch_rate=.1;
 auto pitch_pause=pitched.update(pitched_in,c,k,model,pitch_geom);
 assert(!pitch_pause.fault && !pitch_pause.recover_roll &&
        pitch_pause.thrust[0]>=pitched_in.s.thrust[0]);
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
 // Either sign of outward roll hands over to recovery, not success or a
 // terminal timeout; a stable bias and inward rotation do not request it.
 for(double sign:{-1.,1.}) {
   auto r=in;r.s.bend={{1.5,1.5}};r.s.thrust={{1.,1.}};
   r.s.roll=sign*.13;r.s.roll_rate=sign*.1;r.s.rate={{0.,0.}};
   SinglePass recover;recover.slip_tested=true;recover.calibrated=true;
   recover.observed_extension=k.extension_rad;recover.begin(r);recover.enter(SinglePass::Lift,r);
   auto handoff=recover.update(r,c,k,model,geom);
   assert(!handoff.recover_roll); // one angular-rate sample is not enough
   for(int i=0;i<5;++i)handoff=recover.update(r,c,k,model,geom);
   assert(handoff.recover_roll && !handoff.fault && recover.calibrated);
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
 assert(slide.phase==SinglePass::Slide && slide.slip_tested && slide.lift_probe_entries==0);
 // A detected slide is useful only while it advances the retained position.
 // Alternating positive velocity samples at essentially the same x must hand
 // control to motion 2 after the three-second no-progress window.
 for(int i=0;i<151;++i) {
   in.x=.05+(i%2?2.e-4:0.);in.s.speed=(i%2)?.01:-.01;
   o=slide.update(in,c,k,model,geom);
 }
 assert(slide.phase==SinglePass::StopForLift && !slide.sliding_effective);
 // Genuine, even slow, monotonic progress refreshes that watchdog and keeps
 // the low-friction sliding strategy active.
 SinglePass progressing_slide;auto progressing=in;
 progressing.x=0.;progressing.s.speed=.01;progressing.s.thrust={{2.,2.}};
 progressing_slide.begin(progressing);
 progressing_slide.update(progressing,c,k,model,geom);
 progressing.x=.05;
 for(int i=0;i<20;++i)progressing_slide.update(progressing,c,k,model,geom);
 assert(progressing_slide.phase==SinglePass::Slide);
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
 assert(passive_motion.phase==SinglePass::Slide);
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
 // A sustained soft roll brake pauses the probe ramp but no longer labels the
 // whole slip test as failed. The independent timeout/hard guard still owns
 // termination.
 SinglePass soft_brake;auto soft=in;soft.x=0.;soft.s.speed=0.;
 soft.s.roll=.13;soft.s.roll_rate=.1;soft_brake.begin(soft);
 for(int i=0;i<10;++i)soft_brake.update(soft,c,k,model,geom);
 assert(soft_brake.phase==SinglePass::SlipProbe && !soft_brake.slip_tested &&
     soft_brake.roll_risk_time>=k.soft_roll_confirm-1e-9);
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
 pulling.s.bend={{1.57,1.57}};pulling.s.thrust={{2.,2.}};pulling.s.rate={{0.,0.}};
 pull.begin(pulling);pulling.s.bend={{1.30,1.30}};pull.enter(SinglePass::Pull,pulling);
 pull.lift_force=2.;pull.lift_up=1.;pull.pull_forward=4.;
 auto early=pull.update(pulling,c,k,model,geom);
 assert(early.thrust[0]<=2. && early.rear_pressure==0.);
 pulling.s.bend={{1.45,1.45}};
 auto flexed=pull.update(pulling,c,k,model,geom);
 assert(flexed.thrust[0]>early.thrust[0]);
 assert(flexed.thrust[0]<=2.+c.thrust_rise*pulling.dt+1e-8);
 assert(!flexed.regrip && flexed.rear_pressure==0.);
 std::cout<<"PASS: one slip/lift pass, cyclic horizontal rear, rear held low through actual front catch, bounded outputs and timeout\n";
}
'''
with tempfile.TemporaryDirectory(prefix='hugmy_single_pass_') as folder:
    binary=str(Path(folder)/'test')
    subprocess.run(['g++','-std=c++14','-O2','-I'+str(root/'include'),'-x','c++','-','-o',binary],input=source,text=True,check=True)
    subprocess.run([binary],check=True)
