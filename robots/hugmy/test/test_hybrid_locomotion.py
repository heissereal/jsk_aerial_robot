#!/usr/bin/env python3
"""Compile/run the production inner MPC and outer search without ROS/hardware."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CODE = r'''
#include <hugmy/control/hybrid_locomotion_search.h>
#include <hugmy/control/hybrid_measurement.h>
#include <cassert>
#include <iostream>
using namespace hugmy::hybrid;
int main() {
 Config c; c.budget_ms=1000.; assert(c.valid());
 assert(c.forward_force_max>=2*c.thrust_max);
 auto rear_config=c;rear_config.sliding_rear_pressure=5.;assert(rear_config.valid());
 rear_config.sliding_rear_pressure=6.01;assert(!rear_config.valid());
 rear_config.sliding_rear_pressure=NAN;assert(!rear_config.valid());
 // Transverse balancing reduces only, uses direction not motor index, and
 // restores roll without breaking the normal thrust slew constraints.
 auto balanced=balanceLateral({{4.,3.}},{{.6,-.6}},0.,0.);
 assert(std::abs(balanced[0]-3.)<1e-10 && balanced[1]==3.);
 auto restoring=balanceLateral({{4.,4.}},{{.6,-.6}},-.1,0.);
 assert(restoring[0]<4. && restoring[1]==4.);
 auto mirrored=balanceLateral({{4.,4.}},{{-.6,.6}},.1,0.);
 assert(std::abs(restoring[0]-mirrored[0])<1e-10 && restoring[1]==mirrored[1]);
 // Low bottom targets share one tolerance at preparation and roll boundaries.
 assert(bottomPressureReady(3.,0.,2.));assert(bottomPressureReady(5.,0.,2.));
 assert(bottomPressureReady(-5.,0.,2.));assert(!bottomPressureReady(5.01,0.,2.));
 assert(bottomPressureReady(9.9,5.,2.));assert(!bottomPressureReady(8.1,6.,2.));
 assert(!bottomPressureReady(NAN,0.,2.));assert(!bottomPressureReady(0.,NAN,2.));
 // Action-D-like breakaway exploration uses real powered elapsed time.
 ThrustSearch demand_search;demand_search.reset(c);
 for(int k=0;k<4;++k) demand_search.observe(0.,.1,c);
 assert(demand_search.target==1.);
 demand_search.observe(0.,.1,c);assert(demand_search.target==1.5);
 for(int k=0;k<5;++k) demand_search.observe(.01,.1,c);
 assert(demand_search.target==1.5); // moving -> hold, not more thrust
 for(int k=0;k<5;++k) demand_search.observe(-.01,.1,c);
 assert(demand_search.target==2.); // backwards -> exploration, not a lowered cap
 demand_search.observe(0.,10.,c);assert(demand_search.target==2. && demand_search.time==0.);
 for(int k=0;k<100;++k) demand_search.observe(0.,.1,c);
 assert(demand_search.target==6.);
 auto low_cap=c;low_cap.thrust_max=.6;demand_search.reset(low_cap);
 assert(demand_search.target==.6);
 PositionWindow window;
 for(int i=0;i<=80;++i) window.add(i*.01,.123+((i%2)?1.:-1.)*.0001);
 assert(window.stable(.8) && std::abs(window.mean()-.123)<1e-6);
 PositionWindow noisy;
 for(int i=0;i<=80;++i) noisy.add(i*.01,.123+((i%2)?1.:-1.)*.0013);
 assert(noisy.stable(.8)); // stationary 1-mm-class mocap noise is not body motion
 auto count=window.samples.size();window.add(.8,100.);assert(window.samples.size()==count);
 assert(!window.stable(1.1)); // stale data cannot complete an evaluation
 window.add(.81,.14);assert(!window.stable(.81)); // transient jump
 window.add(2.,.123);assert(!window.ready(2.)); // dropped stream
 window.reset();for(int i=0;i<=80;++i) window.add(i*.01,.01*i*.01);
 assert(!window.stable(.8) && std::abs(window.speed()-.01)<1e-9);
 window.add(.1,0.);assert(!window.ready(.1)); // backwards clock
 BottomAssist bottom;bottom.reset(0.);
 for(int i=0;i<100;++i) bottom.update(.02,0.,true,false,true,0.,true);
 assert(bottom.command==0. && !bottom.probing); // thrust search first
 for(int i=0;i<101;++i) bottom.update(.02,0.,true,true,true,0.,true);
 assert(bottom.probing && bottom.target==5. && bottom.command<.05);
 double old=bottom.command;
 for(int i=0;i<450 && bottom.probing;++i) {
   bottom.update(.02,0.,true,true,true,bottom.command,true);
   assert(bottom.command>=old && bottom.command-old<=.020001);old=bottom.command;
 }
 assert(!bottom.probing && bottom.command==5.);
 for(int i=0;i<500;++i) bottom.update(.02,.01,true,true,true,bottom.command,true);
 assert(bottom.command==5. && !bottom.probing); // sliding: hold bottom
 bottom.probing=true;bottom.settling=0.;
 for(int i=0;i<410;++i) bottom.update(.02,0.,false,true,false,0.,true);
 assert(!bottom.probing); // no indefinite thrust freeze on unready sensors
 bottom.probing=true;bottom.target=10.;bottom.command=6.;
 bottom.update(.02,.01,true,true,true,6.,true);
 assert(!bottom.probing && bottom.command==6. && bottom.target==6.);
 auto bad=c;bad.period=0;assert(!bad.valid());bad=c;bad.thrust_max=6.01;assert(!bad.valid());
 c.thrust_max=5.; // preserve the original waveform-comparison fixture below
 State s;s.fresh=true;s.now=1.;s.pressure={{25.,25.}};s.command=s.pressure;
 s.bend={{1.3,1.3}};s.start_bend=s.bend;s.thrust={{1.,1.}};
 auto eq=[](size_t,double p,double t) {return 1.3+.014*(p-25.)-.06*(t-1.);};
 auto geom=[](size_t j,double a,double,double) {
   Force f;f.forward=.7*std::sin(a);f.up=std::cos(a);
   f.roll_moment=(j==0?1.:-1.)*.08*std::cos(a);return f;
 };
 Planner planner;planner.reset(c);
 auto plan=planner.update(s,c,1.,eq,geom);
 assert(plan.feasible && plan.evaluated>20 && plan.duration>=c.run_min);
 // Opposite transient acceleration estimates must not become indefinitely
 // increasing future bend velocities while measured angles are stationary.
 Planner biased;biased.reset(c);biased.model.bend_bias={{.5,-.5}};
 auto constant_eq=[](size_t,double,double){return 1.3;};
 auto constant_geometry=[](size_t,double,double,double){Force f;f.forward=.7;return f;};
 assert(biased.update(s,c,1.,constant_eq,constant_geometry).feasible);
 // A previous 12-ms exhaustion must not suppress the next coordinate sweep.
 auto clean=planner,previously_exhausted=planner;
 previously_exhausted.last.budget_exhausted=true;
 auto later=s;later.now+=.1;
 auto clean_plan=clean.update(later,c,1.,eq,geom);
 auto resumed_plan=previously_exhausted.update(later,c,1.,eq,geom);
 assert(resumed_plan.feasible && !resumed_plan.budget_exhausted && resumed_plan.evaluated>6);
 assert(clean_plan.evaluated==resumed_plan.evaluated && clean_plan.score==resumed_plan.score);
 assert(clean.refinement_cursor==2 && previously_exhausted.refinement_cursor==2);
 // Coarse +0.8-N changes are unsafe here, but +0.1 N is feasible. The
 // predictor must find the small move without relaxing the extension guard.
 auto narrow_config=c;narrow_config.thrust_max=6.;narrow_config.extension_limit=.12;
 auto narrow_state=s;narrow_state.thrust.fill(.3);
 Planner narrow;narrow.reset(narrow_config);
 auto steep=[](size_t,double,double t){return 1.3-.4*(t-.3);};
 auto narrow_plan=narrow.update(narrow_state,narrow_config,1.,steep,constant_geometry);
 assert(narrow_plan.feasible && narrow_plan.pull_thrust[0]>.3 && narrow_plan.pull_thrust[0]<.6);
 // The increasing target changes the actual optimized waveform, not merely
 // a diagnostic counter. Keep geometry stable to isolate breakaway behavior.
 Planner stalled;stalled.reset(c);auto still=s;still.now=0.;still.elapsed=0.;still.thrust.fill(0.);
 auto fixed_eq=[](size_t,double,double){return 1.3;};
 auto fixed_geometry=[](size_t,double,double,double){Force f;f.forward=.7;return f;};
 const auto initial=stalled.update(still,c,1.,fixed_eq,fixed_geometry);
 Plan raised;
 for(int k=1;k<=10;++k) {
   still.now=k*.1;still.elapsed=k*.1;
   raised=stalled.update(still,c,1.,fixed_eq,fixed_geometry);assert(raised.feasible);
 }
 assert(stalled.thrust_search.target==2.);
 const auto before_bottom_probe=stalled.thrust_search.target;
 stalled.hold_thrust_search=true;
 for(int k=11;k<=16;++k) {
   still.now=k*.1;still.elapsed=k*.1;
   assert(stalled.update(still,c,1.,fixed_eq,fixed_geometry).feasible);
 }
 assert(stalled.thrust_search.target==before_bottom_probe);
 assert(raised.pull_thrust[0]+raised.pull_thrust[1]>initial.pull_thrust[0]+initial.pull_thrust[1]);
 auto continuous=stalled;continuous.hold_thrust_search=false;
 auto continuous_state=still;
 for(int k=17;k<=120;++k) {
   continuous_state.now=k*.1;continuous_state.elapsed=k*.1;
   auto p=continuous.update(continuous_state,c,1.,fixed_eq,fixed_geometry);
   assert(p.feasible && p.pull_thrust[0]<=continuous.thrust_search.target+1e-9);
 }
 assert(continuous.thrust_search.target==c.thrust_max); // no 3-s stop/reset
 stalled.beginRun();assert(stalled.thrust_search.target==2. && stalled.thrust_search.time==0.);
 still.now=2.;still.elapsed=0.;stalled.update(still,c,1.,fixed_eq,fixed_geometry);
 const auto before_failure=stalled.thrust_search;
 still.now+=.1;auto impossible=[](size_t,double,double){return NAN;};
 assert(!stalled.update(still,c,1.,impossible,fixed_geometry).feasible);
 assert(stalled.thrust_search.target==before_failure.target && stalled.thrust_search.time==before_failure.time);
 auto first=planner.firstThrust(s,c,plan,.02,geom);
 for(int j=0;j<2;++j) {
   assert(first[j]<=s.thrust[j]+c.thrust_rise*.02+1e-9);
   assert(first[j]>=s.thrust[j]-c.thrust_fall*.02-1e-9);
   assert(plan.pressure[j]>=0 && plan.pressure[j]<=c.pressure_max);
   assert(plan.delay[j]==0.); // horizontal endpoint has no windup phase
 }
 // Advancing time replans; pressure/pull thrust/duration are real inner
 // decisions, not just an outer choice of bottom pressure and mixing ratio.
 const auto horizontal=plan;
 planner.reset(c);s.now=2.;
 auto mixed=planner.update(s,c,.6,eq,geom);
 assert(mixed.feasible);
 assert(std::abs(mixed.score-horizontal.score)>1e-6);
 assert(mixed.pressure!=horizontal.pressure || mixed.delay!=horizontal.delay ||
        mixed.thrust!=horizontal.thrust || mixed.pull_thrust!=horizontal.pull_thrust ||
        mixed.duration!=horizontal.duration);
 // Measured response updates even without a completed locomotion cycle.
 Model response;response.reset(c);State p=s;p.now=0;p.command={{0.,0.}};p.pressure={{0.,0.}};
 response.observe(p,c,eq);
 p.now=.1;p.command={{30.,30.}};response.observe(p,c,eq);
 for(int k=2;k<=14;++k) {p.now=k*.1;p.pressure.fill(std::min(30.,(k-2)*2.2));response.observe(p,c,eq);}
 assert(response.pneumatic[0][1].samples>0);
 assert(response.pneumatic[0][0].samples==0); // simultaneous-load bucket separated
 assert(response.pneumatic[0][1].fill>15. && response.pneumatic[0][1].fill<30.);
 // Slow measured inflation changes planning; 25 kPa needs about 1.2 s,
 // rather than treating a pressure target as an instantaneous arm angle.
 Planner slow_air,fast_air;slow_air.reset(c);fast_air.reset(c);
 for(auto& arm:slow_air.model.pneumatic) for(auto& r:arm){r.fill=21.;r.delay=.15;}
 for(auto& arm:fast_air.model.pneumatic) for(auto& r:arm){r.fill=70.;r.delay=.02;}
 auto delayed=slow_air.update(s,c,.6,eq,geom),quick=fast_air.update(s,c,.6,eq,geom);
 assert(delayed.feasible && quick.feasible);
 assert(std::abs(delayed.score-quick.score)>1e-5);
 // A committed flexion of the leading arm cannot be undone by a later
 // replan while the other arm is still winding up.
 Plan committed;committed.feasible=true;committed.delay.fill(.8);
 committed.thrust.fill(.1);committed.pull_thrust.fill(2.);
 auto actuator=s;actuator.thrust.fill(1.);actuator.elapsed=.1;
 planner.pull_latched={{true,false}};
 auto split=planner.firstThrust(actuator,c,committed,.02,geom);
 assert(split[0]>1. && split[1]<1.);
 // Actual-angle upward-force limit blocks premature pull force; flexion
 // makes a larger horizontal pull possible without increasing that lift.
 auto lift_config=c;lift_config.up_force_max=3.;
 actuator.thrust.fill(3.);actuator.bend.fill(.8);actuator.start_bend=actuator.bend;
 committed.delay.fill(0.);committed.pull_thrust.fill(5.);
 auto before=planner.firstThrust(actuator,lift_config,committed,.1,geom);
 actuator.bend.fill(1.4);actuator.start_bend=actuator.bend;
 auto after=planner.firstThrust(actuator,lift_config,committed,.1,geom);
 assert(after[0]>before[0] && after[1]>before[1]);
 assert(before[0]*std::cos(.8)+before[1]*std::cos(.8)<=3.+1e-9);
 assert(planner.firstThrust(actuator,c,committed,NAN,geom)==Pair{});
 // NaN model inputs cannot poison future identification.
 auto corrupt=p;corrupt.pressure[0]=NAN;response.observe(corrupt,c,eq);
 assert(!response.initialized && std::isfinite(response.bend_error[0]));
 auto ramp=p;ramp.now=0.;ramp.pressure.fill(0.);ramp.command.fill(0.);
 Model ramp_response;ramp_response.reset(c);ramp_response.observe(ramp,c,eq);
 for(int k=1;k<=4;++k){ramp.now=k*.02;ramp.command.fill(k*1.6);ramp_response.observe(ramp,c,eq);}
 assert(ramp_response.edge_time[0]>=0.); // slew-limited targets still identify a response edge
 // No fictitious friction/contact-force observation is required by the model.
 const double estimate=response.pneumatic[0][1].fill;response.newWindow();
 assert(response.pneumatic[0][1].fill==estimate && !response.initialized);
 auto invalid=s;invalid.fresh=false;
 assert(!planner.update(invalid,c,.6,eq,geom).feasible);
 invalid=s;invalid.bend[0]=NAN;assert(!Planner::valid(invalid,c));
 invalid=s;invalid.roll=c.roll_limit;assert(!Planner::valid(invalid,c));
 invalid=s;invalid.bend[0]=s.start_bend[0]-c.extension_limit;assert(!Planner::valid(invalid,c));
 invalid=s;invalid.floor[0]=s.bend[0]+.01;assert(!Planner::valid(invalid,c));
 planner.reset(c);planner.update(s,c,.8,eq,geom);s.now-=.1;
 assert(!planner.update(s,c,.8,eq,geom).feasible); // no cached command across clock reset
 auto budget=c;budget.budget_ms=1e-12;planner.reset(budget);
 const auto demand=planner.force_request;
 auto expired=planner.update(s,budget,.8,eq,geom);
 assert(expired.budget_exhausted && !expired.feasible && planner.force_request==demand);
 // Outer selector needs actual settled measurements and includes negative
 // results. A tiny translation under a hard guard cannot become a best.
 Result r;r.context.fill(0.);r.seconds=4.;r.progress=.02;r.completed=r.safe=true;
 Search search;search.record(r);assert(search.samples.size()==1);
 auto next=search.select(r.context);assert(next.bottom==5. && next.minimum_ratio==1.);
 next=search.select(r.context);assert(next.bottom==10.);
 next=search.select(r.context);assert(next.minimum_ratio<1.);
 auto unsafe=r;unsafe.safe=false;assert(unsafe.score()<0.);
 auto no_motion=r;no_motion.progress=0.;assert(no_motion.score()<r.score());
 auto slow=r;slow.seconds=8.;assert(slow.score()<r.score());
 auto costly=r;costly.effort=80.;auto cheap=r;cheap.effort=20.;
 assert(cheap.score()>costly.score());
 auto idle=costly;idle.seconds+=60.;assert(idle.score()<costly.score());
 auto fast=costly;fast.seconds=1.;assert(cheap.score()>fast.score());
 auto tiny=cheap;tiny.progress=.0001;assert(!tiny.productive() && tiny.score()<0.);
 auto recovered=cheap;recovered.effort+=10.;recovered.recovery_seconds=10.;
 assert(recovered.score()<cheap.score());
 Search coarse;coarse.current={12.,1.};auto failed=r;failed.progress=-.003;
 for(double m:{.75,.5,.25,0.,1.}) {
   failed.candidate=coarse.current;coarse.finishProbe(failed);
   auto proposal=coarse.select(failed.context);
   assert(proposal.minimum_ratio==m && coarse.ratio_probe_active);
   assert(proposal.bottom==(m==1.?17.:12.));
 }
 cheap.candidate={5.,1.};costly.candidate={10.,.25};
 coarse.record(cheap);coarse.record(costly);coarse.finishProbe(failed);
 next=coarse.select(failed.context);assert(next.minimum_ratio==1. && next.bottom==5.);
 // Continue a productive condition, rather than exhaustive optimization.
 coarse.finishProbe(cheap);next=coarse.select(cheap.context);assert(next.bottom==5. && next.minimum_ratio==1.);
 auto recent_failure=costly;recent_failure.progress=0.;coarse.record(recent_failure);
 coarse.finishProbe(failed);next=coarse.select(failed.context);
 assert(next.bottom==5. && next.minimum_ratio==.75); // no resurrection of failed old best
 Search no_measurement;failed.completed=false;no_measurement.finishProbe(failed);
 assert(!no_measurement.alternative_pending);failed.completed=true;
 Search disabled;disabled.enabled=false;disabled.finishProbe(failed);assert(!disabled.alternative_pending);
 ProbeBudget probe;bool cut=false;
 for(int i=0;i<301;++i) cut=probe.update(.02,0.,32.,true);
 assert(cut && probe.effort>=24.); // expensive no-motion probe is bounded
 probe.reset();for(int i=0;i<1000;++i) assert(!probe.update(.02,.001*i*.02,32.,true));
 probe.reset();for(int i=0;i<301;++i) assert(!probe.update(.02,0.,1.,true));
 assert(!probe.update(.02,0.,32.,false) && !probe.initialized);
 probe.update(.02,0.,32.,true);assert(!probe.update(-.1,0.,32.,true) && !probe.initialized);
 auto unmeasured=r;unmeasured.seconds=NAN;search.record(unmeasured);assert(search.samples.size()==1);
 Context changed=r.context;changed[0]=3.;next=search.select(changed);
 assert(next.bottom==5. && next.minimum_ratio==1.);
 search.enabled=false;const auto fixed=search.current;next=search.select(r.context);
 assert(next.bottom==fixed.bottom && next.minimum_ratio==fixed.minimum_ratio);
 Search bottom_search;bottom_search.current={5.,.9};bottom_search.requestBottomProbe();
 next=bottom_search.select(r.context);
 assert(next.bottom==10. && next.minimum_ratio==.9 && bottom_search.bottom_probe_active);
 next=bottom_search.select(r.context);assert(next.bottom==10. && !bottom_search.bottom_probe_active);
 bottom_search.current.bottom=30.;bottom_search.requestBottomProbe();next=bottom_search.select(r.context);
 assert(next.bottom==25. && bottom_search.bottom_probe_active);
 bottom_search.current.bottom=0.;bottom_search.requestBottomProbe();next=bottom_search.select(r.context);
 assert(next.bottom==5.);
 bottom_search.enabled=false;bottom_search.requestBottomProbe();next=bottom_search.select(r.context);
 assert(next.bottom==5. && !bottom_search.bottom_probe_active);
 std::cout<<"PASS: hybrid inner search "<<horizontal.evaluated<<" candidates; early/pull force, pressure, timing and duration, scenario guards, pressure identification, context outer search\n";
 std::cout<<"horizontal: T="<<horizontal.thrust[0]<<","<<horizontal.pull_thrust[0]<<" P="<<horizontal.pressure[0]<<" duration="<<horizontal.duration<<" score="<<horizontal.score<<" compute="<<horizontal.compute_ms<<" ms\n";
 std::cout<<"mixed: T="<<mixed.thrust[0]<<","<<mixed.pull_thrust[0]<<" P="<<mixed.pressure[0]<<" delay="<<mixed.delay[0]<<" duration="<<mixed.duration<<" score="<<mixed.score<<"\n";
}
'''

def main():
    with tempfile.TemporaryDirectory(prefix="hugmy_hybrid_test_") as directory:
        binary = Path(directory) / "test"
        subprocess.run(["g++", "-std=c++14", "-O2", "-Wall", "-Wextra", "-x", "c++", "-",
                        "-I", str(ROOT / "include"), "-o", str(binary)], input=CODE, text=True, check=True)
        subprocess.run([str(binary)], check=True)
    source = (ROOT / "src/control/hybrid_locomotion.cpp").read_text()
    controller = (ROOT / "src/control/attitude_pressure_controller.cpp").read_text()
    assert "hybrid_single_.update" in source
    assert "hybrid_planner_.update" not in source and "hybrid_search_.select" not in source
    assert "publishPressureTarget(inchworm_pressure_" in source
    assert "if (!plan.feasible)" not in source or "fallback" in source
    assert "action_c_optimizer_trial_active_" not in controller
    assert "finishHybridRecovery(now)" in controller
    assert "finishHybridLocomotion(now,hybrid_phase_==2 && !hybrid_terminal_settle_,true)" in source
    assert 'hybrid_locomotion_shadow_' not in source
    assert 'return inchworm_action_c_enabled_ && !inchworm_action_d_enabled_;' in source
    assert 's.elapsed>=plan.duration' not in source and 's.elapsed>=hybrid_config_.run_max' not in source
    assert 'hybrid_search_.requestBottomProbe()' not in source
    assert 'hybrid_end_mean_x_=hybrid_settled_window_.mean()' in source
    print("PASS: single-pass C is the sole execution path; legacy planner cannot command actuators; recovery accounting remains connected")

if __name__ == "__main__":
    main()
