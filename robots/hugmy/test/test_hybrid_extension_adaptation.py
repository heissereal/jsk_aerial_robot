#!/usr/bin/env python3
"""Production-header tests: breakaway memory, fast approach, rate FB, no pair matching."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
code=r'''
#include <hugmy/control/hybrid_single_pass.h>
#include <cassert>
#include <iostream>
using namespace hugmy::hybrid;
int main() {
 Config c; SinglePassConfig k; Model model; model.reset(c);
 auto flat=[](size_t,double a,double,double){Force f;f.forward=1.;f.up=std::max(0.,std::cos(a));return f;};
 SinglePassInput s;s.s.fresh=true;s.s.bend={{1.6,1.6}};s.s.start_bend=s.s.bend;
 s.s.floor={{.7,.7}};s.s.command={{30.,30.}};s.s.pressure=s.s.command;
 s.s.thrust={{3.,3.}};s.rear_released=true;
 SinglePass p;p.slip_tested=true;p.begin(s);p.enter(SinglePass::Lift,s);
 s.s.pressure={{0.,0.}};s.s.command=s.s.pressure;
 // Initial search has no learned shortcut, nor false onset from angle alone.
 auto initial=p.update(s,c,k,model,flat);
 assert(!p.fast_approach[0] && initial.thrust[0]<=3.+k.lift_rise*s.dt+1e-9);
 s.s.bend={{1.58,1.58}};
 for(int i=0;i<6;++i)p.update(s,c,k,model,flat);
 assert(p.onset[0].samples==0);
 s.s.rate={{-.04,-.04}};
 for(int i=0;i<4;++i){for(double& a:s.s.bend)a-=.04*s.dt;p.update(s,c,k,model,flat);}
 assert(p.onset[0].samples==0);
 for(double& a:s.s.bend)a-=.04*s.dt;
 p.update(s,c,k,model,flat);
 assert(p.onset[0].samples==1 && p.onset[1].samples==1);
 assert(p.onset[0].thrust==3. && p.onset[0].pressure==0.);
 assert(p.onset_identified && !p.calibrated);
 for(int i=0;i<10;++i)p.update(s,c,k,model,flat);
 assert(p.onset[0].samples==1); // not learned every control tick
 // A retained rate sample without subsequent angle movement cannot confirm.
 SinglePass stuck;auto stale=s;stale.s.bend={{1.6,1.6}};stuck.slip_tested=true;stuck.begin(stale);stuck.enter(SinglePass::Lift,stale);
 stale.s.bend={{1.58,1.58}};
 for(int i=0;i<8;++i)stuck.update(stale,c,k,model,flat);
 assert(stuck.onset[0].samples==0);
 // Once both first-pass onsets are identified, a single rate spike above
 // 45 deg must not remove thrust. Sustained fast extension pauses the gentle
 // search, but still does not command a thrust decrease.
 SinglePass capture;auto capture_in=s;capture_in.s.bend={{1.6,1.6}};
 capture_in.s.thrust={{2.5,2.5}};capture_in.s.rate={{0.,0.}};
 capture.slip_tested=true;capture.begin(capture_in);
 capture.enter(SinglePass::Lift,capture_in);
 capture.onset[0].observe(.6,0.,1.6);capture.onset[1].observe(.8,0.,1.6);
 capture.onset_identified=true;capture.lift_onset_seen={{true,true}};
 capture.identification_tracking={{true,true}};
 capture.identification_hold_thrust={{1.4,1.5}};
 capture.lift_ramp={{2.5,2.5}};
 capture_in.s.bend={{1.40,1.39}};capture_in.s.rate={{-1.,-1.}};
 capture_in.height=.01;
 auto capture_out=capture.update(capture_in,c,k,model,flat);
 assert(std::abs(capture_out.thrust[0]-capture_in.s.thrust[0])<1e-9 &&
     std::abs(capture_out.thrust[1]-capture_in.s.thrust[1])<1e-9);
 for(int i=0;i<20;++i) {
   capture_in.s.thrust=capture_out.thrust;
   capture_out=capture.update(capture_in,c,k,model,flat);
 }
 assert(capture.identification_fast_rate_time[0]>=k.identification_rate_confirm);
 const double fast_hold=capture_out.thrust[0];
 capture_in.s.thrust=capture_out.thrust;
 capture_out=capture.update(capture_in,c,k,model,flat);
 assert(std::abs(capture_out.thrust[0]-fast_hold)<1e-9);
 assert(!capture.lift_target_reached[0] && !capture.lift_target_reached[1]);
 // Slow/stalled arms must keep increasing beyond breakaway; onset is a
 // feed-forward starting point, never a completion force or force ceiling.
 SinglePass advance;auto advance_in=s;advance_in.s.bend={{1.6,1.6}};
 advance_in.s.thrust={{1.4,1.5}};advance_in.s.rate={{0.,0.}};
 advance_in.s.pressure={{0.,0.}};advance_in.s.command={{0.,0.}};
 advance.slip_tested=true;advance.begin(advance_in);advance.enter(SinglePass::Lift,advance_in);
 advance.onset[0].observe(.8,0.,1.6);advance.onset[1].observe(1.0,0.,1.6);
 advance.onset_identified=true;advance.lift_onset_seen={{true,true}};
 advance.identification_tracking={{true,true}};
 advance.identification_hold_thrust={{1.4,1.5}};advance.lift_ramp=advance_in.s.thrust;
 advance_in.s.bend={{1.35,1.35}};
 auto advance_out=advance.update(advance_in,c,k,model,flat);
 // The common gentle search first brings the lower-demand side up to the
 // existing pair demand; it must increase total authority without lowering
 // the arm that already carries the larger breakaway force.
 assert(advance_out.thrust[0]>advance_in.s.thrust[0] &&
     advance_out.thrust[1]>=advance_in.s.thrust[1] &&
     advance_out.thrust[0]+advance_out.thrust[1]>
         advance_in.s.thrust[0]+advance_in.s.thrust[1]);
 // With no motion, the common pulse increases in discrete bounded steps
 // instead of remaining permanently at the initial 2-N level.
 for(int i=0;i<400;++i) {
   advance_in.s.thrust=advance_out.thrust;
   advance_out=advance.update(advance_in,c,k,model,flat);
 }
 assert(advance_out.thrust[0]>k.initial_pulse_max-.2 &&
     advance_out.thrust[1]>k.initial_pulse_max-.2 &&
     advance_out.thrust[0]<=k.initial_pulse_max+1e-9 &&
     advance_out.thrust[1]<=k.initial_pulse_max+1e-9);
 // A lower-target override still supports the generic below-45-deg path: even
 // a large sub-safety-limit rate sample is ignored until the measured terminal
 // zone. Production uses the safer 60-deg unloading target.
 auto low_target_k=k;low_target_k.identification_bend=20.*M_PI/180.;
 low_target_k.identification_extension_limit=80.*M_PI/180.;
 SinglePass low_angle=advance;auto low_input=advance_in;
 low_input.s.bend={{40.*M_PI/180.,40.*M_PI/180.}};
 low_input.s.thrust={{2.,2.}};low_input.s.rate={{-1.,-1.}};
 low_angle.lift_ramp=low_input.s.thrust;
 auto low_out=low_angle.update(low_input,c,low_target_k,model,flat);
 assert(low_out.thrust[0]>low_input.s.thrust[0] &&
     low_angle.identification_fast_rate_time[0]==0.);
 // An artificial low absolute-angle target cannot replace the measured
 // unloading event or chop the simultaneous pulse.
 SinglePass terminal=advance;auto terminal_input=advance_in;
 terminal_input.s.bend={{23.*M_PI/180.,23.*M_PI/180.}};
 terminal_input.s.thrust={{2.,2.}};terminal_input.s.rate={{0.,0.}};
 terminal.lift_ramp=terminal_input.s.thrust;
 auto terminal_out=terminal.update(terminal_input,c,low_target_k,model,flat);
 assert(terminal_out.thrust[0]>=terminal_input.s.thrust[0] &&
     terminal_out.thrust[0]<=terminal_input.s.thrust[0]+
         low_target_k.initial_pulse_rise*terminal_input.dt+1e-9 &&
     terminal_out.pressure[0]==0. && terminal_out.pressure[1]==0.);
 // A realistic long first reach has more than ten seconds available and can
 // establish both absolute targets before handing directly to motion 3.
 SinglePass reaches;auto reach_in=s;reach_in.s.bend={{1.90,1.90}};
 reach_in.s.thrust={{1.2,1.2}};reach_in.s.rate={{0.,0.}};
 reach_in.s.pressure={{0.,0.}};reach_in.s.command={{0.,0.}};
 reaches.slip_tested=true;reaches.begin(reach_in);reaches.enter(SinglePass::Lift,reach_in);
 reaches.onset[0].observe(.8,0.,1.9);reaches.onset[1].observe(.8,0.,1.9);
 reaches.onset_identified=true;reaches.lift_onset_seen={{true,true}};
 reaches.identification_tracking={{true,true}};
 reaches.identification_hold_thrust={{1.2,1.2}};reaches.lift_ramp=reach_in.s.thrust;
 for(int i=0;i<740 && reaches.phase==SinglePass::Lift;++i) {
   auto o=reaches.update(reach_in,c,k,model,flat);
   reach_in.s.thrust=o.thrust;reach_in.s.command=o.pressure;
   const double physical_rate=std::min(k.extension_rate,
       std::max(0.,reach_in.s.thrust[0]-.8)*.5);
   reach_in.s.rate={{-physical_rate,-physical_rate}};
   for(double& a:reach_in.s.bend)a+=reach_in.s.rate[0]*reach_in.dt;
   // Motion 2 now ends on measured body unloading, not on a fixed bend.
   // Emulate the small mocap rise produced once both released arms have made
   // the minimum paired extension. The signed forward-pitch corroboration is
   // part of the same measured load-relief event.
   if(1.90-reach_in.s.bend[0]>=k.unload_min_extension) {
     reach_in.height=.003;
     reach_in.directional_pitch=k.unload_min_pitch+.01;
   }
 }
 assert(reaches.phase==SinglePass::Pull && reaches.calibrated);
 assert(std::isfinite(reaches.identified_bend_target[0]));
 // A pressure-assisted horizontal slip is a different load case and must not
 // identify the reusable zero-pressure Lift threshold.
 SinglePass sliding;auto slip=s;slip.s.bend={{1.6,1.6}};sliding.begin(slip);
 slip.s.bend={{1.58,1.58}};
 for(int i=0;i<6;++i){for(double& a:slip.s.bend)a-=.04*slip.dt;sliding.update(slip,c,k,model,flat);}
 assert(sliding.onset[0].samples==0 && !sliding.onset_identified);
 sliding.enter(SinglePass::Lift,slip);
 assert(!sliding.onset_this_stroke[0] && sliding.onset[0].samples==0);
 // An unfinished calibration restarts the simultaneous pulse. A stale onset
 // hint must not bring back the slow one-arm approach.
 s.s.bend={{1.6,1.6}};s.s.rate={{0.,0.}};s.s.thrust={{0.,0.}};
 p.lift_tested=true;p.calibrated=false;p.observed_extension=.1;
 s.rear_released=false;p.begin(s);p.enter(SinglePass::Lift,s);
 auto fast=p.update(s,c,k,model,flat);
 assert(p.fast_approach[0] && p.fast_approach[1]);
 assert(fast.thrust[0]<=k.initial_pulse_rise*s.dt+1e-9);
 for(int i=0;i<90;++i){s.s.thrust=fast.thrust;fast=p.update(s,c,k,model,flat);}
 // If no extension occurs, discrete pulse-amplitude steps continue the
 // bounded search instead of stopping below an old breakaway estimate.
 assert(fast.thrust[0]>1. && fast.thrust[0]<=k.initial_pulse_max+1e-9);
 assert(fast.thrust[0]<=c.thrust_max+1e-9);
 // The old 8-deg event is now a pneumatic timing hint, not a completion or
 // thrust-braking target.  Without fresh body unloading, a reused stroke must
 // keep the bounded search alive instead of dropping thrust at that angle.
 SinglePass shaped;auto near=s;near.s.bend={{1.6,1.6}};
 near.s.start_bend=near.s.bend;near.s.floor={{.7,.7}};
 near.s.pressure={{0.,0.}};near.s.command={{0.,0.}};
 near.s.thrust={{4.,4.}};near.s.rate={{0.,0.}};near.rear_pressure=15.;
 shaped.slip_tested=true;shaped.calibrated=true;shaped.observed_extension=k.extension_rad;
 shaped.onset[0].observe(3.,0.,1.6);shaped.onset[1].observe(3.,0.,1.6);
 shaped.onset_identified=true;
 shaped.begin(near);shaped.enter(SinglePass::Lift,near);
 near.s.bend={{1.6-k.extension_rad+M_PI/180.,
               1.6-k.extension_rad+M_PI/180.}};
 auto shaped_o=shaped.update(near,c,k,model,flat);
 assert(shaped_o.thrust[0]>=near.s.thrust[0] &&
        shaped_o.thrust[0]<=near.s.thrust[0]+k.reused_lift_rise*near.dt+1e-9);
 // Early motion overrides the fast shortcut, but a single rate sample no
 // longer removes thrust from the first identification pass.
 s.s.thrust=fast.thrust;s.s.bend={{1.58,1.58}};s.s.rate={{-.3,-.3}};
 auto brake=p.update(s,c,k,model,flat);
 assert(!p.fast_approach[0] && brake.thrust[0]>=s.s.thrust[0]);
 // Different pressure/geometry => no fast approach. A single angular-rate
 // sample is intentionally not a permanent calibration/recovery decision.
 for(int mode=0;mode<2;++mode){
   auto bad=s;bad.s.bend={{1.6,1.6}};bad.s.rate={{0.,0.}};bad.s.thrust={{0.,0.}};
   if(mode==0)bad.s.pressure={{20.,20.}};
   if(mode==1)bad.s.bend={{1.9,1.9}};
   auto q=p;q.begin(bad);q.enter(SinglePass::Lift,bad);q.update(bad,c,k,model,flat);assert(!q.fast_approach[0]);
 }
 // One identified arm cannot trigger a one-sided shortcut on a later Lift.
 SinglePass partial;partial.slip_tested=true;
 partial.onset[0].observe(3.,0.,1.6);partial.lift_onset_seen[0]=true;
 auto partial_in=s;partial_in.s.bend={{1.6,1.6}};
 partial_in.s.thrust={{0.,0.}};partial.begin(partial_in);
 partial.enter(SinglePass::Lift,partial_in);
 partial.update(partial_in,c,k,model,flat);
 assert(!partial.onset_identified && !partial.fast_approach[0] &&
     !partial.fast_approach[1]);
 // Continuing observations are not capped at three rounds.
 ExtensionOnset local;for(int i=0;i<12;++i)local.observe(3.+.1*i,30.,1.6);
 assert(local.samples==12 && local.thrust>3. && local.thrust<4.1);
 p.reset();assert(!p.onset[0].samples); // explicit new task discards old context
 // A weak first lift is a hint, not a permanent ceiling in changed contact.
 SinglePass retry;auto immobile=s;immobile.s.bend={{1.6,1.6}};
 immobile.s.thrust={{0.,0.}};immobile.s.rate={{0.,0.}};
 retry.slip_tested=true;retry.calibrated=true;retry.observed_extension=.1;
 retry.learned_lift_thrust=.5;retry.begin(immobile);retry.enter(SinglePass::Lift,immobile);
 for(int i=0;i<250;++i){auto o=retry.update(immobile,c,k,model,flat);
   assert(!o.fault && o.thrust[0]<=c.thrust_max);
   immobile.s.thrust=o.thrust;immobile.s.command=o.pressure;
 }
 assert(immobile.s.thrust[0]>3. && retry.phase==SinglePass::Lift);
 // Predictive pressure lead must not turn an unachieved calibration into Pull.
 s.s.bend={{1.6,1.6}};s.s.rate={{0.,0.}};s.s.thrust={{2.,2.}};
 SinglePass lead;lead.slip_tested=true;lead.begin(s);lead.enter(SinglePass::Lift,s);
 s.s.bend={{1.49,1.49}};s.s.rate={{-.15,-.15}};
 lead.update(s,c,k,model,flat);assert(lead.phase==SinglePass::Lift);
 // Stable roll + unequal relative extension does not reduce one arm solely
 // to equalize angles; this slip fixture has identical force geometry.
 SinglePass unequal;s.s.bend={{1.6,1.6}};s.s.rate={{0.,0.}};s.s.thrust={{1.,1.}};
 unequal.begin(s);s.s.bend={{1.50,1.60}};
 auto u=unequal.update(s,c,k,model,flat);assert(u.thrust[0]==u.thrust[1]);
 // Fast approach is still subject to individual floor/extension protection.
 SinglePass protected_p;protected_p.slip_tested=true;
 protected_p.onset[0].observe(4.,0.,1.6);protected_p.onset[1].observe(4.,0.,1.6);
 protected_p.onset_identified=true;
 protected_p.calibrated=true;protected_p.observed_extension=.1;
 s.s.bend={{1.6,1.6}};s.s.floor={{1.59,1.59}};protected_p.begin(s);protected_p.enter(SinglePass::Lift,s);
 auto protected_o=protected_p.update(s,c,k,model,flat);assert(protected_o.thrust[0]<=s.s.thrust[0]);
 std::cout<<"PASS: per-arm onset confirmation/reuse/reset, context-gated fast approach, rate hold, full fill timing, no angle equality cost\n";
}
'''
with tempfile.TemporaryDirectory(prefix='hybrid_extension_') as folder:
    binary=str(Path(folder)/'test')
    subprocess.run(['g++','-std=c++14','-O2','-I'+str(root/'include'),'-x','c++','-','-o',binary],input=code,text=True,check=True)
    subprocess.run([binary],check=True)
