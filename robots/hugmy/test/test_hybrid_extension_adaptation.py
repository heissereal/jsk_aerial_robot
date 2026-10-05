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
 auto flat=[](size_t,double,double,double){Force f;f.forward=1.;return f;};
 SinglePassInput s;s.s.fresh=true;s.s.bend={{1.6,1.6}};s.s.start_bend=s.s.bend;
 s.s.floor={{.7,.7}};s.s.command={{30.,30.}};s.s.pressure=s.s.command;
 s.s.thrust={{3.,3.}};s.rear_released=true;
 SinglePass p;p.slip_tested=true;p.begin(s);p.enter(SinglePass::Lift,s);
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
 assert(p.onset[0].thrust==3. && p.onset[0].pressure==30.);
 for(int i=0;i<10;++i)p.update(s,c,k,model,flat);
 assert(p.onset[0].samples==1); // not learned every control tick
 // A retained rate sample without subsequent angle movement cannot confirm.
 SinglePass stuck;auto stale=s;stale.s.bend={{1.6,1.6}};stuck.slip_tested=true;stuck.begin(stale);stuck.enter(SinglePass::Lift,stale);
 stale.s.bend={{1.58,1.58}};
 for(int i=0;i<8;++i)stuck.update(stale,c,k,model,flat);
 assert(stuck.onset[0].samples==0);
 // Breakaway may happen in the sliding ramp, before the Lift reference reset.
 SinglePass sliding;auto slip=s;slip.s.bend={{1.6,1.6}};sliding.begin(slip);
 slip.s.bend={{1.58,1.58}};
 for(int i=0;i<6;++i){for(double& a:slip.s.bend)a-=.04*slip.dt;sliding.update(slip,c,k,model,flat);}
 assert(sliding.onset[0].samples==1);
 sliding.enter(SinglePass::Lift,slip);
 assert(sliding.onset_this_stroke[0] && sliding.onset[0].samples==1);
 // New stroke reuses memory with the Action-C rise, never past the learned
 // onset margin. Rear release is deliberately not a prerequisite: motion 2
 // now keeps the rear pair grasped until the predicted pre-vent.
 s.s.bend={{1.6,1.6}};s.s.rate={{0.,0.}};s.s.thrust={{0.,0.}};
 p.lift_tested=true;p.calibrated=true;p.observed_extension=.1;
 s.rear_released=false;p.begin(s);p.enter(SinglePass::Lift,s);
 auto fast=p.update(s,c,k,model,flat);
 assert(p.fast_approach[0] && fast.thrust[0]>c.thrust_rise*s.dt);
 assert(fast.thrust[0]<=k.reused_lift_rise*s.dt+1e-9);
 for(int i=0;i<55;++i){s.s.thrust=fast.thrust;fast=p.update(s,c,k,model,flat);}
 // The onset is an approach hint, not a ceiling: if contact changed and no
 // extension occurs, the reused stroke continues its bounded fast search.
 assert(fast.thrust[0]>p.onset[0].thrust-k.onset_margin_n);
 assert(fast.thrust[0]<=c.thrust_max+1e-9);
 // A reused stroke keeps the fast approach but brakes before the 8-deg
 // target.  Its demand must remain above the measured sub-breakaway hold;
 // braking to zero makes one arm rebound while rear pressure is exhausting.
 SinglePass shaped;auto near=s;near.s.bend={{1.6,1.6}};
 near.s.start_bend=near.s.bend;near.s.floor={{.7,.7}};
 near.s.pressure={{0.,0.}};near.s.command={{0.,0.}};
 near.s.thrust={{4.,4.}};near.s.rate={{0.,0.}};near.rear_pressure=15.;
 shaped.slip_tested=true;shaped.calibrated=true;shaped.observed_extension=k.extension_rad;
 shaped.onset[0].observe(3.,0.,1.6);shaped.onset[1].observe(3.,0.,1.6);
 shaped.begin(near);shaped.enter(SinglePass::Lift,near);
 near.s.bend={{1.6-k.extension_rad+M_PI/180.,
               1.6-k.extension_rad+M_PI/180.}};
 auto shaped_o=shaped.update(near,c,k,model,flat);
 const double measured_hold=3.-k.onset_margin_n;
 assert(shaped_o.thrust[0]<near.s.thrust[0]);
 assert(shaped_o.thrust[0]>=measured_hold-1e-9);
 // Early motion overrides shortcut even before the confirmation finishes.
 s.s.thrust=fast.thrust;s.s.bend={{1.58,1.58}};s.s.rate={{-.3,-.3}};
 auto brake=p.update(s,c,k,model,flat);
 assert(!p.fast_approach[0] && brake.thrust[0]<s.s.thrust[0]);
 // Different pressure/geometry or roll motion => no fast approach.
 for(int mode=0;mode<3;++mode){
   auto bad=s;bad.s.bend={{1.6,1.6}};bad.s.rate={{0.,0.}};bad.s.thrust={{0.,0.}};
   if(mode==0)bad.s.pressure={{20.,20.}};
   if(mode==1)bad.s.bend={{1.9,1.9}};
   if(mode==2)bad.s.roll_rate=.15;
   auto q=p;q.begin(bad);q.enter(SinglePass::Lift,bad);q.update(bad,c,k,model,flat);assert(!q.fast_approach[0]);
 }
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
 SinglePass protected_p;protected_p.slip_tested=true;protected_p.onset[0].observe(4.,30.,1.6);
 protected_p.calibrated=true;protected_p.observed_extension=.1;
 s.s.bend={{1.6,1.6}};s.s.floor={{1.59,1.59}};protected_p.begin(s);protected_p.enter(SinglePass::Lift,s);
 auto protected_o=protected_p.update(s,c,k,model,flat);assert(protected_o.thrust[0]<=s.s.thrust[0]);
 std::cout<<"PASS: per-arm onset confirmation/reuse/reset, context-gated fast approach, rate brake, full fill timing, no angle equality cost\n";
}
'''
with tempfile.TemporaryDirectory(prefix='hybrid_extension_') as folder:
    binary=str(Path(folder)/'test')
    subprocess.run(['g++','-std=c++14','-O2','-I'+str(root/'include'),'-x','c++','-','-o',binary],input=code,text=True,check=True)
    subprocess.run([binary],check=True)
