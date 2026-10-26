#!/usr/bin/env python3
"""Online choices change only from measured, settled results in matched context."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
code=r'''
#include <hugmy/control/hybrid_trajectory_adaptation.h>
#include <cassert>
#include <iostream>
using namespace hugmy::hybrid;
int main() {
 TrajectoryAdaptation a;Pair bend{{1.4,1.45}};
 a.record(bend,0.,0.,.1,10.,1.,0.,false,true,true);
 assert(a.samples.empty()); // no airborne/noisy endpoint credit
 // One settled forward baseline starts a bounded local probe, but it is not
 // yet a distance model. Two repeatable baselines establish that model.
 a.record(bend,0.,0.,.0095,10.,20.,0.,true,true,true);
 assert(!std::isfinite(a.estimatedProgress(bend,0.,0.)));
 assert(std::abs(a.provisionalProgressForReduction(bend,0.,0.)-.0095)<1e-9);
 a.select(bend,0.,0.);assert(a.current==1);
 a.current=0; // collect the second unchanged result for distance estimation
 a.record(bend,0.,0.,.0105,10.,20.,0.,true,true,true);
 assert(std::abs(a.estimatedProgress(bend,0.,0.)-.010)<1e-9);
 a.select(bend,0.,0.);assert(a.current==1);
 // An exploratory candidate is repeated once before advancing.
 a.record(bend,0.,0.,.011,10.,20.,0.,true,true,true);
 a.select(bend,0.,0.);assert(a.current==1);
 a.record(bend,0.,0.,.0112,10.,20.,0.,true,true,true);
 a.select(bend,0.,0.);assert(a.current==2);
 // Scaled results update the baseline response after normalization and must
 // also unlock the outer search: long-distance goal conditioning must not
 // leave candidate zero selected forever.
 TrajectoryAdaptation scaled;
 scaled.record(bend,0.,0.,.0096,10.,20.,0.,true,true,true,1.2);
 scaled.record(bend,0.,0.,.0104,10.,20.,0.,true,true,true,1.3);
 assert(std::abs(scaled.estimatedProgress(bend,0.,0.)-.008)<1e-9);
 scaled.select(bend,0.,0.);assert(scaled.current==1);
 // Merely positive is not enough: a strongly scattered response must not be
 // used to decide the number of remaining strokes or to strengthen a command.
 TrajectoryAdaptation scattered;
 scattered.record(bend,0.,0.,.0020,10.,20.,0.,true,true,true);
 scattered.record(bend,0.,0.,.0050,10.,20.,0.,true,true,true);
 assert(!std::isfinite(scattered.estimatedProgress(bend,0.,0.)));
 // The provisional estimate deliberately uses only the latest comparable
 // safe baseline and is consumed by SinglePass only when it reduces a final
 // stroke; it can never strengthen from this scattered pair.
 assert(std::abs(scattered.provisionalProgressForReduction(bend,0.,0.)-.005)<1e-9);
 // A safely completed but losing baseline must not be repeated blindly at a
 // changed contact. It may start one bounded local probe immediately; this
 // does not relax the two-sample rule used for distance amplification.
 TrajectoryAdaptation losing;
 losing.record(bend,0.,0.,-.004,10.,20.,0.,true,true,true);
 losing.select(bend,0.,0.);assert(losing.current==1);
 // A saturated rear-support arm is physically identical to the baseline and
 // is skipped without fabricating a failed or successful observation.
 TrajectoryAdaptation unavailable;
 unavailable.record(bend,0.,0.,.004,10.,20.,0.,true,true,true);
 unavailable.markUnavailable(1);
 unavailable.select(bend,0.,0.);
 assert(unavailable.current==2 && unavailable.samples.size()==1);
 // Distance tiling: exact long goals repeat; a weak last stroke is spread;
 // an almost-full remainder becomes one slightly weaker extra stroke.
 assert(std::abs(TrajectoryAdaptation::planStride(.050,.010,.010,.25,1.5)-.010)<1e-9);
 assert(std::abs(TrajectoryAdaptation::planStride(.020,.008,.010,.25,1.5)-.010)<1e-9);
 assert(std::abs(TrajectoryAdaptation::planStride(.039,.010,.010,.25,1.5)-.00975)<1e-9);
 a.record(bend,0.,0.,.1,10.,20.,0.,true,false,false);
 assert(a.samples.back().progress==0. && a.samples.back().score<0.);
 a.record(bend,0.,0.,.011,10.,20.,0.,true,false,true);
 assert(a.samples.back().progress==.011 && a.samples.back().score>0.);
 a.select({{.5,.5}},.5,.5);assert(a.current==0); // different support context
 assert(std::abs(a.choice(1).force_scale-1.10)<1e-9);
 assert(std::abs(a.choice(2).rear_support_offset+.50)<1e-9);
 assert(std::abs(a.choice(3).rear_support_offset-1.0)<1e-9);
 assert(std::abs(a.choice(4).unload_scale-1.05)<1e-9);
 assert(std::abs(a.choice(5).mix_offset+.10)<1e-9 &&
        std::abs(a.choice(5).force_scale-1.10)<1e-9);
 for(unsigned i=0;i<TrajectoryAdaptation::choice_count;++i){
   auto c=a.choice(i);
   assert(c.force_scale>=.90 && c.force_scale<=1.40 &&
          c.mix_offset>=-.30 && c.mix_offset<=0. &&
          c.unload_scale>=1. && c.unload_scale<=1.35 &&
          c.rear_support_offset>=-.50 && c.rear_support_offset<=1.0);
 }
 // Once every bounded candidate has measured evidence, a backward candidate
 // can never win by saving energy or time.  The deployed choice must have at
 // least one retained-forward observation and no unsafe/reverse observation.
 TrajectoryAdaptation no_reverse_winner;
 for(unsigned i=0;i<TrajectoryAdaptation::choice_count;++i) {
   no_reverse_winner.current=i;
   const double dx=i==4?-.004:.004+.0001*i;
   no_reverse_winner.record(bend,0.,0.,dx,10.,20.,0.,true,true,true);
 }
 no_reverse_winner.baseline_revalidation_remaining=0;
 no_reverse_winner.current=0;
 no_reverse_winner.select(bend,0.,0.);
 assert(no_reverse_winner.current==5);
 assert(no_reverse_winner.samples[4].score<0.);
 // A failed exploratory stroke forces one safely completed baseline
 // revalidation before the next untried local candidate may run. The
 // baseline need not advance: on high friction that is the reason to search.
 TrajectoryAdaptation guarded;
 for(int i=0;i<2;++i)guarded.record(bend,0.,0.,.015,10.,20.,0.,true,true,true);
 guarded.select(bend,0.,0.);assert(guarded.current==1);
 guarded.record(bend,0.,0.,0.,10.,20.,0.,true,false,false);
 guarded.select(bend,0.,0.);assert(guarded.current==0);
 guarded.record(bend,0.,0.,-.002,10.,20.,0.,true,true,true);
 guarded.select(bend,0.,0.);assert(guarded.current==2);
 // Safe retained travel from an early guard is valid evidence: repeated
 // forward guarded trials must not leave the optimizer stuck at baseline.
 TrajectoryAdaptation guarded_progress;
 for(int i=0;i<2;++i)
   guarded_progress.record(bend,0.,0.,.011,10.,20.,0.,true,false,true);
 guarded_progress.select(bend,0.,0.);assert(guarded_progress.current==1);
 guarded_progress.record(bend,0.,0.,.010,10.,20.,0.,true,false,true);
 // A safe retained endpoint does not need a baseline interleave merely
 // because a guard shortened the waveform. Repeat it once for reproducibility.
 guarded_progress.select(bend,0.,0.);assert(guarded_progress.current==1);
 guarded_progress.record(bend,0.,0.,.009,10.,20.,0.,true,true,true);
 guarded_progress.select(bend,0.,0.);assert(guarded_progress.current==2);
 a.enabled=false;a.select(bend,0.,0.);assert(a.current==0);
 auto before=a.samples.size();
 a.record(bend,0.,0.,.021,10.,20.,0.,true,true,true);
 assert(a.samples.size()==before+1); // measurement supports goal scaling
 std::cout<<"PASS: bounded measured-result search, retained safe incomplete progress, context reset and no unsafe/idle reward\n";
}
'''
with tempfile.TemporaryDirectory(prefix='hybrid_trajectory_') as folder:
    binary=str(Path(folder)/'test')
    subprocess.run(['g++','-std=c++14','-O2','-I'+str(root/'include'),'-x','c++','-','-o',binary],input=code,text=True,check=True)
    subprocess.run([binary],check=True)
