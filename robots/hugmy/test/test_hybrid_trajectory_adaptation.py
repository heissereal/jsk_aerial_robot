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
 // One lucky stroke is not a model. Two repeatable measurements establish a
 // 10-mm baseline and permit the first horizontal-mix candidate.
 a.record(bend,0.,0.,.0095,10.,20.,0.,true,true,true);
 assert(!std::isfinite(a.estimatedProgress(bend,0.,0.)));
 a.select(bend,0.,0.);assert(a.current==0);
 a.record(bend,0.,0.,.0105,10.,20.,0.,true,true,true);
 assert(std::abs(a.estimatedProgress(bend,0.,0.)-.010)<1e-9);
 a.select(bend,0.,0.);assert(a.current==1);
 // An exploratory candidate is repeated once before advancing.
 a.record(bend,0.,0.,.011,10.,20.,0.,true,true,true);
 a.select(bend,0.,0.);assert(a.current==1);
 a.record(bend,0.,0.,.0112,10.,20.,0.,true,true,true);
 a.select(bend,0.,0.);assert(a.current==2);
 // Scaled results update the baseline response after normalization, but are
 // excluded from direct candidate score comparison.
 TrajectoryAdaptation scaled;
 scaled.record(bend,0.,0.,.0096,10.,20.,0.,true,true,true,1.2);
 scaled.record(bend,0.,0.,.0104,10.,20.,0.,true,true,true,1.3);
 assert(std::abs(scaled.estimatedProgress(bend,0.,0.)-.008)<1e-9);
 // Merely positive is not enough: a strongly scattered response must not be
 // used to decide the number of remaining strokes or to strengthen a command.
 TrajectoryAdaptation scattered;
 scattered.record(bend,0.,0.,.0020,10.,20.,0.,true,true,true);
 scattered.record(bend,0.,0.,.0050,10.,20.,0.,true,true,true);
 assert(!std::isfinite(scattered.estimatedProgress(bend,0.,0.)));
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
 for(unsigned i=0;i<7;++i){auto c=a.choice(i);assert(c.force_scale>=.95 && c.force_scale<=1.05 && std::abs(c.mix_offset)<=.05);}
 // A failed exploratory stroke forces two successful baseline revalidation
 // cycles before the next untried local candidate may run.
 TrajectoryAdaptation guarded;
 for(int i=0;i<2;++i)guarded.record(bend,0.,0.,.015,10.,20.,0.,true,true,true);
 guarded.select(bend,0.,0.);assert(guarded.current==1);
 guarded.record(bend,0.,0.,0.,10.,20.,0.,true,false,false);
 guarded.select(bend,0.,0.);assert(guarded.current==0);
 guarded.record(bend,0.,0.,.015,10.,20.,0.,true,true,true);
 guarded.select(bend,0.,0.);assert(guarded.current==0);
 guarded.record(bend,0.,0.,.015,10.,20.,0.,true,true,true);
 guarded.select(bend,0.,0.);assert(guarded.current==2);
 // Safe retained travel from an early guard is valid evidence: repeated
 // forward guarded trials must not leave the optimizer stuck at baseline.
 TrajectoryAdaptation guarded_progress;
 for(int i=0;i<2;++i)
   guarded_progress.record(bend,0.,0.,.011,10.,20.,0.,true,false,true);
 guarded_progress.select(bend,0.,0.);assert(guarded_progress.current==1);
 guarded_progress.record(bend,0.,0.,.010,10.,20.,0.,true,false,true);
 guarded_progress.select(bend,0.,0.);assert(guarded_progress.current==0);
 guarded_progress.record(bend,0.,0.,.009,10.,20.,0.,true,false,true);
 guarded_progress.select(bend,0.,0.);assert(guarded_progress.current==0);
 guarded_progress.record(bend,0.,0.,.009,10.,20.,0.,true,false,true);
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
