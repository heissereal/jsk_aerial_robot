#!/usr/bin/env python3
"""Production pressure gates, with optional read-only cd_5 bag replay.

Replay checks pressure prerequisites only, NOT resulting motion or stability.
"""
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CODE = r'''
#include <hugmy/control/hybrid_measurement.h>
#include <hugmy/control/pressure_ready_gate.h>
#include <cassert>
#include <iostream>
using namespace hugmy::hybrid;
int main(int argc,char**) {
 BodyRateFilter gyro;gyro.update({{0.,0.}},0.);
 for(int i=1;i<150;++i) {
   auto rate=gyro.update({{i%2?.18:-.18,i%2?-.18:.18}},i*.02);
   assert(std::abs(rate[0])<.1 && std::abs(rate[1])<.1);
 }
 const auto previous=gyro.value;
 assert(gyro.update({{10.,10.}},gyro.stamp)==previous); // no duplicate filtering
 gyro.update({{.5,.5}},3.);assert(gyro.value[0]>.05);
 gyro.update({{NAN,0.}},3.02);assert(gyro.stamp<0.);
 assert(gyro.update({{.9,0.}},4.)[0]==.9); // reset does not hide a fast rotation
 PressureGuard g; std::array<double,4> p{{40,30,30,40}};
 auto tick=[&](double t,bool fresh=true){return g.update(p,t,fresh,45.,60.,.08);};
 assert(tick(0)==PressureGuard::Ready);
 p[0]=50.4872;assert(tick(.02)==PressureGuard::Pending);
 p[0]=39.7082;assert(tick(.04)==PressureGuard::Ready);
 p[0]=49.2649;assert(tick(.06)==PressureGuard::Pending);
 p[0]=40.6713;assert(tick(.08)==PressureGuard::Ready);
 p[2]=49.;assert(tick(.10)==PressureGuard::Pending);
 assert(tick(.17)==PressureGuard::Pending);assert(tick(.18)==PressureGuard::Fault);
 // One impossible hard-threshold ADC sample must not abort a good stroke.
 g.reset();p[2]=99.3447;assert(tick(1.)==PressureGuard::Pending);
 p[2]=30.;assert(tick(1.02)==PressureGuard::Ready);
 // A real, sustained hard overpressure still fails after 80 ms.
 g.reset();p[2]=60.;assert(tick(2.)==PressureGuard::Pending);
 assert(tick(2.07)==PressureGuard::Pending);
 assert(tick(2.08)==PressureGuard::Fault);
 p[2]=NAN;assert(tick(2.1)==PressureGuard::Fault);
 g.reset();p[2]=-5.1;assert(tick(3.)==PressureGuard::Pending);
 p[2]=30.;assert(tick(3.02)==PressureGuard::Ready);
 p[2]=-5.1;assert(tick(3.04)==PressureGuard::Pending);
 assert(tick(3.12)==PressureGuard::Fault);
 // Opposite ADC rails within a few samples are not a physical chamber
 // response. Hold the command through the burst, then resume only after a
 // complete quiet confirmation interval.
 g.reset();p[2]=10.;assert(tick(10.)==PressureGuard::Ready);
 p[2]=-5.56;assert(tick(10.02)==PressureGuard::Pending);
 p[2]=49.89;assert(tick(10.04)==PressureGuard::Pending);
 assert(g.rejectingAlternatingSpike());assert(g.alternatingSpikeMask()==4u);
 p[2]=48.97;assert(tick(10.06)==PressureGuard::Pending);
 p[2]=-5.52;assert(tick(10.08)==PressureGuard::Pending);
 p[2]=49.82;assert(tick(10.10)==PressureGuard::Pending);
 p[2]=10.;assert(tick(10.12)==PressureGuard::Pending);
 assert(tick(10.18)==PressureGuard::Pending);
 assert(tick(10.20)==PressureGuard::Ready);
 assert(!g.rejectingAlternatingSpike());
 // Do not mask a failed sensor forever: persistent alternating rails fault
 // after the bounded 0.5-s filter interval.
 g.reset();p[2]=-5.5;assert(tick(20.)==PressureGuard::Pending);
 p[2]=50.;assert(tick(20.02)==PressureGuard::Pending);
 for(int i=2;i<25;++i) {
   p[2]=i%2 ? 50. : -5.5;
   assert(tick(20.+.02*i)==PressureGuard::Pending);
 }
 p[2]=50.;assert(tick(20.50)==PressureGuard::Fault);
 // A real one-sided overpressure remains governed by the original 80-ms
 // safety confirmation and is never classified as alternating noise.
 g.reset();p[2]=60.;assert(tick(21.)==PressureGuard::Pending);
 assert(tick(21.07)==PressureGuard::Pending);
 assert(tick(21.08)==PressureGuard::Fault);
 p[2]=30.;assert(tick(4.,false)==PressureGuard::Fault);
 assert(tick(5.)==PressureGuard::Ready);assert(tick(4.)==PressureGuard::Fault);
 assert(g.update(p,6.,true,45.,45.,.08)==PressureGuard::Fault);
 assert(g.update(p,6.,true,45.,60.,NAN)==PressureGuard::Fault);
 Config c;c.pressure_max=40.;State s;s.fresh=true;s.now=1.;
 s.bend={{1.3,1.3}};s.start_bend=s.bend;s.pressure={{50.,30.}};s.command={{30.,30.}};
 assert(!Planner::valid(s,c));assert(Planner::valid(s,c,true));
 s.rate[0]=c.bend_rate_limit;assert(!Planner::valid(s,c,true));s.rate[0]=0.;
 s.roll=c.roll_limit;assert(!Planner::valid(s,c,true));s.roll=0.;
 s.pitch=c.pitch_change_limit;assert(!Planner::valid(s,c,true));s.pitch=0.;
 s.bend[0]=c.bend_min;assert(!Planner::valid(s,c,true));s.bend[0]=1.3;
 s.pressure[0]=NAN;assert(!Planner::valid(s,c,true));
 hugmy::PressureReadyGate ready;
 std::array<double,4> target{{30,30,30,30}},tol{{2,2,2,2}};
 p=target;PositionWindow window;double since=-1.;bool complete=false;
 for(int i=0;i<=100;++i) {
   double t=i*.02;p[2]=(i%10==5)?34.:30.;
   if(ready.update(p,target,tol,t,true)) window.add(t,.12);
   else {window.reset();since=-1.;}
   if(window.stable(t)) {if(since<0.) since=t;complete=t-since>=.4;}
 }
 assert(complete); // periodic ripple no longer starves .8 + .4 s settlement
 if(argc==1) {std::cout<<"PASS hybrid pressure: transient/persistent/hard/invalid, motion guards, preparation hysteresis\n";return 0;}
 g.reset();ready.reset();window.reset();since=-1.;
 double t,first_t=-1.,last_t=-1.,old_since=-1.,old_best=0.,new_since=-1.,new_best=0.;
 int pending=0,fault=0,old_fault=0;double first_fault=-1.;
 while(std::cin>>t>>p[0]>>p[1]>>p[2]>>p[3]) {
   if(first_t<0.) first_t=t;last_t=t;
   auto status=tick(t);pending+=status==PressureGuard::Pending;fault+=status==PressureGuard::Fault;
   if(status==PressureGuard::Fault && first_fault<0.) first_fault=t;
   bool old_ready=true;
   for(double v:p) {old_ready &= std::abs(v-30.)<=2.;old_fault+=v>45.;}
   bool new_ready=ready.update(p,target,tol,t,true);
   if(old_ready) {if(old_since<0.) old_since=t;old_best=std::max(old_best,t-old_since);} else old_since=-1.;
   if(new_ready) {if(new_since<0.) new_since=t;new_best=std::max(new_best,t-new_since);} else new_since=-1.;
 }
 std::cout<<"old_peak_faults="<<old_fault<<" pending="<<pending<<" confirmed_faults="<<fault
          <<" first_fault="<<first_fault
          <<" old_ready_longest="<<old_best<<" new_ready_longest="<<new_best<<std::endl;
 assert(fault==0);
 if(old_fault) assert(pending>0);
 else if(last_t-first_t>=1.2) assert(new_best>=1.2 && new_best>old_best);
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix='hugmy_hybrid_pressure_') as directory:
        binary = str(Path(directory) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(ROOT/'include'),
                        '-x', 'c++', '-', '-o', binary], input=CODE, text=True, check=True)
        subprocess.run([binary], check=True)
        if len(sys.argv) > 1:
            import rosbag
            with rosbag.Bag(sys.argv[1]) as bag:
                start = bag.get_start_time()
                duration = bag.get_end_time()-start
                windows = [(37.5, 41.), (53., 57.), (59.1, 62.7),
                           (95., 103.), (116., 142.), (170., 201.)]
                if duration >= 209.:
                    windows.append((202.2, 209.))
                samples = [[] for _ in windows]
                topic = '/quadrotor/independent_arm_pressure_controller/pressure'
                for _, msg, stamp in bag.read_messages(topics=[topic]):
                    t = stamp.to_sec()-start
                    for i, (lo, hi) in enumerate(windows):
                        if lo <= t <= hi:
                            samples[i].append(' '.join(map(str, [t]+list(msg.data))))
            for window, rows in zip(windows, samples):
                assert rows, 'missing pressure data'
                print('cd_5 pressure-only replay', window, flush=True)
                subprocess.run([binary, 'replay'], input='\n'.join(rows), text=True, check=True)


if __name__ == '__main__':
    main()
