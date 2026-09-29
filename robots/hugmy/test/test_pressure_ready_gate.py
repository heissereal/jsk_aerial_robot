#!/usr/bin/env python3
"""Test actual hysteresis + settling/confirmation, optionally replay a bag.

The replay evaluates sensor prerequisites only; no ROS nodes or actuators run.
"""
import pathlib
import subprocess
import sys
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    assert 'pressure_valid && actionCPressureReady(' in source
    assert 'posture_boundary && actionCPressureReady(inchworm_pressure_, now)' in source
    assert 'action_c_pressure_ready_gate_.reset();' in source
    cpp = r'''
#include <hugmy/control/pressure_ready_gate.h>
#include <hugmy/control/posture_recovery_gate.h>
#include <cassert>
#include <cstdio>
#include <iostream>
int main(int argc,char**) {
 using G=hugmy::PressureReadyGate; using A=G::Four;
 G g; A p{{30,30,30,30}}, target=p, tol{{2,2,2,2}};
 auto tick=[&](double t,bool fresh=true){return g.update(p,target,tol,t,fresh);};
 p[2]=34;assert(!tick(0)); // Never acquire from widened band alone.
 p[2]=30;assert(tick(.02));
 p[2]=34;assert(tick(.04));assert(tick(2)); // Normal regulation ripple.
 p[2]=36;assert(tick(2.02));assert(tick(2.10));assert(!tick(2.13));
 p[2]=34;assert(!tick(2.14)); // Must re-enter narrow band after timeout.
 p[2]=30;assert(tick(2.16));
 p[2]=39;assert(!tick(2.18)); // Severe deviation has NO grace.
 p[2]=30;assert(tick(2.2));p[2]=21;assert(!tick(2.22));
 p[2]=30;assert(tick(2.24));p[2]=NAN;assert(!tick(2.26));
 p[2]=30;assert(tick(2.28));assert(!tick(2.3,false));
 assert(tick(2.32));assert(!tick(1.)); // Backward clock invalidates latch.
 assert(tick(3.));target[0]=0;assert(!tick(3.02)); // Changed target cannot inherit readiness.
 p[0]=4;tol[0]=5;assert(tick(3.04)); // Released-front arrival tolerance.
 g.reset();p=target={{30,30,30,30}};tol.fill(2);
 hugmy::PostureRecoveryGate settle, confirm;
 bool confirmed=false;
 for(int i=0;i<100;++i) {
   double t=.02*i;p[2]=(i%10==5)?34:30;
   bool ready=tick(t);
   confirmed=confirm.update(settle.update(ready,t,.5),t,.3);
 }
 assert(confirmed); // 34-kPa spikes no longer starve the combined 0.8-s window.
 if(argc==1) {puts("PASS: strict acquisition, hysteresis, finite grace, severe/stale/NaN/reset/target faults, combined recovery gates");return 0;}
 g.reset();settle.reset();confirm.reset();
 double t,bottom,rate; bool found=false;
 while(std::cin>>t>>p[0]>>p[1]>>p[2]>>p[3]>>bottom>>rate) {
   bool ready=tick(t);
   bool stable=settle.update(ready && std::abs(bottom)<=5 && std::abs(rate)<=10*M_PI/180,t,.5);
   if(confirm.update(stable,t,.3) && !found) {
     std::cout<<"REPLAY recovery sensor prerequisites confirmed at "<<t<<" s\n";found=true;
   }
 }
 if(!found) {std::cerr<<"No confirmed prerequisites in replay\n";return 1;}
}
'''
    with tempfile.TemporaryDirectory(prefix='hugmy_pressure_ready_') as directory:
        binary = str(pathlib.Path(directory) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                        '-x', 'c++', '-', '-o', binary], input=cpp, text=True, check=True)
        subprocess.run([binary], check=True)
        if len(sys.argv) > 1:
            import rosbag
            bottom, rate = float('nan'), float('nan')
            rows = []
            prefix = '/quadrotor/independent_arm_pressure_controller/'
            with rosbag.Bag(sys.argv[1]) as bag:
                start = bag.get_start_time()
                for topic, msg, stamp in bag.read_messages(topics=[prefix+'pressure', prefix+'bottom_pressure', '/quadrotor/imu']):
                    t = stamp.to_sec()-start
                    if t > 44.0:
                        break
                    if topic.endswith('bottom_pressure'):
                        bottom = msg.data
                    elif topic.endswith('/imu'):
                        rate = msg.gyro[0]
                    elif t >= 24.075:
                        rows.append(' '.join(map(str, [t]+list(msg.data)+[bottom, rate])))
            subprocess.run([binary, 'replay'], input='\n'.join(rows), text=True, check=True)


if __name__ == '__main__':
    main()
