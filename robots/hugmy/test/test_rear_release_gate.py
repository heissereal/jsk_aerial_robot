#!/usr/bin/env python3
"""Production release gate + optional recorded ripple replay (no ROS master)."""
import pathlib
import subprocess
import sys
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    src = (root/'src/control/attitude_pressure_controller.cpp').read_text()
    enter = src[src.index('void AttitudePressureController::enterState('):]
    enter = enter[:enter.index('if (state_ == PULL)')]
    assert enter.index('action_c_rear_release_gate_.reset();') < enter.index('state_ = state;')
    adapter = (root/'src/control/hybrid_locomotion.cpp').read_text()
    assert 'input.rear_released=actionCRearReleased(now)' in adapter
    assert 'hybrid rear release lost' not in adapter  # intentional periodic grasp is not a fault
    single=(root/'include/hugmy/control/hybrid_single_pass.h').read_text()
    assert 'release_seen=release_seen || in.rear_released' in single
    assert 'phase==SlipProbe && !posture_brake' in single
    assert 'pressure_target[j]=0.' in single
    assert 'in.rear_grasped' in single
    replay = ''
    if len(sys.argv)>1:
        import rosbag
        with rosbag.Bag(sys.argv[1]) as bag:
            origin = bag.get_start_time()
            samples = [(t.to_sec()-origin, list(m.data)) for _, m, t in bag.read_messages(
                topics=['/quadrotor/independent_arm_pressure_controller/pressure'])]
        # Actual ON edge and the pressure-ripple cutoff in the 16:57:28 bag.
        for start, stop in ((34.083,34.123),(55.083,55.123),(208.014,208.055)):
            lo = min(range(len(samples)), key=lambda i:abs(samples[i][0]-start))
            hi = min(range(len(samples)), key=lambda i:abs(samples[i][0]-stop))
            replay += 'g.reset();\n'
            for t,p in samples[lo:hi+1]:
                replay += 'assert(g.update({{%s}},front,%.9f,true,8,10,12,.1));\n' % (
                    ','.join(repr(x) for x in p),t)
    code = r'''
#include <hugmy/control/rear_release_gate.h>
#include <cassert>
#include <limits>
int main() {
 hugmy::RearReleaseGate g;
 std::array<bool,4> front{{true,false,false,true}};
 auto tick=[&](double t,double p,bool fresh=true) {
   return g.update({{30,p,p,30}},front,t,fresh,8,10,12,.1);
 };
 assert(!tick(0,8.08)); // cannot acquire with the wider hold band
 assert(tick(.02,7.85));assert(tick(.04,8.08));assert(tick(.06,10));
 assert(tick(.08,11));assert(tick(.16,11)); // bounded grace
 assert(!tick(.20,11));assert(!tick(.22,9)); // reacquisition still <=8
 assert(tick(.24,7));assert(!tick(.26,12.01)); // hard breach: no grace
 assert(tick(.28,7));assert(!tick(.30,7,false));
 assert(!tick(.32,9));assert(tick(.34,7));
 assert(!tick(.36,std::numeric_limits<double>::quiet_NaN()));
 assert(tick(.38,7));assert(!tick(.37,7)); // backwards clock resets
 assert(!tick(.40,9));assert(tick(.42,7));
 g.reset();assert(!tick(.44,9)); // new pulse must release again
 assert(tick(.46,7));
 front={{false,true,true,false}};assert(!tick(.48,7)); // pair swap resets, new rear=30
 assert(g.update({{7,30,30,7}},front,.5,true,8,10,12,.1));
 assert(g.update({{8.08,30,30,9}},front,.52,true,8,10,12,.1));
 assert(!g.update({{7,30,30,13}},front,.54,true,8,10,12,.1));
 front={{true,false,false,true}};
 g.reset(); assert(tick(1,7)); assert(tick(1.02,11)); assert(tick(1.08,9));
 assert(tick(1.1,11)); assert(tick(1.18,11)); assert(!tick(1.22,11));
 assert(!g.update({{7,7,7,7}},{{true,true,true,true}},2,true,8,10,12,.1));
 assert(!g.update({{7,7,7,7}},front,3,true,8,7,12,.1));
 REPLAY
}
'''.replace('REPLAY',replay)
    with tempfile.TemporaryDirectory(prefix='hugmy_release_gate_') as tmp:
        binary = str(pathlib.Path(tmp)/'test')
        subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-Werror',
                        '-I'+str(root/'include'),'-x','c++','-','-o',binary],
                       input=code,text=True,check=True)
        subprocess.run([binary],check=True)
    print('PASS: rear release acquisition/hold/grace/hard/fault/mask/reset checks' +
          (' and three recorded ripple cutoffs' if replay else ''))


if __name__ == '__main__':
    main()
