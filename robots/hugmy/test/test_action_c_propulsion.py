#!/usr/bin/env python3
"""Offline tests of production geometry, measured timing and adaptation gates."""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    assert source.index('if (finishActionCPropulsionTrial(recovery_required, retained_dx)) return;') < source.index('action_c_coordination_.finishTrial(')
    assert 'action_c_propulsion_.regrip(valid_progress);' in source
    assert 'powered_complete && action_c_propulsion_.both_powered_sec>=.1' in source
    assert 'state_==ACTION_C_EVALUATE || state_==ACTION_C_REGRIP || state_==SETTLE' in source
    assert 'extension_guard = guard_reasons != 0;' in source
    assert 'action_c_coordination_.lag_count.fill(0)' in source
    code = r'''
#include <hugmy/control/action_c_propulsion.h>
#include <cassert>
#include <cstdio>
int main() {
 using P=hugmy::ActionCPropulsion;
 const std::array<bool,4> front{{true,false,false,true}};
 const double deg=M_PI/180.;
 auto projection=[](int arm,double bend,double roll=0.,double pitch=0.,double heading=0.,double direction=1.) {
   return hugmy::actionCForwardProjection(bend,(-135+90*arm)*M_PI/180.,6.5*M_PI/180.,
       arm==0 || arm==2 ? 1. : -1.,roll,pitch,heading,direction);
 };
 assert(projection(0,M_PI/2)>.7 && projection(3,M_PI/2)>.7);
 assert(std::abs(projection(0,M_PI/2)-projection(3,M_PI/2))<1e-12);
 assert(projection(1,M_PI/2,0,0,0,-1)>.7 && projection(2,M_PI/2,0,0,0,-1)>.7);
 assert(projection(0,M_PI/2,0,0,M_PI)<0);
 assert(std::abs(projection(0,M_PI/2,0,0,M_PI/2))<=1.);
 assert(projection(0,NAN)==0.);
 for(int arm=0;arm<4;++arm) for(int i=0;i<200;++i)
   assert(std::abs(projection(arm,i*.014,.2,.3,.4))<=1.000001);
 P p;p.beginTrial();p.beginPulse();
 auto& r=p.response[0];
 r.command(1.,5.,30.,1.4);
 r.command(1.1,6.,30.,1.3);assert(r.start==1.); // command edge not reset
 r.observe(1.02,1.02,5.,1.39,-.5,true);
 r.observe(1.06,1.06,5.6,1.38,0.,true);
 assert(r.delay>.05 && r.delay<.07);
 r.observe(1.12,1.12,8.,1.40,.2,true);
 r.observe(1.18,1.18,20.,1.42,.2,true);
 assert(r.rate>1. && r.pending>0. && r.pending<=.20);
 assert(r.lead==0.); // frozen while collecting
 const double pending=r.pending;
 r.observe(1.2,1.18,20.,2.,1.,true);assert(r.pending==pending); // duplicate pressure sample
 p.force(1.2,1.);p.force(1.22,1.);assert(p.forward_impulse>0.);
 p.complete(true);p.regrip(true);p.regrip(true);assert(p.completed_pulls==1);
 assert(p.finish(false,1,0.,.002,1.05,1.5,front)==0); // recovery/guard
 assert(p.finish(true,2,0.,.002,1.05,1.5,front)==0); // incomplete burst
 assert(p.finish(true,1,-.001,.002,1.05,1.5,front)==0); // backward
 assert(p.commitTiming(front));
 assert(r.lead==.03 && p.extra_n==0.); // timing ONLY
 assert(!r.early(.2,.6,0.,8*deg,-1.));
 assert(!r.early(.1,.6,8*deg,8*deg,-1.));
 assert(r.early(.59,.6,5*deg,8*deg,-1.));
 assert(!r.early(.2,.6,5*deg,8*deg,-.01));
 // Motion failure does not erase pneumatic identification; no IMU is needed
 // to measure pressure delay/rise, but it cannot manufacture flexion success.
 P identify;identify.beginTrial();identify.beginPulse();
 auto& q=identify.response[3];q.command(2,0,30,NAN);
 q.observe(2.05,2.05,1,NAN,NAN,true,false);
 q.observe(2.8,2.8,20,NAN,NAN,true,false);
 assert(q.pressure_identified && q.pending>0 && q.flex_at<0);
 assert(identify.completed_pulls==0 && identify.commitTiming(front));
 assert(q.lead==.03 && identify.extra_n==0.);
 assert(identify.finish(false,1,0,.002,1.05,1.5,front)==0);
 using hugmy::actionCFlexionHandoff;
 assert(!actionCFlexionHandoff(.05,8*deg,8*deg,-1,10*deg,M_PI,.08));
 assert(!actionCFlexionHandoff(.3,0,8*deg,-1,10*deg,M_PI,.08));
 assert(actionCFlexionHandoff(.3,3*deg,8*deg,-100*deg,12*deg,M_PI,.08));
 assert(actionCFlexionHandoff(.3,8*deg,8*deg,0,12*deg,M_PI,.08));
 assert(!actionCFlexionHandoff(.3,3*deg,8*deg,.2,12*deg,M_PI,.08));
 assert(!actionCFlexionHandoff(.3,NAN,8*deg,-1,12*deg,M_PI,.08));
 P phases;phases.beginPulse();phases.startFlexion(3,2.4,1.05);
 assert(phases.flexing[3] && !phases.flexing[0] && !phases.allFlexing(front));
 phases.startFlexion(3,3,1.5);assert(phases.flex_ceiling[3]==1.05); // no re-boost
 phases.startFlexion(0,.7,1.05);assert(phases.allFlexing(front) && phases.flex_ceiling[0]==.7);
 assert(!phases.regripReady(1,true));assert(!phases.regripReady(1.1,false));
 assert(!phases.regripReady(1.2,true));assert(phases.regripReady(1.36,true));
 phases.beginPulse();assert(!phases.allFlexing(front) && phases.regrip_start<0);
 const std::array<bool,4> back{{false,true,true,false}};
 phases.startFlexion(1,.8,1.05);assert(!phases.allFlexing(back));
 phases.startFlexion(2,1.2,1.05);assert(phases.allFlexing(back));
 // Rebound does not erase attained extension or manufacture pair mismatch.
 std::array<double,4> starts{{1.3,1.4,1.4,1.6}}, minima=starts;
 minima[0]-=8*deg;minima[3]-=9*deg;
 assert(std::abs(hugmy::actionCPeakExtensionMismatch(front,starts,minima)-deg)<1e-10);
 minima[3]-=15*deg;
 assert(hugmy::actionCPeakExtensionMismatch(front,starts,minima)>15*deg);
 P stale;stale.beginPulse();stale.response[0].command(1,0,30,1.);
 stale.response[0].observe(1.1,1.1,20,1.1,1.,false);
 assert(stale.response[0].censored && stale.response[0].pending<0.);
 P backward_clock;backward_clock.beginPulse();backward_clock.response[0].command(2,0,30,1.);
 backward_clock.response[0].observe(1.9,1.9,5,1.1,1.,true);
 assert(backward_clock.response[0].censored);
 P boost;boost.beginTrial();boost.beginPulse();boost.force(1,1);boost.force(1.02,1);
 boost.complete(false);boost.regrip(true);
 assert(boost.finish(true,1,0.,.002,1.05,1.5,front)==0); // no flexion
 boost.complete(true);boost.regrip(true);
 assert(boost.finish(true,1,0.,.002,1.05,1.5,front)==2);
 assert(std::abs(boost.extra_n-.1)<1e-10);
 assert(boost.finish(true,1,.003,.002,1.05,1.5,front)==0); // progress sufficient
 for(int i=0;i<20;++i) boost.finish(true,1,0.,.002,1.05,1.5,front);
 assert(std::abs(boost.extra_n-.45)<1e-10);
 assert(boost.finish(true,1,0.,.002,1.05,1.5,front)==0); // cap
 boost.beginTrial();assert(boost.completed_pulls==0 && boost.trial_forward_impulse==0.);
 boost=P();assert(boost.extra_n==0. && boost.response[0].lead==0.); // new task/direction
 puts("PASS: cant/tilt/direction projection, measured pressure delay/rise/flexion, immutable trial timing, completed-only bounded pull learning");
}
'''
    with tempfile.TemporaryDirectory(prefix='hugmy_propulsion_') as directory:
        binary = str(pathlib.Path(directory) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                        '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
        subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
