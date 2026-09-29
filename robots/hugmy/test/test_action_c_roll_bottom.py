#!/usr/bin/env python3
"""Offline tests for roll trend and the production bottom setpoint helper."""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root/'src/control/attitude_pressure_controller.cpp').read_text()
    start = source.index('double AttitudePressureController::actionCBottomTarget()')
    helper = source[start:source.index('bool AttitudePressureController::actionCPressureReady(', start)]
    off = source[source.index('action_c_bottom_post_pulse_ = true;'):]
    assert off.index('publishBottomTarget(actionCBottomTarget())') < off.index('enterState(ACTION_C_EVALUATE, now)')
    cycle = source[source.index('void AttitudePressureController::startInchwormCycle('):]
    cycle = cycle[:cycle.index('void AttitudePressureController::beginInchwormArmReach(')]
    assert 'action_c_bottom_post_pulse_ = false;' in cycle
    assert 'if (state_ == ACTION_C_PREPARE) action_c_bottom_post_pulse_ = false;' in source
    assert 'bottom_pressure_kpa_ - action_c_bottom_hold_kpa_' not in source
    code = r'''
#include <hugmy/control/roll_trend_gate.h>
#include <hugmy/control/bottom_pulse_timing.h>
#include <algorithm>
#include <cassert>
#include <cstdio>
class AttitudePressureController {
public:
 double action_c_bottom_hold_kpa_=30,action_c_bottom_post_pulse_drop_kpa_=3;
 bool action_c_bottom_post_pulse_=false;
 enum {SETTLE, ACTION_C_PREPARE, ON, OFF}; int state_=ON;
 bool action_c_bottom_sync_enabled_=false;
 double action_c_bottom_wait_kpa_=5;
 hugmy::BottomPulseTiming action_c_bottom_timing_;
 double actionCBottomTarget() const;
};
HELPER
int main() {
 const double d=M_PI/180.;hugmy::RollTrendGate g;
 for(int i=0;i<=50;++i)g.update(true,10*d,0,.02*i);
 assert(g.ready && !g.worsening);
 g.reset();for(int i=0;i<=50;++i)g.update(true,(9+.06*i)*d,3*d,.02*i);
 assert(g.ready && g.worsening);
 g.reset();for(int i=0;i<=50;++i)g.update(true,(-9-.06*i)*d,-3*d,.02*i);
 assert(g.ready && g.worsening);
 g.reset();for(int i=0;i<=50;++i)g.update(true,(9+.06*i)*d,-3*d,.02*i);
 assert(!g.worsening); // Disagreeing gyro cannot certify growth.
 g.update(false,12*d,0,1.02);assert(!g.ready && !g.worsening);
 g.update(true,12*d,0,2);g.update(true,NAN,0,2.02);assert(!g.ready);
 g.update(true,12*d,0,3);g.update(true,12*d,0,3.5);assert(!g.ready); // Gap resets.
 g.update(true,12*d,0,2);assert(!g.ready); // Clock reset.
 AttitudePressureController c;
 for(int i=0;i<10;++i) {
   c.action_c_bottom_post_pulse_=false;assert(c.actionCBottomTarget()==30.);
   c.action_c_bottom_post_pulse_=true;assert(c.actionCBottomTarget()==27.);
 }
 c.action_c_bottom_hold_kpa_=25;assert(c.actionCBottomTarget()==22.);
 c.action_c_bottom_hold_kpa_=2;assert(c.actionCBottomTarget()==0.);
 c.action_c_bottom_post_pulse_drop_kpa_=0;assert(c.actionCBottomTarget()==2.);
 c.action_c_bottom_hold_kpa_=30;c.action_c_bottom_post_pulse_drop_kpa_=3;
 c.action_c_bottom_sync_enabled_=true;
 for(int i=0;i<5;++i) {
  c.state_=AttitudePressureController::ACTION_C_PREPARE;
  c.action_c_bottom_post_pulse_=false;c.action_c_bottom_timing_.beginCycle();
  assert(c.actionCBottomTarget()==5.);
  c.action_c_bottom_timing_.filling=true;assert(c.actionCBottomTarget()==30.);
  c.state_=AttitudePressureController::ON;assert(c.actionCBottomTarget()==30.);
  c.state_=AttitudePressureController::OFF;c.action_c_bottom_post_pulse_=true;
  assert(c.actionCBottomTarget()==27.);
  c.state_=AttitudePressureController::SETTLE;assert(c.actionCBottomTarget()==5.);
 }
 puts("PASS: signed roll trend, agreement/fault/reset gates, pulse OFF drop, no accumulation, candidate independence");
}
'''.replace('HELPER', helper)
    with tempfile.TemporaryDirectory(prefix='hugmy_roll_bottom_') as directory:
        binary = str(pathlib.Path(directory)/'test')
        subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-I',str(root/'include'),'-x','c++','-','-o',binary],
                       input=code,text=True,check=True)
        subprocess.run([binary],check=True)


if __name__ == '__main__':
    main()
