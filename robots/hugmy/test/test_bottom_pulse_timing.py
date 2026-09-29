#!/usr/bin/env python3
"""Offline pressure-plant tests against the actual bottom timing helper."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root/'src/control/attitude_pressure_controller.cpp').read_text()
assert 'action_c_bottom_timing_.observe(msg->data' in source
assert 'action_c_bottom_timing_.ready(action_c_bottom_hold_kpa_' in source
assert 'action_c_bottom_timing_.peak_at = now.toSec() + actionCBottomPeakLead()' in source
code = r'''
#include <hugmy/control/bottom_pulse_timing.h>
#include <cassert>
#include <cstdio>
double cycle(hugmy::BottomPulseTiming& s,double start,double lag,double rate) {
  s.beginCycle();s.observe(5,start,30);s.startFill(start);
  double on=-1.;
  for(int i=1;i<1000;++i) {
    double t=start+i*.01;
    double p=std::min(30.,5+rate*std::max(0.,t-start-lag));
    s.observe(p,t,30);
    if(on<0 && s.ready(30,2,.375,t)) {
      on=t; s.peak_at=t+.375;
      assert(p<30.); // No full inflation + dwell before normal pulse.
    }
    if(on>=0 && t>=on+.38) {
      assert(p>=28. && p<=30.); assert(std::isfinite(s.peak_error));
      assert(on>=start+lag); return on;
    }
  }
  assert(false);return -1;
}
int main() {
  hugmy::BottomPulseTiming s;
  const double fast=cycle(s,0.,.2,40.);
  assert(s.learned && s.delay>0 && s.rate>0);
  // Fresh local slope/delay, NOT optimistic reuse from the previous fill.
  const double slow=cycle(s,5.,.6,10.);
  assert(slow-5.>fast);
  s.rate=200.;s.learned=true;
  const double slow_after_fast=cycle(s,9.,.2,10.);
  assert(slow_after_fast-9.>2.); // Never launch using an old optimistic slope.
  s.beginCycle();s.observe(5,20.,30);s.startFill(20.);
  for(int i=1;i<=100;++i) {
    s.observe(5,20+i*.01,30);
    assert(!s.ready(30,2,.375,20+i*.01));
  }
  assert(!s.ready(30,2,.375,22.)); // Stale sensor cannot release pulse.
  s.observe(NAN,22.,30);assert(!s.learned && !s.fresh(22.));
  s.observe(5,23.,30);s.startFill(23.);s.observe(6,22.,30);
  assert(!s.filling && !s.learned); // Clock reset cancels preparation.
  puts("PASS: low-wait / fresh-rate inflation, varying lag/rate, pressure at nominal peak, no-rise/stale/NaN/clock guards");
}
'''
with tempfile.TemporaryDirectory(prefix='hugmy_bottom_sync_') as tmp:
    binary=str(pathlib.Path(tmp)/'test')
    subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-I',str(root/'include'),
                    '-x','c++','-','-o',binary],input=code,text=True,check=True)
    subprocess.run([binary],check=True)
