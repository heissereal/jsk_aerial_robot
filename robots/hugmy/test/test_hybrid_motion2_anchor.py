#!/usr/bin/env python3
"""Opt-in Motion-2 anchor search and direct paired-extension handoff tests."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = r'''
#include <hugmy/control/hybrid_single_pass.h>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace hugmy::hybrid;

int main() {
  Motion2RearAnchorSearch rear;
  rear.reset(30.);
  for(int i=0;i<100;++i)
    assert(rear.update(30.,20.,.10,.2618,0.,false,true,.02)==30.);
  // A powered pair with zero progress is given time before rear relaxation.
  for(int i=0;i<59;++i)
    assert(rear.update(30.,20.,.10,.2618,0.,true,true,.02)==30.);
  assert(rear.update(30.,20.,.10,.2618,0.,true,true,.02)==27.5);
  assert(rear.release_steps==1);
  // Meaningful progress resets the stall clock.
  assert(rear.update(30.,20.,.10,.2618,.04,true,true,.02)==27.5);
  for(int i=0;i<29;++i)
    assert(rear.update(30.,20.,.10,.2618,.04,true,true,.02)==27.5);
  assert(rear.update(30.,20.,.10,.2618,.04,true,true,.02)==25.);
  for(int i=0;i<200;++i)
    rear.update(30.,20.,.10,.2618,.04,true,true,.02);
  assert(rear.pressure==20.);
  assert(rear.update(30.,20.,.10,.2618,.04,true,false,.02)==30.);

  SinglePassConfig k;
  k.motion2_rear_anchor_search_enabled=true;
  assert(k.valid());
  auto invalid=k; invalid.motion2_rear_min_pressure=31.;
  assert(!invalid.valid());
  invalid=k;invalid.motion2_release_start_ratio=.9;
  assert(!invalid.valid());

  // Two powered front arms reach the same *relative* travel on a new object.
  // The body never rises or pitches: old mode would stay in Lift searching
  // for load relief, while the short-pulse mode must enter Pull immediately.
  Config c;Model model;model.reset(c);
  auto geometry=[](size_t,double a,double,double){
    Force f;f.forward=1.;f.up=std::cos(a);
    f.roll_moment=0.;f.pitch_moment=0.;return f;
  };
  SinglePassInput in;
  in.s.fresh=true;
  in.s.bend={{1.5,1.7}};
  in.s.start_bend=in.s.bend;
  in.s.floor={{.5,.5}};
  in.s.command={{0.,0.}};in.s.pressure=in.s.command;
  in.s.thrust={{2.,2.}};
  in.rear_pressure=30.;
  SinglePass p;
  p.slip_tested=true;
  p.begin(in);
  p.enter(SinglePass::Lift,in);
  p.onset_this_stroke={{true,true}};
  p.lift_onset_seen={{true,true}};
  p.powered_extension_reference_valid={{true,true}};
  p.powered_extension_reference=in.s.bend;
  p.powered_body_reference_valid=true;
  in.s.bend={{1.20,1.40}};
  in.s.rate={{-.10,-.10}};
  const auto out=p.update(in,c,k,model,geometry);
  assert(!out.fault);
  assert(p.phase==SinglePass::Pull);
  assert(p.motion2_direct_extension_handoff);
  assert(!p.unload_event_detected); // no fictitious body load relief
  assert(!p.load_relief_detected);
  assert(p.motion2_rear_anchor.release_steps==0);
  auto legacy_k=k;legacy_k.motion2_rear_anchor_search_enabled=false;
  SinglePass legacy;legacy.slip_tested=true;
  auto before=in;before.s.bend={{1.5,1.7}};
  legacy.begin(before);legacy.enter(SinglePass::Lift,before);
  legacy.onset_this_stroke={{true,true}};
  legacy.lift_onset_seen={{true,true}};
  legacy.powered_extension_reference_valid={{true,true}};
  legacy.powered_extension_reference=before.s.bend;
  legacy.powered_body_reference_valid=true;
  auto old_out=legacy.update(in,c,legacy_k,model,geometry);
  assert(!old_out.fault && legacy.phase==SinglePass::Lift);
  std::cout<<"PASS: rear pressure searches only after stall; paired travel hands off without false unload event\n";
}
'''

with tempfile.TemporaryDirectory() as tmp:
    cpp = Path(tmp) / 'motion2_anchor.cpp'
    exe = Path(tmp) / 'motion2_anchor'
    cpp.write_text(source)
    subprocess.run([
        'g++', '-std=c++14', '-O2', '-Wall', '-Wextra',
        '-I' + str(root / 'include'), str(cpp), '-o', str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
