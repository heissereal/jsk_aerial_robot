#!/usr/bin/env python3
"""Opt-in Motion-2 anchor search and direct paired-extension handoff tests."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

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
  invalid=k;invalid.motion2_rear_anchor_search_enabled=false;
  invalid.motion2_paired_extension_handoff_enabled=true;
  invalid.motion2_handoff_max_extension=.1;
  assert(!invalid.valid());
  invalid=k;invalid.motion2_paired_extension_handoff_enabled=true;
  assert(!invalid.valid()); // pressure-search and measured-release policies conflict
  invalid=k;invalid.motion2_rear_anchor_search_enabled=false;
  invalid.motion2_paired_extension_handoff_enabled=true;
  invalid.motion2_handoff_stall_seconds=0.;
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
  // Independent policy: rear pressure is released continuously rather than
  // by the opt-in 2.5-kPa/20-kPa anchor-search steps. Merely reaching 15 deg
  // while still extending may not end the FIRST identification stroke.
  auto adaptive_k=legacy_k;
  adaptive_k.motion2_paired_extension_handoff_enabled=true;
  assert(adaptive_k.valid());
  auto start=before;
  SinglePass adaptive;adaptive.slip_tested=true;
  adaptive.begin(start);adaptive.enter(SinglePass::Lift,start);
  adaptive.onset_this_stroke={{true,true}};
  adaptive.lift_onset_seen={{true,true}};
  adaptive.powered_extension_reference_valid={{true,true}};
  adaptive.powered_extension_reference=start.s.bend;
  adaptive.powered_body_reference_valid=true;
  auto probe=in;
  probe.s.rate={{-.2,-.2}};
  probe.rear_pressure=7.5;
  for(int i=0;i<4;++i) {
    auto sample=adaptive.update(probe,c,adaptive_k,model,geometry);
    assert(!sample.fault && adaptive.phase==SinglePass::Lift);
  }
  assert(adaptive.motion2_handoff_target_extension>=
         adaptive_k.trajectory_min_extension);
  assert(adaptive.motion2_rear_anchor.release_steps==0);
  // Stalled extension at adequate travel can hand off without manufacturing
  // a pitch/height cue, but only after the slower real rear pressure follows.
  probe.s.rate={{-.02,-.02}};
  probe.rear_pressure=30.;
  for(int i=0;i<17;++i) {
    auto sample=adaptive.update(probe,c,adaptive_k,model,geometry);
    assert(!sample.fault && adaptive.phase==SinglePass::Lift);
  }
  // The integrated low-pressure schedule commands 0 kPa here.  Completion
  // must wait for the measured chamber pressure to enter the configured
  // 3-kPa tolerance instead of accepting the older 7.5-kPa floor.
  probe.rear_pressure=3.0;
  bool handoff=false;
  for(int i=0;i<20 && !handoff;++i) {
    auto sample=adaptive.update(probe,c,adaptive_k,model,geometry);
    assert(!sample.fault);
    handoff=adaptive.phase==SinglePass::Pull;
  }
  if(!handoff)
    std::cerr<<"adaptive handoff diagnostic: target="
             <<adaptive.motion2_handoff_target_extension
             <<" stall="<<adaptive.motion2_handoff_stall_time
             <<" rear_cmd="<<adaptive.rear_pressure_command
             <<" rear_release_commanded="<<adaptive.rear_release_commanded
             <<" phase="<<adaptive.phase
             <<" load_relief="<<adaptive.load_relief_detected
             <<" direct="<<adaptive.motion2_direct_extension_handoff
             <<"\n";
  assert(handoff);
  assert(adaptive.motion2_direct_extension_handoff);
  assert(!adaptive.unload_event_detected);
  // Reuse the first paired-extension observation; later strokes with a larger
  // measured reach must not silently fall back to the fixed 15-deg shortcut.
  auto learned_k=adaptive_k;
  SinglePass learned;learned.slip_tested=true;
  learned.begin(start);learned.enter(SinglePass::Lift,start);
  learned.calibrated=true;
  learned.observed_extension_by_arm={{.55,.60}};
  learned.identified_extension_thrust={{2.,2.}};
  learned.identified_bend_target={{.9,1.1}};
  learned.onset_this_stroke={{true,true}};
  learned.lift_onset_seen={{true,true}};
  learned.powered_extension_reference_valid={{true,true}};
  learned.powered_extension_reference=start.s.bend;
  learned.powered_body_reference_valid=true;
  auto insufficient=probe;
  insufficient.s.bend={{1.2,1.4}};
  auto sample=learned.update(insufficient,c,learned_k,model,geometry);
  assert(!sample.fault && learned.phase==SinglePass::Lift);
  assert(learned.motion2_handoff_target_extension>.55);
  auto sufficient=insufficient;
  sufficient.s.bend={{.90,1.10}};
  sample=learned.update(sufficient,c,learned_k,model,geometry);
  assert(!sample.fault && learned.phase==SinglePass::Pull);
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

# The opt-in policy must remain off in all launch presets, including the
# previously successful human-arm configuration, and must reach the ROS node.
name = 'hybrid_motion2_paired_extension_handoff_enabled'
for launch in ('attitude_pressure_controller.launch', 'bringup.launch',
               'human_arm_locomotion.launch'):
    tree = ET.parse(root / 'launch' / launch)
    matching = [arg for arg in tree.findall('.//arg')
                if arg.get('name') == name]
    assert matching, launch
    if launch == 'attitude_pressure_controller.launch':
        assert any(arg.get('default') == 'false' for arg in matching)
        assert tree.find(".//param[@name='" + name + "']") is not None
    elif launch == 'human_arm_locomotion.launch':
        assert any(arg.get('default') == 'false' for arg in matching)
    assert any(arg.get('value') == '$(arg ' + name + ')' for arg in matching) \
        or launch == 'attitude_pressure_controller.launch'
assert name in (root / 'config' / 'attitude_pressure_controller.yaml').read_text()
