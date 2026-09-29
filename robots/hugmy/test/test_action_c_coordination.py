#!/usr/bin/env python3
"""Test the production residual learner, arbitration and efficiency helpers.

No ROS master or actuator access. C++ code is compiled from the actual header.
"""
import pathlib
import subprocess
import tempfile
import yaml


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'config/attitude_pressure_controller.yaml').read_text())
    assert config['action_c_coordination_enabled']
    assert config['action_c_efficiency_objective_enabled']
    assert config['action_c_max_thrust_n'] == 5.0
    assert config['action_c_coordination_lag_kp_n_rad'] == 6.0
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    # Single ownership and boundaries, including zero assist in experimental modes.
    assert 'action_c_reference_pulse_mode_ || action_c_individual_extension_enabled_' in source
    assert 'action_c_online_pressure_adaptation_enabled_ || !action_c_adaptive_burst_enabled_' in source
    assert 'reduction > 1e-6 ? 0.0 : coordinated_assist[arm]' in source
    assert 'action_c_coordination_rebend_target_[arm] : std::min(' in source
    finish = source[source.index('void AttitudePressureController::finishActionCOptimizerTrial('):]
    finish = finish[:finish.index('void AttitudePressureController::requestActionCPostureRecovery(')]
    assert finish.index('const double score =') < finish.index('action_c_coordination_.finishTrial(')
    assert finish.index('same T/P re-evaluation') < finish.index('selectNextActionCCandidate();')
    assert 'action_c_optimizer_valid_pulses_ == action_c_optimizer_completed_pulses_' in finish
    assert 'action_c_optimizer_recovery_sec_ += clock_step' in source
    start = source.index('const double extra = reduction > 1e-6')
    end = source.rfind('if (', start, source.index('action_c_coordination_.earlyPressure', start))
    output = source[start:end]
    code = r'''
#include <hugmy/control/action_c_coordination.h>
#include <hugmy/control/action_c_feedback.h>
#include <cassert>
#include <cstdio>
#include <numeric>
double outputAt(double previous_thrust,double hard,double base,double assist,double reduction=0.) {
 const size_t arm=0;
 const double action_c_max_thrust_n_=5., action_c_thrust_down_rate_n_s_=30., action_c_thrust_ramp_n_s_=8.;
 const double common_front_thrust=base;
 hugmy::ActionCThrustSafety action_c_thrust_safety_;action_c_thrust_safety_.reset(hard);
 action_c_thrust_safety_.pair_max_n=3.; // Pair cap must not erase lagging-arm assist.
 std::array<double,4> coordinated_assist{{assist,0,0,0}}, inchworm_thrust_{};
 auto approach=[](double x,double y,double rate){return x<y?std::min(y,x+rate*.02):std::max(y,x-rate*.02);};
 OUTPUT
 return inchworm_thrust_[0];
}
int main() {
 assert(std::abs(outputAt(3.,5.,3.,.3)-3.16)<1e-9);
 assert(std::abs(outputAt(3.16,5.,3.,.3)-3.3)<1e-9);
 assert(outputAt(3.,3.,3.,.3)==3.); // Physical guard limit still blocks extra thrust.
 assert(outputAt(5.,5.,5.,.3)==5.); // 5 N includes feedback.
 assert(outputAt(3.,5.,3.,.3,.1)<3.); // Do not fight angle/roll braking.
  using C=hugmy::ActionCCoordination;
  const double deg=M_PI/180, target=8*deg;
  std::array<bool,4> front{{true,false,false,true}};
  C::Four extension{{2*deg,0,0,8*deg}}, pressure{{20,0,0,20}};
  C c; c.startPulse();
  auto extra=c.assist(front,extension,target,0,0,0,10*deg,.3,8);
  assert(extra[0]>0 && extra[3]==0 && extra[1]==0 && extra[2]==0);
  assert(std::accumulate(extra.begin(),extra.end(),0.)<=.3);
  assert(std::abs(extra[0]-.3)<1e-9); // Previously only about 0.105 N for this lag.
  auto high_roll=c.assist(front,extension,target,8*deg,0,0,10*deg,.3,8);
  auto high_rate=c.assist(front,extension,target,0,.3,0,10*deg,.3,8);
  auto high_tilt=c.assist(front,extension,target,0,0,5*deg,10*deg,.3,8);
  assert(high_roll[0]==0 && high_rate[0]==0 && high_tilt[0]==0);
  auto invalid=extension; invalid[3]=NAN;
  assert(c.assist(front,invalid,target,0,0,0,10*deg,.3,8)[0]==0);
  std::array<bool,4> reverse{{false,true,true,false}};
  assert(c.assist(reverse,{{0,2*deg,8*deg,0}},target,0,0,0,10*deg,.3,8)[1]>0);
  // Empirical command-to-half-pressure rise; immutable until trial commit.
  c.startPressure(0,10,20,30); c.startPressure(3,10,20,30);
  c.observePressure(10.05,pressure,true); assert(c.observed_lag[0]<0);
  pressure[0]=26; c.observePressure(10.2,pressure,true);
  pressure[3]=26; c.observePressure(10.4,pressure,true);
  c.observePulse(front,extension,target,true);
  assert(c.offset[0]==0 && c.lead[0]==0);
  assert(c.finishTrial(true));
  assert(c.offset[0]>0 && c.offset[0]<=.05 && c.offset[3]==0);
  assert(c.lead[0]<=.030001 && c.lead[3]<=.030001);
  assert(!c.earlyPressure(0,.59,.6,2*deg,target,-1)); // Not yet extended.
  assert(!c.earlyPressure(0,.05,.6,7*deg,target,-1)); // Too early in pulse.
  assert(c.earlyPressure(0,.59,.6,5*deg,target,-1));
  assert(c.earlyPressure(0,.25,.6,7*deg,target,-1)); // Angle, not only fixed time.
  assert(!c.earlyPressure(0,.25,.6,5*deg,target,NAN));
  // Faulted pressure traces remain censored, even after fresh samples return.
  c.startPulse(); c.startPressure(0,20,20,30);
  c.observePressure(20.1,pressure,false); c.observePressure(20.2,pressure,true);
  assert(c.observed_lag[0]==-2);
  c.startPressure(0,20.3,20,30); assert(c.pressure_start[0]==20);
  auto offsets=c.offset; auto leads=c.lead;
  c.observePulse(front,extension,target,true);
  assert(!c.finishTrial(false)); // recovery/guard => no contaminated update
  assert(c.offset==offsets && c.lead==leads);
  c.observePulse(front,extension,target,false); assert(!c.finishTrial(true));
  // Three bounded learning rounds, then freeze so the optimizer can finish.
  for(int n=0;n<8;++n) {
    c.startPulse();c.startPressure(0,30,20,30);
    c.observePressure(30.2,pressure,true);
    c.observePulse(front,extension,target,true); c.finishTrial(true);
  }
  assert(c.rounds==3 && c.lead[0]<=.12);
  assert(std::accumulate(c.offset.begin(),c.offset.end(),0.)<=.300001);
  offsets=c.offset;leads=c.lead;
  c.observePulse(front,{{8*deg,0,0,2*deg}},target,true);
  assert(!c.finishTrial(true));assert(offsets==c.offset && leads==c.lead);
  // Neither arm moved: do not learn spurious asymmetry or block completion.
  C still;still.startPulse();still.observePulse(front,{{0,0,0,0}},target,true);
  assert(!still.finishTrial(true));assert(still.offset[0]==0 && still.offset[3]==0);
  // Changing burst count cannot win merely by collecting more total motion.
  const auto score=[](double dx,double t,double effort=0,double rt=0) {
    return hugmy::actionCEfficiencyScore(dx,t,effort,.0001,rt,.001);
  };
  assert(std::abs(score(.02,2)-score(.04,4))<1e-10);
  assert(score(.02,2)>score(.02,10,0,8));
  assert(score(.0001,2)<score(.02,2));
  assert(score(.02,2,10)<score(.02,2));
  assert(!std::isfinite(score(.02,0)) && !std::isfinite(score(NAN,2)));
  assert(std::abs(hugmy::actionCRetainedAfterRecovery(.02,true,-.01)-.01)<1e-10);
  assert(hugmy::actionCRetainedAfterRecovery(.02,true,.01)==.02);
  assert(hugmy::actionCRetainedAfterRecovery(0,true,.02)==0);
  std::puts("PASS: bounded pair assist, reverse mapping, brake priority wiring, pressure delay/lead gates, trial freeze/commit, censored faults, finite calibration, recovery-inclusive efficiency");
}
'''.replace('OUTPUT', output)
    with tempfile.TemporaryDirectory(prefix='hugmy_coordination_') as directory:
        binary = str(pathlib.Path(directory) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                        '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
        subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
