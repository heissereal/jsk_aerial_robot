#!/usr/bin/env python3
"""Compile/run scenario-MPC and downstream guards offline; no ROS actuation."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
branch = source[source.index('case PREPARE:'):source.index('case THRUST_CORRECTION:')]
assert 'if(!config.shadow)' in branch
assert 'continuous_roll_recovery_.fault=true;' in branch
assert 'predictive_target = 0.0;' in branch
assert 'braking this cycle and replanning from measured contact state' in branch
assert 'predictive_target,' in branch
assert 'recovery_response_faults_enabled &&' in branch
assert 'recovery_wrong_geometry_response_observed_' in branch
assert 'geometry_authority_limit)' in branch
assert 'recovery_predictive_.reset();' in source
assert 'recoveryRotorLever(arms[j],angle)' in branch
assert 'force+bend_model_thrust_offset_n_' in branch

code = r'''
#include <hugmy/control/recovery_predictive_control.h>
#include <hugmy/model/pressure_thrust_bend_model.h>
#include <cassert>
#include <chrono>
#include <cstdio>
using namespace hugmy;
int main() {
  RecoveryPredictiveConfig cfg;RollRecoveryConfig bounds;
  cfg.budget_ms=1000.; // deterministic exhaustive test, independent of test-host load
  assert(cfg.shadow && cfg.enabled && bounds.limit==4.);
  RecoveryPredictiveState s;s.error=.4;s.bend={{1.4,1.4}};
  auto eq=[](size_t,double p,double f){return recoveryClamp(1.4+.02*p-.65*f,0.,2.9);};
  auto lever=[](size_t,double a){return .06*std::cos(a);};
  RecoveryPredictiveControl planner;
  auto plan=planner.update(s,cfg,bounds,eq,lever);
  assert(plan.feasible && plan.evaluated==216 && plan.rejected<plan.evaluated);
  assert(plan.thrust>0. && plan.thrust<=cfg.thrust_step*.5+1e-9);
  assert(plan.pressure[0]>=0. && plan.pressure[0]<=2.);
  assert(plan.pressure[1]>=0. && plan.pressure[1]<=2.);
  // Current wrong-sign lever does not discard future extension trajectories.
  auto changing=[](size_t,double a){return .04-.04*a;};
  RecoveryPredictiveControl changing_planner;
  auto changing_plan=changing_planner.update(s,cfg,bounds,eq,changing);
  assert(changing_plan.feasible && changing_plan.thrust>0.);
  // Cached first move; malformed state / clock rollback cannot replay it.
  s.now=.02;auto cached=planner.update(s,cfg,bounds,eq,lever);
  assert(cached.thrust==plan.thrust);
  s.now=-.01;assert(!planner.update(s,cfg,bounds,eq,lever).feasible);
  s.now=0.;s.bend[0]=std::numeric_limits<double>::quiet_NaN();
  assert(!planner.update(s,cfg,bounds,eq,lever).feasible);
  s.bend[0]=1.4;
  auto bad=[](size_t,double,double){return std::numeric_limits<double>::quiet_NaN();};
  assert(!planner.update(s,cfg,bounds,bad,lever).feasible);
  planner.reset();auto bad_lever=[](size_t,double){return std::numeric_limits<double>::quiet_NaN();};
  assert(!planner.update(s,cfg,bounds,eq,bad_lever).feasible);
  planner.reset();auto expired=cfg;expired.budget_ms=1e-12;
  auto deadline=planner.update(s,expired,bounds,eq,lever);
  assert(deadline.budget_exhausted && !deadline.feasible);
  planner.reset();auto invalid_config=cfg;
  invalid_config.moment_gain=std::numeric_limits<double>::quiet_NaN();
  assert(!planner.update(s,invalid_config,bounds,eq,lever).feasible);
  // No feasible trajectory is distinct from a feasible zero-force command.
  planner.reset();s.rate=-.75;
  assert(!planner.update(s,cfg,bounds,eq,lever).feasible);
  s.rate=0.;
  // Downstream controller tests actual actuator output, not just predictions.
  RollRecoveryModel model;model.valid=true;model.moment=.1;model.lever=.1;
  RollRecoveryResponse response;
  ContinuousRollRecovery actual;actual.reset(.4,0.);
  actual.thrust=2.;actual.boost=1.;
  const double output=actual.update(.4,.3,.02,.02,model,bounds,response,4.);
  assert(output<2. && actual.speed_braking && actual.boost==1.);
  actual.update(.4,.81,.04,.02,model,bounds,response,4.);
  assert(actual.fault && actual.thrust==0.);
  actual.reset(.4,0.);assert(actual.update(.4,0.,.02,.02,model,bounds,response,100.)<=.030001);
  actual.reset(.4,0.);actual.thrust=1.;
  // Include both the 1-N thrust rundown and the configured low-rate settle
  // dwell; entering the angular band alone is deliberately not completion.
  for(int i=1;i<=25;++i)actual.update(.01,0.,i*.02,.02,model,bounds,response,4.);
  assert(actual.finished && actual.thrust==0.);
  // Shadow calculations cannot alter baseline FF/PI state or actuator output.
  ContinuousRollRecovery base,shadow;base.reset(.4,0.);shadow.reset(.4,0.);
  RollRecoveryResponse rb,rs;RecoveryPredictiveControl passive;
  for(int i=1;i<120;++i) {
    s.now=i*.02;s.thrust=shadow.thrust;
    passive.update(s,cfg,bounds,eq,lever); // result deliberately not applied
    const double a=base.update(.4,0.,s.now,.02,model,bounds,rb);
    const double b=shadow.update(.4,0.,s.now,.02,model,bounds,rs);
    assert(a==b && base.boost==shadow.boost);
  }
  // Bounded local adaptation and reset on likely disturbance/contact change.
  planner.reset();s=RecoveryPredictiveState();s.error=.4;s.bend={{1.4,1.4}};
  planner.update(s,cfg,bounds,eq,lever);
  for(int i=1;i<=10;++i){s.now=i*.1;planner.update(s,cfg,bounds,eq,lever);}
  assert(planner.accepted_observations>0 && planner.confidence>0.);
  s.now+=.1;s.rate=.3;planner.update(s,cfg,bounds,eq,lever);
  assert(planner.confidence==0. && planner.accepted_observations==0);
  planner.reset();assert(planner.bend_bias[0]==0. && !planner.initialized);
  // Real calibration table, including existing 1.8-N coordinate offset.
  PressureThrustBendModel table;
  auto calibrated=[&](size_t,double p,double f){return table.angleRad(p,f+1.8);};
  s=RecoveryPredictiveState();s.error=.4;s.bend={{1.4,1.4}};
  const auto begin=std::chrono::steady_clock::now();
  for(int i=0;i<20;++i){planner.reset();plan=planner.update(s,cfg,bounds,calibrated,lever);}
  const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()/20.;
  assert(plan.feasible);
  printf("PASS: scenario search, future extension, bounded first move, pressure candidates, shadow isolation, adaptation/reset, infeasible/fault/brake priority; %.2f ms/plan (this PC)\n",ms);
}
'''
with tempfile.TemporaryDirectory(prefix='hugmy_predictive_') as tmp:
    binary = str(pathlib.Path(tmp) / 'test')
    subprocess.run(['g++', '-std=c++14', '-O2', '-Wall', '-Wextra', '-I', str(root / 'include'),
                    '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
    subprocess.run([binary], check=True)
