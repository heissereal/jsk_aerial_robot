#!/usr/bin/env python3
"""Regression for the supplied baseline diff, using production C++ functions.

No ROS master, bag playback or actuator writes. Checks launch/YAML defaults,
actual calibrated-table pressure selection, J-based best selection and all
three coordinate-search variables. Does not claim physical gait equivalence.
"""
import math
import pathlib
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    header = (root / 'include/hugmy/control/attitude_pressure_controller.h').read_text()
    config = yaml.safe_load((root / 'config/attitude_pressure_controller.yaml').read_text())
    for name in ('reference_pulse_mode', 'pulse_search_enabled',
                 'individual_extension_enabled', 'online_pressure_adaptation_enabled'):
        assert config['action_c_' + name] is False, name
        assert re.search(r'pnh_\.param\("action_c_' + name +
                         r'",\s*action_c_' + name + r'_, false\)', source), name
    assert config['action_c_pressure_pull_enabled'] is True
    assert 'const bool pressure_pull_mode = action_c_pressure_pull_enabled_;' in source
    assert config['action_c_optimizer_enabled'] is True
    assert config['action_c_optimize_bottom_enabled'] is True
    assert config['action_c_extension_sync_kp_n_rad'] == 0
    assert config['action_c_objective_recovery_weight'] == .030
    assert math.isclose(config['action_c_max_extension_mismatch_rad'], math.radians(15))
    for filename in ('bringup.launch', 'attitude_pressure_controller.launch'):
        args = {a.attrib['name']: a.attrib.get('default')
                for a in ET.parse(root / 'launch' / filename).getroot().findall('arg')}
        assert args['action_c_reference_pulse_mode'] == 'false'
        assert args['action_c_pulse_search_enabled'] == 'false'
    # Recovery is independent: preserve anti-overshoot prediction/slowdown.
    assert config['correction_roll_only'] is True
    assert math.isclose(config['correction_slowdown_angle_rad'], math.radians(10), abs_tol=1e-7)
    assert config['correction_prediction_horizon_sec'] == .20
    assert config['correction_max_thrust_n'] == 4.
    assert 'attitude_error - std::max(0.0, correction_omega)' in source
    assert 'motion_hold_thrust_[arm] = prepare_shape_thrust_' not in source  # whole array capture, not a new gait loop
    # Predictions must remain diagnostic, never invoke recovery themselves.
    pred = source[source.index('if (action_c_adaptive_burst_enabled_ && !target_reached &&'):]
    pred = pred[:pred.index('enterState(SETTLE, now);')]
    assert 'requestActionCPostureRecovery(' not in pred

    start = source.index('double best_pressure =', source.index('case ACTION_C_PREPARE:'))
    feedforward = source[start:source.index('// Once one pulse has been observed', start)]
    start = source.index('void AttitudePressureController::selectNextActionCCandidate()')
    end = source.index('void AttitudePressureController::requestActionCPostureRecovery(', start)
    methods = source[start:end]
    helper_start = source.index('bool AttitudePressureController::finishActionCPropulsionTrial(')
    helper_end = source.index('void AttitudePressureController::observeActionCPropulsion(', helper_start)
    methods += source[helper_start:helper_end]
    assert 'void AttitudePressureController::finishActionCOptimizerTrial(' in methods
    fields = sorted(set(re.findall(r'\baction_c_\w+_\b', methods)))
    declarations = []
    for field in fields:
        if field == 'action_c_propulsion_':
            declarations.append('hugmy::ActionCPropulsion action_c_propulsion_;')
            continue
        if field == 'action_c_coordination_':
            declarations.append('hugmy::ActionCCoordination action_c_coordination_;')
            continue
        if field == 'action_c_thrust_safety_':
            declarations.append('hugmy::ActionCThrustSafety action_c_thrust_safety_;')
            continue
        match = re.search(r'\b(unsigned int|bool|int|double|size_t|PressureArray)\s+[^;{}]*?\b'
                          + re.escape(field) + r'\b', header)
        assert match, field
        declarations.append(f'{match[1]} {field}{{}};')
    # Use YAML objective weights, not hand-copied test-only values.
    assignments = []
    for field in fields:
        key = field[:-1]
        if key in config and type(config[key]) in (bool, int, float):
            value = str(config[key]).lower()
            assignments.append(f'c.{field}={value};')
    code = r'''
#include <hugmy/control/action_c_feedback.h>
#include <hugmy/control/action_c_coordination.h>
#include <hugmy/control/action_c_propulsion.h>
#include <hugmy/model/pressure_thrust_bend_model.h>
#include <array>
#include <cassert>
#include <cstdio>
#include <limits>
#include <string>
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
struct Params {
 double last_score=0.;
 void setParam(const char* key, double value) {
   if(std::string(key)=="action_c_optimizer_last_score") last_score=value;
 }
 template<class T> void setParam(const char*, const T&) {}
};
class AttitudePressureController {
public:
using PressureArray=std::array<double,4>;
FIELDS
Params pnh_;
std::array<double,3> mocap_position_{{10.,0.,0.}};
std::array<bool,4> inchworm_front_{{true,false,false,true}};
void selectNextActionCCandidate();
void finishActionCOptimizerTrial(bool recovery_required,const char* reason);
bool finishActionCPropulsionTrial(bool,double);
};
METHODS
double selectedPressure(double thrust, bool timed=false, bool adaptation=false, double bias=0., bool release=true) {
 using Trend=hugmy::PressureThrustBendModel::Trend;
 hugmy::PressureThrustBendModel bend_model_;
 const double maximum_pressure_kpa_=40., action_c_max_pressure_kpa_=50.;
 const double action_c_thrust_n_=thrust, bend_model_thrust_offset_n_=1.8;
 const double action_c_target_angle_rad_=M_PI/2, action_c_front_pulse_pressure_kpa_=5.;
 const bool action_c_online_pressure_adaptation_enabled_=adaptation;
 const bool action_c_reference_pulse_mode_=timed;
 const bool action_c_front_release_enabled_=release;
 const size_t arm=0;
 const std::array<double,4> action_c_pressure_bias_kpa_{{bias,0,0,0}};
 FEEDFORWARD
 return selected_pressure;
}
AttitudePressureController initial() {
 AttitudePressureController c;
 CONFIG
 c.action_c_efficiency_objective_enabled_=false; // Explicit legacy A/B mode.
 c.action_c_coordination_enabled_=false;
 c.action_c_adaptive_burst_enabled_=true;
 c.action_c_thrust_n_=3.; c.action_c_bottom_hold_kpa_=30.;
 c.action_c_optimizer_burst_pulses_=1;
 c.action_c_optimizer_best_score_=-std::numeric_limits<double>::infinity();
 c.action_c_optimizer_safe_thrust_max_n_=6.;c.action_c_thrust_safety_.reset(6.);
 c.action_c_optimizer_thrust_step_n_=.25;c.action_c_optimizer_bottom_step_kpa_=5.;
 c.action_c_direction_sign_x_=1.;
 c.action_c_optimizer_completed_pulses_=c.action_c_optimizer_valid_pulses_=1;
 c.action_c_optimizer_successful_pulses_=1;
 c.action_c_optimizer_target_progress_m_=.002;
 c.action_c_optimizer_trial_active_=true;
 return c;
}
int main() {
 // Disabling online adaptation still performs a real table lookup at thrust.
 const double p3=selectedPressure(3.,false,false,0.,false), p4=selectedPressure(4.,false,false,0.,false);
 assert(p3>5.1 && std::abs(p3-p4)>.1);
 assert(selectedPressure(3.,false,false,15.,false)==p3); // rejected online bias
 assert(std::abs(selectedPressure(3.,false,true,2.,false)-p3-2.)<1e-9);
 assert(selectedPressure(3.)==0. && selectedPressure(6.)==0.);
 assert(selectedPressure(3.,false,true,15.)==0.); // no pressure preload override
 assert(selectedPressure(3.,true)==5.); // explicit experiment only
 auto c=initial();
 assert(!c.action_c_reference_pulse_mode_ && !c.action_c_individual_extension_enabled_);
 assert(c.action_c_optimize_bottom_enabled_);
 c.action_c_optimizer_retained_progress_m_=.010;
 c.finishActionCOptimizerTrial(false,"baseline");
 assert(std::abs(c.action_c_optimizer_best_score_-.010)<1e-10);
 assert(c.action_c_optimizer_best_thrust_n_==3.);
 assert(c.action_c_optimizer_best_bottom_kpa_==30.);
 // The J-selected best, not the latest trial, anchors a bottom probe.
 assert(c.action_c_bottom_hold_kpa_==25. && c.action_c_thrust_n_==3.);
 c.action_c_optimizer_trial_active_=true;
 c.action_c_thrust_n_=4.;c.action_c_bottom_hold_kpa_=25.;
 c.action_c_optimizer_retained_progress_m_=.020;
 c.finishActionCOptimizerTrial(true,"more motion but recovery");
 assert(std::abs(c.action_c_optimizer_best_score_-.010)<1e-10); // .020-.030 loses
 assert(c.action_c_optimizer_best_thrust_n_==3.);
 c.action_c_optimizer_trial_active_=true;
 c.action_c_thrust_n_=4.;c.action_c_bottom_hold_kpa_=25.;
 c.finishActionCOptimizerTrial(false,"same motion without recovery");
 assert(std::abs(c.action_c_optimizer_best_score_-.020)<1e-10);
 assert(c.action_c_optimizer_best_thrust_n_==4.);
 assert(c.action_c_optimizer_best_bottom_kpa_==25.);
 // Burst count is an active search coordinate, not permanently frozen at 1.
 c.action_c_optimizer_direction_index_=4;
 c.selectNextActionCCandidate();
 assert(c.action_c_optimizer_burst_pulses_==2);
 // A large raw mocap displacement cannot replace retained cycle progress.
 c.action_c_optimizer_trial_active_=true;
 c.action_c_optimizer_retained_progress_m_=0.;c.mocap_position_[0]=100.;
 c.finishActionCOptimizerTrial(false,"raw drift only");
 assert(std::abs(c.action_c_optimizer_best_score_-.020)<1e-10);
 // Table feed-forward changes are not online pressure-sign learning.
 auto e=initial();e.action_c_efficiency_objective_enabled_=true;
 e.action_c_optimizer_clock_valid_=true;e.action_c_optimizer_elapsed_sec_=2;
 e.action_c_optimizer_retained_progress_m_=.02;
 e.finishActionCOptimizerTrial(false,"efficient");
 assert(std::abs(e.action_c_optimizer_best_score_-.01)<1e-10);
 // No extension: do not learn/re-evaluate or probe bottom around tiny drift.
 auto stalled=initial();stalled.action_c_efficiency_objective_enabled_=true;
 stalled.action_c_coordination_enabled_=true;stalled.action_c_optimizer_clock_valid_=true;
 stalled.action_c_optimizer_elapsed_sec_=10.;
 stalled.action_c_optimizer_successful_pulses_=0;
 stalled.action_c_optimizer_retained_progress_m_=.00001;
 stalled.action_c_coordination_.startPulse();
 // Even accidental staged calibration cannot commit an unestablished trial.
 stalled.action_c_coordination_.observePulse({{true,false,false,true}},{{.03,0,0,.14}},.14,true);
 stalled.finishActionCOptimizerTrial(false,"unestablished");
 assert(!std::isfinite(stalled.action_c_optimizer_best_score_));
 assert(stalled.action_c_coordination_.rounds==0);
 assert(stalled.action_c_thrust_n_==3.25 && stalled.action_c_bottom_hold_kpa_==30.);
 assert(std::abs(stalled.pnh_.last_score-(.00001/10.-.010*(1.-.00001/.002)))<1e-10);
 // Deficit cost is not divided by duration; retreat costs more, excess has no penalty.
 auto scoreAt=[](double dx,double seconds) {
   auto x=initial();x.action_c_efficiency_objective_enabled_=true;
   x.action_c_optimizer_clock_valid_=true;x.action_c_optimizer_elapsed_sec_=seconds;
   x.action_c_optimizer_retained_progress_m_=dx;
   x.finishActionCOptimizerTrial(false,"objective test");return x.pnh_.last_score;
 };
 assert(std::abs(scoreAt(0.,1.)+.010)<1e-10);
 assert(std::abs(scoreAt(0.,100.)+.010)<1e-10);
 assert(scoreAt(-.001,10.)<scoreAt(0.,10.));
 assert(std::abs(scoreAt(.001,10.)-(.0001-.005))<1e-10);
 assert(std::abs(scoreAt(.004,10.)-.0004)<1e-10);
 e.action_c_optimizer_trial_active_=true;e.action_c_optimizer_recovery_pending_=true;
 e.action_c_optimizer_elapsed_sec_=10;e.action_c_optimizer_recovery_sec_=8;
 e.action_c_optimizer_recovery_start_x_m_=10.01; // Recovery slipped back 10 mm.
 e.action_c_optimizer_retained_progress_m_=.02;
 e.finishActionCOptimizerTrial(true,"slow recovery and slip");
 assert(std::abs(e.action_c_optimizer_best_score_-.01)<1e-10);
 e.action_c_optimizer_trial_active_=true;e.action_c_optimizer_clock_valid_=false;
 e.action_c_optimizer_retained_progress_m_=1.;
 e.finishActionCOptimizerTrial(false,"clock reset must not win");
 assert(std::abs(e.action_c_optimizer_best_score_-.01)<1e-10);
 e.action_c_optimizer_trial_active_=true;e.action_c_optimizer_clock_valid_=true;
 e.action_c_optimizer_trial_start_x_m_=10.01;e.mocap_position_[0]=10.;
 e.action_c_optimizer_elapsed_sec_=1;e.action_c_optimizer_recovery_sec_=0;
 e.action_c_optimizer_retained_progress_m_=1.;
 e.finishActionCOptimizerTrial(false,"settle slip must not earn positive credit");
 assert(std::abs(e.action_c_optimizer_best_score_-.01)<1e-10);
 // Actual controller commit: same T/P, single pulse, no optimizer NEXT.
 auto learning=initial();learning.action_c_coordination_enabled_=true;
 learning.action_c_optimizer_clock_valid_=true;
 learning.action_c_optimizer_retained_progress_m_=.02;
 learning.action_c_coordination_.startPulse();
 learning.action_c_coordination_.observePulse({{true,false,false,true}},
     {{.03,0,0,.14}},.14,true);
 learning.finishActionCOptimizerTrial(false,"calibration boundary");
 assert(learning.action_c_coordination_.offset[0]>0);
 assert(learning.action_c_thrust_n_==3. && learning.action_c_bottom_hold_kpa_==30.);
 assert(learning.action_c_optimizer_burst_pulses_==1);
 assert(!std::isfinite(learning.action_c_optimizer_best_score_));
 const auto frozen=learning.action_c_coordination_.offset;
 learning.action_c_optimizer_trial_active_=true;
 learning.action_c_coordination_.observePulse({{true,false,false,true}},
     {{.03,0,0,.14}},.14,true);
 learning.finishActionCOptimizerTrial(true,"recovery rejects pending calibration");
 assert(learning.action_c_coordination_.offset==frozen);
 // Production arbitration: a complete under-goal pull raises ONLY residual,
 // not the ON amplitude/bottom/burst, even when old calibration is pending.
 auto pull=initial();pull.action_c_optimizer_clock_valid_=true;
 pull.action_c_coordination_enabled_=true;
 pull.action_c_optimizer_retained_progress_m_=.001;
 pull.action_c_optimizer_successful_pulses_=0;
 pull.action_c_propulsion_.beginTrial();pull.action_c_propulsion_.beginPulse();
 pull.action_c_propulsion_.force(1,1);pull.action_c_propulsion_.force(1.02,1);
 pull.action_c_propulsion_.complete(true);pull.action_c_propulsion_.regrip(true);
 pull.action_c_coordination_.observePulse(pull.inchworm_front_,{{.03,0,0,.14}},.14,true);
 pull.finishActionCOptimizerTrial(false,"completed pull below goal");
 assert(std::abs(pull.action_c_propulsion_.extra_n-.1)<1e-9);
 assert(pull.action_c_thrust_n_==3. && pull.action_c_bottom_hold_kpa_==30.);
 assert(pull.action_c_optimizer_burst_pulses_==1);
 assert(pull.action_c_coordination_.rounds==0 && !std::isfinite(pull.action_c_optimizer_best_score_));
 auto timing=initial();timing.action_c_optimizer_clock_valid_=true;
 timing.action_c_optimizer_retained_progress_m_=.001;
 timing.action_c_propulsion_.completed_pulls=1;
 timing.action_c_propulsion_.trial_forward_impulse=.1;
 timing.action_c_propulsion_.response[0].pending=.12;
 timing.finishActionCOptimizerTrial(false,"timing before strength");
 assert(timing.action_c_propulsion_.response[0].lead==.03);
 assert(timing.action_c_propulsion_.extra_n==0 && timing.action_c_thrust_n_==3.);
 // Actual failed-motion path: pressure timing can learn without a positive
 // safe baseline; amplitude learning and old coordination still cannot.
 auto identification=initial();identification.action_c_optimizer_clock_valid_=true;
 identification.action_c_optimizer_retained_progress_m_=-.002;
 identification.action_c_optimizer_valid_pulses_=0;
 identification.action_c_optimizer_successful_pulses_=0;
 identification.action_c_optimizer_trial_had_extension_guard_=true;
 identification.action_c_optimizer_trial_guard_reasons_=hugmy::ACTION_C_EXTENSION_RATE;
 identification.action_c_propulsion_.response[3].pending=.2;
 identification.finishActionCOptimizerTrial(false,"zero-thrust pressure identification after guard");
 assert(identification.action_c_propulsion_.response[3].lead==.03);
 assert(identification.action_c_propulsion_.extra_n==0 && identification.action_c_thrust_n_==3.);
 assert(identification.action_c_thrust_safety_.hard_max_n==3.); // do not defeat hard cap
 assert(identification.action_c_optimizer_successful_pulses_==0);
 auto recovered=initial();recovered.action_c_optimizer_clock_valid_=true;
 recovered.action_c_optimizer_retained_progress_m_=.001;
 recovered.action_c_propulsion_.completed_pulls=1;
 recovered.action_c_propulsion_.trial_forward_impulse=.1;
 recovered.action_c_propulsion_.response[0].pending=.12;
 recovered.finishActionCOptimizerTrial(true,"no adaptation after recovery");
 assert(recovered.action_c_propulsion_.response[0].lead==0 && recovered.action_c_propulsion_.extra_n==0);
 printf("PASS: restored defaults, table feed-forward %.1f/%.1f kPa at 3/4 N, J-based best and thrust/bottom/burst search\n",p3,p4);
}
'''
    code = code.replace('FIELDS', '\n'.join(declarations)).replace('METHODS', methods)
    code = code.replace('CONFIG', '\n'.join(assignments)).replace('FEEDFORWARD', feedforward)
    with tempfile.TemporaryDirectory(prefix='hugmy_restored_optimizer_') as directory:
        binary = str(pathlib.Path(directory) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                        '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
        subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
