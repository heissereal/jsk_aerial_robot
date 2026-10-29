#!/usr/bin/env python3
"""Compile actual recovery geometry with synthetic measured bends, no ROS."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()

def function(name, result):
    start = source.index('AttitudePressureController::' + name + '(')
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return result + ' ' + source[start:end]

functions = '\n'.join(function(name, result) for name, result in [
    ('measuredRecoveryModel', 'hugmy::RollRecoveryModel'),
    ('recoverySelectedPairPressure', 'double'),
    ('recoveryRotorLever', 'double'),
    ('recoveryContactPivot', 'AttitudePressureController::Vector3'),
    ('recoveryRotorRawLeverAbout', 'double'),
    ('recoveryRotorRawLever', 'double'),
    ('rotorPositionFromAngle', 'AttitudePressureController::Vector3'),
    ('armContactPointFromAngle', 'AttitudePressureController::Vector3'),
    ('rotateAboutLocalY', 'AttitudePressureController::Vector3'),
    ('rotateLocalToBody', 'AttitudePressureController::Vector3'),
    ('cross', 'AttitudePressureController::Vector3'),
    ('wrapAngle', 'double'),
])
code = r'''
#include <hugmy/control/continuous_roll_recovery.h>
#include <hugmy/control/recovery_predictive_control.h>
#include <hugmy/control/circular_grasp_geometry.h>
#include <hugmy/model/pressure_thrust_bend_model.h>
#include <cassert>
#include <cstdio>
class AttitudePressureController {
public:
 using Vector3=std::array<double,3>;
 using PressureArray=std::array<double,4>;
 static constexpr size_t ARM_COUNT=4;
 hugmy::RollRecoveryConfig roll_recovery_config_;
 bool gravity_compensation_=true;
 bool correction_pair_reverse_spinal_mapping_=false;
 double roll_moment_kp_=.1, target_roll_=0., roll_=-.3, mass_kg_=1.244;
 double recovery_rotor_cant_rad_=6.5*M_PI/180., bottom_pressure_kpa_=1.;
 std::array<double,2> correction_axis_{{1.,0.}};
 std::array<bool,4> prepare_arm_active_{{false,false,true,true}};
 bool recovery_selected_pair_release_=true;
 double recovery_selected_pair_pressure_kpa_=0.;
 bool recovery_selected_contact_pivot_enabled_=true;
 std::array<double,4> measured_pressure_{{10,11,30,31}}, angles{{1.57,1.57,1.57,1.57}};
 std::array<double,4> grasp_pressure_{{30,30,30,30}};
 std::array<double,4> recovery_selected_release_start_pressure_{{30,30,30,30}};
 double recovery_selected_support_loss_moment_nm_=0.;
 double minimum_prepare_pressure_kpa_=0.,recovery_release_pressure_kpa_=0.;
 double base_pressure_kpa_=30.,maximum_pressure_kpa_=40.;
 double recovery_circle_contact_pressure_kpa_=10.;
 std::array<double,4> arm_yaw_{{-3*M_PI/4,-M_PI/4,M_PI/4,3*M_PI/4}};
 std::array<Vector3,4> arm_base_position_{{{{.03812,.03812,-.039}},{{-.03812,.03812,-.039}},
     {{-.03812,-.03812,-.039}},{{.03812,-.03812,-.039}}}};
 Vector3 center_of_mass_{{-.00348932,.00065510,.01200916}};
 Vector3 contact_position_{{0.,0.,-.07}}, gravity_body_{{0.,0.,-9.81}};
 double measuredArmBendAngle(size_t arm)const{return angles[arm];}
 Vector3 rotorPositionFromAngle(size_t,double)const;
 Vector3 armContactPointFromAngle(size_t,double)const;
 static Vector3 rotateAboutLocalY(const Vector3&,double);
 static Vector3 rotateLocalToBody(const Vector3&,double);
 static Vector3 cross(const Vector3&,const Vector3&);
 static double wrapAngle(double);
 hugmy::RollRecoveryModel measuredRecoveryModel()const;
 double recoverySelectedPairPressure(size_t)const;
 Vector3 recoveryContactPivot(const std::array<bool,4>&,bool,
     const PressureArray* = nullptr,const PressureArray* = nullptr)const;
 double recoveryRotorRawLeverAbout(size_t,double,const Vector3&)const;
 double recoveryRotorRawLever(size_t,double)const;
 double recoveryRotorLever(size_t,double)const;
};
FUNCTIONS
int main() {
 AttitudePressureController c;
 auto m=c.measuredRecoveryModel();
 assert(m.valid && m.lever>0.015 && m.moment>0.);
 // About the old fixed bottom pivot this pair has the opposite sign.  The
 // retained 0+1 contacts are the physical pivot when 2+3 are released.
 const double fixed_pair_lever=c.recoveryRotorRawLever(2,c.angles[2])+
     c.recoveryRotorRawLever(3,c.angles[3]);
 assert(fixed_pair_lever<0. && m.lever>0.);
 assert(m.context[0]==10 && m.context[4]==1 && m.context[5]==1.57);
 auto first=m;
 // Perched correction requests only additional restoring moment. Gravity,
 // mass/COM and the legacy flag cannot inflate the model thrust.
 assert(std::abs(first.moment-.1*(.3-c.roll_recovery_config_.stop))<1e-10);
 c.mass_kg_=10.;c.gravity_body_={{3.,5.,-8.}};c.center_of_mass_={{1.,2.,3.}};
 m=c.measuredRecoveryModel();assert(m.moment==first.moment);
 c.gravity_compensation_=false;m=c.measuredRecoveryModel();assert(m.moment==first.moment);
 c.gravity_compensation_=true;
 c.roll_=-22.58*M_PI/180.;m=c.measuredRecoveryModel();
 assert(std::abs(m.moment-.1*(22.58*M_PI/180.-c.roll_recovery_config_.stop))<1e-10);
 assert(m.moment>.0306 && m.moment<.0308); // bag c13: previously 0.415 Nm
 assert(m.moment/.017<1.81); // about 1.80 N, not gravity-saturated 4 N
 c.roll_=-.3;
 // Pressure does not change the measured bend, but it now moves the estimated
 // load-weighted CoP. The requested restoring moment itself is unchanged.
 c.measured_pressure_.fill(50);m=c.measuredRecoveryModel();
 assert(std::abs(m.lever-first.lever)>1e-6 && m.moment==first.moment);
 c.angles[0]=1.0;m=c.measuredRecoveryModel();assert(std::abs(m.lever-first.lever)>.001);
 // A larger measured load on one retained contact moves the CoP toward that
 // contact instead of using the old arithmetic midpoint.
 c.angles.fill(1.57);c.measured_pressure_={{10.,0.,30.,0.}};
 const auto weighted=c.recoveryContactPivot(c.prepare_arm_active_,true);
 c.measured_pressure_={{20.,0.,20.,0.}};
 const auto equal=c.recoveryContactPivot(c.prepare_arm_active_,true);
 assert(std::abs(weighted[1]-equal[1])>1e-4 ||
        std::abs(weighted[2]-equal[2])>1e-4);
 c.measured_pressure_.fill(30.);c.angles.fill(1.57);
 c.prepare_arm_active_={{false,false,true,true}};
 c.correction_axis_[0]=1.;c.roll_=-.3;
 const auto symmetric_reference=c.measuredRecoveryModel();
 c.roll_=.3;c.correction_axis_[0]=-1.;
 c.prepare_arm_active_={{true,true,false,false}};
 m=c.measuredRecoveryModel();assert(m.lever>.015);
 assert(std::abs(m.lever-symmetric_reference.lever)<1e-8); // mirrored correcting pair
 c.prepare_arm_active_={{false,false,true,true}};
 // Signed geometry rejects a pair which would drive away from the target.
 m=c.measuredRecoveryModel();assert(m.lever<0.);
 c.correction_axis_[0]=1.;c.gravity_compensation_=false;c.roll_=-.3;
 m=c.measuredRecoveryModel();assert(std::abs(m.moment-.1*(.3-c.roll_recovery_config_.stop))<1e-8);
 // Validate cant for each motor against an independent matrix calculation.
 for(size_t arm=0;arm<4;++arm) {
   c.prepare_arm_active_.fill(false);c.prepare_arm_active_[arm]=true;
   const double a=c.angles[arm],y=c.arm_yaw_[arm],cant=c.recovery_rotor_cant_rad_;
   const double sign=(arm==0||arm==2)?1.:-1.;
   const double fx=-std::sin(a)*std::cos(cant),fy=sign*std::sin(cant),fz=std::cos(a)*std::cos(cant);
   const double body_fy=std::sin(y)*fx+std::cos(y)*fy;
   auto pos=c.rotorPositionFromAngle(arm,a);
   const auto pivot=c.recoveryContactPivot(
       c.prepare_arm_active_,c.recovery_selected_pair_release_);
   double expected=(pos[1]-pivot[1])*fz-(pos[2]-pivot[2])*body_fy;
   m=c.measuredRecoveryModel();
   assert(std::abs(m.lever-expected*c.correction_axis_[0])<1e-10);
 }
 c.angles[0]=std::numeric_limits<double>::quiet_NaN();assert(!c.measuredRecoveryModel().valid);
 // The same production kinematics infer a plausible 50-mm-class circle and
 // classify a 105-deg grasp as just below the object side.
 std::array<std::array<double,2>,4> contact{};
 for(size_t arm=0;arm<4;++arm) {
   const auto point=c.armContactPointFromAngle(arm,105.*M_PI/180.);
   contact[arm]={{point[1],point[2]}};
 }
 const auto circle=hugmy::estimateCircularGrasp(contact,
     {{true,true,true,true}},{{0.,-.07}},{{0.,-9.81}},.025,.15,.015);
 assert(circle.valid && circle.radius>.035 && circle.radius<.050);
 for(double depth:circle.depth_angle)assert(depth>M_PI/2);
 // Exercise the real forward geometry and calibration table with the planner.
 hugmy::PressureThrustBendModel table;
 hugmy::RecoveryPredictiveConfig config;config.budget_ms=1000.;
 for(double sign:{-1.,1.}) {
   c.correction_axis_[0]=sign;
   hugmy::RecoveryPredictiveControl planner;hugmy::RecoveryPredictiveState input;
   input.error=.4;input.bend={{1.4,1.4}};
   auto equilibrium=[&](size_t,double p,double f){return table.angleRad(p,f+1.8);};
   auto geometry=[&](size_t j,double a){return c.recoveryRotorLever((sign>0?0:2)+j,a);};
   const auto begin=std::chrono::steady_clock::now();
   auto plan=planner.update(input,config,c.roll_recovery_config_,equilibrium,geometry);
   const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
   assert(plan.feasible && std::isfinite(plan.score) && plan.thrust>=0. && plan.thrust<=.3+1e-9);
   printf("Production geometry + table: sign %.0f, first thrust %.2f N, feasible %u/%u, %.2f ms/plan (unoptimized exhaustive test)\n",
          sign,plan.thrust,plan.evaluated-plan.rejected,plan.evaluated,ms);
 }
 puts("PASS: measured-angle restoring-only model, gravity independence, c13 moment, pair lever, cant signs and invalid geometry");
}
'''.replace('FUNCTIONS', functions)
with tempfile.TemporaryDirectory(prefix='hugmy_recovery_geometry_') as tmp:
    binary = str(pathlib.Path(tmp) / 'test')
    subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                    '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
    subprocess.run([binary], check=True)
