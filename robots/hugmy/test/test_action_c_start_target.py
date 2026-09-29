#!/usr/bin/env python3
"""Run production command callback against data-only stubs, no publications."""
import pathlib
import subprocess
import tempfile

root=pathlib.Path(__file__).resolve().parents[1]
src=(root/'src/control/attitude_pressure_controller.cpp').read_text()
start=src.index('void AttitudePressureController::inchwormCommandCallback(')
method=src[start:src.index('bool AttitudePressureController::enableCallback(',start)]
code=r'''
#include <array>
#include <cmath>
#include <cassert>
#define ROS_ERROR(...) ((void)0)
namespace ros {struct Time {static Time now(){return {};}};}
namespace geometry_msgs {
struct Vector3Stamped {using ConstPtr=const Vector3Stamped*;
 struct {double x=1,y=0,z=.02;} vector;};
}
struct AttitudePressureController {
 using Vector2=std::array<double,2>;static constexpr int ARM_COUNT=4;
 bool enabled_=true,inchworm_action_c_enabled_=true,inchworm_action_d_enabled_=false;
 bool mocap_received_=true,target_received_=false;
 std::array<double,4> measured_pressure_{{30,30,30,30}};
 double base_pressure_kpa_=30,pressure_tolerance_kpa_=2;
 bool continuous_recovery_blocked_=false,continuous_recovery_started_=false;
 bool continuous_recovery_waiting_for_settle_=false,action_c_resume_pending_=false;
 bool action_c_preserve_adaptation_on_start_=false;
 int starts=0,stops=0;
 void stopInchwormToGrasp(ros::Time,const char*){++stops;}
 void startInchwormStep(Vector2,double,ros::Time){++starts;}
 void inchwormCommandCallback(const geometry_msgs::Vector3Stamped::ConstPtr&);
};
METHOD
int main(){
 geometry_msgs::Vector3Stamped m;
 AttitudePressureController c;c.inchwormCommandCallback(&m);assert(c.starts==0);
 c.target_received_=true;c.inchwormCommandCallback(&m);assert(c.starts==1);
 c.target_received_=false;m.vector.z=0;c.inchwormCommandCallback(&m);assert(c.stops==1);
 m.vector.z=.02;c.inchworm_action_c_enabled_=false;c.inchwormCommandCallback(&m);assert(c.starts==2);
 c.inchworm_action_c_enabled_=true;c.inchworm_action_d_enabled_=true;
 c.inchwormCommandCallback(&m);assert(c.starts==3); // other gaits unchanged
 c.inchworm_action_d_enabled_=false;c.target_received_=true;c.mocap_received_=false;
 c.inchwormCommandCallback(&m);assert(c.starts==3);
 c.mocap_received_=true;c.measured_pressure_[0]=10;c.inchwormCommandCallback(&m);assert(c.starts==3);
}
'''.replace('METHOD',method)
with tempfile.TemporaryDirectory(prefix='hugmy_start_target_') as tmp:
    binary=str(pathlib.Path(tmp)/'test')
    subprocess.run(['g++','-std=c++14','-x','c++','-','-o',binary],input=code,text=True,check=True)
    subprocess.run([binary],check=True)
print('PASS: missing Action C target blocks start, explicit target enables, stop/other gaits unchanged')
