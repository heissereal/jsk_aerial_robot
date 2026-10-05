#!/usr/bin/env python3
"""Production cant/geometry and calibration table through the hybrid planner.

No ROS, no actuators. Predictions are not a substitute for physics validation.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src/control/attitude_pressure_controller.cpp").read_text()
adapter = (ROOT / "src/control/hybrid_locomotion.cpp").read_text()

def extract(text, name, result):
    start = text.index("AttitudePressureController::" + name + "(")
    end = text.index("{", start) + 1
    depth = 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return result + " " + text[start:end]

functions = extract(adapter, "hybridRotorForce", "hugmy::hybrid::Force")
for name in ("rotorPositionFromAngle", "rotateAboutLocalY", "rotateLocalToBody", "cross"):
    functions += "\n" + extract(source, name, "AttitudePressureController::Vector3")

CODE = r'''
#include <hugmy/control/hybrid_locomotion_mpc.h>
#include <hugmy/model/pressure_thrust_bend_model.h>
#include <cassert>
#include <iostream>
class AttitudePressureController {
public:
 using Vector3=std::array<double,3>;
 double recovery_rotor_cant_rad_=6.5*M_PI/180.,target_roll_=0.,action_c_direction_sign_x_=1.;
 std::array<double,4> arm_yaw_{{-3*M_PI/4,-M_PI/4,M_PI/4,3*M_PI/4}};
 std::array<Vector3,4> arm_base_position_{{{{.03812,.03812,-.039}},{{-.03812,.03812,-.039}},
    {{-.03812,-.03812,-.039}},{{.03812,-.03812,-.039}}}};
 Vector3 contact_position_{{0.,0.,-.07}};
 Vector3 hybrid_contact_position_{{0.,0.,-.07}};
 Vector3 rotorPositionFromAngle(size_t,double)const;
 static Vector3 rotateAboutLocalY(const Vector3&,double);
 static Vector3 rotateLocalToBody(const Vector3&,double);
 static Vector3 cross(const Vector3&,const Vector3&);
 hugmy::hybrid::Force hybridRotorForce(size_t,double,double,double)const;
};
FUNCTIONS
int main() {
 using namespace hugmy::hybrid;
 AttitudePressureController robot;hugmy::PressureThrustBendModel table;
 // Support plane errors can reverse roll authority for the same rotor.
 // The force direction itself must be independent of the support estimate.
 const auto shallow=robot.hybridRotorForce(0,80.*M_PI/180.,0.,0.);
 robot.hybrid_contact_position_[2]=-.12;
 const auto deep=robot.hybridRotorForce(0,80.*M_PI/180.,0.,0.);
 assert(shallow.roll_moment>0. && deep.roll_moment<0.);
 assert(shallow.forward==deep.forward && shallow.up==deep.up);
 robot.hybrid_contact_position_[2]=-.07;
 Config c;c.budget_ms=1000.;
 auto eq=[&](size_t,double p,double t){return table.angleRad(p,t+1.8);};
 unsigned feasible=0,attempts=0,increases=0;double worst_ms=0.;
 for(double sign:{1.,-1.}) {
  robot.action_c_direction_sign_x_=sign;
  const std::array<size_t,2> arms=sign>0?std::array<size_t,2>{{0,3}}:std::array<size_t,2>{{1,2}};
  auto geom=[&](size_t j,double a,double roll,double pitch){return robot.hybridRotorForce(arms[j],a,roll,pitch);};
  auto left=geom(0,1.4,0.,0.),right=geom(1,1.4,0.,0.);
  assert(left.forward>0. && right.forward>0.);
  assert(std::abs(left.roll_moment+right.roll_moment)<1e-10);
  // Independently verify cant and the body-to-world projection.
  const size_t arm=arms[0];const double a=1.4,y=robot.arm_yaw_[arm],cant=robot.recovery_rotor_cant_rad_;
  const double local_x=-std::sin(a)*std::cos(cant);
  const double local_y=(arm==0||arm==2?1.:-1.)*std::sin(cant);
  assert(std::abs(left.forward-sign*(std::cos(y)*local_x-std::sin(y)*local_y))<1e-10);
  // Initial sliding probe and the first no-motion escalation. At 1 N the
  // selected waveform may be flat; powered-flexion increase must remain
  // available once the breakaway search asks for more thrust.
  for(double demand:{1.,1.5}) for(double bend:{1.,1.4,1.8,2.}) for(double pressure:{10.,30.}) for(double ratio:{0.,.6,1.}) {
   State s;s.fresh=true;s.now=1.;s.bend.fill(bend);s.start_bend=s.bend;
   s.pressure.fill(pressure);s.command=s.pressure;
   c.search_initial_thrust=demand;
   Planner planner;planner.reset(c);
   auto plan=planner.update(s,c,ratio,eq,geom);++attempts;
   if(!plan.feasible) continue;
   ++feasible;worst_ms=std::max(worst_ms,plan.compute_ms);
   increases+=plan.pull_thrust[0]>plan.thrust[0]+.01 || plan.pull_thrust[1]>plan.thrust[1]+.01;
   auto first=planner.firstThrust(s,c,plan,.02,geom);
   assert(first[0]>=0 && first[1]>=0 && first[0]<=.080001 && first[1]<=.080001);
   assert(first[0]*std::max(0.,left.up)+first[1]*std::max(0.,right.up)<=c.up_force_max);
  }
 }
 assert(feasible>=48 && increases>0);
 std::cout<<"PASS: actual cant + geometry + pressure/bend table: "<<feasible<<"/"<<attempts
   <<" feasible starts, "<<increases<<" plans increase pull thrust, max "<<worst_ms<<" ms\n";
}
'''.replace("FUNCTIONS", functions)

with tempfile.TemporaryDirectory(prefix="hugmy_hybrid_geometry_") as directory:
    binary = str(Path(directory) / "test")
    subprocess.run(["g++", "-std=c++14", "-O2", "-Wall", "-Wextra", "-I", str(ROOT / "include"),
                    "-x", "c++", "-", "-o", binary], input=CODE, text=True, check=True)
    subprocess.run([binary], check=True)
