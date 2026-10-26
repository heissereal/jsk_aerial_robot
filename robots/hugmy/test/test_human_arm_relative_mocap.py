#!/usr/bin/env python3
"""Position-only hand reference, common-mode rejection, and launch wiring."""
import pathlib
import subprocess
import tempfile
import xml.etree.ElementTree as ET

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
start = source.index('AttitudePressureController::Vector3\nAttitudePressureController::locomotionPosition() const')
method = source[start:source.index('\nbool AttitudePressureController::locomotionMocapReady()', start)]

code = r'''
#include <array>
#include <cassert>
#include <cmath>
class AttitudePressureController {
public:
 using Vector3=std::array<double,3>;
 bool hand_relative_mocap_enabled_=true;
 double hand_progress_direction_sign_x_=1.;
 Vector3 mocap_position_{{0,0,0}},hand_mocap_position_{{.3,0,0}};
 Vector3 locomotionPosition()const;
};
METHOD
int main() {
 AttitudePressureController c;
 assert(std::abs(c.locomotionPosition()[0]+.3)<1e-12);
 c.mocap_position_={{1,2,3}};c.hand_mocap_position_={{1.3,2,99}};
 assert(std::abs(c.locomotionPosition()[0]+.3)<1e-12); // common translation
 c.hand_mocap_position_={{1,2.3,3}};
 assert(std::abs(c.locomotionPosition()[0]+.3)<1e-12); // common rigid rotation
 c.mocap_position_={{1,2.05,3}};
 assert(std::abs(c.locomotionPosition()[0]+.25)<1e-12); // 50 mm closer
 c.hand_progress_direction_sign_x_=-1.;
 assert(std::abs(c.locomotionPosition()[0]-.25)<1e-12);
 c.hand_relative_mocap_enabled_=false;c.mocap_position_={{4,5,6}};
 assert(c.locomotionPosition()==c.mocap_position_); // legacy behavior
}
'''.replace('METHOD', method)
with tempfile.TemporaryDirectory(prefix='hugmy_hand_relative_') as tmp:
    binary = str(pathlib.Path(tmp) / 'test')
    subprocess.run(['g++', '-std=c++17', '-x', 'c++', '-', '-o', binary],
                   input=code, text=True, check=True)
    subprocess.run([binary], check=True)

callback_start = source.index('void AttitudePressureController::updateHandMocap(')
callback = source[callback_start:source.index('\n}\n', callback_start) + 3]
callback_code = '\n'.join(line.split('//', 1)[0] for line in callback.splitlines())
assert 'pose.orientation' not in callback_code
assert 'pose.position.z' not in callback_code

mocap = ET.parse(root / 'launch/includes/human_arm_mocap.launch.xml').getroot()
args = {arg.get('name'): arg.get('default') for arg in mocap.findall('arg')}
assert args['robot_rigid_body_id'] == '1'
assert args['hand_rigid_body_id'] == '2'
text = (root / 'launch/includes/human_arm_mocap.launch.xml').read_text()
assert 'mocap/pose' in text and 'mocap/hand_pose' in text

human = ET.parse(root / 'launch/human_arm_locomotion.launch').getroot()
args = {arg.get('name'): arg.get('default') for arg in human.findall('arg')}
assert abs(float(args['forward_distance_m']) - .10) < 1e-12
assert args['enable_button'] == '4' and args['disable_button'] == '5'
assert args['forward_button'] == '3' and args['stop_button'] == '1'
assert abs(float(args['ppg_settle_sec'])) < 1e-12
assert abs(float(args['hybrid_lift_rear_release_start_ratio']) - .10) < 1e-12
assert abs(float(args['hybrid_lift_rear_pressure_floor_kpa'])) < 1e-12
assert abs(float(args['hybrid_rear_support_probe_stable']) + .75) < 1e-12
assert abs(float(args['hybrid_regrip_rear_hold_pressure_kpa']) - 7.5) < 1e-12
assert args['hybrid_locomotion_outer_enabled'] == 'true'
include = human.find('include')
forwarded = {arg.get('name'): arg.get('value') for arg in include.findall('arg')}
assert forwarded['hand_relative_mocap_enabled'] == 'true'
assert forwarded['hand_target_selects_direction'] == 'true'
assert forwarded['use_mocap_attitude_for_inchworm'] == 'false'
assert forwarded['ppg_settle_sec'] == '$(arg ppg_settle_sec)'
assert forwarded['hybrid_locomotion_outer_enabled'] == '$(arg hybrid_locomotion_outer_enabled)'
assert forwarded['hybrid_lift_rear_release_start_ratio'] == '$(arg hybrid_lift_rear_release_start_ratio)'
assert forwarded['hybrid_lift_rear_pressure_floor_kpa'] == '$(arg hybrid_lift_rear_pressure_floor_kpa)'
assert forwarded['hybrid_rear_support_probe_stable'] == '$(arg hybrid_rear_support_probe_stable)'
assert forwarded['hybrid_regrip_rear_hold_pressure_kpa'] == '$(arg hybrid_regrip_rear_hold_pressure_kpa)'
print('PASS: ID2 position-only reference, rigid-motion rejection, 100-mm Joy goal, online optimization enabled')
