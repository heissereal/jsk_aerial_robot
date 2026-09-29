#!/usr/bin/env python3
"""Read-only launch wiring checks; never start ROS nodes."""
import pathlib
import xml.etree.ElementTree as ET
import yaml

root = pathlib.Path(__file__).resolve().parents[1]
config = yaml.safe_load((root / 'config/attitude_pressure_controller.yaml').read_text())
controller = ET.parse(root / 'launch/attitude_pressure_controller.launch').getroot()
bringup = ET.parse(root / 'launch/bringup.launch').getroot()
for name in ('recovery_predictive_enabled', 'recovery_predictive_shadow'):
    assert config[name] is True
    for launch in (controller, bringup):
        assert launch.find("arg[@name='{}']".format(name)).get('default') == 'true'
    node = controller.find(".//node[@name='attitude_pressure_controller']")
    assert node.find("param[@name='{}']".format(name)).get('value') == '$(arg {})'.format(name)
    include = next(e for e in bringup.findall('include')
                   if e.get('file', '').endswith('/attitude_pressure_controller.launch'))
    assert include.find("arg[@name='{}']".format(name)).get('value') == '$(arg {})'.format(name)
assert config['prepare_timeout_sec'] == 60.0
assert config['recovery_predictive_budget_ms'] <= 10.0
print('PASS: shadow defaults, explicit activation, launch forwarding, total timeout unchanged')
