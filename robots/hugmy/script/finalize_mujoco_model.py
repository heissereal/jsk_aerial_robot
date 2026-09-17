#!/usr/bin/env python3

"""Prepare the generated Hugmy MJCF for pneumatic effort control."""

import argparse
import math
import xml.etree.ElementTree as ET


ARM_COUNT = 4
JOINT_COUNT = 5


def finalize(model_path):
    tree = ET.parse(model_path)
    root = tree.getroot()

    joints = {}
    bodies = {}
    for body in root.iter("body"):
        name = body.get("name")
        if name:
            bodies[name] = body
        for joint in body.findall("joint"):
            joint_name = joint.get("name", "")
            if joint_name.startswith("joint_"):
                joint.set("damping", "0.002")
                joint.set("frictionloss", "0")
                # The cable-carrier links bear against one another at the
                # straight pose.  MuJoCo's default joint limit is deliberately
                # soft (20 ms time constant), which lets rotor thrust push
                # these very light links far through the zero-angle stop.
                # Model the link-to-link support as a stiff, critically damped
                # unilateral joint stop.  A small margin engages the stop just
                # before zero without changing the kinematic joint range.
                joint.set("solreflimit", "0.004 1")
                joint.set("solimplimit", "0.99 0.999 0.001 0.5 2")
                joint.set("margin", "0.002")
                joints[joint_name] = joint

    expected = {
        "joint_{}_{}".format(arm, joint)
        for arm in range(1, ARM_COUNT + 1)
        for joint in range(1, JOINT_COUNT + 1)
    }
    missing_joints = sorted(expected.difference(joints))
    if missing_joints:
        raise RuntimeError("missing Hugmy arm joints: {}".format(", ".join(missing_joints)))

    actuator = root.find("actuator")
    if actuator is None:
        raise RuntimeError("generated MJCF has no actuator section")
    actuator_by_name = {element.get("name"): element for element in list(actuator)}
    for arm in range(1, ARM_COUNT + 1):
        rotor = actuator_by_name.get("rotor{}".format(arm))
        if rotor is None:
            raise RuntimeError("missing rotor actuator {}".format(arm))
        # The 3D propellers generate signed thrust about the 0.75 neutral PWM.
        rotor.set("ctrllimited", "true")
        rotor.set("ctrlrange", "-20 20")
    for joint_name in sorted(expected):
        old = actuator_by_name.get(joint_name)
        if old is None:
            raise RuntimeError("missing actuator for {}".format(joint_name))
        index = list(actuator).index(old)
        actuator.remove(old)
        motor = ET.Element(
            "motor",
            name=joint_name,
            joint=joint_name,
            ctrllimited="true",
            ctrlrange="-10 10",
            gear="1",
        )
        actuator.insert(index, motor)

    for arm in range(1, ARM_COUNT + 1):
        body_name = "link_{}_rotor".format(arm)
        body = bodies.get(body_name)
        if body is None:
            raise RuntimeError("missing rotor body {}".format(body_name))
        site_name = "neuron_imu_{}".format(arm)
        if not any(site.get("name") == site_name for site in body.findall("site")):
            # Match the virtual propeller frame published by real Spinal: the
            # physical arm IMU is unchanged, while the propeller is canted by
            # -spin_sign * 6.5 deg about the arm-local X axis.
            spin_sign = 1.0 if arm in (1, 3) else -1.0
            cant_rad = math.radians(6.5)
            body.append(ET.Element(
                "site", name=site_name, pos="-0.020 0 -0.0131",
                euler="{} 0 0".format(-spin_sign * cant_rad)))

    main_body = bodies.get("main_body")
    if main_body is None:
        raise RuntimeError("generated MJCF has no main_body")
    if not any(
        geom.get("name") == "hugmy_bottom_inflatable"
        for geom in main_body.findall("geom")
    ):
        # Collision is represented by an explicit pressure-dependent force in
        # HugmyPneumaticHWSim. This visual follows the selected side and grows
        # with simulated bottom pressure.
        main_body.append(
            ET.Element(
                "geom",
                name="hugmy_bottom_inflatable",
                type="ellipsoid",
                pos="-0.04 0 -0.05",
                size="0.03 0.035 0.003",
                rgba="0.2 0.55 1.0 0.15",
                mass="0",
                contype="0",
                conaffinity="0",
            )
        )

    sensor = root.find("sensor")
    if sensor is None:
        sensor = ET.SubElement(root, "sensor")
    sensor_names = {element.get("name") for element in list(sensor)}
    for arm in range(1, ARM_COUNT + 1):
        site_name = "neuron_imu_{}".format(arm)
        acc_name = "neuron_acc_{}".format(arm)
        gyro_name = "neuron_gyro_{}".format(arm)
        if acc_name not in sensor_names:
            sensor.append(ET.Element("accelerometer", name=acc_name, site=site_name))
        if gyro_name not in sensor_names:
            sensor.append(ET.Element("gyro", name=gyro_name, site=site_name))

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("generated MJCF has no worldbody")
    if not any(body.get("name") == "hugmy_human_arm" for body in worldbody.findall("body")):
        human = ET.SubElement(worldbody, "body", name="hugmy_human_arm", pos="0 0 0.05")
        ET.SubElement(
            human,
            "geom",
            name="hugmy_human_arm_forearm",
            # MuJoCo cylinders are aligned with local z by default.  fromto
            # gives the dummy forearm the same local-x longitudinal axis used
            # by the pneumatic contact model and gait controller.
            type="cylinder",
            fromto="-0.25 0 0 0.25 0 0",
            size="0.05",
            rgba="0.9 0.58 0.42 1",
            mass="0",
            contype="1",
            conaffinity="1",
            friction="2.5 0.05 0.01",
        )

    tree.write(model_path, encoding="unicode", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="generated articulated Hugmy MJCF (robot.xml)")
    args = parser.parse_args()
    finalize(args.model)


if __name__ == "__main__":
    main()
