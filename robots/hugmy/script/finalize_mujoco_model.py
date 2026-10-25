#!/usr/bin/env python3

"""Prepare the generated Hugmy MJCF for pneumatic effort control."""

import argparse
import math
import xml.etree.ElementTree as ET


ARM_COUNT = 4
JOINT_COUNT = 5

# AIST Japanese anthropometric database 1991--92, young adult men.
# Forearm length C8: 251.6 mm, maximum breadth D14: 92.6 mm,
# maximum circumference F26: 259.2 mm, minimum circumference F27: 167.7 mm.
# The depths below are the ellipse depths obtained from those circumferences
# while retaining the measured maximum breadth / inferred depth aspect ratio.
AIST_FOREARM_LENGTH_M = 0.2516
AIST_FOREARM_PROXIMAL_WIDTH_M = 0.0926
AIST_FOREARM_PROXIMAL_DEPTH_M = 0.07175
AIST_FOREARM_DISTAL_WIDTH_M = 0.0599
AIST_FOREARM_DISTAL_DEPTH_M = 0.0464
# A conservative rigid test fixture uses the mean of the proximal and distal
# AIST sections.  Removing the taper prevents a pressure-normal force from
# acquiring an axial component and being mistaken for gait progress.
AIST_FOREARM_MEAN_WIDTH_M = 0.5 * (
    AIST_FOREARM_PROXIMAL_WIDTH_M + AIST_FOREARM_DISTAL_WIDTH_M)
AIST_FOREARM_MEAN_DEPTH_M = 0.5 * (
    AIST_FOREARM_PROXIMAL_DEPTH_M + AIST_FOREARM_DISTAL_DEPTH_M)


def tapered_elliptic_forearm_mesh(length=AIST_FOREARM_LENGTH_M,
                                  proximal_width=AIST_FOREARM_PROXIMAL_WIDTH_M,
                                  proximal_depth=AIST_FOREARM_PROXIMAL_DEPTH_M,
                                  distal_width=AIST_FOREARM_DISTAL_WIDTH_M,
                                  distal_depth=AIST_FOREARM_DISTAL_DEPTH_M,
                                  segments=32):
    """Return MJCF inline mesh strings for a convex tapered forearm.

    Local +x points from elbow to wrist.  Width is local y and depth is local
    z.  The two end rings describe a convex elliptic frustum, which gives
    MuJoCo a stable collision hull without approximating the arm by a round
    cylinder.
    """
    half = 0.5 * length
    vertices = []
    for x, width, depth in ((-half, proximal_width, proximal_depth),
                            (half, distal_width, distal_depth)):
        for index in range(segments):
            angle = 2.0 * math.pi * index / segments
            vertices.extend((x, 0.5 * width * math.cos(angle),
                             0.5 * depth * math.sin(angle)))
    vertices.extend((-half, 0.0, 0.0, half, 0.0, 0.0))
    proximal_center = 2 * segments
    distal_center = proximal_center + 1
    faces = []
    for index in range(segments):
        following = (index + 1) % segments
        # Side surface, then proximal and distal caps.  The winding is chosen
        # to keep the visual normals outward; collision uses the convex hull.
        faces.extend((index, segments + index, segments + following,
                      index, segments + following, following,
                      proximal_center, following, index,
                      distal_center, segments + index, segments + following))
    return (" ".join("{:.8g}".format(value) for value in vertices),
            " ".join(str(value) for value in faces))


def finalize(model_path, forearm_profile="tapered"):
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
    bottom_geom = next((
        geom for geom in main_body.findall("geom")
        if geom.get("name") == "hugmy_bottom_inflatable"
    ), None)
    if bottom_geom is None:
        bottom_geom = ET.SubElement(
            main_body, "geom", name="hugmy_bottom_inflatable")
    # Collision is represented by the pressure-dependent radial force in
    # HugmyPneumaticHWSim.  The centred ellipsoid visualises the single rounded
    # support; its wide lateral radius discourages interpreting it as a second
    # independently controlled front/rear chamber.
    bottom_geom.set("type", "ellipsoid")
    bottom_geom.set("pos", "0 0 -0.05")
    bottom_geom.set("size", "0.04 0.045 0.003")
    bottom_geom.set("rgba", "0.2 0.55 1.0 0.15")
    bottom_geom.set("mass", "0")
    bottom_geom.set("contype", "0")
    bottom_geom.set("conaffinity", "0")

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
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    forearm_mesh = next((mesh for mesh in asset.findall("mesh")
                         if mesh.get("name") == "hugmy_aist_male_forearm"), None)
    if forearm_mesh is None:
        forearm_mesh = ET.SubElement(asset, "mesh", name="hugmy_aist_male_forearm")
    if forearm_profile == "uniform":
        vertices, faces = tapered_elliptic_forearm_mesh(
            AIST_FOREARM_LENGTH_M,
            AIST_FOREARM_MEAN_WIDTH_M,
            AIST_FOREARM_MEAN_DEPTH_M,
            AIST_FOREARM_MEAN_WIDTH_M,
            AIST_FOREARM_MEAN_DEPTH_M)
    else:
        vertices, faces = tapered_elliptic_forearm_mesh()
    forearm_mesh.set("vertex", vertices)
    forearm_mesh.set("face", faces)

    if not any(body.get("name") == "hugmy_human_arm" for body in worldbody.findall("body")):
        human = ET.SubElement(worldbody, "body", name="hugmy_human_arm", pos="0 0 0.05")
        ET.SubElement(
            human,
            "geom",
            name="hugmy_human_arm_forearm",
            type="mesh",
            mesh="hugmy_aist_male_forearm",
            rgba="0.9 0.58 0.42 1",
            mass="0",
            contype="1",
            conaffinity="1",
            friction="2.5 0.05 0.01",
        )
    else:
        human = next(body for body in worldbody.findall("body")
                     if body.get("name") == "hugmy_human_arm")
        geom = next((item for item in human.findall("geom")
                     if item.get("name") == "hugmy_human_arm_forearm"), None)
        if geom is None:
            raise RuntimeError("hugmy_human_arm has no forearm geom")
        geom.attrib.pop("fromto", None)
        geom.attrib.pop("size", None)
        geom.set("type", "mesh")
        geom.set("mesh", "hugmy_aist_male_forearm")

    tree.write(model_path, encoding="unicode", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="generated articulated Hugmy MJCF (robot.xml)")
    parser.add_argument(
        "--forearm-profile", choices=("tapered", "uniform"), default="tapered",
        help=("tapered keeps the measured AIST end sections; uniform uses their "
              "mean ellipse as a conservative no-wedge locomotion fixture"))
    args = parser.parse_args()
    finalize(args.model, args.forearm_profile)


if __name__ == "__main__":
    main()
