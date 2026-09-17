#!/usr/bin/env python3
"""Robust fixed-tilt optimization for the HUGMY Quad-to-Bi transition.

Only the measured total bend of each folding arm is assumed to be known. The
distribution over joints 1--3 is treated as an uncertainty:

    q1 + q2 + q3 = beta
    0 <= q1 <= 50 deg, 0 <= q2,q3 <= 60 deg

The set is sampled on a configurable grid. By default both opposing arms share
the same unknown distribution. Asymmetric combinations of extreme shapes can
also be enabled. This avoids assuming either equal joint angles or an
unverified "proximal joint bends first" law.

The controlled outputs [Fz, Mx, My, Mz] are solved exactly. Fx and Fy are
dependent outputs of this under-actuated allocation; their resultant
acceleration is bounded rather than silently ignored.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull


# =============================================================================
# Analysis defaults
# =============================================================================
FOLDED_ARMS = (2, 4)
JOINT_LIMITS_DEG = (50.0, 60.0, 60.0)

BETA_MIN_DEG = 0.0
BETA_MAX_DEG = 90.0
BETA_STEP_DEG = 5.0
JOINT_SAMPLE_STEP_DEG = 5.0

GAMMA_MIN_DEG = 0.0
GAMMA_MAX_DEG = 45.0
GAMMA_STEP_DEG = 0.5

M_F_RATE_FORWARD = 0.0092  # [Nm/N], signed Q/F for positive thrust
M_F_RATE_REVERSE = 0.0279  # [Nm/N], placeholder until reverse data are available
REVERSE_THRUST_RATIO = 0.4
MAX_LATERAL_ACCEL_M_S2 = 0.5
MAX_ALLOCATION_CONDITION = 1.0e8
ARM_UNCERTAINTY = "symmetric"

GRAVITY = 9.81
CONTROL_ROWS = (2, 3, 4, 5)  # Fz, Mx, My, Mz
CONTROL_LABELS = ("Fz", "Mx", "My", "Mz")
NUMERIC_TOL = 1.0e-9

DEFAULT_XACRO = Path(__file__).resolve().parents[1] / "urdf" / "hugmy2.urdf.xacro"


# =============================================================================
# Data types
# =============================================================================
@dataclass
class LinkMass:
    mass: float
    com_local: np.ndarray


@dataclass
class ArmSpec:
    arm_id: int
    base_offset: np.ndarray
    base_yaw: float
    spin_sign: int


@dataclass
class HugmyModel:
    link_length_n: float
    link_length_r: float
    max_thrust: float
    min_thrust_urdf: float
    m_f_rate_forward: float
    m_f_rate_reverse: float
    main_body: LinkMass
    battery_mass: float
    battery_positions: List[np.ndarray]
    arm_link_masses: Dict[str, LinkMass]
    thrust_mass: LinkMass
    arms: List[ArmSpec]
    prop_origin_in_rotor: np.ndarray

    @property
    def total_mass_nominal(self) -> float:
        per_arm = sum(link.mass for link in self.arm_link_masses.values())
        per_arm += self.thrust_mass.mass
        return (
            self.main_body.mass
            + len(self.battery_positions) * self.battery_mass
            + len(self.arms) * per_arm
        )


@dataclass(frozen=True)
class JointScenario:
    """Joint-angle hypotheses for the two folding arms, in degrees."""

    label: str
    arm2_deg: Tuple[float, float, float]
    arm4_deg: Tuple[float, float, float]

    def angles_deg(self, arm_id: int) -> Tuple[float, float, float]:
        if arm_id == FOLDED_ARMS[0]:
            return self.arm2_deg
        if arm_id == FOLDED_ARMS[1]:
            return self.arm4_deg
        return (0.0, 0.0, 0.0)


@dataclass
class AnalysisSettings:
    reverse_thrust_ratio: float
    max_lateral_accel: float
    joint_step_deg: float
    arm_uncertainty: str
    max_allocation_condition: float = MAX_ALLOCATION_CONDITION


@dataclass
class ShapeEvaluation:
    beta_deg: float
    gamma_deg: float
    scenario: JointScenario
    feasible: bool
    reason: str
    mass: float
    com: np.ndarray
    rotors: List[dict]
    allocation_condition: float
    hover_thrusts: np.ndarray
    hover_wrench: np.ndarray
    required_thrust: float
    lateral_accel: float
    margins: Dict[str, float]
    normalized_margins: Dict[str, float]
    rho: float


@dataclass
class RobustEvaluation:
    beta_deg: float
    gamma_deg: float
    scenario_count: int
    failed_scenarios: int
    feasible: bool
    worst_rho: float
    worst_margin_case: ShapeEvaluation
    worst_required_thrust: float
    worst_thrust_case: ShapeEvaluation
    worst_lateral_accel: float


@dataclass
class SignRegion:
    """Linear allocation valid for one combination of thrust signs."""

    signs: Tuple[int, ...]
    matrix: np.ndarray
    inverse_control: np.ndarray
    condition: float


# =============================================================================
# Rigid transforms and Xacro loading
# =============================================================================
def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def transform(
    rotation: np.ndarray | None = None,
    translation: Sequence[float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def transform_point(frame: np.ndarray, point: Sequence[float]) -> np.ndarray:
    homogeneous = np.r_[np.asarray(point, dtype=float), 1.0]
    return (frame @ homogeneous)[:3]


def parse_vector(text: str) -> np.ndarray:
    return np.array([float(value) for value in text.strip().split()], dtype=float)


def strip_xacro_expression(text: str) -> str:
    text = text.strip()
    if text.startswith("${") and text.endswith("}"):
        return text[2:-1]
    return text


def safe_eval(expression: str, variables: Dict[str, float]) -> float:
    allowed = {"pi": math.pi, **variables}
    return float(eval(strip_xacro_expression(expression), {"__builtins__": {}}, allowed))


def load_hugmy_xacro(path: str | Path) -> HugmyModel:
    root = ET.parse(Path(path)).getroot()
    namespace = {"xacro": "http://www.ros.org/wiki/xacro"}

    properties: Dict[str, float] = {}
    for element in root.findall("xacro:property", namespace):
        name = element.attrib.get("name")
        if not name:
            continue
        try:
            properties[name] = safe_eval(element.attrib.get("value", ""), properties)
        except Exception:
            # Boolean/string Xacro properties are irrelevant to this parser.
            continue

    main_body = root.find("./link[@name='main_body']/inertial")
    if main_body is None:
        raise ValueError("main_body inertial was not found in the Xacro")
    main_body_mass = LinkMass(
        mass=float(main_body.find("mass").attrib["value"]),
        com_local=parse_vector(main_body.find("origin").attrib.get("xyz", "0 0 0")),
    )

    battery_mass = 0.0
    battery_positions: List[np.ndarray] = []
    for battery_id in (0, 1):
        link = root.find(f"./link[@name='battery_{battery_id}']")
        joint = root.find(f"./joint[@name='battery_{battery_id}_joint']")
        if link is None or joint is None:
            continue
        battery_mass = float(link.find("inertial/mass").attrib["value"])
        battery_positions.append(parse_vector(joint.find("origin").attrib["xyz"]))

    arm_macro = next(
        (element for element in root.findall("xacro:macro", namespace)
         if element.attrib.get("name") == "arm_module"),
        None,
    )
    if arm_macro is None:
        raise ValueError("arm_module macro was not found in the Xacro")
    link_names = {
        "link_${id}_1": "link1",
        "link_${id}_2": "link2",
        "link_${id}_rotor": "rotor",
        "link_${id}_4": "link4",
        "link_${id}_end": "end",
    }
    arm_link_masses: Dict[str, LinkMass] = {}
    for link in arm_macro.findall("link"):
        short_name = link_names.get(link.attrib.get("name", ""))
        if short_name is None:
            continue
        inertial = link.find("inertial")
        arm_link_masses[short_name] = LinkMass(
            mass=float(inertial.find("mass").attrib["value"]),
            com_local=parse_vector(inertial.find("origin").attrib.get("xyz", "0 0 0")),
        )

    propeller_macro = next(
        (element for element in root.findall("xacro:macro", namespace)
         if element.attrib.get("name") == "propeller_module"),
        None,
    )
    if propeller_macro is None:
        raise ValueError("propeller_module macro was not found in the Xacro")
    thrust_inertial = propeller_macro.find("./link[@name='thrust${id}']/inertial")
    thrust_mass = LinkMass(
        mass=float(thrust_inertial.find("mass").attrib["value"]),
        com_local=parse_vector(thrust_inertial.find("origin").attrib.get("xyz", "0 0 0")),
    )
    rotor_joint = propeller_macro.find("./joint[@name='rotor${id}']")
    prop_origin = parse_vector(rotor_joint.find("origin").attrib["xyz"])

    base_offsets = {
        int(element.attrib["id"]): parse_vector(element.attrib["base_offset"])
        for element in root.findall("xacro:arm_module", namespace)
    }
    spin_signs = {
        int(element.attrib["id"]): int(round(safe_eval(element.attrib["direction"], properties)))
        for element in root.findall("xacro:propeller_module", namespace)
    }
    arms = [
        ArmSpec(
            arm_id=arm_id,
            base_offset=base_offsets[arm_id],
            base_yaw=-3.0 * math.pi / 4.0 + math.pi / 2.0 * (arm_id - 1),
            spin_sign=spin_signs[arm_id],
        )
        for arm_id in sorted(base_offsets)
    ]

    m_f_rate_element = root.find("m_f_rate")
    xacro_m_f_rate = (
        float(m_f_rate_element.attrib["value"])
        if m_f_rate_element is not None else 0.0
    )
    return HugmyModel(
        link_length_n=properties["link_length_n"],
        link_length_r=properties["link_length_r"],
        max_thrust=properties["max_force"],
        min_thrust_urdf=properties["min_force"],
        m_f_rate_forward=xacro_m_f_rate,
        m_f_rate_reverse=xacro_m_f_rate,
        main_body=main_body_mass,
        battery_mass=battery_mass,
        battery_positions=battery_positions,
        arm_link_masses=arm_link_masses,
        thrust_mass=thrust_mass,
        arms=arms,
        prop_origin_in_rotor=prop_origin,
    )


# =============================================================================
# Joint-angle uncertainty
# =============================================================================
def unique_angle_tuples(
    values: Iterable[Sequence[float]],
) -> List[Tuple[float, float, float]]:
    unique: Dict[Tuple[float, float, float], Tuple[float, float, float]] = {}
    for value in values:
        angle = tuple(float(component) for component in value)
        key = tuple(round(component, 8) for component in angle)
        unique[key] = angle
    return sorted(unique.values())


def interval_samples(lower: float, upper: float, step: float) -> List[float]:
    if upper < lower - NUMERIC_TOL:
        return []
    values = [lower, upper]
    first_grid = math.ceil((lower - NUMERIC_TOL) / step) * step
    values.extend(float(value) for value in np.arange(first_grid, upper + NUMERIC_TOL, step))
    return sorted({round(value, 10) for value in values})


def joint_polytope_vertices(beta_deg: float) -> List[Tuple[float, float, float]]:
    """Vertices of q1+q2+q3=beta intersected with the joint limits."""
    limits = np.asarray(JOINT_LIMITS_DEG)
    vertices = []
    for fixed_indices in itertools.combinations(range(3), 2):
        free_index = next(index for index in range(3) if index not in fixed_indices)
        for at_upper in itertools.product((False, True), repeat=2):
            q = np.zeros(3)
            for index, use_upper in zip(fixed_indices, at_upper):
                q[index] = limits[index] if use_upper else 0.0
            q[free_index] = beta_deg - float(np.sum(q))
            if np.all(q >= -NUMERIC_TOL) and np.all(q <= limits + NUMERIC_TOL):
                vertices.append(np.clip(q, 0.0, limits))
    return unique_angle_tuples(vertices)


def equal_joint_distribution(beta_deg: float) -> Tuple[float, float, float]:
    value = beta_deg / 3.0
    return value, value, value


def proximal_first_distribution(beta_deg: float) -> Tuple[float, float, float]:
    remaining = beta_deg
    result = []
    for limit in JOINT_LIMITS_DEG:
        angle = min(max(remaining, 0.0), limit)
        result.append(angle)
        remaining -= angle
    if remaining > NUMERIC_TOL:
        raise ValueError(f"beta={beta_deg:g} deg exceeds the available joint range")
    return tuple(result)


def joint_angle_grid(beta_deg: float, step_deg: float) -> List[Tuple[float, float, float]]:
    """Sample every feasible symmetric joint distribution on a 2-D grid."""
    limit1, limit2, limit3 = JOINT_LIMITS_DEG
    candidates: List[Sequence[float]] = []
    for q1 in interval_samples(0.0, min(limit1, beta_deg), step_deg):
        q2_lower = max(0.0, beta_deg - q1 - limit3)
        q2_upper = min(limit2, beta_deg - q1)
        for q2 in interval_samples(q2_lower, q2_upper, step_deg):
            q3 = beta_deg - q1 - q2
            if -NUMERIC_TOL <= q3 <= limit3 + NUMERIC_TOL:
                candidates.append((q1, q2, min(max(q3, 0.0), limit3)))

    # Keep important shapes even when they do not lie on the sampling grid.
    candidates.extend(joint_polytope_vertices(beta_deg))
    candidates.append(equal_joint_distribution(beta_deg))
    candidates.append(proximal_first_distribution(beta_deg))
    return unique_angle_tuples(candidates)


def make_joint_scenarios(
    beta_deg: float,
    step_deg: float,
    arm_uncertainty: str,
) -> List[JointScenario]:
    """Build symmetric shapes plus the requested arm-to-arm uncertainty."""
    grid = joint_angle_grid(beta_deg, step_deg)
    scenarios = [JointScenario(f"symmetric_{index:04d}", q, q) for index, q in enumerate(grid)]
    if arm_uncertainty == "symmetric":
        return scenarios

    if arm_uncertainty == "full":
        pair_source = grid
    elif arm_uncertainty == "extremes":
        pair_source = unique_angle_tuples(
            joint_polytope_vertices(beta_deg)
            + [equal_joint_distribution(beta_deg), proximal_first_distribution(beta_deg)]
        )
    else:
        raise ValueError(f"unknown arm uncertainty mode: {arm_uncertainty}")

    existing = {(scenario.arm2_deg, scenario.arm4_deg) for scenario in scenarios}
    for index, (arm2, arm4) in enumerate(itertools.product(pair_source, repeat=2)):
        if (arm2, arm4) not in existing:
            scenarios.append(JointScenario(f"asymmetric_{index:04d}", arm2, arm4))
    return scenarios


# =============================================================================
# Configuration-dependent geometry and wrench allocation
# =============================================================================
def arm_transforms(
    model: HugmyModel,
    arm: ArmSpec,
    joint_angles_deg: Sequence[float],
) -> Dict[str, np.ndarray]:
    q1, q2, q3 = np.radians(np.asarray(joint_angles_deg, dtype=float))
    link1 = (
        transform(translation=arm.base_offset)
        @ transform(rot_z(arm.base_yaw))
        @ transform(rot_y(-q1))
    )
    link2 = (
        link1
        @ transform(translation=(-model.link_length_n, 0.0, 0.0))
        @ transform(rot_y(-q2))
    )
    rotor = (
        link2
        @ transform(translation=(-model.link_length_n, 0.0, 0.0))
        @ transform(rot_y(-q3))
    )
    link4 = rotor @ transform(translation=(-model.link_length_r, 0.0, 0.0))
    end = link4 @ transform(translation=(-model.link_length_n, 0.0, 0.0))
    propeller = rotor @ transform(translation=model.prop_origin_in_rotor)
    return {
        "link1": link1, "link2": link2, "rotor": rotor,
        "link4": link4, "end": end, "prop": propeller,
    }


def local_canted_axis(model: HugmyModel, arm: ArmSpec, gamma_rad: float) -> np.ndarray:
    """Cant so geometric yaw opposes the positive-thrust reaction torque."""
    reaction_sign = (
        math.copysign(1.0, model.m_f_rate_forward)
        if model.m_f_rate_forward else 1.0
    )
    # In these arm frames, local +Y produces negative body yaw moment.
    tangent_sign = reaction_sign * arm.spin_sign
    tangent = tangent_sign * np.array([0.0, 1.0, 0.0])
    return math.cos(gamma_rad) * np.array([0.0, 0.0, 1.0]) + math.sin(gamma_rad) * tangent


def configuration_geometry(
    model: HugmyModel,
    gamma_deg: float,
    scenario: JointScenario,
) -> Tuple[float, np.ndarray, List[dict]]:
    gamma_rad = math.radians(gamma_deg)
    mass_points: List[Tuple[float, np.ndarray]] = [
        (model.main_body.mass, model.main_body.com_local.copy())
    ]
    mass_points.extend(
        (model.battery_mass, position.copy())
        for position in model.battery_positions
    )

    rotor_frames = []
    for arm in model.arms:
        frames = arm_transforms(model, arm, scenario.angles_deg(arm.arm_id))
        for link_name in ("link1", "link2", "rotor", "link4", "end"):
            link = model.arm_link_masses[link_name]
            mass_points.append((link.mass, transform_point(frames[link_name], link.com_local)))
        mass_points.append(
            (model.thrust_mass.mass, transform_point(frames["prop"], model.thrust_mass.com_local))
        )

        axis = frames["prop"][:3, :3] @ local_canted_axis(model, arm, gamma_rad)
        axis /= np.linalg.norm(axis)
        rotor_frames.append(
            (arm, frames["prop"][:3, 3].copy(), frames["rotor"][:3, 3].copy(), axis)
        )

    total_mass = sum(mass for mass, _ in mass_points)
    center_of_mass = sum(mass * point for mass, point in mass_points) / total_mass
    rotors = []
    for arm, propeller_position, rotor_frame_position, axis in rotor_frames:
        lever = propeller_position - center_of_mass
        rotors.append({
            "id": arm.arm_id,
            "spin": arm.spin_sign,
            "position_body": propeller_position,
            "rotor_frame_body": rotor_frame_position,
            "position": lever,
            "lever_perp": lever - np.dot(lever, axis) * axis,
            "axis": axis,
        })
    return total_mass, center_of_mass, rotors


def wrench_matrix(
    model: HugmyModel,
    gamma_deg: float,
    scenario: JointScenario,
    thrust_signs: Sequence[int] | None = None,
) -> Tuple[np.ndarray, float, np.ndarray, List[dict]]:
    """Return the wrench map for one positive/negative-thrust sign region.

    Both rates are signed Q/F values. For a negative scalar thrust, the
    reverse rate multiplies that negative thrust directly; callers must not
    negate the rate a second time.
    """
    mass, center_of_mass, rotors = configuration_geometry(model, gamma_deg, scenario)
    matrix = np.zeros((6, len(rotors)))
    if thrust_signs is None:
        thrust_signs = (1,) * len(rotors)
    for column, rotor in enumerate(rotors):
        axis = rotor["axis"]
        m_f_rate = (
            model.m_f_rate_forward
            if thrust_signs[column] >= 0 else model.m_f_rate_reverse
        )
        matrix[:3, column] = axis
        matrix[3:, column] = (
            np.cross(rotor["position"], axis)
            + m_f_rate * rotor["spin"] * axis
        )
    return matrix, mass, center_of_mass, rotors


def thrust_bounds(model: HugmyModel, settings: AnalysisSettings) -> List[Tuple[float, float]]:
    lower = -settings.reverse_thrust_ratio * model.max_thrust
    return [(lower, model.max_thrust)] * len(model.arms)


def required_thrust_rating(thrusts: np.ndarray, reverse_ratio: float) -> float:
    """Minimum forward rating t satisfying -reverse_ratio*t <= fi <= t."""
    forward = max(0.0, float(np.max(thrusts)))
    reverse = max(0.0, float(-np.min(thrusts)))
    if reverse <= NUMERIC_TOL:
        return forward
    if reverse_ratio <= NUMERIC_TOL:
        return math.inf
    return max(forward, reverse / reverse_ratio)


# =============================================================================
# Exact four-output trim and margins with lateral-acceleration limits
# =============================================================================
def intersect_linear_bounds(
    interval: Tuple[float, float],
    offset: float,
    slope: float,
    lower: float,
    upper: float,
) -> Tuple[float, float] | None:
    """Intersect delta with lower <= offset + slope*delta <= upper."""
    if abs(slope) <= NUMERIC_TOL:
        return interval if lower - NUMERIC_TOL <= offset <= upper + NUMERIC_TOL else None
    first = (lower - offset) / slope
    second = (upper - offset) / slope
    constraint_lower, constraint_upper = sorted((first, second))
    result = max(interval[0], constraint_lower), min(interval[1], constraint_upper)
    return result if result[0] <= result[1] + NUMERIC_TOL else None


def intersect_lateral_acceleration(
    interval: Tuple[float, float],
    force_offset: np.ndarray,
    force_slope: np.ndarray,
    force_limit: float,
) -> Tuple[float, float] | None:
    """Intersect delta with ||force_offset + force_slope*delta|| <= limit."""
    quadratic = float(force_slope @ force_slope)
    linear = 2.0 * float(force_offset @ force_slope)
    constant = float(force_offset @ force_offset) - force_limit**2
    if quadratic <= NUMERIC_TOL:
        return interval if constant <= NUMERIC_TOL else None
    discriminant = linear**2 - 4.0 * quadratic * constant
    if discriminant < -NUMERIC_TOL:
        return None
    root = math.sqrt(max(discriminant, 0.0))
    lower = (-linear - root) / (2.0 * quadratic)
    upper = (-linear + root) / (2.0 * quadratic)
    result = max(interval[0], lower), min(interval[1], upper)
    return result if result[0] <= result[1] + NUMERIC_TOL else None


def sign_region_bounds(
    signs: Sequence[int],
    bounds: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    """Intersect actuator bounds with the sign assumptions of a region."""
    return [
        (max(lower, 0.0), upper) if sign >= 0 else (lower, min(upper, 0.0))
        for sign, (lower, upper) in zip(signs, bounds)
    ]


def build_sign_regions(
    model: HugmyModel,
    gamma_deg: float,
    scenario: JointScenario,
    max_condition: float,
) -> Tuple[List[SignRegion], float, np.ndarray, List[dict]]:
    """Construct all nonsingular forward/reverse piecewise-linear maps."""
    forward, mass, center_of_mass, rotors = wrench_matrix(
        model, gamma_deg, scenario, (1,) * len(model.arms)
    )
    reverse, _, _, _ = wrench_matrix(
        model, gamma_deg, scenario, (-1,) * len(model.arms)
    )
    regions: List[SignRegion] = []
    for signs in itertools.product((-1, 1), repeat=len(model.arms)):
        matrix = forward.copy()
        for column, sign in enumerate(signs):
            if sign < 0:
                matrix[:, column] = reverse[:, column]
        control = matrix[np.asarray(CONTROL_ROWS), :]
        condition = float(np.linalg.cond(control))
        if np.isfinite(condition) and condition <= max_condition:
            regions.append(
                SignRegion(signs, matrix, np.linalg.inv(control), condition)
            )
    return regions, mass, center_of_mass, rotors


def thrusts_match_signs(thrusts: np.ndarray, signs: Sequence[int]) -> bool:
    return all(
        thrust >= -NUMERIC_TOL if sign >= 0 else thrust <= NUMERIC_TOL
        for thrust, sign in zip(thrusts, signs)
    )


def axis_delta_interval_for_region(
    region: SignRegion,
    target: np.ndarray,
    axis_index: int,
    bounds: Sequence[Tuple[float, float]],
    lateral_force_limit: float,
) -> Tuple[float, float] | None:
    """Reachable delta interval inside one forward/reverse sign region."""
    thrust_offset = region.inverse_control @ target
    thrust_slope = region.inverse_control[:, axis_index]
    interval = (-math.inf, math.inf)
    region_bounds = sign_region_bounds(region.signs, bounds)
    for thrust, slope, (lower, upper) in zip(
        thrust_offset, thrust_slope, region_bounds
    ):
        interval = intersect_linear_bounds(interval, thrust, slope, lower, upper)
        if interval is None:
            return None
    return intersect_lateral_acceleration(
        interval,
        region.matrix[:2] @ thrust_offset,
        region.matrix[:2] @ thrust_slope,
        lateral_force_limit,
    )


def merge_intervals(
    intervals: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    """Merge overlapping reachable intervals from adjacent sign regions."""
    merged: List[Tuple[float, float]] = []
    for lower, upper in sorted(intervals):
        if not merged or lower > merged[-1][1] + NUMERIC_TOL:
            merged.append((lower, upper))
        else:
            merged[-1] = (merged[-1][0], max(merged[-1][1], upper))
    return merged


def axis_delta_interval_piecewise(
    regions: Sequence[SignRegion],
    target: np.ndarray,
    axis_index: int,
    bounds: Sequence[Tuple[float, float]],
    lateral_force_limit: float,
) -> Tuple[float, float] | None:
    """Return the connected reachable interval around the trim output."""
    intervals = []
    for region in regions:
        interval = axis_delta_interval_for_region(
            region, target, axis_index, bounds, lateral_force_limit
        )
        if interval is not None:
            intervals.append(interval)
    for interval in merge_intervals(intervals):
        if interval[0] <= NUMERIC_TOL and interval[1] >= -NUMERIC_TOL:
            return interval
    return None


def failed_shape_evaluation(
    beta_deg: float,
    gamma_deg: float,
    scenario: JointScenario,
    reason: str,
    mass: float,
    center_of_mass: np.ndarray,
    rotors: List[dict],
    allocation_condition: float = math.inf,
    hover_thrusts: np.ndarray | None = None,
    hover_wrench: np.ndarray | None = None,
    required_thrust: float = math.inf,
    lateral_accel: float = math.inf,
) -> ShapeEvaluation:
    return ShapeEvaluation(
        beta_deg, gamma_deg, scenario, False, reason, mass, center_of_mass, rotors,
        allocation_condition,
        np.full(4, np.nan) if hover_thrusts is None else hover_thrusts,
        np.full(6, np.nan) if hover_wrench is None else hover_wrench,
        required_thrust, lateral_accel,
        {label: 0.0 for label in CONTROL_LABELS},
        {label: 0.0 for label in CONTROL_LABELS},
        -math.inf,
    )


def evaluate_shape(
    model: HugmyModel,
    beta_deg: float,
    gamma_deg: float,
    scenario: JointScenario,
    settings: AnalysisSettings,
    characteristic_radius: float,
) -> ShapeEvaluation:
    regions, mass, center_of_mass, rotors = build_sign_regions(
        model, gamma_deg, scenario, settings.max_allocation_condition
    )
    if not regions:
        return failed_shape_evaluation(
            beta_deg, gamma_deg, scenario, "singular allocation",
            mass, center_of_mass, rotors,
        )

    target = np.array([mass * GRAVITY, 0.0, 0.0, 0.0])
    trim_candidates = []
    for region in regions:
        thrusts = region.inverse_control @ target
        if thrusts_match_signs(thrusts, region.signs):
            rating = required_thrust_rating(thrusts, settings.reverse_thrust_ratio)
            trim_candidates.append((rating, float(np.linalg.norm(thrusts)), region, thrusts))
    if not trim_candidates:
        return failed_shape_evaluation(
            beta_deg, gamma_deg, scenario, "no sign-consistent trim",
            mass, center_of_mass, rotors,
        )

    required_thrust, _, trim_region, hover_thrusts = min(
        trim_candidates, key=lambda candidate: candidate[:2]
    )
    matrix = trim_region.matrix
    condition = trim_region.condition
    hover_wrench = matrix @ hover_thrusts
    lateral_accel = float(np.linalg.norm(hover_wrench[:2]) / mass)

    if not np.all(np.isfinite(hover_thrusts)):
        return failed_shape_evaluation(
            beta_deg, gamma_deg, scenario, "non-finite trim",
            mass, center_of_mass, rotors, condition,
        )
    if lateral_accel > settings.max_lateral_accel + NUMERIC_TOL:
        return failed_shape_evaluation(
            beta_deg, gamma_deg, scenario, "lateral acceleration limit",
            mass, center_of_mass, rotors, condition,
            hover_thrusts, hover_wrench, required_thrust, lateral_accel,
        )

    bounds = thrust_bounds(model, settings)
    if not all(
        lower - NUMERIC_TOL <= thrust <= upper + NUMERIC_TOL
        for thrust, (lower, upper) in zip(hover_thrusts, bounds)
    ):
        return failed_shape_evaluation(
            beta_deg, gamma_deg, scenario, "thrust limit",
            mass, center_of_mass, rotors, condition,
            hover_thrusts, hover_wrench, required_thrust, lateral_accel,
        )

    margins: Dict[str, float] = {}
    normalized: Dict[str, float] = {}
    force_scale = mass * GRAVITY
    moment_scale = force_scale * characteristic_radius
    for axis_index, label in enumerate(CONTROL_LABELS):
        interval = axis_delta_interval_piecewise(
            regions, target, axis_index, bounds,
            mass * settings.max_lateral_accel,
        )
        if interval is None or not (interval[0] <= NUMERIC_TOL <= interval[1]):
            return failed_shape_evaluation(
                beta_deg, gamma_deg, scenario, f"no {label} margin",
                mass, center_of_mass, rotors, condition,
                hover_thrusts, hover_wrench, required_thrust, lateral_accel,
            )
        margin = max(0.0, min(-interval[0], interval[1]))
        margins[label] = margin
        normalized[label] = margin / (force_scale if label == "Fz" else moment_scale)

    return ShapeEvaluation(
        beta_deg, gamma_deg, scenario, True, "", mass, center_of_mass, rotors,
        condition, hover_thrusts, hover_wrench, required_thrust, lateral_accel,
        margins, normalized, min(normalized.values()),
    )


def evaluate_robust_configuration(
    model: HugmyModel,
    beta_deg: float,
    gamma_deg: float,
    scenarios: Sequence[JointScenario],
    settings: AnalysisSettings,
    characteristic_radius: float,
) -> RobustEvaluation:
    evaluations = [
        evaluate_shape(
            model, beta_deg, gamma_deg, scenario, settings, characteristic_radius
        )
        for scenario in scenarios
    ]
    failed = sum(not evaluation.feasible for evaluation in evaluations)
    worst_margin_case = min(evaluations, key=lambda evaluation: evaluation.rho)
    worst_thrust_case = max(evaluations, key=lambda evaluation: evaluation.required_thrust)
    return RobustEvaluation(
        beta_deg=beta_deg,
        gamma_deg=gamma_deg,
        scenario_count=len(evaluations),
        failed_scenarios=failed,
        feasible=failed == 0,
        worst_rho=worst_margin_case.rho if failed == 0 else -math.inf,
        worst_margin_case=worst_margin_case,
        worst_required_thrust=worst_thrust_case.required_thrust,
        worst_thrust_case=worst_thrust_case,
        worst_lateral_accel=max(evaluation.lateral_accel for evaluation in evaluations),
    )


# =============================================================================
# Robust gamma scan
# =============================================================================
def inclusive_range(start: float, stop: float, step: float) -> np.ndarray:
    values = list(np.arange(start, stop + NUMERIC_TOL, step))
    if not values or values[-1] < stop - NUMERIC_TOL:
        values.append(stop)
    return np.asarray(values, dtype=float)


def characteristic_rotor_radius(model: HugmyModel) -> float:
    straight = JointScenario("straight", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    _, _, rotors = configuration_geometry(model, 0.0, straight)
    return float(np.mean([np.linalg.norm(rotor["position"][:2]) for rotor in rotors]))


def scan_transition(
    model: HugmyModel,
    settings: AnalysisSettings,
    beta_max_deg: float,
):
    betas = inclusive_range(BETA_MIN_DEG, beta_max_deg, BETA_STEP_DEG)
    gammas = inclusive_range(GAMMA_MIN_DEG, GAMMA_MAX_DEG, GAMMA_STEP_DEG)
    radius = characteristic_rotor_radius(model)
    scenarios_by_beta = {
        float(beta): make_joint_scenarios(
            float(beta), settings.joint_step_deg, settings.arm_uncertainty
        )
        for beta in betas
    }

    results: List[RobustEvaluation] = []
    gamma_summary = []
    for gamma in gammas:
        per_beta = []
        for beta in betas:
            evaluation = evaluate_robust_configuration(
                model, float(beta), float(gamma), scenarios_by_beta[float(beta)],
                settings, radius,
            )
            results.append(evaluation)
            per_beta.append(evaluation)

        all_feasible = all(evaluation.feasible for evaluation in per_beta)
        worst_margin = min(per_beta, key=lambda evaluation: evaluation.worst_rho)
        worst_thrust = max(per_beta, key=lambda evaluation: evaluation.worst_required_thrust)
        gamma_summary.append({
            "gamma_deg": float(gamma),
            "all_feasible": all_feasible,
            "worst_rho": worst_margin.worst_rho if all_feasible else -math.inf,
            "worst_beta_deg": worst_margin.beta_deg if all_feasible else None,
            "required_thrust_N": worst_thrust.worst_required_thrust,
            "required_thrust_beta_deg": worst_thrust.beta_deg,
            "required_thrust_scenario": worst_thrust.worst_thrust_case.scenario,
            "worst_lateral_accel": max(evaluation.worst_lateral_accel for evaluation in per_beta),
        })

    feasible = [summary for summary in gamma_summary if summary["all_feasible"]]
    best_margin = max(feasible, key=lambda summary: summary["worst_rho"]) if feasible else None
    finite_capacity = [
        summary for summary in gamma_summary
        if np.isfinite(summary["required_thrust_N"])
        and summary["worst_lateral_accel"] <= settings.max_lateral_accel + NUMERIC_TOL
    ]
    best_capacity = (
        min(finite_capacity, key=lambda summary: summary["required_thrust_N"])
        if finite_capacity else None
    )
    return betas, gammas, results, gamma_summary, best_margin, best_capacity


# =============================================================================
# CSV and plots
# =============================================================================
def scenario_columns(prefix: str) -> List[str]:
    return [
        f"{prefix}_arm2_q1_deg", f"{prefix}_arm2_q2_deg", f"{prefix}_arm2_q3_deg",
        f"{prefix}_arm4_q1_deg", f"{prefix}_arm4_q2_deg", f"{prefix}_arm4_q3_deg",
    ]


def scenario_values(scenario: JointScenario) -> List[float]:
    return [*scenario.arm2_deg, *scenario.arm4_deg]


def write_csv(
    results: Sequence[RobustEvaluation],
    gamma_summary: Sequence[dict],
    out_dir: Path,
) -> None:
    with (out_dir / "transition_scan.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "beta_deg", "gamma_deg", "scenario_count", "failed_scenarios", "feasible",
            "worst_rho", "worst_margin_scenario", "worst_margin_failure_reason",
            *scenario_columns("margin"),
            "worst_required_thrust_N", "worst_thrust_scenario", *scenario_columns("thrust"),
            "hover_f1_N", "hover_f2_N", "hover_f3_N", "hover_f4_N",
            "hover_Fx_N", "hover_Fy_N", "lateral_accel_m_s2", "allocation_condition",
            "Fz_margin_N", "Mx_margin_Nm", "My_margin_Nm", "Mz_margin_Nm",
        ])
        for robust in results:
            margin_case = robust.worst_margin_case
            thrust_case = robust.worst_thrust_case
            margins = margin_case.margins
            writer.writerow([
                robust.beta_deg, robust.gamma_deg, robust.scenario_count,
                robust.failed_scenarios, int(robust.feasible),
                robust.worst_rho if robust.feasible else "",
                margin_case.scenario.label, margin_case.reason,
                *scenario_values(margin_case.scenario),
                robust.worst_required_thrust, thrust_case.scenario.label,
                *scenario_values(thrust_case.scenario), *thrust_case.hover_thrusts.tolist(),
                thrust_case.hover_wrench[0], thrust_case.hover_wrench[1],
                thrust_case.lateral_accel, thrust_case.allocation_condition,
                margins.get("Fz", ""), margins.get("Mx", ""),
                margins.get("My", ""), margins.get("Mz", ""),
            ])

    with (out_dir / "gamma_summary.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "gamma_deg", "all_scenarios_feasible", "worst_rho", "worst_beta_deg",
            "required_thrust_N", "required_thrust_beta_deg", "required_thrust_scenario",
            *scenario_columns("thrust"), "worst_lateral_accel_m_s2",
        ])
        for summary in gamma_summary:
            scenario = summary["required_thrust_scenario"]
            writer.writerow([
                summary["gamma_deg"], int(summary["all_feasible"]),
                summary["worst_rho"], summary["worst_beta_deg"],
                summary["required_thrust_N"], summary["required_thrust_beta_deg"],
                scenario.label, *scenario_values(scenario), summary["worst_lateral_accel"],
            ])


def plot_transition_heatmaps(
    betas: np.ndarray,
    gammas: np.ndarray,
    results: Sequence[RobustEvaluation],
    model: HugmyModel,
    out_dir: Path,
) -> Path:
    rho = np.full((len(gammas), len(betas)), np.nan)
    required = np.full_like(rho, np.nan)
    lookup = {
        (round(result.gamma_deg, 8), round(result.beta_deg, 8)): result
        for result in results
    }
    for gamma_index, gamma in enumerate(gammas):
        for beta_index, beta in enumerate(betas):
            result = lookup[(round(float(gamma), 8), round(float(beta), 8))]
            if result.feasible:
                rho[gamma_index, beta_index] = result.worst_rho
            if np.isfinite(result.worst_required_thrust):
                required[gamma_index, beta_index] = result.worst_required_thrust

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    extent = [betas[0], betas[-1], gammas[0], gammas[-1]]
    margin_image = axes[0].imshow(rho, origin="lower", aspect="auto", extent=extent)
    axes[0].set_title("Robust normalized control margin")
    fig.colorbar(margin_image, ax=axes[0], label="worst-case rho")
    thrust_image = axes[1].imshow(required, origin="lower", aspect="auto", extent=extent)
    if np.nanmin(required) <= model.max_thrust <= np.nanmax(required):
        axes[1].contour(
            betas, gammas, required, levels=[model.max_thrust],
            colors="white", linewidths=1.2,
        )
    axes[1].set_title("Worst-case hover thrust rating")
    fig.colorbar(thrust_image, ax=axes[1], label="required thrust [N/rotor]")
    for axis in axes:
        axis.set_xlabel("Total arm bend beta [deg]")
        axis.set_ylabel("Fixed cant gamma [deg]")
    fig.tight_layout()
    path = out_dir / "robust_transition_heatmaps.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_gamma_summary(gamma_summary: Sequence[dict], model: HugmyModel, out_dir: Path) -> Path:
    gamma = np.array([summary["gamma_deg"] for summary in gamma_summary])
    rho = np.array([
        summary["worst_rho"] if np.isfinite(summary["worst_rho"]) else np.nan
        for summary in gamma_summary
    ])
    required = np.array([summary["required_thrust_N"] for summary in gamma_summary])
    fig, axes = plt.subplots(2, 1, figsize=(8.5, 8.0), sharex=True)
    axes[0].plot(gamma, rho, marker="o", markersize=3)
    axes[0].set_ylabel("Worst-case normalized margin")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(gamma, required, marker="o", markersize=3)
    axes[1].axhline(model.max_thrust, color="tab:red", linestyle="--", label="configured limit")
    axes[1].set_xlabel("Fixed cant gamma [deg]")
    axes[1].set_ylabel("Required thrust [N/rotor]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle("Robust fixed-cant scan")
    fig.tight_layout()
    path = out_dir / "gamma_robust_summary.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def exact_piecewise_wrench_vertices(
    model: HugmyModel,
    gamma_deg: float,
    scenario: JointScenario,
    bounds: Sequence[Tuple[float, float]],
) -> np.ndarray:
    """Map thrust-box corners with the rate matching each corner's signs."""
    corners = itertools.product(*[(lower, upper) for lower, upper in bounds])
    vertices = []
    for thrusts in corners:
        thrusts_array = np.asarray(thrusts, dtype=float)
        signs = tuple(1 if thrust >= 0.0 else -1 for thrust in thrusts_array)
        matrix, _, _, _ = wrench_matrix(model, gamma_deg, scenario, signs)
        vertices.append(matrix @ thrusts_array)
    return np.asarray(vertices)


def plot_wrench_projection(
    model: HugmyModel,
    settings: AnalysisSettings,
    beta_deg: float,
    gamma_deg: float,
    scenario: JointScenario,
    out_dir: Path,
) -> Path:
    _, mass, _, _ = wrench_matrix(model, gamma_deg, scenario)
    vertices = exact_piecewise_wrench_vertices(
        model, gamma_deg, scenario, thrust_bounds(model, settings)
    )
    gravity = np.array([0.0, 0.0, mass * GRAVITY, 0.0, 0.0, 0.0])
    moments = (vertices - gravity)[:, 3:6]
    fig = plt.figure(figsize=(8, 7))
    axis = fig.add_subplot(111, projection="3d")
    try:
        hull = ConvexHull(moments, qhull_options="QJ")
        axis.plot_trisurf(
            hull.points[:, 0], hull.points[:, 1], hull.points[:, 2],
            triangles=hull.simplices, alpha=0.35, edgecolor="k", linewidth=0.4,
        )
    except Exception:
        pass
    axis.scatter(moments[:, 0], moments[:, 1], moments[:, 2], s=16)
    axis.set_xlabel("Mx [Nm]")
    axis.set_ylabel("My [Nm]")
    axis.set_zlabel("Mz [Nm]")
    axis.set_title(
        f"Worst-shape torque projection: beta={beta_deg:.1f}, gamma={gamma_deg:.1f} deg"
    )
    fig.tight_layout()
    path = out_dir / f"torque_projection_beta{beta_deg:.0f}_gamma{gamma_deg:.1f}.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def write_geometry_debug_csv(
    model: HugmyModel,
    betas: Sequence[float],
    gamma_deg: float,
    settings: AnalysisSettings,
    out_dir: Path,
) -> Path:
    path = out_dir / "geometry_debug.csv"
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "beta_deg", "gamma_deg", "scenario", *scenario_columns("shape"),
            "rotor_id", "spin", "com_x_m", "com_y_m", "com_z_m",
            "prop_body_x_m", "prop_body_y_m", "prop_body_z_m",
            "rel_x_m", "rel_y_m", "rel_z_m", "axis_x", "axis_y", "axis_z",
        ])
        for beta in betas:
            scenarios = make_joint_scenarios(
                float(beta), settings.joint_step_deg, settings.arm_uncertainty
            )
            for scenario in scenarios:
                _, center_of_mass, rotors = configuration_geometry(model, gamma_deg, scenario)
                for rotor in rotors:
                    writer.writerow([
                        float(beta), gamma_deg, scenario.label, *scenario_values(scenario),
                        rotor["id"], rotor["spin"], *center_of_mass.tolist(),
                        *rotor["position_body"].tolist(), *rotor["position"].tolist(),
                        *rotor["axis"].tolist(),
                    ])
    return path


def print_model(model: HugmyModel, settings: AnalysisSettings) -> None:
    straight = JointScenario("straight", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    mass, center_of_mass, rotors = configuration_geometry(model, 0.0, straight)
    print("\n=== HUGMY robust fixed-tilt model ===")
    print(f"mass                         : {mass:.4f} kg")
    print(f"CoM at beta=0, gamma=0       : {center_of_mass}")
    print(f"configured thrust limit      : {model.max_thrust:.3f} N / rotor")
    print(f"forward signed m_f_rate      : {model.m_f_rate_forward:.6f} Nm/N")
    print(f"reverse signed m_f_rate      : {model.m_f_rate_reverse:.6f} Nm/N")
    print(f"reverse/forward thrust ratio : {settings.reverse_thrust_ratio:.3f}")
    print(
        f"reverse thrust limit         : "
        f"{settings.reverse_thrust_ratio * model.max_thrust:.3f} N / rotor"
    )
    print(f"max resultant lateral accel  : {settings.max_lateral_accel:.3f} m/s^2")
    print(f"arm uncertainty mode         : {settings.arm_uncertainty}")
    print(f"joint-distribution grid      : {settings.joint_step_deg:.3f} deg")
    for rotor in rotors:
        print(
            f"  rotor {rotor['id']}: p={np.array2string(rotor['position_body'], precision=5)}, "
            f"spin={rotor['spin']:+d}, n={np.array2string(rotor['axis'], precision=4)}"
        )


# =============================================================================
# Command-line entry point
# =============================================================================
def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Robust fixed motor-cant optimization for HUGMY",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("xacro", nargs="?", default=str(DEFAULT_XACRO))
    parser.add_argument("--out", default="fixed_tilt_results_2")
    parser.add_argument("--beta-max", type=float, default=BETA_MAX_DEG)
    parser.add_argument("--max-thrust", type=float, default=None, help="forward limit [N/rotor]")
    parser.add_argument(
        "--m-f-rate-forward", type=float, default=M_F_RATE_FORWARD,
        help="signed reaction-torque/force ratio for positive thrust [Nm/N]",
    )
    parser.add_argument(
        "--m-f-rate-reverse", type=float, default=M_F_RATE_REVERSE,
        help="signed reaction-torque/force ratio for negative thrust [Nm/N]",
    )
    parser.add_argument("--reverse-ratio", type=float, default=REVERSE_THRUST_RATIO)
    parser.add_argument(
        "--max-reverse-thrust", type=float, default=None,
        help="absolute negative-thrust limit [N/rotor]; overrides --reverse-ratio",
    )
    parser.add_argument(
        "--max-lateral-accel", type=float, default=MAX_LATERAL_ACCEL_M_S2,
        help="maximum resultant sqrt(Fx^2+Fy^2)/mass [m/s^2]",
    )
    parser.add_argument("--joint-step", type=float, default=JOINT_SAMPLE_STEP_DEG)
    parser.add_argument(
        "--arm-uncertainty", choices=("symmetric", "extremes", "full"),
        default=ARM_UNCERTAINTY,
        help="arm2/arm4 pairing; extremes/full include asymmetric shapes",
    )
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--show-plots", action="store_true")
    parser.add_argument("--debug-geometry", action="store_true")
    parser.add_argument("--debug-gamma", type=float, default=0.0)
    return parser.parse_args()


def validate_arguments(args: argparse.Namespace) -> None:
    if not (BETA_MIN_DEG <= args.beta_max <= BETA_MAX_DEG):
        raise ValueError(
            f"beta-max must be in [{BETA_MIN_DEG}, {BETA_MAX_DEG}] deg "
            "for the Quad-to-Bi study"
        )
    if args.max_thrust is not None and args.max_thrust <= 0.0:
        raise ValueError("max-thrust must be positive")
    if args.reverse_ratio < 0.0:
        raise ValueError("reverse-ratio must be non-negative")
    if args.max_reverse_thrust is not None and args.max_reverse_thrust < 0.0:
        raise ValueError("max-reverse-thrust must be non-negative")
    if args.max_lateral_accel < 0.0:
        raise ValueError("max-lateral-accel must be non-negative")
    if args.joint_step <= 0.0:
        raise ValueError("joint-step must be positive")


def main() -> None:
    args = parse_arguments()
    validate_arguments(args)
    model = load_hugmy_xacro(args.xacro)
    model.m_f_rate_forward = float(args.m_f_rate_forward)
    model.m_f_rate_reverse = float(args.m_f_rate_reverse)
    if args.max_thrust is not None:
        model.max_thrust = float(args.max_thrust)

    reverse_ratio = float(args.reverse_ratio)
    if args.max_reverse_thrust is not None:
        reverse_ratio = float(args.max_reverse_thrust) / model.max_thrust
    settings = AnalysisSettings(
        reverse_thrust_ratio=reverse_ratio,
        max_lateral_accel=float(args.max_lateral_accel),
        joint_step_deg=float(args.joint_step),
        arm_uncertainty=args.arm_uncertainty,
    )
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print_model(model, settings)
    print(
        f"\nScanning beta={BETA_MIN_DEG:.1f}..{args.beta_max:.1f} deg, "
        f"gamma={GAMMA_MIN_DEG:.1f}..{GAMMA_MAX_DEG:.1f} deg"
    )
    betas, gammas, results, summary, best_margin, best_capacity = scan_transition(
        model, settings, args.beta_max
    )
    write_csv(results, summary, out_dir)

    print("\n=== Robust result ===")
    if best_margin is None:
        print(
            "No gamma satisfies every joint-shape scenario with the configured "
            f"{model.max_thrust:.3f} N/rotor limit."
        )
    else:
        print(f"best gamma by control margin : {best_margin['gamma_deg']:.2f} deg")
        print(f"worst-case rho              : {best_margin['worst_rho']:.6f}")
        print(f"worst beta                  : {best_margin['worst_beta_deg']:.1f} deg")
        print(f"required thrust             : {best_margin['required_thrust_N']:.3f} N/rotor")

    if best_capacity is None:
        print(
            "No gamma kept every geometry within the allocation and "
            "lateral-acceleration constraints."
        )
    else:
        scenario = best_capacity["required_thrust_scenario"]
        print("\nMinimum-capacity solution (hover only):")
        print(f"  gamma                      : {best_capacity['gamma_deg']:.2f} deg")
        print(f"  worst required thrust      : {best_capacity['required_thrust_N']:.3f} N/rotor")
        print(
            f"  worst beta                 : "
            f"{best_capacity['required_thrust_beta_deg']:.1f} deg"
        )
        print(f"  worst arm 2 joints         : {np.asarray(scenario.arm2_deg)} deg")
        print(f"  worst arm 4 joints         : {np.asarray(scenario.arm4_deg)} deg")
        print(f"  worst lateral acceleration : {best_capacity['worst_lateral_accel']:.4f} m/s^2")

    plot_files: List[Path] = []
    if not args.no_plots:
        plot_files.append(plot_transition_heatmaps(betas, gammas, results, model, out_dir))
        plot_files.append(plot_gamma_summary(summary, model, out_dir))
        selected = best_margin or best_capacity
        if selected is not None:
            plot_files.append(plot_wrench_projection(
                model, settings, float(selected["required_thrust_beta_deg"]),
                float(selected["gamma_deg"]), selected["required_thrust_scenario"], out_dir,
            ))

    if args.debug_geometry:
        debug_dir = out_dir / "geometry_debug"
        debug_dir.mkdir(parents=True, exist_ok=True)
        geometry_csv = write_geometry_debug_csv(
            model, betas, float(args.debug_gamma), settings, debug_dir
        )
        print(f"geometry scenarios written to: {geometry_csv.resolve()}")

    if plot_files:
        print("\nGenerated figures:")
        for path in plot_files:
            print(f"  {path.resolve()}")
        if args.show_plots:
            for path in plot_files:
                image = plt.imread(path)
                figure, axis = plt.subplots(figsize=(10, 7))
                axis.imshow(image)
                axis.axis("off")
                axis.set_title(path.stem)
                figure.tight_layout()
            plt.show()

    print(f"\nResults written to: {out_dir.resolve()}")


if __name__ == "__main__":
    main()
