#!/usr/bin/env python3
"""
Fixed-tilt optimization for HUGMY morphing quadrotor.

Based on the user's feasible_wrench_space.py, but adapted to:
  * read geometry/dynamics from hugmy2.urdf.xacro,
  * compute rotor positions/orientations and the configuration-dependent CoM,
  * morph two opposing arms from quad (beta=0 deg) toward birotor-like form,
  * add a fixed motor cant angle gamma about each arm's local radial axis,
  * build the exact 6x4 wrench allocation matrix at the CoM,
  * evaluate hover feasibility and Fz/Mx/My/Mz control margins,
  * scan gamma and beta to choose a robust fixed-cant angle,
  * optionally add a perception-weighted haptic-force metric.

IMPORTANT modeling assumptions (easy to change in CONFIG below):
  1. Arms 2 and 4 morph; arms 1 and 3 remain straight.
  2. Total arm bend beta is distributed equally over joints 1..3.
     (The rotor is mounted after joint 3 in the Xacro.)
  3. Fixed cant is tangential and alternates with rotor spin direction.
  4. 3D propeller thrust is initially modeled symmetrically: [-Tmax, +Tmax].
     Replace with measured forward/reverse bounds for final design.
  5. Reaction torque is Q = sigma * spin_sign * thrust along rotor axis.
     Replace sigma with measured Q/T for the 513D + selected motor.

Coordinate convention follows the URDF body frame.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linprog, lsq_linear
from scipy.spatial import ConvexHull


# =============================================================================
# CONFIG: edit here first
# =============================================================================
FOLDED_ARMS = (2, 4)          # opposing arms that morph toward the birotor-like form
BETA_MIN_DEG = 0.0
BETA_MAX_DEG = 150.0           # first study: 0 -> 90 deg. Can be extended to 150.
BETA_STEP_DEG = 5.0

GAMMA_MIN_DEG = 0.0           # fixed motor cant magnitude
GAMMA_MAX_DEG = 45.0
GAMMA_STEP_DEG = 0.5

USE_REVERSE_THRUST = True     # ideal 3D propeller model
REVERSE_THRUST_RATIO = 0.4    # |T_reverse,max| / T_forward,max; replace by measurement

# Fixed-cant sign pattern.
# "spin_alternating": gamma_i = spin_i * gamma. This preserves zero yaw bias
# at equal thrust in the symmetric quad and usually improves yaw authority.
# "same": all rotors use +gamma (mainly for comparison).
CANT_PATTERN = "spin_alternating"

# Transition score uses normalized symmetric margins around hover.
# Fz is normalized by mg; moments by mg * characteristic rotor radius.
TRANSITION_AXES = ("Fz", "Mx", "My", "Mz")

# Stage 2: human force-perception compensation.
# required physical FB/LR ratio: 1.0 = disabled/isotropic.
# Example: 1.4 means FB should be 1.4x stronger physically than LR.
PERCEPTION_FB_TO_LR_REQUIRED = 1.4
HAPTIC_BETA_DEG = 0.0
# Minimum fraction of Stage-1 transition score that Stage-2 candidates must retain.
TRANSITION_RETENTION = 0.95
STAGE2_BIAS_MIN_DEG = 0.0
STAGE2_BIAS_MAX_DEG = 90.0
STAGE2_BIAS_STEP_DEG = 5.0
STAGE2_GAMMA_STEP_DEG = 1.0

GRAVITY = 9.81
HOVER_RESIDUAL_TOL = 5e-3     # normalized least-squares residual


# =============================================================================
# Basic transforms
# =============================================================================
def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tf(R: np.ndarray | None = None, p: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = np.asarray(p, dtype=float)
    return T


def transform_point(T: np.ndarray, p: Sequence[float]) -> np.ndarray:
    ph = np.r_[np.asarray(p, dtype=float), 1.0]
    return (T @ ph)[:3]


def parse_vec(text: str) -> np.ndarray:
    return np.array([float(x) for x in text.strip().split()], dtype=float)


# =============================================================================
# Xacro model extraction (tailored to the supplied HUGMY Xacro)
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
    sigma: float
    main_body: LinkMass
    battery_mass: float
    battery_positions: List[np.ndarray]
    arm_link_masses: Dict[str, LinkMass]
    thrust_mass: LinkMass
    arms: List[ArmSpec]
    prop_origin_in_rotor: np.ndarray

    @property
    def total_mass_nominal(self) -> float:
        # Same masses irrespective of configuration.
        per_arm = sum(v.mass for v in self.arm_link_masses.values()) + self.thrust_mass.mass
        return self.main_body.mass + len(self.battery_positions) * self.battery_mass + 4 * per_arm


def _strip_xacro_expr(s: str) -> str:
    s = s.strip()
    if s.startswith("${") and s.endswith("}"):
        return s[2:-1]
    return s


def _safe_eval(expr: str, variables: Dict[str, float]) -> float:
    expr = _strip_xacro_expr(expr)
    allowed = {"pi": math.pi, **variables}
    return float(eval(expr, {"__builtins__": {}}, allowed))


def load_hugmy_xacro(path: str | Path) -> HugmyModel:
    path = Path(path)
    root = ET.parse(path).getroot()
    ns = {"xacro": "http://www.ros.org/wiki/xacro"}

    props: Dict[str, float] = {}
    # Most relevant properties are scalar numerics.
    for e in root.findall("xacro:property", ns):
        name = e.attrib.get("name")
        value = e.attrib.get("value", "")
        if not name:
            continue
        try:
            props[name] = _safe_eval(value, props)
        except Exception:
            pass

    link_length_n = props["link_length_n"]
    link_length_r = props["link_length_r"]
    max_thrust = props["max_force"]
    min_thrust_urdf = props["min_force"]

    # m_f_rate is a plain custom XML element in this file.
    mf = root.find("m_f_rate")
    sigma = abs(float(mf.attrib["value"])) if mf is not None else 0.0

    # Main body inertial.
    main_body_link = root.find("./link[@name='main_body']")
    if main_body_link is None:
        raise ValueError("main_body link not found")
    inertial = main_body_link.find("inertial")
    main_mass = float(inertial.find("mass").attrib["value"])
    main_com = parse_vec(inertial.find("origin").attrib.get("xyz", "0 0 0"))

    # Batteries.
    battery_mass = 0.0
    battery_positions: List[np.ndarray] = []
    for bid in (0, 1):
        link = root.find(f"./link[@name='battery_{bid}']")
        joint = root.find(f"./joint[@name='battery_{bid}_joint']")
        if link is not None and joint is not None:
            battery_mass = float(link.find("inertial/mass").attrib["value"])
            battery_positions.append(parse_vec(joint.find("origin").attrib["xyz"]))

    # Arm macro link masses/coms.
    arm_macro = None
    for e in root.findall("xacro:macro", ns):
        if e.attrib.get("name") == "arm_module":
            arm_macro = e
            break
    if arm_macro is None:
        raise ValueError("xacro arm_module macro not found")

    arm_link_masses: Dict[str, LinkMass] = {}
    suffix_map = {
        "link_${id}_1": "link1",
        "link_${id}_2": "link2",
        "link_${id}_rotor": "rotor",
        "link_${id}_4": "link4",
        "link_${id}_end": "end",
    }
    for link in arm_macro.findall("link"):
        name = link.attrib.get("name")
        if name not in suffix_map:
            continue
        inert = link.find("inertial")
        mass = float(inert.find("mass").attrib["value"])
        com = parse_vec(inert.find("origin").attrib.get("xyz", "0 0 0"))
        arm_link_masses[suffix_map[name]] = LinkMass(mass, com)

    # Thrust-link mass and propeller joint origin are in propeller_module macro.
    prop_macro = None
    for e in root.findall("xacro:macro", ns):
        if e.attrib.get("name") == "propeller_module":
            prop_macro = e
            break
    if prop_macro is None:
        raise ValueError("xacro propeller_module macro not found")

    thrust_link = prop_macro.find("./link[@name='thrust${id}']")
    thrust_inert = thrust_link.find("inertial")
    thrust_mass = LinkMass(
        float(thrust_inert.find("mass").attrib["value"]),
        parse_vec(thrust_inert.find("origin").attrib.get("xyz", "0 0 0")),
    )
    rotor_joint = prop_macro.find("./joint[@name='rotor${id}']")
    prop_origin = parse_vec(rotor_joint.find("origin").attrib["xyz"])

    # Arm calls: get base offsets.
    base_offsets: Dict[int, np.ndarray] = {}
    for e in root.findall("xacro:arm_module", ns):
        aid = int(e.attrib["id"])
        base_offsets[aid] = parse_vec(e.attrib["base_offset"])

    # Propeller calls: get spin signs.
    spin_signs: Dict[int, int] = {}
    for e in root.findall("xacro:propeller_module", ns):
        aid = int(e.attrib["id"])
        spin_signs[aid] = int(round(_safe_eval(e.attrib["direction"], props)))

    arms: List[ArmSpec] = []
    for aid in sorted(base_offsets):
        # Copied from joint_${id}_1 origin rpy expression in the Xacro.
        yaw = -3.0 * math.pi / 4.0 + math.pi / 2.0 * (aid - 1)
        arms.append(ArmSpec(aid, base_offsets[aid], yaw, spin_signs[aid]))

    return HugmyModel(
        link_length_n=link_length_n,
        link_length_r=link_length_r,
        max_thrust=max_thrust,
        min_thrust_urdf=min_thrust_urdf,
        sigma=sigma,
        main_body=LinkMass(main_mass, main_com),
        battery_mass=battery_mass,
        battery_positions=battery_positions,
        arm_link_masses=arm_link_masses,
        thrust_mass=thrust_mass,
        arms=arms,
        prop_origin_in_rotor=prop_origin,
    )


# =============================================================================
# Configuration-dependent kinematics and CoM
# =============================================================================
def arm_joint_angles(total_bend_rad: float) -> Tuple[float, float, float, float, float]:
    """Constant-curvature approximation: total rotor bend equally over joints 1..3."""
    q = total_bend_rad / 3.0
    # Supplied limits: joint1 <= 50 deg, joint2/3 <= 60 deg.
    if q > math.radians(50.0) + 1e-9:
        raise ValueError(
            f"Requested total bend {math.degrees(total_bend_rad):.1f} deg exceeds "
            "the current equal-distribution model (joint1 limit 50 deg => total 150 deg)."
        )
    return q, q, q, 0.0, 0.0


def arm_transforms(model: HugmyModel, arm: ArmSpec, total_bend_rad: float):
    q1, q2, q3, q4, q5 = arm_joint_angles(total_bend_rad)

    # URDF joint axis is local (0,-1,0), therefore rotation is Ry(-q).
    T1 = tf(p=arm.base_offset) @ tf(rot_z(arm.base_yaw)) @ tf(rot_y(-q1))
    T2 = T1 @ tf(p=(-model.link_length_n, 0, 0)) @ tf(rot_y(-q2))
    Tr = T2 @ tf(p=(-model.link_length_n, 0, 0)) @ tf(rot_y(-q3))
    T4 = Tr @ tf(p=(-model.link_length_r, 0, 0)) @ tf(rot_y(-q4))
    Te = T4 @ tf(p=(-model.link_length_n, 0, 0)) @ tf(rot_y(-q5))
    Tp = Tr @ tf(p=model.prop_origin_in_rotor)
    return {"link1": T1, "link2": T2, "rotor": Tr, "link4": T4, "end": Te, "prop": Tp}


def configuration_geometry(
    model: HugmyModel,
    beta_deg: float,
    gamma_deg: float,
    folded_arms: Sequence[int] = FOLDED_ARMS,
    cant_pattern: str = CANT_PATTERN,
    haptic_bias_deg: float = 0.0,
):
    beta = math.radians(beta_deg)
    gamma = math.radians(gamma_deg)

    mass_points: List[Tuple[float, np.ndarray]] = []
    mass_points.append((model.main_body.mass, model.main_body.com_local.copy()))
    for p in model.battery_positions:
        mass_points.append((model.battery_mass, p.copy()))

    rotor_tmp = []
    for arm in model.arms:
        bend = beta if arm.arm_id in folded_arms else 0.0
        Ts = arm_transforms(model, arm, bend)

        for lname in ("link1", "link2", "rotor", "link4", "end"):
            lm = model.arm_link_masses[lname]
            mass_points.append((lm.mass, transform_point(Ts[lname], lm.com_local)))
        mass_points.append((model.thrust_mass.mass, transform_point(Ts["prop"], model.thrust_mass.com_local)))

        # Fixed motor cant.  At haptic_bias_deg=0 this is the Stage-1
        # spin-alternating tangential cant.  Stage 2 blends that direction
        # toward an FB-biased pattern while keeping equal-thrust lateral force
        # balanced in the nominal quad configuration.
        Rrot = Ts["prop"][:3, :3]
        if cant_pattern == "spin_alternating":
            # Local +y is tangential for the supplied arm frames.
            v_yaw_local = arm.spin_sign * np.array([0.0, 1.0, 0.0])

            # FB-biased pattern: motors on +x and -x sides cant in opposite
            # global body-x directions. Convert that desired direction into
            # the *nominal* local rotor frame so it stays fixed in the mount.
            Ts0 = arm_transforms(model, arm, 0.0)
            R0 = Ts0["prop"][:3, :3]
            x_sign = 1.0 if Ts0["prop"][0, 3] >= 0.0 else -1.0
            v_fb_global = np.array([x_sign, 0.0, 0.0])
            v_fb_local = R0.T @ v_fb_global
            v_fb_local[2] = 0.0
            v_fb_local /= np.linalg.norm(v_fb_local)

            eta = math.radians(haptic_bias_deg)
            v_local = math.cos(eta) * v_yaw_local + math.sin(eta) * v_fb_local
            v_local[2] = 0.0
            v_local /= np.linalg.norm(v_local)
            n_local = math.cos(gamma) * np.array([0.0, 0.0, 1.0]) + math.sin(gamma) * v_local
        elif cant_pattern == "same":
            n_local = rot_x(gamma) @ np.array([0.0, 0.0, 1.0])
        else:
            raise ValueError(f"Unknown cant pattern: {cant_pattern}")

        n = Rrot @ n_local
        n /= np.linalg.norm(n)
        rotor_tmp.append((arm, Ts["prop"][:3, 3].copy(), n))

    M = sum(m for m, _ in mass_points)
    com = sum(m * p for m, p in mass_points) / M

    rotors = []
    for arm, p, n in rotor_tmp:
        rotors.append(
            {
                "id": arm.arm_id,
                "spin": arm.spin_sign,
                # Absolute position in the URDF/main_body frame.
                "position_body": p.copy(),
                # Position relative to the current configuration-dependent CoM.
                "position": p - com,
                "axis": n,
            }
        )

    return M, com, rotors


# =============================================================================
# Wrench model
# =============================================================================
def wrench_matrix(model: HugmyModel, beta_deg: float, gamma_deg: float, haptic_bias_deg: float = 0.0):
    mass, com, rotors = configuration_geometry(model, beta_deg, gamma_deg, haptic_bias_deg=haptic_bias_deg)
    B = np.zeros((6, 4))
    for j, rtr in enumerate(rotors):
        r = rtr["position"]
        n = rtr["axis"]
        spin = rtr["spin"]
        B[:3, j] = n
        B[3:, j] = np.cross(r, n) + model.sigma * spin * n
    return B, mass, com, rotors


def thrust_bounds(model: HugmyModel) -> List[Tuple[float, float]]:
    if USE_REVERSE_THRUST:
        lo = -REVERSE_THRUST_RATIO * model.max_thrust
    else:
        lo = model.min_thrust_urdf
    return [(lo, model.max_thrust)] * 4


def exact_wrench_vertices(B: np.ndarray, bounds: Sequence[Tuple[float, float]]) -> np.ndarray:
    """Exact vertices generated by the 2^4 thrust-box corners for fixed rotor axes."""
    corners = itertools.product(*[(lo, hi) for lo, hi in bounds])
    return np.array([B @ np.asarray(f, dtype=float) for f in corners])


# =============================================================================
# Hover / control margin calculations
# =============================================================================
def characteristic_radius(model: HugmyModel) -> float:
    _, _, rotors = configuration_geometry(model, beta_deg=0.0, gamma_deg=0.0)
    rs = [np.linalg.norm(r["position"][:2]) for r in rotors]
    return float(np.mean(rs))


def hover_trim_full_wrench(B: np.ndarray, mass: float, bounds, r_char: float):
    """
    Find a level-hover trim against the full 6D wrench target [0,0,mg,0,0,0].
    Uses normalized bounded least squares because the system is overdetermined (6x4).
    """
    target = np.array([0.0, 0.0, mass * GRAVITY, 0.0, 0.0, 0.0])
    fscale = max(mass * GRAVITY, 1e-9)
    mscale = max(mass * GRAVITY * r_char, 1e-9)
    S = np.diag([1 / fscale] * 3 + [1 / mscale] * 3)
    lo = np.array([b[0] for b in bounds])
    hi = np.array([b[1] for b in bounds])
    sol = lsq_linear(S @ B, S @ target, bounds=(lo, hi), lsmr_tol="auto")
    residual = float(np.linalg.norm(S @ (B @ sol.x - target)))
    return sol.x, residual, B @ sol.x - target


def lp_axis_limits(A: np.ndarray, target: np.ndarray, axis_idx: int, bounds):
    """
    Extremes of one output of A f while keeping the other outputs at target.
    Here A is the reduced [Fz, Mx, My, Mz] allocation matrix.
    """
    other = [i for i in range(A.shape[0]) if i != axis_idx]
    Aeq = A[other, :]
    beq = target[other]

    # max
    res_max = linprog(-A[axis_idx, :], A_eq=Aeq, b_eq=beq, bounds=bounds, method="highs")
    # min
    res_min = linprog(A[axis_idx, :], A_eq=Aeq, b_eq=beq, bounds=bounds, method="highs")
    if not (res_max.success and res_min.success):
        return None
    vmax = float(A[axis_idx, :] @ res_max.x)
    vmin = float(A[axis_idx, :] @ res_min.x)
    return vmin, vmax, res_min.x, res_max.x


def evaluate_configuration(model: HugmyModel, beta_deg: float, gamma_deg: float, r_char: float, haptic_bias_deg: float = 0.0):
    B, mass, com, rotors = wrench_matrix(model, beta_deg, gamma_deg, haptic_bias_deg=haptic_bias_deg)
    bounds = thrust_bounds(model)

    f_hover, hover_residual, hover_error = hover_trim_full_wrench(B, mass, bounds, r_char)
    if hover_residual > HOVER_RESIDUAL_TOL:
        return {
            "feasible": False,
            "beta_deg": beta_deg,
            "gamma_deg": gamma_deg,
            "hover_residual": hover_residual,
            "B": B,
            "mass": mass,
            "com": com,
            "rotors": rotors,
        }

    # Reduced controllable outputs for a quadrotor-like attitude controller.
    rows = [2, 3, 4, 5]  # Fz, Mx, My, Mz
    A = B[rows, :]
    target = np.array([mass * GRAVITY, 0.0, 0.0, 0.0])
    labels = ["Fz", "Mx", "My", "Mz"]

    margins = {}
    limits = {}
    for j, lab in enumerate(labels):
        lim = lp_axis_limits(A, target, j, bounds)
        if lim is None:
            return {
                "feasible": False,
                "beta_deg": beta_deg,
                "gamma_deg": gamma_deg,
                "hover_residual": hover_residual,
                "B": B,
                "mass": mass,
                "com": com,
                "rotors": rotors,
            }
        vmin, vmax, _, _ = lim
        center = target[j]
        neg = center - vmin
        pos = vmax - center
        margins[lab] = min(neg, pos)
        limits[lab] = (vmin, vmax)

    # Normalize force and moments into comparable, dimensionless margins.
    force_scale = mass * GRAVITY
    moment_scale = mass * GRAVITY * r_char
    normalized = {
        "Fz": margins["Fz"] / force_scale,
        "Mx": margins["Mx"] / moment_scale,
        "My": margins["My"] / moment_scale,
        "Mz": margins["Mz"] / moment_scale,
    }
    rho = min(normalized[a] for a in TRANSITION_AXES)

    # Full-wrench hover trim diagnostic.
    lateral_hover = float(np.linalg.norm((B @ f_hover)[:2]))
    max_abs = max(max(abs(lo), abs(hi)) for lo, hi in bounds)
    hover_util = float(np.max(np.abs(f_hover)) / max_abs)

    return {
        "feasible": True,
        "beta_deg": beta_deg,
        "gamma_deg": gamma_deg,
        "hover_residual": hover_residual,
        "hover_error": hover_error,
        "hover_thrusts": f_hover,
        "hover_lateral_force": lateral_hover,
        "hover_utilization": hover_util,
        "margins": margins,
        "normalized": normalized,
        "limits": limits,
        "rho": rho,
        "B": B,
        "mass": mass,
        "com": com,
        "rotors": rotors,
    }


# =============================================================================
# Haptic force metric (Stage 2 scaffold)
# =============================================================================
def force_projection_extents(B: np.ndarray, bounds) -> Dict[str, float]:
    """Raw support of the feasible force projection in +/- body x/y directions."""
    F = B[:3, :]
    out = {}
    for lab, d in {
        "+FB": np.array([1.0, 0.0, 0.0]),
        "-FB": np.array([-1.0, 0.0, 0.0]),
        "+LR": np.array([0.0, 1.0, 0.0]),
        "-LR": np.array([0.0, -1.0, 0.0]),
    }.items():
        c = -(d @ F)
        res = linprog(c, bounds=bounds, method="highs")
        out[lab] = float(d @ F @ res.x) if res.success else float("nan")
    out["FB_sym"] = min(out["+FB"], out["-FB"])
    out["LR_sym"] = min(out["+LR"], out["-LR"])
    out["ratio_FB_LR"] = out["FB_sym"] / max(out["LR_sym"], 1e-12)
    return out


def haptic_score(model: HugmyModel, gamma_deg: float, haptic_bias_deg: float = 0.0) -> Dict[str, float]:
    B, _, _, _ = wrench_matrix(model, HAPTIC_BETA_DEG, gamma_deg, haptic_bias_deg=haptic_bias_deg)
    ext = force_projection_extents(B, thrust_bounds(model))
    target_ratio = PERCEPTION_FB_TO_LR_REQUIRED
    # Score in (0,1], 1 is exact ratio match; logarithm treats reciprocal errors symmetrically.
    ratio = max(ext["ratio_FB_LR"], 1e-12)
    score = math.exp(-abs(math.log(ratio / target_ratio)))
    ext["score"] = score
    return ext


# =============================================================================
# Scan and optimize
# =============================================================================
def frange(start: float, stop: float, step: float) -> np.ndarray:
    n = int(round((stop - start) / step))
    return np.linspace(start, stop, n + 1)


def scan_transition(model: HugmyModel, beta_max_deg: float = BETA_MAX_DEG):
    betas = frange(BETA_MIN_DEG, beta_max_deg, BETA_STEP_DEG)
    gammas = frange(GAMMA_MIN_DEG, GAMMA_MAX_DEG, GAMMA_STEP_DEG)
    r_char = characteristic_radius(model)

    results = []
    gamma_summary = []
    for gamma in gammas:
        per_beta = []
        for beta in betas:
            ev = evaluate_configuration(model, float(beta), float(gamma), r_char)
            results.append(ev)
            per_beta.append(ev)

        feasible = [e for e in per_beta if e["feasible"]]
        all_feasible = len(feasible) == len(per_beta)
        worst_rho = min((e["rho"] for e in feasible), default=-np.inf) if all_feasible else -np.inf
        worst_beta = None
        if all_feasible:
            worst_beta = min(feasible, key=lambda e: e["rho"])["beta_deg"]
        gamma_summary.append(
            {
                "gamma_deg": float(gamma),
                "all_feasible": all_feasible,
                "worst_rho": float(worst_rho),
                "worst_beta_deg": worst_beta,
            }
        )

    feasible_summaries = [g for g in gamma_summary if g["all_feasible"]]
    best = max(feasible_summaries, key=lambda g: g["worst_rho"]) if feasible_summaries else None
    return betas, gammas, results, gamma_summary, best


def stage2_select(model: HugmyModel, best_stage1):
    """
    Stage 2 scans fixed cant magnitude gamma and a bias angle eta.
    eta=0 keeps the Stage-1 tangential/yaw pattern; increasing eta biases the
    fixed mount directions toward body FB (x) capability.  Candidates must
    preserve at least TRANSITION_RETENTION of the Stage-1 worst-case margin.
    """
    if best_stage1 is None:
        return None, []

    rho_best = best_stage1["worst_rho"]
    r_char = characteristic_radius(model)
    betas = frange(BETA_MIN_DEG, BETA_MAX_DEG, BETA_STEP_DEG)
    gammas = frange(GAMMA_MIN_DEG, GAMMA_MAX_DEG, STAGE2_GAMMA_STEP_DEG)
    etas = frange(STAGE2_BIAS_MIN_DEG, STAGE2_BIAS_MAX_DEG, STAGE2_BIAS_STEP_DEG)

    candidates = []
    for eta in etas:
        for gamma in gammas:
            evs = [evaluate_configuration(model, float(beta), float(gamma), r_char, haptic_bias_deg=float(eta)) for beta in betas]
            if not all(e["feasible"] for e in evs):
                continue
            worst_rho = min(e["rho"] for e in evs)
            if worst_rho < TRANSITION_RETENTION * rho_best:
                continue
            hs = haptic_score(model, float(gamma), haptic_bias_deg=float(eta))
            candidates.append({
                "gamma_deg": float(gamma),
                "haptic_bias_deg": float(eta),
                "worst_rho": float(worst_rho),
                **{f"haptic_{k}": v for k, v in hs.items()},
            })

    best = max(candidates, key=lambda x: x["haptic_score"]) if candidates else None
    return best, candidates


# =============================================================================
# Visualization / CSV
# =============================================================================
def plot_transition_heatmap(betas, gammas, results, out_dir: Path):
    rho = np.full((len(gammas), len(betas)), np.nan)
    lookup = {(round(e["gamma_deg"], 8), round(e["beta_deg"], 8)): e for e in results}
    for ig, g in enumerate(gammas):
        for ib, b in enumerate(betas):
            e = lookup[(round(float(g), 8), round(float(b), 8))]
            if e["feasible"]:
                rho[ig, ib] = e["rho"]

    fig, ax = plt.subplots(figsize=(9, 5.5))
    im = ax.imshow(
        rho,
        origin="lower",
        aspect="auto",
        extent=[betas[0], betas[-1], gammas[0], gammas[-1]],
    )
    ax.set_xlabel("Arm bend beta [deg]")
    ax.set_ylabel("Fixed cant gamma [deg]")
    ax.set_title("Normalized control margin over Quad -> Bi transition")
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("rho = minimum normalized Fz/Mx/My/Mz margin")
    fig.tight_layout()
    fig.savefig(out_dir / "transition_margin_heatmap.png", dpi=180)
    plt.close(fig)


def plot_gamma_summary(gamma_summary, out_dir: Path):
    xs = [g["gamma_deg"] for g in gamma_summary]
    ys = [g["worst_rho"] if np.isfinite(g["worst_rho"]) else np.nan for g in gamma_summary]
    fig, ax = plt.subplots(figsize=(8, 4.8))
    ax.plot(xs, ys, marker="o", markersize=3)
    ax.set_xlabel("Fixed cant gamma [deg]")
    ax.set_ylabel("Worst-case normalized margin")
    ax.set_title("Robust fixed-cant optimization")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "gamma_worst_case_margin.png", dpi=180)
    plt.close(fig)


def plot_wrench_projection(model: HugmyModel, beta_deg: float, gamma_deg: float, out_dir: Path):
    B, mass, _, _ = wrench_matrix(model, beta_deg, gamma_deg)
    V = exact_wrench_vertices(B, thrust_bounds(model))
    gravity = np.array([0.0, 0.0, mass * GRAVITY, 0.0, 0.0, 0.0])
    Voff = V - gravity
    M = Voff[:, 3:6]

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    try:
        h = ConvexHull(M, qhull_options="QJ")
        ax.plot_trisurf(
            h.points[:, 0], h.points[:, 1], h.points[:, 2], triangles=h.simplices,
            alpha=0.35, edgecolor="k", linewidth=0.4,
        )
    except Exception:
        pass
    ax.scatter(M[:, 0], M[:, 1], M[:, 2], s=16)
    ax.set_xlabel("Mx [Nm]")
    ax.set_ylabel("My [Nm]")
    ax.set_zlabel("Mz [Nm]")
    ax.set_title(f"Raw torque projection: beta={beta_deg:.1f} deg, gamma={gamma_deg:.1f} deg")
    fig.tight_layout()
    fig.savefig(out_dir / f"torque_projection_beta{beta_deg:.0f}_gamma{gamma_deg:.1f}.png", dpi=180)
    plt.close(fig)


def write_csv(results, gamma_summary, out_dir: Path):
    with open(out_dir / "transition_scan.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "beta_deg", "gamma_deg", "feasible", "rho", "hover_residual", "hover_utilization",
            "Fz_margin_N", "Mx_margin_Nm", "My_margin_Nm", "Mz_margin_Nm",
        ])
        for e in results:
            if e["feasible"]:
                m = e["margins"]
                w.writerow([
                    e["beta_deg"], e["gamma_deg"], 1, e["rho"], e["hover_residual"], e["hover_utilization"],
                    m["Fz"], m["Mx"], m["My"], m["Mz"],
                ])
            else:
                w.writerow([e["beta_deg"], e["gamma_deg"], 0, "", e["hover_residual"], "", "", "", "", ""])

    with open(out_dir / "gamma_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gamma_deg", "all_beta_feasible", "worst_rho", "worst_beta_deg"])
        for g in gamma_summary:
            w.writerow([g["gamma_deg"], int(g["all_feasible"]), g["worst_rho"], g["worst_beta_deg"]])




def _set_equal_2d(ax, xs, ys, pad=0.02):
    """Set equal XY scaling with a small symmetric padding."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    xmin, xmax = float(xs.min()), float(xs.max())
    ymin, ymax = float(ys.min()), float(ys.max())
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    half = max(0.5 * (xmax - xmin), 0.5 * (ymax - ymin), 0.02) + pad
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal", adjustable="box")


def write_geometry_debug_csv(model: HugmyModel, betas, gamma_deg: float, out_dir: Path):
    """Write absolute/main_body and CoM-relative rotor geometry for every beta."""
    path = out_dir / "geometry_debug.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "beta_deg", "gamma_deg", "rotor_id", "spin",
            "com_x_m", "com_y_m", "com_z_m",
            "body_x_m", "body_y_m", "body_z_m",
            "rel_x_m", "rel_y_m", "rel_z_m",
            "axis_x", "axis_y", "axis_z",
        ])
        for beta in betas:
            _, com, rotors = configuration_geometry(model, float(beta), float(gamma_deg))
            for r in rotors:
                pb = r["position_body"]
                pr = r["position"]
                n = r["axis"]
                w.writerow([
                    float(beta), float(gamma_deg), r["id"], r["spin"],
                    *com.tolist(), *pb.tolist(), *pr.tolist(), *n.tolist(),
                ])
    return path


def plot_geometry_debug_for_beta(model: HugmyModel, beta_deg: float, gamma_deg: float, out_dir: Path):
    """
    Debug geometry for one morphing angle beta.

    Four panels are produced:
      1) top view in main_body/URDF coordinates,
      2) x-z side view in main_body/URDF coordinates,
      3) top view centered at the current CoM,
      4) x-z side view centered at the current CoM.

    The small arrows show each rotor thrust axis projection.
    """
    _, com, rotors = configuration_geometry(model, beta_deg, gamma_deg)
    body = np.array([r["position_body"] for r in rotors])
    rel = np.array([r["position"] for r in rotors])
    axes = np.array([r["axis"] for r in rotors])

    fig, axs = plt.subplots(2, 2, figsize=(11, 10))
    ax_abs_xy, ax_abs_xz = axs[0]
    ax_rel_xy, ax_rel_xz = axs[1]

    # Absolute top view: main_body origin, CoM, and rotor positions.
    ax_abs_xy.scatter([0.0], [0.0], marker="+", s=120, label="main_body origin")
    ax_abs_xy.scatter([com[0]], [com[1]], marker="x", s=100, label="CoM")
    for r, p, n in zip(rotors, body, axes):
        ax_abs_xy.scatter(p[0], p[1], s=55)
        ax_abs_xy.text(p[0], p[1], f" R{r['id']}", va="bottom")
        ax_abs_xy.arrow(p[0], p[1], 0.025*n[0], 0.025*n[1],
                        head_width=0.003, length_includes_head=True)
    _set_equal_2d(ax_abs_xy,
                  np.r_[body[:,0], 0.0, com[0]],
                  np.r_[body[:,1], 0.0, com[1]])
    ax_abs_xy.set_xlabel("x in main_body [m]")
    ax_abs_xy.set_ylabel("y in main_body [m]")
    ax_abs_xy.set_title("Top view: absolute rotor positions")
    ax_abs_xy.grid(True, alpha=0.3)
    ax_abs_xy.legend(loc="best", fontsize=8)

    # Absolute x-z side view.
    ax_abs_xz.scatter([0.0], [0.0], marker="+", s=120, label="main_body origin")
    ax_abs_xz.scatter([com[0]], [com[2]], marker="x", s=100, label="CoM")
    for r, p, n in zip(rotors, body, axes):
        ax_abs_xz.scatter(p[0], p[2], s=55)
        ax_abs_xz.text(p[0], p[2], f" R{r['id']}", va="bottom")
        ax_abs_xz.arrow(p[0], p[2], 0.025*n[0], 0.025*n[2],
                        head_width=0.003, length_includes_head=True)
    _set_equal_2d(ax_abs_xz,
                  np.r_[body[:,0], 0.0, com[0]],
                  np.r_[body[:,2], 0.0, com[2]])
    ax_abs_xz.set_xlabel("x in main_body [m]")
    ax_abs_xz.set_ylabel("z in main_body [m]")
    ax_abs_xz.set_title("Side view (x-z): absolute rotor positions")
    ax_abs_xz.grid(True, alpha=0.3)

    # CoM-relative top view.
    ax_rel_xy.scatter([0.0], [0.0], marker="x", s=100, label="CoM")
    body_origin_rel = -com
    ax_rel_xy.scatter([body_origin_rel[0]], [body_origin_rel[1]], marker="+", s=120, label="main_body origin")
    for r, p, n in zip(rotors, rel, axes):
        ax_rel_xy.scatter(p[0], p[1], s=55)
        ax_rel_xy.text(p[0], p[1], f" R{r['id']}", va="bottom")
        ax_rel_xy.arrow(p[0], p[1], 0.025*n[0], 0.025*n[1],
                        head_width=0.003, length_includes_head=True)
    _set_equal_2d(ax_rel_xy,
                  np.r_[rel[:,0], 0.0, body_origin_rel[0]],
                  np.r_[rel[:,1], 0.0, body_origin_rel[1]])
    ax_rel_xy.set_xlabel("x relative to CoM [m]")
    ax_rel_xy.set_ylabel("y relative to CoM [m]")
    ax_rel_xy.set_title("Top view: CoM-relative rotor positions")
    ax_rel_xy.grid(True, alpha=0.3)
    ax_rel_xy.legend(loc="best", fontsize=8)

    # CoM-relative x-z side view.
    ax_rel_xz.scatter([0.0], [0.0], marker="x", s=100, label="CoM")
    ax_rel_xz.scatter([body_origin_rel[0]], [body_origin_rel[2]], marker="+", s=120, label="main_body origin")
    for r, p, n in zip(rotors, rel, axes):
        ax_rel_xz.scatter(p[0], p[2], s=55)
        ax_rel_xz.text(p[0], p[2], f" R{r['id']}", va="bottom")
        ax_rel_xz.arrow(p[0], p[2], 0.025*n[0], 0.025*n[2],
                        head_width=0.003, length_includes_head=True)
    _set_equal_2d(ax_rel_xz,
                  np.r_[rel[:,0], 0.0, body_origin_rel[0]],
                  np.r_[rel[:,2], 0.0, body_origin_rel[2]])
    ax_rel_xz.set_xlabel("x relative to CoM [m]")
    ax_rel_xz.set_ylabel("z relative to CoM [m]")
    ax_rel_xz.set_title("Side view (x-z): CoM-relative rotor positions")
    ax_rel_xz.grid(True, alpha=0.3)

    fig.suptitle(
        f"Geometry debug: beta={beta_deg:.1f} deg, gamma={gamma_deg:.1f} deg\n"
        f"CoM(main_body)=[{com[0]:+.4f}, {com[1]:+.4f}, {com[2]:+.4f}] m",
        fontsize=12,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    path = out_dir / f"geometry_beta{beta_deg:05.1f}_gamma{gamma_deg:04.1f}.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def generate_geometry_debug(model: HugmyModel, betas, gamma_deg: float, out_dir: Path):
    """Generate per-beta geometry figures and one CSV table."""
    dbg_dir = out_dir / "geometry_debug"
    dbg_dir.mkdir(parents=True, exist_ok=True)
    csv_path = write_geometry_debug_csv(model, betas, gamma_deg, dbg_dir)
    pngs = [plot_geometry_debug_for_beta(model, float(beta), gamma_deg, dbg_dir) for beta in betas]
    return dbg_dir, csv_path, pngs


def print_geometry(model: HugmyModel):
    mass, com, rotors = configuration_geometry(model, 0.0, 0.0)
    print("\n=== Parsed HUGMY model ===")
    print(f"total mass from Xacro inertials : {mass:.4f} kg")
    print(f"CoM at beta=0, gamma=0        : {com}")
    print(f"max thrust from Xacro          : {model.max_thrust:.3f} N / rotor")
    print(f"drag moment ratio sigma        : {model.sigma:.6f} Nm/N")
    print("rotors at beta=0, gamma=0:")
    for r in rotors:
        print(
            f"  rotor {r['id']}: "
            f"p_body={np.array2string(r['position_body'], precision=5)}, "
            f"r_com={np.array2string(r['position'], precision=5)}, "
            f"spin={r['spin']:+d}, n={np.array2string(r['axis'], precision=4)}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("xacro", nargs="?", default="/home/miyamichi/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/hugmy/urdf/hugmy2.urdf.xacro")
    parser.add_argument("--out", default="fixed_tilt_results")
    parser.add_argument("--beta-max", type=float, default=BETA_MAX_DEG, help="90 for Quad->Bi study; up to 150 supported")
    parser.add_argument("--max-thrust", type=float, default=None, help="override Xacro max_force [N/rotor], useful for F60/F80+513D studies")
    parser.add_argument("--reverse-ratio", type=float, default=None, help="override |T_reverse,max|/T_forward,max")
    parser.add_argument("--perception-ratio", type=float, default=None, help="required physical FB/LR force ratio for Stage 2")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--show-plots", action="store_true", help="show generated figures interactively after saving")
    parser.add_argument(
        "--debug-geometry", action="store_true",
        help="save per-beta top/side geometry plots and geometry_debug.csv"
    )
    parser.add_argument(
        "--debug-gamma", type=float, default=0.0,
        help="fixed cant angle used only for thrust-axis arrows in geometry debug plots [deg]; positions are unaffected"
    )
    args = parser.parse_args()

    global REVERSE_THRUST_RATIO, PERCEPTION_FB_TO_LR_REQUIRED
    model = load_hugmy_xacro(args.xacro)
    if args.max_thrust is not None:
        model.max_thrust = float(args.max_thrust)
    if args.reverse_ratio is not None:
        REVERSE_THRUST_RATIO = float(args.reverse_ratio)
    if args.perception_ratio is not None:
        PERCEPTION_FB_TO_LR_REQUIRED = float(args.perception_ratio)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print_geometry(model)
    print(f"\nFolded arms: {FOLDED_ARMS}")
    print(f"3D thrust bounds: {thrust_bounds(model)[0]} N per rotor")
    print(f"Scanning beta={BETA_MIN_DEG:.1f}..{args.beta_max:.1f} deg, gamma={GAMMA_MIN_DEG:.1f}..{GAMMA_MAX_DEG:.1f} deg")

    betas, gammas, results, summary, best1 = scan_transition(model, beta_max_deg=args.beta_max)
    write_csv(results, summary, out_dir)

    if args.debug_geometry:
        dbg_dir, dbg_csv, dbg_pngs = generate_geometry_debug(
            model, betas, gamma_deg=float(args.debug_gamma), out_dir=out_dir
        )
        print("\n=== Geometry debug output ===")
        print(f"geometry CSV : {dbg_csv.resolve()}")
        print(f"figure dir   : {dbg_dir.resolve()}")
        print(f"figures      : {len(dbg_pngs)} files (one per beta)")

    print("\n=== Stage 1: Quad -> Bi robust fixed-cant optimization ===")
    if best1 is None:
        print("No gamma kept a feasible level-hover trim over the complete beta range.")
        print("Inspect transition_scan.csv / heatmap. This may indicate the cant pattern or folded-arm pair should be changed.")
    else:
        print(f"best gamma            : {best1['gamma_deg']:.2f} deg")
        print(f"worst-case rho        : {best1['worst_rho']:.5f}")
        print(f"worst beta            : {best1['worst_beta_deg']:.1f} deg")

        # Print detailed endpoints and bottleneck (without duplicate beta values).
        r_char = characteristic_radius(model)
        beta_report = list(dict.fromkeys([0.0, best1["worst_beta_deg"], args.beta_max]))
        for b in beta_report:
            e = evaluate_configuration(model, b, best1["gamma_deg"], r_char)
            if e["feasible"]:
                print(
                    f"  beta={b:6.1f}: rho={e['rho']:.5f}, "
                    f"margins [Fz={e['margins']['Fz']:.3f} N, "
                    f"Mx={e['margins']['Mx']:.4f}, My={e['margins']['My']:.4f}, Mz={e['margins']['Mz']:.4f} Nm], "
                    f"hover f={np.array2string(e['hover_thrusts'], precision=3)}"
                )

    print("\n=== Stage 2: perception-weighted candidate selection ===")
    if abs(PERCEPTION_FB_TO_LR_REQUIRED - 1.0) < 1e-12:
        print("PERCEPTION_FB_TO_LR_REQUIRED=1.0, so perception compensation is currently disabled.")
        print("Set it to your measured physical FB/LR compensation ratio (e.g. 1.4) and rerun.")
        best2 = None
    else:
        best2, candidates = stage2_select(model, best1)
        if best2 is None:
            print("No Stage-2 candidate met the transition-retention constraint.")
        else:
            print(f"target physical FB/LR ratio : {PERCEPTION_FB_TO_LR_REQUIRED:.3f}")
            print(f"selected gamma              : {best2['gamma_deg']:.2f} deg")
            print(f"selected haptic bias eta    : {best2['haptic_bias_deg']:.2f} deg")
            print(f"transition rho              : {best2['worst_rho']:.5f}")
            print(f"predicted FB/LR force ratio : {best2['haptic_ratio_FB_LR']:.3f}")
            print(f"haptic ratio-match score    : {best2['haptic_score']:.5f}")

    if not args.no_plots:
        plot_transition_heatmap(betas, gammas, results, out_dir)
        plot_gamma_summary(summary, out_dir)
        if best1 is not None:
            plot_wrench_projection(model, 0.0, best1["gamma_deg"], out_dir)
            plot_wrench_projection(model, args.beta_max, best1["gamma_deg"], out_dir)

        plot_files = [
            out_dir / "transition_margin_heatmap.png",
            out_dir / "gamma_worst_case_margin.png",
        ]
        if best1 is not None:
            plot_files += [
                out_dir / f"torque_projection_beta0_gamma{best1['gamma_deg']:.1f}.png",
                out_dir / f"torque_projection_beta{args.beta_max:.0f}_gamma{best1['gamma_deg']:.1f}.png",
            ]
        print("\nGenerated figures:")
        for q in plot_files:
            print(f"  {q.resolve()}")

        if args.show_plots:
            # The plotting functions save-and-close their figures, so reopen the
            # saved PNGs in one Matplotlib window set. This keeps batch mode clean
            # while allowing interactive inspection on a desktop.
            for q in plot_files:
                if not q.exists():
                    continue
                img = plt.imread(q)
                fig, ax = plt.subplots(figsize=(9, 6))
                ax.imshow(img)
                ax.axis("off")
                ax.set_title(q.stem)
                fig.tight_layout()
            plt.show()

    print(f"\nResults written to: {out_dir.resolve()}")


if __name__ == "__main__":
    main()
