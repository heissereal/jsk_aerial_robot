#!/usr/bin/env python3
"""Guarded one-stride trajectory player for a secured Hugmy test article.

The node starts in shadow mode and never actuates unless both ``shadow`` is
false and ``allow_actuation`` is true.  Every arming permits exactly one
trajectory.  A stale mocap/IMU/pressure stream or a safety-envelope violation
immediately neutralizes thrust and leaves the arms at the configured grasp
pressure.
"""

from __future__ import annotations

import json
import math
import threading
import time
from bisect import bisect_right
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np
import rosgraph
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from spinal.msg import Imu, NeuronAdcStates, NeuronImuStates, Pressure, PwmTest
from std_msgs.msg import Float32, Float32MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


ARM_COUNT = 4
FRONT_ARMS = (0, 3)
REAR_ARMS = (1, 2)


def topic_names(value: Any) -> List[str]:
    """Accept one ROS topic or an ordered list of equivalent topic routes."""
    values = value if isinstance(value, list) else [value]
    result = []
    for item in values:
        name = str(item)
        if name and name not in result:
            result.append(name)
    if not result:
        raise ValueError("sensor topic route must not be empty")
    return result


def quaternion_matrix(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        raise ValueError("invalid zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def relative_roll_pitch(initial: Sequence[float], current: Sequence[float]) -> Tuple[float, float]:
    relative = quaternion_matrix(initial).T @ quaternion_matrix(current)
    roll = math.atan2(relative[2, 1], relative[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -relative[2, 0])))
    return roll, pitch


def load_trajectory(path: str) -> Dict[str, Any]:
    with Path(path).expanduser().open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if document.get("format") != "hugmy_stride_trajectory" or int(document.get("version", 0)) != 1:
        raise ValueError("trajectory is not hugmy_stride_trajectory version 1")
    samples = document.get("samples", [])
    if not samples:
        raise ValueError("trajectory has no samples")
    previous_time = -1.0
    for index, sample in enumerate(samples):
        stamp = float(sample["time_sec"])
        if stamp <= previous_time:
            raise ValueError(f"sample {index} time is not strictly increasing")
        previous_time = stamp
        if len(sample["pressure_kpa"]) != ARM_COUNT or len(sample["thrust_n"]) != ARM_COUNT:
            raise ValueError(f"sample {index} must contain four pressures and four thrusts")
        values = [*sample["pressure_kpa"], *sample["thrust_n"], sample["bottom_pressure_kpa"]]
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"sample {index} contains a non-finite command")
        if "expected_arm_bend_rad" in sample:
            bend = sample["expected_arm_bend_rad"]
            if (len(bend) != ARM_COUNT
                    or not all(math.isfinite(float(value)) for value in bend)):
                raise ValueError(
                    f"sample {index} expected_arm_bend_rad must contain four finite values")
    direction = np.asarray(document.get("direction_body_xyz", []), dtype=np.float64)
    if direction.shape != (3,) or np.linalg.norm(direction) < 1.0e-9:
        raise ValueError("trajectory direction_body_xyz must be a nonzero 3-vector")
    document["direction_body_xyz"] = list(direction / np.linalg.norm(direction))
    return document


class MotorModel:
    def __init__(self, prefix: str = "/quadrotor/motor_info") -> None:
        self.minimum_pwm = float(rospy.get_param(prefix + "/min_pwm"))
        self.neutral_pwm = float(rospy.get_param(prefix + "/neutral_pwm", 0.75))
        self.maximum_pwm = float(rospy.get_param(prefix + "/max_pwm"))
        self.voltage = float(rospy.get_param(prefix + "/simulation/voltage", 21.2))
        self.conversion_mode = int(rospy.get_param(prefix + "/pwm_conversion_mode", 0))
        self.positive_below = bool(rospy.get_param(
            prefix + "/positive_thrust_below_neutral", True))

        def curve(root: str) -> Tuple[float, Tuple[float, float, float]]:
            count = int(rospy.get_param(root + "/vel_ref_num"))
            candidates = []
            for index in range(1, count + 1):
                reference = root + f"/ref{index}"
                reference_voltage = float(rospy.get_param(reference + "/voltage"))
                coefficients = tuple(float(rospy.get_param(
                    reference + f"/polynominal{order}")) for order in range(3))
                candidates.append((abs(reference_voltage - self.voltage),
                                   reference_voltage, coefficients))
            if not candidates:
                raise RuntimeError(f"no motor calibration curve below {root}")
            _, reference_voltage, coefficients = min(candidates, key=lambda item: item[0])
            return reference_voltage, coefficients

        # MotorInfo_3D_V keeps the low-PWM branch under /reverse for file-format
        # compatibility even though positive_thrust_below_neutral makes it the
        # positive-force branch.
        self.high_pwm_curve = curve(prefix)
        self.low_pwm_curve = curve(prefix + "/reverse")

    def force_from_pwm(self, pwm: float) -> float:
        if abs(pwm - self.neutral_pwm) <= 1.0e-9:
            return 0.0
        low = pwm < self.neutral_pwm
        reference_voltage, polynomial = self.low_pwm_curve if low else self.high_pwm_curve
        percent = 100.0 * pwm
        force = polynomial[0] + (
            polynomial[1] * percent + polynomial[2] * percent * percent) / 10.0
        exponent = 1.5 if self.conversion_mode == 1 else 2.0
        force *= (self.voltage / reference_voltage) ** exponent
        positive = low == self.positive_below
        return abs(force) if positive else -abs(force)

    def thrust_to_pwm(self, thrust_n: float) -> float:
        if abs(thrust_n) <= 0.01:
            return self.neutral_pwm
        low = (thrust_n > 0.0) == self.positive_below
        edge = self.minimum_pwm if low else self.maximum_pwm
        target = min(abs(thrust_n), abs(self.force_from_pwm(edge)))
        lower, upper = 0.0, 1.0
        for _ in range(50):
            fraction = 0.5 * (lower + upper)
            pwm = self.neutral_pwm + fraction * (edge - self.neutral_pwm)
            if abs(self.force_from_pwm(pwm)) < target:
                lower = fraction
            else:
                upper = fraction
        return self.neutral_pwm + 0.5 * (lower + upper) * (edge - self.neutral_pwm)


class RealStrideRunner:
    def __init__(self) -> None:
        trajectory_path = rospy.get_param("~trajectory")
        self.trajectory = load_trajectory(trajectory_path)
        self.samples: List[Mapping[str, Any]] = self.trajectory["samples"]
        self.shadow = bool(rospy.get_param("~shadow", True))
        self.allow_actuation = bool(rospy.get_param("~allow_actuation", False))
        self.rate_hz = float(rospy.get_param("~control_rate_hz", 50.0))
        self.sensor_timeout = float(rospy.get_param("~sensor_timeout_sec", 0.30))
        self.pressure_timeout = float(rospy.get_param("~pressure_timeout_sec", 0.75))
        self.maximum_duration = float(rospy.get_param("~maximum_duration_sec", 12.0))
        self.maximum_arm_pressure = float(rospy.get_param("~maximum_arm_pressure_kpa", 50.0))
        self.maximum_bottom_pressure = float(rospy.get_param("~maximum_bottom_pressure_kpa", 30.0))
        self.maximum_forward_thrust = float(rospy.get_param("~maximum_forward_thrust_n", 5.0))
        self.maximum_reverse_thrust = float(rospy.get_param("~maximum_reverse_thrust_n", 2.5))
        self.maximum_thrust_rate = float(rospy.get_param("~maximum_thrust_rate_n_s", 10.0))
        self.maximum_thrust_integral = float(rospy.get_param(
            "~maximum_thrust_integral_n_s", 60.0))
        if self.maximum_thrust_integral <= 0.0:
            raise ValueError("maximum_thrust_integral_n_s must be positive")
        feedback = rospy.get_param("~pressure_feedback", {})
        self.pressure_feedback_enabled = bool(feedback.get("enabled", True))
        self.pressure_feedback_arm_indices = [int(index) for index in feedback.get(
            "arm_indices", list(range(ARM_COUNT)))]
        if (not self.pressure_feedback_arm_indices
                or len(set(self.pressure_feedback_arm_indices))
                != len(self.pressure_feedback_arm_indices)
                or any(index < 0 or index >= ARM_COUNT
                       for index in self.pressure_feedback_arm_indices)):
            raise ValueError(
                "pressure_feedback/arm_indices must contain unique zero-based "
                "arm indices in [0, 3]")
        self.minimum_playback_rate = float(feedback.get(
            "minimum_playback_rate", 0.15))
        self.full_speed_arm_error = float(feedback.get(
            "full_speed_arm_error_kpa", 2.0))
        self.slow_arm_error = float(feedback.get(
            "slow_arm_error_kpa", 10.0))
        self.full_speed_bottom_error = float(feedback.get(
            "full_speed_bottom_error_kpa", 1.0))
        self.slow_bottom_error = float(feedback.get(
            "slow_bottom_error_kpa", 5.0))
        self.playback_rate_filter_tau = float(feedback.get(
            "rate_filter_time_constant_sec", 0.25))
        self.lead_in_duration = float(feedback.get(
            "lead_in_duration_sec", 1.0))
        if not 0.0 < self.minimum_playback_rate <= 1.0:
            raise ValueError("pressure_feedback/minimum_playback_rate must be in (0, 1]")
        if not 0.0 <= self.full_speed_arm_error < self.slow_arm_error:
            raise ValueError("arm pressure feedback error thresholds are invalid")
        if not 0.0 <= self.full_speed_bottom_error < self.slow_bottom_error:
            raise ValueError("bottom pressure feedback error thresholds are invalid")
        if self.playback_rate_filter_tau < 0.0 or self.lead_in_duration < 0.0:
            raise ValueError("pressure feedback timing parameters must be non-negative")
        angle_feedback = rospy.get_param("~angle_feedback", {})
        self.angle_feedback_enabled = bool(angle_feedback.get("enabled", True))
        self.angle_feedback_required = bool(angle_feedback.get("required", True))
        self.angle_feedback_arm_indices = [int(index) for index in
                                           angle_feedback.get(
                                               "playback_arm_indices", [0, 1])]
        if (not self.angle_feedback_arm_indices
                or len(set(self.angle_feedback_arm_indices))
                != len(self.angle_feedback_arm_indices)
                or any(index < 0 or index >= ARM_COUNT
                       for index in self.angle_feedback_arm_indices)):
            raise ValueError(
                "angle_feedback/playback_arm_indices must contain unique "
                "zero-based arm indices in [0, 3]")
        self.neuron_imu_timeout = float(angle_feedback.get(
            "neuron_timeout_sec", 0.30))
        self.neuron_acc_lpf_tau = float(angle_feedback.get(
            "acc_lpf_tau_sec", 0.20))
        self.arm_yaw_rad = np.asarray(angle_feedback.get(
            "arm_yaw_rad", [-2.3561944902, -0.7853981634,
                            0.7853981634, 2.3561944902]), dtype=np.float64)
        self.arm_bend_offset_rad = np.asarray(angle_feedback.get(
            "bend_offset_rad", [0.0] * ARM_COUNT), dtype=np.float64)
        self.angle_deadband = float(angle_feedback.get(
            "deadband_rad", math.radians(5.0)))
        self.full_speed_angle_error = float(angle_feedback.get(
            "full_speed_error_rad", math.radians(8.0)))
        self.slow_angle_error = float(angle_feedback.get(
            "slow_error_rad", math.radians(25.0)))
        self.angle_pressure_kp = float(angle_feedback.get(
            "pressure_kp_kpa_rad", 8.0))
        self.maximum_angle_pressure_correction = float(angle_feedback.get(
            "maximum_pressure_correction_kpa", 3.0))
        self.angle_pressure_correction_rate = float(angle_feedback.get(
            "pressure_correction_rate_kpa_s", 5.0))
        self.angle_thrust_kp = float(angle_feedback.get(
            "thrust_kp_n_rad", 0.5))
        self.maximum_angle_thrust_correction = float(angle_feedback.get(
            "maximum_thrust_correction_n", 0.25))
        self.angle_thrust_correction_rate = float(angle_feedback.get(
            "thrust_correction_rate_n_s", 0.5))
        self.pressure_bend_sign = np.asarray(angle_feedback.get(
            "pressure_bend_sign", [1.0] * ARM_COUNT), dtype=np.float64)
        self.thrust_bend_sign = np.asarray(angle_feedback.get(
            "thrust_bend_sign", [-1.0] * ARM_COUNT), dtype=np.float64)
        vector_parameters = {
            "arm_yaw_rad": self.arm_yaw_rad,
            "bend_offset_rad": self.arm_bend_offset_rad,
            "pressure_bend_sign": self.pressure_bend_sign,
            "thrust_bend_sign": self.thrust_bend_sign,
        }
        for name, values in vector_parameters.items():
            if values.shape != (ARM_COUNT,) or not np.all(np.isfinite(values)):
                raise ValueError(
                    f"angle_feedback/{name} must contain four finite values")
        if (self.neuron_imu_timeout <= 0.0 or self.neuron_acc_lpf_tau < 0.0
                or self.angle_deadband < 0.0
                or not 0.0 <= self.full_speed_angle_error < self.slow_angle_error
                or self.angle_pressure_kp < 0.0
                or self.maximum_angle_pressure_correction < 0.0
                or self.angle_pressure_correction_rate < 0.0
                or self.angle_thrust_kp < 0.0
                or self.maximum_angle_thrust_correction < 0.0
                or self.angle_thrust_correction_rate < 0.0):
            raise ValueError("angle_feedback contains an invalid negative value or threshold")
        self.trajectory_has_arm_bend = all(
            "expected_arm_bend_rad" in sample for sample in self.samples)
        if (self.angle_feedback_enabled and self.angle_feedback_required
                and not self.trajectory_has_arm_bend):
            raise ValueError(
                "angle feedback requires expected_arm_bend_rad in every trajectory sample; "
                "re-export the 20260921 policy trajectory")
        if self.angle_feedback_enabled and not self.trajectory_has_arm_bend:
            rospy.logwarn(
                "Trajectory has no expected_arm_bend_rad; angle feedback is disabled")
            self.angle_feedback_enabled = False
        self.maximum_progress = float(rospy.get_param("~maximum_progress_m", 0.100))
        self.maximum_lateral = float(rospy.get_param("~maximum_lateral_m", 0.050))
        self.maximum_vertical = float(rospy.get_param("~maximum_vertical_m", 0.080))
        self.maximum_roll = float(rospy.get_param("~maximum_relative_roll_rad", math.pi / 4))
        self.maximum_pitch = float(rospy.get_param("~maximum_relative_pitch_rad", math.pi / 3))
        self.maximum_measured_pressure = float(rospy.get_param(
            "~maximum_measured_pressure_kpa", 60.0))
        self.minimum_start_pressure = float(rospy.get_param(
            "~minimum_start_pressure_kpa", 20.0))
        self.maximum_start_speed = float(rospy.get_param(
            "~maximum_start_speed_m_s", 0.03))
        self.stop_grip_pressure = float(rospy.get_param("~stop_grip_pressure_kpa", 30.0))
        self.disable_pressure_on_stop = bool(rospy.get_param(
            "~disable_pressure_controller_on_stop", False))
        self.reject_other_pwm_publishers = bool(rospy.get_param(
            "~reject_other_pwm_publishers", True))
        self.reject_other_command_publishers = bool(rospy.get_param(
            "~reject_other_command_publishers", True))
        self.arm_slave_ids = [int(value) for value in rospy.get_param(
            "~arm_slave_ids", [1, 2, 3, 4])]
        if len(self.arm_slave_ids) != ARM_COUNT:
            raise ValueError("arm_slave_ids must contain four entries")
        topics = rospy.get_param("~topics")
        self.pwm_topic = str(topics["pwm_test"])
        self.pressure_target_topic = str(topics["pressure_target"])
        self.bottom_target_topic = str(topics["bottom_pressure_target"])
        self.pressure_enable_name = str(topics["pressure_enable_service"])

        self.lock = threading.RLock()
        self.armed = False
        self.running = False
        self.output_active = False
        self.stop_requested = False
        self.mocap: Dict[str, Any] = {}
        self.imu_time = 0.0
        self.body_up = np.full(3, np.nan)
        self.neuron_acc = np.zeros((ARM_COUNT, 3), dtype=np.float64)
        self.neuron_acc_initialized = np.zeros(ARM_COUNT, dtype=bool)
        self.neuron_imu_time = np.zeros(ARM_COUNT, dtype=np.float64)
        self.pressures = np.full(ARM_COUNT, np.nan)
        self.pressure_time = 0.0
        self.bottom_pressure = float("nan")
        self.bottom_pressure_time = 0.0
        self.mocap_speed = float("inf")
        self.last_conflict_check = 0.0
        self.cached_command_publishers: Dict[str, List[str]] = {}
        self.last_thrust = np.zeros(ARM_COUNT)
        self.angle_pressure_correction = np.zeros(ARM_COUNT)
        self.angle_thrust_correction = np.zeros(ARM_COUNT)
        self.last_result: Dict[str, Any] = {}
        self.sample_times = [float(sample["time_sec"]) for sample in self.samples]

        self.pressure_pub = rospy.Publisher(
            self.pressure_target_topic, Float32MultiArray, queue_size=1)
        self.bottom_pub = rospy.Publisher(
            self.bottom_target_topic, Float32, queue_size=1)
        self.pwm_pub = rospy.Publisher(self.pwm_topic, PwmTest, queue_size=1)
        self.command_debug_pub = rospy.Publisher(
            "~command", Float32MultiArray, queue_size=1)
        self.feedback_debug_pub = rospy.Publisher(
            "~playback_feedback", Float32MultiArray, queue_size=1)
        self.angle_feedback_debug_pub = rospy.Publisher(
            "~angle_feedback", Float32MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher("~result", String, queue_size=1, latch=True)
        self.sensor_subscribers = []
        for topic in topic_names(topics["mocap_pose"]):
            self.sensor_subscribers.append(rospy.Subscriber(
                topic, PoseStamped, self.mocap_callback, queue_size=10))
        for topic in topic_names(topics["spinal_imu"]):
            self.sensor_subscribers.append(rospy.Subscriber(
                topic, Imu, self.imu_callback, queue_size=10))
        for topic in topic_names(topics["neuron_adc"]):
            self.sensor_subscribers.append(rospy.Subscriber(
                topic, NeuronAdcStates, self.pressure_callback, queue_size=10))
        for topic in topic_names(topics["neuron_imu"]):
            self.sensor_subscribers.append(rospy.Subscriber(
                topic, NeuronImuStates, self.neuron_imu_callback, queue_size=10))
        for topic in topic_names(topics["bottom_pressure"]):
            self.sensor_subscribers.append(rospy.Subscriber(
                topic, Float32, self.bottom_pressure_callback, queue_size=10))
        for topic in topic_names(topics["bottom_pressure_sensor"]):
            self.sensor_subscribers.append(rospy.Subscriber(
                topic, Pressure, self.bottom_pressure_sensor_callback, queue_size=10))
        self.pressure_enable = rospy.ServiceProxy(self.pressure_enable_name, SetBool)
        rospy.Service("~arm", SetBool, self.arm_callback)
        rospy.Service("~run_once", Trigger, self.run_once_callback)
        rospy.Service("~stop", Trigger, self.stop_callback)
        self.motor_model = MotorModel(str(rospy.get_param(
            "~motor_info_prefix", "/quadrotor/motor_info")))
        self.publish_status("idle")
        rospy.on_shutdown(self.shutdown)

    def publish_status(self, status: str) -> None:
        rospy.loginfo("Hugmy stride runner: %s", status)
        self.status_pub.publish(String(data=status))

    def mocap_callback(self, message: PoseStamped) -> None:
        now = time.monotonic()
        position = np.asarray([
            message.pose.position.x, message.pose.position.y,
            message.pose.position.z], dtype=np.float64)
        with self.lock:
            if self.mocap:
                dt = now - float(self.mocap["time"])
                if 1.0e-4 < dt < 1.0:
                    self.mocap_speed = float(np.linalg.norm(
                        position - self.mocap["position"]) / dt)
            self.mocap = {
                "time": now,
                "position": position,
                "quaternion": np.asarray([
                    message.pose.orientation.x, message.pose.orientation.y,
                    message.pose.orientation.z, message.pose.orientation.w],
                    dtype=np.float64),
            }

    def imu_callback(self, message: Imu) -> None:
        quaternion = np.asarray(message.quaternion, dtype=np.float64)
        if (quaternion.shape != (4,) or not np.all(np.isfinite(quaternion))
                or np.linalg.norm(quaternion) < 1.0e-6):
            return
        body_up = quaternion_matrix(quaternion).T @ np.asarray(
            [0.0, 0.0, 1.0], dtype=np.float64)
        with self.lock:
            self.imu_time = time.monotonic()
            self.body_up = body_up

    def neuron_imu_callback(self, message: NeuronImuStates) -> None:
        now = time.monotonic()
        with self.lock:
            for imu in message.imus:
                if imu.slave_id not in self.arm_slave_ids:
                    continue
                arm = self.arm_slave_ids.index(imu.slave_id)
                acceleration = np.asarray(imu.acc, dtype=np.float64)
                if acceleration.shape != (3,) or not np.all(np.isfinite(acceleration)):
                    continue
                if self.neuron_acc_initialized[arm]:
                    dt = max(0.0, min(0.1, now - self.neuron_imu_time[arm]))
                    alpha = dt / max(1.0e-6, self.neuron_acc_lpf_tau + dt)
                    self.neuron_acc[arm] += alpha * (
                        acceleration - self.neuron_acc[arm])
                else:
                    self.neuron_acc[arm] = acceleration
                    self.neuron_acc_initialized[arm] = True
                self.neuron_imu_time[arm] = now

    @staticmethod
    def arm_bend_from_acceleration(
            body_up: np.ndarray, acceleration: np.ndarray,
            arm_yaw_rad: float, bend_offset_rad: float) -> float:
        """Match QuadBiController's gravity-based Spinal--Neuron bend."""
        body_norm = float(np.linalg.norm(body_up))
        measured_norm = float(np.linalg.norm(acceleration))
        if (not math.isfinite(body_norm) or not math.isfinite(measured_norm)
                or body_norm < 1.0e-6 or measured_norm < 1.0e-6):
            return float("nan")
        body = np.asarray(body_up, dtype=np.float64) / body_norm
        measured = np.asarray(acceleration, dtype=np.float64) / measured_norm
        cosine = math.cos(arm_yaw_rad)
        sine = math.sin(arm_yaw_rad)
        local_up_x = cosine * body[0] + sine * body[1]
        local_up_z = body[2]
        dot_xz = local_up_x * measured[0] + local_up_z * measured[2]
        cross_xz = local_up_z * measured[0] - local_up_x * measured[2]
        return float(np.clip(
            -math.atan2(cross_xz, dot_xz) - bend_offset_rad, 0.0, math.pi))

    def measured_arm_bend(self) -> np.ndarray:
        with self.lock:
            body_up = self.body_up.copy()
            acceleration = self.neuron_acc.copy()
        return np.asarray([
            self.arm_bend_from_acceleration(
                body_up, acceleration[arm], self.arm_yaw_rad[arm],
                self.arm_bend_offset_rad[arm])
            for arm in range(ARM_COUNT)
        ], dtype=np.float64)

    def pressure_callback(self, message: NeuronAdcStates) -> None:
        values = np.full(ARM_COUNT, np.nan)
        for adc in message.adcs:
            if adc.slave_id in self.arm_slave_ids:
                values[self.arm_slave_ids.index(adc.slave_id)] = float(adc.pressure)
        if np.all(np.isfinite(values)):
            with self.lock:
                self.pressures = values
                self.pressure_time = time.monotonic()

    def bottom_pressure_callback(self, message: Float32) -> None:
        if math.isfinite(message.data):
            with self.lock:
                self.bottom_pressure = float(message.data)
                self.bottom_pressure_time = time.monotonic()

    def bottom_pressure_sensor_callback(self, message: Pressure) -> None:
        if message.valid and math.isfinite(message.pressure):
            with self.lock:
                self.bottom_pressure = float(message.pressure)
                self.bottom_pressure_time = time.monotonic()

    def other_command_publishers(self) -> Dict[str, List[str]]:
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        except Exception as exc:  # ROS master query should not defeat E-stop.
            rospy.logwarn("Cannot inspect PWM publishers: %s", exc)
            return {}
        command_topics = {
            self.pwm_topic, self.pressure_target_topic,
            self.bottom_target_topic}
        return {
            topic: [node for node in nodes if node != rospy.get_name()]
            for topic, nodes in publishers
            if topic in command_topics
            and any(node != rospy.get_name() for node in nodes)
        }

    def readiness_error(self) -> str:
        now = time.monotonic()
        with self.lock:
            if not self.mocap or now - self.mocap.get("time", 0.0) > self.sensor_timeout:
                return "mocap/pose is missing or stale"
            quaternion = np.asarray(self.mocap.get("quaternion", []), dtype=float)
            if (quaternion.shape != (4,) or not np.all(np.isfinite(quaternion))
                    or np.linalg.norm(quaternion) < 1.0e-6):
                return "mocap orientation is invalid"
            if now - self.imu_time > self.sensor_timeout:
                return "Spinal IMU is missing or stale"
            if self.angle_feedback_enabled:
                if not np.all(np.isfinite(self.body_up)):
                    return "Spinal IMU orientation is unavailable for arm-angle feedback"
                if not np.all(self.neuron_acc_initialized):
                    return "four Neuron IMUs are unavailable for arm-angle feedback"
                if np.any(now - self.neuron_imu_time > self.neuron_imu_timeout):
                    return "four Neuron IMUs are missing or stale"
            if now - self.pressure_time > self.pressure_timeout:
                return "four arm pressures are missing or stale"
            if now - self.bottom_pressure_time > self.pressure_timeout:
                return "bottom pressure is missing or stale"
            if np.max(self.pressures) > self.maximum_measured_pressure:
                return "measured arm pressure exceeds the safety limit"
        if not self.shadow and not self.allow_actuation:
            return "actuation is locked; set both shadow:=false and allow_actuation:=true"
        if not self.shadow:
            # A ROS-master query at the 50 Hz command rate adds avoidable
            # latency. Refresh the competing-publisher check at 2 Hz.
            if now - self.last_conflict_check >= 0.5:
                self.cached_command_publishers = self.other_command_publishers()
                self.last_conflict_check = now
            rejected = {}
            for topic, nodes in self.cached_command_publishers.items():
                if ((topic == self.pwm_topic and self.reject_other_pwm_publishers)
                        or (topic != self.pwm_topic
                            and self.reject_other_command_publishers)):
                    rejected[topic] = nodes
            if rejected:
                description = "; ".join(
                    f"{topic}: {', '.join(nodes)}"
                    for topic, nodes in sorted(rejected.items()))
                return "other command publisher(s) are active: " + description
            if self.pwm_pub.get_num_connections() == 0:
                return f"no subscriber is connected to {self.pwm_topic}"
            if self.pressure_pub.get_num_connections() == 0:
                return "pressure target controller is not connected"
        return ""

    def arm_callback(self, request: Any) -> SetBoolResponse:
        if not request.data:
            self.stop_requested = True
            self.safe_stop("disarmed")
            return SetBoolResponse(success=True, message="disarmed and thrust neutralized")
        with self.lock:
            if self.running:
                return SetBoolResponse(success=False, message="one-stride trial is running")
        error = self.readiness_error()
        if error:
            return SetBoolResponse(success=False, message=error)
        with self.lock:
            if float(np.min(self.pressures)) < self.minimum_start_pressure:
                return SetBoolResponse(
                    success=False,
                    message="initial arm pressure is below the grasp threshold")
            if self.mocap_speed > self.maximum_start_speed:
                return SetBoolResponse(
                    success=False,
                    message="robot is moving too quickly to arm a stride")
        if not self.shadow:
            try:
                rospy.wait_for_service(self.pressure_enable_name, timeout=2.0)
                response = self.pressure_enable(True)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                return SetBoolResponse(success=False, message=str(exc))
            if not response.success:
                return SetBoolResponse(success=False, message=response.message)
        with self.lock:
            self.armed = True
            self.output_active = not self.shadow
            self.stop_requested = False
        mode = "SHADOW" if self.shadow else "ACTUATION"
        self.publish_status(f"armed for exactly one stride ({mode})")
        return SetBoolResponse(success=True, message=f"armed in {mode} mode")

    def stop_callback(self, _request: Any) -> TriggerResponse:
        self.stop_requested = True
        self.safe_stop("operator stop")
        return TriggerResponse(success=True, message="stop requested")

    def scales(self) -> Dict[str, float]:
        values = {name: float(rospy.get_param(f"~scales/{name}")) for name in (
            "front_pressure", "rear_pressure", "forward_thrust",
            "reverse_thrust", "bottom_pressure", "time")}
        bounds = rospy.get_param("~optimization/parameters", {})
        for name, value in values.items():
            if not math.isfinite(value):
                raise ValueError(f"scale {name} is not finite")
            if name in bounds and not (
                    float(bounds[name]["min"]) <= value <= float(bounds[name]["max"])):
                raise ValueError(f"scale {name}={value} is outside configured bounds")
        if values["time"] <= 0.0:
            raise ValueError("time scale must be positive")
        return values

    def scaled_command(self, sample: Mapping[str, Any], scales: Mapping[str, float]) -> Tuple[np.ndarray, np.ndarray, float]:
        pressure = np.asarray(sample["pressure_kpa"], dtype=np.float64)
        pressure[list(FRONT_ARMS)] *= scales["front_pressure"]
        pressure[list(REAR_ARMS)] *= scales["rear_pressure"]
        thrust = np.asarray(sample["thrust_n"], dtype=np.float64)
        thrust = np.where(thrust >= 0.0,
                          thrust * scales["forward_thrust"],
                          thrust * scales["reverse_thrust"])
        bottom = float(sample["bottom_pressure_kpa"]) * scales["bottom_pressure"]
        return (
            np.clip(pressure, 0.0, self.maximum_arm_pressure),
            np.clip(thrust, -self.maximum_reverse_thrust, self.maximum_forward_thrust),
            min(self.maximum_bottom_pressure, max(0.0, bottom)),
        )

    def interpolated_command(
            self, trajectory_time: float, scales: Mapping[str, float],
            initial_pressure: np.ndarray,
            initial_bottom: float) -> Tuple[np.ndarray, np.ndarray, float]:
        """Return a continuous command on the pressure-adapted time axis.

        A short lead-in connects the measured initial pneumatic state to the
        first learned pressure action while thrust remains neutral.  This
        avoids applying the first large pneumatic step while a rotor is
        already running.  Once learned playback starts, pressure and thrust
        share exactly the same interpolation factor, so slowing pneumatic
        playback cannot let thrust run ahead.
        """
        first_pressure, first_thrust, first_bottom = self.scaled_command(
            self.samples[0], scales)
        if trajectory_time < 0.0 and self.lead_in_duration > 0.0:
            fraction = np.clip(
                (trajectory_time + self.lead_in_duration) / self.lead_in_duration,
                0.0, 1.0)
            return (
                initial_pressure + fraction * (first_pressure - initial_pressure),
                np.zeros(ARM_COUNT, dtype=np.float64),
                float(initial_bottom + fraction * (first_bottom - initial_bottom)),
            )

        if trajectory_time <= self.sample_times[0]:
            return first_pressure, first_thrust, first_bottom
        if trajectory_time >= self.sample_times[-1]:
            return self.scaled_command(self.samples[-1], scales)

        upper = bisect_right(self.sample_times, trajectory_time)
        lower = upper - 1
        lower_time = self.sample_times[lower]
        upper_time = self.sample_times[upper]
        fraction = (trajectory_time - lower_time) / (upper_time - lower_time)
        lower_pressure, lower_thrust, lower_bottom = self.scaled_command(
            self.samples[lower], scales)
        upper_pressure, upper_thrust, upper_bottom = self.scaled_command(
            self.samples[upper], scales)
        return (
            lower_pressure + fraction * (upper_pressure - lower_pressure),
            lower_thrust + fraction * (upper_thrust - lower_thrust),
            float(lower_bottom + fraction * (upper_bottom - lower_bottom)),
        )

    def interpolated_arm_bend(self, trajectory_time: float) -> np.ndarray:
        """Interpolate the MuJoCo bend state associated with the learned path."""
        first = np.asarray(
            self.samples[0]["expected_arm_bend_rad"], dtype=np.float64)
        if trajectory_time <= self.sample_times[0]:
            return first
        if trajectory_time >= self.sample_times[-1]:
            return np.asarray(
                self.samples[-1]["expected_arm_bend_rad"], dtype=np.float64)
        upper = bisect_right(self.sample_times, trajectory_time)
        lower = upper - 1
        fraction = ((trajectory_time - self.sample_times[lower]) /
                    (self.sample_times[upper] - self.sample_times[lower]))
        lower_bend = np.asarray(
            self.samples[lower]["expected_arm_bend_rad"], dtype=np.float64)
        upper_bend = np.asarray(
            self.samples[upper]["expected_arm_bend_rad"], dtype=np.float64)
        return lower_bend + fraction * (upper_bend - lower_bend)

    @staticmethod
    def slew_array(current: np.ndarray, target: np.ndarray,
                   maximum_rate: float, dt: float) -> np.ndarray:
        maximum_change = max(0.0, maximum_rate * dt)
        return current + np.clip(
            target - current, -maximum_change, maximum_change)

    def apply_angle_feedback(
            self, pressure: np.ndarray, thrust: np.ndarray,
            trajectory_time: float, angle_bias: np.ndarray,
            dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray,
                               np.ndarray, np.ndarray]:
        """Add bounded fine corrections to the learned feed-forward command.

        The IMU mounting-frame offset is removed by ``angle_bias``, captured
        once at the beginning of the stride.  Thus the controller tracks the
        learned change in arm shape without assuming that the real Neuron's
        absolute gravity angle equals MuJoCo's joint-sum coordinate.
        """
        expected = self.interpolated_arm_bend(max(0.0, trajectory_time))
        raw_measured = self.measured_arm_bend()
        measured = np.clip(raw_measured + angle_bias, 0.0, math.pi)
        error = expected - measured
        active_error = np.sign(error) * np.maximum(
            0.0, np.abs(error) - self.angle_deadband)
        if not self.angle_feedback_enabled or trajectory_time < 0.0:
            desired_pressure_correction = np.zeros(ARM_COUNT)
            desired_thrust_correction = np.zeros(ARM_COUNT)
        else:
            desired_pressure_correction = np.clip(
                self.pressure_bend_sign * self.angle_pressure_kp * active_error,
                -self.maximum_angle_pressure_correction,
                self.maximum_angle_pressure_correction)
            desired_thrust_correction = np.clip(
                self.thrust_bend_sign * self.angle_thrust_kp * active_error,
                -self.maximum_angle_thrust_correction,
                self.maximum_angle_thrust_correction)
        self.angle_pressure_correction = self.slew_array(
            self.angle_pressure_correction, desired_pressure_correction,
            self.angle_pressure_correction_rate, dt)
        self.angle_thrust_correction = self.slew_array(
            self.angle_thrust_correction, desired_thrust_correction,
            self.angle_thrust_correction_rate, dt)
        corrected_pressure = np.clip(
            pressure + self.angle_pressure_correction,
            0.0, self.maximum_arm_pressure)
        corrected_thrust = np.clip(
            thrust + self.angle_thrust_correction,
            -self.maximum_reverse_thrust, self.maximum_forward_thrust)
        return corrected_pressure, corrected_thrust, expected, measured, error

    @staticmethod
    def error_playback_rate(error: float, full_speed_error: float,
                            slow_error: float, minimum_rate: float) -> float:
        """Map pressure tracking error to a continuous virtual-time rate."""
        if error <= full_speed_error:
            return 1.0
        if error >= slow_error:
            return minimum_rate
        fraction = (error - full_speed_error) / (slow_error - full_speed_error)
        return 1.0 - fraction * (1.0 - minimum_rate)

    def pressure_playback_rate(
            self, target_pressure: np.ndarray,
            target_bottom: float) -> Tuple[float, float, float, float]:
        """Slow the path using representative arms and the bottom chamber.

        All four arm pressures remain subject to the measured-pressure safety
        limit.  ``pressure_feedback/arm_indices`` only selects which arm
        response sets the shared pressure/thrust playback speed.  This lets a
        known slow or faulty channel remain safely monitored without forcing
        the complete learned trajectory to its minimum rate.
        """
        with self.lock:
            measured_pressure = self.pressures.copy()
            measured_bottom = float(self.bottom_pressure)
        arm_errors = np.abs(target_pressure - measured_pressure)
        reference_arm_error = float(np.max(
            arm_errors[self.pressure_feedback_arm_indices]))
        all_arm_error = float(np.max(arm_errors))
        bottom_error = abs(float(target_bottom) - measured_bottom)
        if not self.pressure_feedback_enabled:
            return 1.0, reference_arm_error, all_arm_error, bottom_error
        arm_rate = self.error_playback_rate(
            reference_arm_error, self.full_speed_arm_error, self.slow_arm_error,
            self.minimum_playback_rate)
        bottom_rate = self.error_playback_rate(
            bottom_error, self.full_speed_bottom_error, self.slow_bottom_error,
            self.minimum_playback_rate)
        return (min(arm_rate, bottom_rate), reference_arm_error,
                all_arm_error, bottom_error)

    def angle_playback_rate(
            self, angle_error: np.ndarray) -> Tuple[float, float, float]:
        """Use representative angle errors to pace the shared trajectory."""
        reference_error = float(np.max(
            np.abs(angle_error[self.angle_feedback_arm_indices])))
        all_error = float(np.max(np.abs(angle_error)))
        if not self.angle_feedback_enabled:
            return 1.0, reference_error, all_error
        return (self.error_playback_rate(
                    reference_error, self.full_speed_angle_error,
                    self.slow_angle_error, self.minimum_playback_rate),
                reference_error, all_error)

    def publish_command(self, pressure: np.ndarray, thrust: np.ndarray,
                        bottom: float) -> None:
        pwms = np.asarray([self.motor_model.thrust_to_pwm(value) for value in thrust])
        self.command_debug_pub.publish(Float32MultiArray(data=[
            *pressure.tolist(), *thrust.tolist(), float(bottom), *pwms.tolist()]))
        if self.shadow:
            return
        self.pressure_pub.publish(Float32MultiArray(data=pressure.tolist()))
        self.bottom_pub.publish(Float32(data=float(bottom)))
        self.pwm_pub.publish(PwmTest(
            motor_index=list(range(ARM_COUNT)), pwms=pwms.tolist()))

    def safety_metrics(self, start: Mapping[str, Any], axis: np.ndarray,
                       lateral_axis: np.ndarray) -> Tuple[Dict[str, float], str]:
        error = self.readiness_error()
        if error:
            return {}, error
        with self.lock:
            position = self.mocap["position"].copy()
            quaternion = self.mocap["quaternion"].copy()
        displacement = position - start["position"]
        progress = float(np.dot(displacement, axis))
        lateral = abs(float(np.dot(displacement, lateral_axis)))
        vertical = abs(float(displacement[2]))
        roll, pitch = relative_roll_pitch(start["quaternion"], quaternion)
        metrics = {
            "progress_m": progress, "lateral_m": lateral,
            "vertical_m": vertical, "relative_roll_rad": roll,
            "relative_pitch_rad": pitch,
        }
        if progress > self.maximum_progress:
            return metrics, "progress exceeded 100 mm runaway limit"
        if lateral > self.maximum_lateral:
            return metrics, "lateral displacement exceeded safety limit"
        if vertical > self.maximum_vertical:
            return metrics, "vertical displacement exceeded safety limit"
        if abs(roll) > self.maximum_roll or abs(pitch) > self.maximum_pitch:
            return metrics, "relative attitude exceeded safety limit"
        return metrics, ""

    def run_once_callback(self, _request: Any) -> TriggerResponse:
        with self.lock:
            if not self.armed:
                return TriggerResponse(success=False, message="arm the runner first")
            if self.running:
                return TriggerResponse(success=False, message="trial already running")
            # Consume the one-shot arm permission before executing.
            self.armed = False
            self.running = True
            self.stop_requested = False
            start = {
                "position": self.mocap["position"].copy(),
                "quaternion": self.mocap["quaternion"].copy(),
            }
        try:
            scales = self.scales()
            body_direction = np.asarray(
                self.trajectory["direction_body_xyz"], dtype=np.float64)
            signed_body_direction = body_direction * float(
                self.trajectory.get("target_direction", 1.0))
            axis = quaternion_matrix(start["quaternion"]) @ signed_body_direction
            axis /= np.linalg.norm(axis)
            lateral_axis = np.cross(np.asarray([0.0, 0.0, 1.0]), axis)
            if np.linalg.norm(lateral_axis) < 1.0e-6:
                lateral_axis = np.asarray([0.0, 1.0, 0.0])
            lateral_axis /= np.linalg.norm(lateral_axis)
        except Exception as exc:
            with self.lock:
                self.running = False
            self.safe_stop("invalid trial setup")
            return TriggerResponse(
                success=False, message=json.dumps({
                    "aborted": True,
                    "reason": f"{type(exc).__name__}: {exc}",
                    "shadow": self.shadow,
                    "scales": {},
                    "metrics": {},
                }, sort_keys=True))
        start_wall = time.monotonic()
        previous_wall = start_wall
        thrust_integral = 0.0
        thrust_change = 0.0
        playback_rate_integral = 0.0
        minimum_actual_playback_rate = 1.0
        maximum_reference_arm_tracking_error = 0.0
        maximum_arm_tracking_error = 0.0
        maximum_bottom_tracking_error = 0.0
        maximum_reference_angle_tracking_error = 0.0
        maximum_angle_tracking_error = 0.0
        maximum_angle_pressure_correction = 0.0
        maximum_angle_thrust_correction = 0.0
        maximum_tilt = 0.0
        aborted = False
        reason = ""
        metrics: Dict[str, float] = {}
        self.last_thrust.fill(0.0)
        self.angle_pressure_correction.fill(0.0)
        self.angle_thrust_correction.fill(0.0)
        with self.lock:
            initial_pressure = self.pressures.copy()
            initial_bottom = float(self.bottom_pressure)
        if self.angle_feedback_enabled:
            raw_initial_bend = self.measured_arm_bend()
            initial_expected_bend = self.interpolated_arm_bend(0.0)
            if not np.all(np.isfinite(raw_initial_bend)):
                with self.lock:
                    self.running = False
                self.safe_stop("invalid initial arm-angle measurement")
                result = {
                    "aborted": True,
                    "reason": "invalid initial arm-angle measurement",
                    "shadow": self.shadow, "scales": scales, "metrics": {},
                }
                self.last_result = result
                return TriggerResponse(
                    success=False, message=json.dumps(result, sort_keys=True))
            angle_bias = initial_expected_bend - raw_initial_bend
        else:
            angle_bias = np.zeros(ARM_COUNT)
        trajectory_time = -self.lead_in_duration
        filtered_playback_rate = 1.0
        self.publish_status("executing one pressure-adaptive stride")
        rate = rospy.Rate(self.rate_hz)
        try:
            trajectory_end = (float(self.samples[-1]["time_sec"])
                              + float(self.trajectory["control_dt"]))
            fastest_duration = (
                trajectory_end + self.lead_in_duration) * scales["time"]
            if fastest_duration > self.maximum_duration:
                raise RuntimeError(
                    "fastest trajectory duration exceeds maximum_duration_sec")
            while not rospy.is_shutdown():
                now = time.monotonic()
                elapsed = now - start_wall
                if self.stop_requested:
                    aborted, reason = True, "operator stop"
                    break
                if trajectory_time >= trajectory_end:
                    break
                if elapsed >= self.maximum_duration:
                    aborted = True
                    reason = "pressure-adaptive playback exceeded maximum_duration_sec"
                    break
                dt = max(0.0, min(0.1, now - previous_wall))
                pressure, requested_thrust, bottom = self.interpolated_command(
                    trajectory_time, scales, initial_pressure, initial_bottom)
                if self.angle_feedback_enabled:
                    (pressure, requested_thrust, expected_bend, measured_bend,
                     angle_error) = self.apply_angle_feedback(
                         pressure, requested_thrust, trajectory_time,
                         angle_bias, dt)
                else:
                    expected_bend = np.zeros(ARM_COUNT)
                    measured_bend = np.zeros(ARM_COUNT)
                    angle_error = np.zeros(ARM_COUNT)
                (requested_playback_rate, reference_arm_error, arm_error,
                 bottom_error) = (
                    self.pressure_playback_rate(pressure, bottom))
                (requested_angle_rate, reference_angle_error,
                 all_angle_error) = self.angle_playback_rate(angle_error)
                # The zero-thrust lead-in intentionally establishes the first
                # pressure command.  Do not let an angle that can only be
                # produced after learned thrust begins stall that lead-in.
                if trajectory_time < 0.0:
                    requested_angle_rate = 1.0
                requested_playback_rate = min(
                    requested_playback_rate, requested_angle_rate)
                if self.playback_rate_filter_tau > 0.0 and dt > 0.0:
                    alpha = dt / (self.playback_rate_filter_tau + dt)
                    filtered_playback_rate += alpha * (
                        requested_playback_rate - filtered_playback_rate)
                else:
                    filtered_playback_rate = requested_playback_rate
                filtered_playback_rate = float(np.clip(
                    filtered_playback_rate, self.minimum_playback_rate, 1.0))
                maximum_delta = self.maximum_thrust_rate * dt
                thrust = self.last_thrust + np.clip(
                    requested_thrust - self.last_thrust,
                    -maximum_delta, maximum_delta)
                thrust_integral += float(np.sum(np.abs(thrust))) * dt
                thrust_change += float(np.sum(np.abs(thrust - self.last_thrust)))
                self.publish_command(pressure, thrust, bottom)
                self.feedback_debug_pub.publish(Float32MultiArray(data=[
                    float(elapsed), float(trajectory_time),
                    filtered_playback_rate, reference_arm_error, bottom_error,
                    arm_error, reference_angle_error, all_angle_error]))
                self.angle_feedback_debug_pub.publish(Float32MultiArray(data=[
                    *expected_bend.tolist(), *measured_bend.tolist(),
                    *angle_error.tolist(),
                    *self.angle_pressure_correction.tolist(),
                    *self.angle_thrust_correction.tolist()]))
                self.last_thrust = thrust
                trajectory_time += (
                    dt * filtered_playback_rate / scales["time"])
                playback_rate_integral += filtered_playback_rate * dt
                minimum_actual_playback_rate = min(
                    minimum_actual_playback_rate, filtered_playback_rate)
                maximum_reference_arm_tracking_error = max(
                    maximum_reference_arm_tracking_error,
                    reference_arm_error)
                maximum_arm_tracking_error = max(
                    maximum_arm_tracking_error, arm_error)
                maximum_bottom_tracking_error = max(
                    maximum_bottom_tracking_error, bottom_error)
                maximum_reference_angle_tracking_error = max(
                    maximum_reference_angle_tracking_error,
                    reference_angle_error)
                maximum_angle_tracking_error = max(
                    maximum_angle_tracking_error, all_angle_error)
                maximum_angle_pressure_correction = max(
                    maximum_angle_pressure_correction,
                    float(np.max(np.abs(self.angle_pressure_correction))))
                maximum_angle_thrust_correction = max(
                    maximum_angle_thrust_correction,
                    float(np.max(np.abs(self.angle_thrust_correction))))
                previous_wall = now
                if thrust_integral > self.maximum_thrust_integral:
                    aborted = True
                    reason = "total thrust integral exceeded safety limit"
                    break
                metrics, reason = self.safety_metrics(start, axis, lateral_axis)
                if reason:
                    aborted = True
                    break
                maximum_tilt = max(maximum_tilt,
                                   abs(metrics["relative_roll_rad"]),
                                   abs(metrics["relative_pitch_rad"]))
                rate.sleep()
            if not metrics:
                metrics, safety_reason = self.safety_metrics(start, axis, lateral_axis)
                if safety_reason:
                    aborted, reason = True, safety_reason
        except Exception as exc:
            aborted, reason = True, f"{type(exc).__name__}: {exc}"
        finally:
            duration = time.monotonic() - start_wall
            metrics.update({
                "duration_sec": duration,
                "thrust_integral_n_s": thrust_integral,
                "thrust_change_n": thrust_change,
                "maximum_relative_tilt_rad": maximum_tilt,
                "trajectory_time_sec": max(0.0, trajectory_time),
                "average_playback_rate": (
                    playback_rate_integral / duration if duration > 0.0 else 0.0),
                "minimum_playback_rate": minimum_actual_playback_rate,
                "pressure_feedback_arm_indices": self.pressure_feedback_arm_indices,
                "maximum_reference_arm_tracking_error_kpa": (
                    maximum_reference_arm_tracking_error),
                "maximum_arm_tracking_error_kpa": maximum_arm_tracking_error,
                "maximum_bottom_tracking_error_kpa": maximum_bottom_tracking_error,
                "angle_feedback_enabled": self.angle_feedback_enabled,
                "angle_feedback_arm_indices": self.angle_feedback_arm_indices,
                "angle_bias_rad": angle_bias.tolist(),
                "maximum_reference_angle_tracking_error_rad": (
                    maximum_reference_angle_tracking_error),
                "maximum_angle_tracking_error_rad": maximum_angle_tracking_error,
                "maximum_angle_pressure_correction_kpa": (
                    maximum_angle_pressure_correction),
                "maximum_angle_thrust_correction_n": (
                    maximum_angle_thrust_correction),
            })
            result = {
                "aborted": aborted, "reason": reason,
                "shadow": self.shadow, "scales": scales, "metrics": metrics,
            }
            self.last_result = result
            self.result_pub.publish(String(data=json.dumps(result, sort_keys=True)))
            self.safe_stop(reason if aborted else "one stride complete")
            with self.lock:
                self.running = False
        return TriggerResponse(success=not aborted, message=json.dumps(result, sort_keys=True))

    def safe_stop(self, reason: str) -> None:
        with self.lock:
            self.armed = False
            self.last_thrust.fill(0.0)
            output_active = self.output_active
            self.output_active = False
        pressure = np.full(ARM_COUNT, min(
            self.stop_grip_pressure, self.maximum_arm_pressure))
        if self.shadow or output_active:
            self.publish_command(pressure, np.zeros(ARM_COUNT), 0.0)
        else:
            # Preserve observability without violating the explicit-arm rule.
            neutral = [self.motor_model.neutral_pwm] * ARM_COUNT
            self.command_debug_pub.publish(Float32MultiArray(data=[
                *pressure.tolist(), *([0.0] * ARM_COUNT), 0.0, *neutral]))
        if output_active:
            # Explicit neutral first, then an empty command to exit Spinal's
            # direct PwmTest mode and hand control back to the normal path.
            neutral = [self.motor_model.neutral_pwm] * ARM_COUNT
            self.pwm_pub.publish(PwmTest(
                motor_index=list(range(ARM_COUNT)), pwms=neutral))
            rospy.sleep(0.04)
            self.pwm_pub.publish(PwmTest())
            if self.disable_pressure_on_stop:
                try:
                    self.pressure_enable(False)
                except rospy.ServiceException as exc:
                    rospy.logerr("Could not disable pressure controller: %s", exc)
        self.publish_status("stopped: " + reason)

    def shutdown(self) -> None:
        self.stop_requested = True
        self.safe_stop("node shutdown")


def main() -> None:
    rospy.init_node("hugmy_real_stride_runner")
    RealStrideRunner()
    rospy.spin()


if __name__ == "__main__":
    main()
