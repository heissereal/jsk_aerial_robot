#!/usr/bin/env python3
"""Gymnasium interface for Hugmy's ROS-connected MuJoCo simulation.

The MuJoCo process keeps running between episodes. ``reset()`` asks the Hugmy
hardware plugin to restore qpos/qvel and pneumatic state at a simulation-thread
safe point, then establishes the initial four-arm grasp.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from spinal.msg import NeuronAdcStates, PwmTest
from std_msgs.msg import Float32, Float32MultiArray, UInt32
from std_srvs.srv import SetBool, Trigger


ARM_COUNT = 4
FRONT_ARMS = (0, 3)
REAR_ARMS = (1, 2)


def quaternion_to_roll_pitch(quaternion: Any) -> Tuple[float, float]:
    """Return intrinsic roll and pitch without depending on tf Python."""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sin_pitch)


class HugmyMujocoEnv(gym.Env):
    """Five-action Hugmy environment using the existing ROS MuJoCo plant.

    Normalized action ``[P_front, P_rear, T_front, T_rear, u_bottom]``:

    * pressure entries: -1 -> 0 kPa, +1 -> configured maximum pressure;
    * thrust entries: negative uses reverse authority, positive uses forward;
    * bottom: -1 -> 0 kPa, +1 -> configured maximum pressure.

    The single bottom chamber is mounted on the body centreline.  It changes
    support load/height but does not command a rocking direction; the arm
    pressures and rotor thrusts must create fore/aft rocking about the rounded
    support.

    Observation layout (33 values) is documented by ``observation_names``.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        robot_ns: str = "/quadrotor",
        control_dt: float = 0.10,
        episode_duration: float = 30.0,
        reset_pressure_kpa: float = 30.0,
        reset_settle_sim_sec: float = 1.5,
        maximum_pressure_kpa: float = 50.0,
        maximum_bottom_pressure_kpa: float = 30.0,
        maximum_forward_thrust_n: float = 6.0,
        maximum_reverse_thrust_n: float = 3.0,
        maximum_thrust_rate_n_s: float = 10.0,
        direction_xyz: Sequence[float] = (1.0, 0.0, 0.0),
        target_direction_mode: str = "forward",
        progress_weight: float = 1000.0,
        detach_penalty: float = 20.0,
        minimum_progress_contacts: int = 2,
        stride_target_m: float = 0.020,
        stride_preferred_maximum_m: float = 0.030,
        stride_failure_maximum_m: float = 0.100,
        stride_success_bonus: float = 20.0,
        stride_overshoot_weight_per_m: float = 500.0,
        stride_failure_penalty: float = 100.0,
        stride_recovery_contacts: int = 3,
        stride_recovery_velocity_m_s: float = 0.03,
        stride_recovery_max_total_thrust_n: float = 4.0,
        stride_recovery_duration_sec: float = 0.5,
        roll_weight: float = 2.0,
        pitch_weight: float = 2.0,
        bottom_usage_weight: float = 0.02,
        bottom_change_weight: float = 0.1,
        thrust_usage_weight_per_n: float = 0.02,
        thrust_change_weight_per_n: float = 0.05,
        thrust_soft_limit_n: float = 4.0,
        thrust_excess_weight_per_n2: float = 0.5,
        roll_safe_rad: float = math.radians(15.0),
        pitch_safe_rad: float = math.radians(30.0),
        fall_roll_rad: float = math.radians(60.0),
        fall_pitch_rad: float = math.radians(75.0),
        maximum_vertical_excursion_m: float = 0.25,
        detached_grace_sec: float = 0.20,
        wall_step_timeout_sec: float = 10.0,
        synchronous_step: bool = True,
        action_latch_wall_sec: float = 0.005,
        curriculum: Optional[Mapping[str, Any]] = None,
        curriculum_stage: int = 0,
    ) -> None:
        super().__init__()
        self.robot_ns = "/" + robot_ns.strip("/")
        self.control_dt = float(control_dt)
        self.episode_duration = float(episode_duration)
        self.reset_pressure_kpa = float(reset_pressure_kpa)
        self.reset_settle_sim_sec = float(reset_settle_sim_sec)
        self.maximum_pressure_kpa = float(maximum_pressure_kpa)
        self.maximum_bottom_pressure_kpa = float(maximum_bottom_pressure_kpa)
        self.maximum_forward_thrust_n = float(maximum_forward_thrust_n)
        self.maximum_reverse_thrust_n = float(maximum_reverse_thrust_n)
        self.maximum_thrust_rate_n_s = float(maximum_thrust_rate_n_s)
        direction = np.asarray(direction_xyz, dtype=np.float64)
        if direction.shape != (3,) or np.linalg.norm(direction) < 1.0e-9:
            raise ValueError("direction_xyz must be a nonzero three-vector")
        self._base_direction_xyz = direction / np.linalg.norm(direction)
        self.direction_xyz = self._base_direction_xyz.copy()
        self.target_direction_mode = str(target_direction_mode).lower()
        if self.target_direction_mode not in ("forward", "backward", "random"):
            raise ValueError(
                "target_direction_mode must be forward, backward, or random")
        self._target_direction = 1.0
        self.progress_weight = float(progress_weight)
        self.detach_penalty = float(detach_penalty)
        self.minimum_progress_contacts = int(minimum_progress_contacts)
        self.stride_target_m = float(stride_target_m)
        self.stride_preferred_maximum_m = float(
            stride_preferred_maximum_m)
        self.stride_failure_maximum_m = float(stride_failure_maximum_m)
        self.stride_success_bonus = float(stride_success_bonus)
        self.stride_overshoot_weight_per_m = float(
            stride_overshoot_weight_per_m)
        self.stride_failure_penalty = float(stride_failure_penalty)
        self.stride_recovery_contacts = int(stride_recovery_contacts)
        self.stride_recovery_velocity_m_s = float(
            stride_recovery_velocity_m_s)
        self.stride_recovery_max_total_thrust_n = float(
            stride_recovery_max_total_thrust_n)
        self.stride_recovery_duration_sec = float(
            stride_recovery_duration_sec)
        self.roll_weight = float(roll_weight)
        self.pitch_weight = float(pitch_weight)
        self.bottom_usage_weight = float(bottom_usage_weight)
        self.bottom_change_weight = float(bottom_change_weight)
        self.thrust_usage_weight_per_n = float(thrust_usage_weight_per_n)
        self.thrust_change_weight_per_n = float(thrust_change_weight_per_n)
        self.thrust_soft_limit_n = float(thrust_soft_limit_n)
        self.thrust_excess_weight_per_n2 = float(
            thrust_excess_weight_per_n2)
        if not 1 <= self.minimum_progress_contacts <= ARM_COUNT:
            raise ValueError("minimum_progress_contacts must be between 1 and 4")
        if not (0.0 < self.stride_target_m
                <= self.stride_preferred_maximum_m
                < self.stride_failure_maximum_m):
            raise ValueError(
                "stride limits must satisfy 0 < target <= preferred < failure")
        if not 1 <= self.stride_recovery_contacts <= ARM_COUNT:
            raise ValueError("stride_recovery_contacts must be between 1 and 4")
        if self.stride_recovery_duration_sec <= 0.0:
            raise ValueError("stride_recovery_duration_sec must be positive")
        if self.maximum_thrust_rate_n_s <= 0.0:
            raise ValueError("maximum_thrust_rate_n_s must be positive")
        self.roll_safe_rad = float(roll_safe_rad)
        self.pitch_safe_rad = float(pitch_safe_rad)
        self.fall_roll_rad = float(fall_roll_rad)
        self.fall_pitch_rad = float(fall_pitch_rad)
        self.maximum_vertical_excursion_m = float(maximum_vertical_excursion_m)
        self.detached_grace_sec = float(detached_grace_sec)
        self.wall_step_timeout_sec = float(wall_step_timeout_sec)
        self.synchronous_step = bool(synchronous_step)
        self.action_latch_wall_sec = float(action_latch_wall_sec)
        self.curriculum = dict(curriculum or {})
        self.curriculum_stage = int(curriculum_stage)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(5,), dtype=np.float32)
        self.observation_names = (
            "velocity_axis", "pitch", "pitch_rate", "roll", "roll_rate",
            *(f"arm_bend_{index + 1}" for index in range(ARM_COUNT)),
            *(f"pressure_{index + 1}" for index in range(ARM_COUNT)),
            *(f"thrust_{index + 1}" for index in range(ARM_COUNT)),
            *(f"contact_{index + 1}" for index in range(ARM_COUNT)),
            *(f"normal_force_{index + 1}" for index in range(ARM_COUNT)),
            *(f"tangential_force_{index + 1}" for index in range(ARM_COUNT)),
            "bottom_pressure", "target_direction",
            "stride_progress_fraction", "stride_recovery_phase",
        )
        low = np.asarray(
            [-2.0, -math.pi, -10.0, -math.pi, -10.0]
            + [0.0] * 4 + [0.0] * 4
            + [-self.maximum_reverse_thrust_n] * 4
            + [0.0] * 4 + [0.0] * 4 + [0.0] * 4
            + [0.0, -1.0, -5.0, 0.0], dtype=np.float32,
        )
        high = np.asarray(
            [2.0, math.pi, 10.0, math.pi, 10.0]
            + [math.pi] * 4 + [60.0] * 4
            + [self.maximum_forward_thrust_n] * 4
            + [1.0] * 4 + [20.0] * 4 + [20.0] * 4
            + [60.0, 1.0, 5.0, 1.0], dtype=np.float32,
        )
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        if not rospy.core.is_initialized():
            rospy.init_node("hugmy_gym_env", anonymous=True, disable_signals=True)
        self._condition = threading.Condition()
        self._odom: Optional[Odometry] = None
        self._odom_sequence = 0
        self._grip = np.zeros(36, dtype=np.float64)
        self._grip_valid = False
        self._pressures = np.zeros(ARM_COUNT, dtype=np.float64)
        self._pressure_valid = False
        self._bottom_pressure = 0.0
        self._bottom_valid = False
        self._reset_count = 0
        self._reset_count_valid = False
        self._step_count = 0
        self._step_count_valid = False
        self._commanded_thrust = np.zeros(ARM_COUNT, dtype=np.float64)
        self._episode_start_position = np.zeros(3, dtype=np.float64)
        self._previous_position = np.zeros(3, dtype=np.float64)
        self._episode_elapsed = 0.0
        self._detached_duration = 0.0
        self._qualified_progress_m = 0.0
        self._stride_start_position = np.zeros(3, dtype=np.float64)
        self._stride_recovery_phase = False
        self._stride_recovery_stable_duration = 0.0
        self._episode_stride_count = 0
        self._episode_completed_stride_distance_m = 0.0
        self._previous_bottom_action = 0.0
        self._previous_reward_thrust = np.zeros(ARM_COUNT, dtype=np.float64)
        self._last_domain: Dict[str, float] = {}

        self._pressure_pub = rospy.Publisher(
            self.robot_ns + "/independent_arm_pressure_controller/target_pressure",
            Float32MultiArray, queue_size=1,
        )
        self._bottom_pressure_pub = rospy.Publisher(
            self.robot_ns + "/independent_arm_pressure_controller/bottom_target_pressure",
            Float32, queue_size=1,
        )
        self._bottom_direction_pub = rospy.Publisher(
            self.robot_ns + "/pneumatic/bottom_force_direction",
            Vector3Stamped, queue_size=1,
        )
        self._pwm_pub = rospy.Publisher(
            self.robot_ns + "/pwm_test", PwmTest, queue_size=1,
        )
        self._subscribers = [
            rospy.Subscriber(self.robot_ns + "/ground_truth", Odometry,
                             self._odom_callback, queue_size=1),
            rospy.Subscriber(self.robot_ns + "/pneumatic/grip_state",
                             Float32MultiArray, self._grip_callback, queue_size=1),
            rospy.Subscriber(self.robot_ns + "/neuron/adc_states",
                             NeuronAdcStates, self._pressure_callback, queue_size=1),
            rospy.Subscriber(self.robot_ns + "/pneumatic/bottom_pressure",
                             Float32, self._bottom_callback, queue_size=1),
            rospy.Subscriber(self.robot_ns + "/pneumatic/reset_count",
                             UInt32, self._reset_count_callback, queue_size=1),
            rospy.Subscriber(self.robot_ns + "/pneumatic/step_count",
                             UInt32, self._step_count_callback, queue_size=1),
        ]
        self._pressure_enable_name = (
            self.robot_ns + "/independent_arm_pressure_controller/enable")
        self._reset_simulation_name = self.robot_ns + "/pneumatic/reset_simulation"
        self._step_simulation_name = self.robot_ns + "/pneumatic/step_simulation"
        self._pressure_enable = rospy.ServiceProxy(self._pressure_enable_name, SetBool)
        self._reset_simulation = rospy.ServiceProxy(self._reset_simulation_name, Trigger)
        self._step_simulation = rospy.ServiceProxy(self._step_simulation_name, Trigger)
        self._wait_for_ros()
        # Services and sensor topics also prove that bringup has finished
        # loading MotorInfo_3D_V.yaml onto the parameter server.
        self._load_motor_model()

    def _load_motor_model(self) -> None:
        prefix = self.robot_ns + "/motor_info"
        self._minimum_pwm = float(rospy.get_param(prefix + "/min_pwm", 0.5))
        self._neutral_pwm = float(rospy.get_param(prefix + "/neutral_pwm", 0.75))
        self._maximum_pwm = float(rospy.get_param(prefix + "/max_pwm", 0.975))
        self._motor_voltage = float(rospy.get_param(prefix + "/simulation/voltage", 21.2))
        self._pwm_conversion_mode = int(rospy.get_param(prefix + "/pwm_conversion_mode", 0))
        self._positive_thrust_below_neutral = bool(rospy.get_param(
            prefix + "/positive_thrust_below_neutral", False))

        def closest_curve(branch: str) -> Tuple[float, Tuple[float, float, float]]:
            root = prefix + ("/reverse" if branch == "reverse" else "")
            count = int(rospy.get_param(root + "/vel_ref_num", 0))
            choices = []
            for index in range(1, count + 1):
                ref = root + f"/ref{index}"
                choices.append((
                    abs(float(rospy.get_param(ref + "/voltage")) - self._motor_voltage),
                    float(rospy.get_param(ref + "/voltage")),
                    (
                        float(rospy.get_param(ref + "/polynominal0")),
                        float(rospy.get_param(ref + "/polynominal1")),
                        float(rospy.get_param(ref + "/polynominal2")),
                    ),
                ))
            if not choices:
                raise RuntimeError(f"No motor curve is configured at {root}")
            _, voltage, coefficients = min(choices, key=lambda item: item[0])
            return voltage, coefficients

        self._forward_curve = closest_curve("forward")
        self._reverse_curve = closest_curve("reverse")

    def _force_from_pwm(self, pwm: float) -> float:
        if abs(pwm - self._neutral_pwm) <= 1.0e-6:
            return 0.0
        low_pwm_branch = pwm < self._neutral_pwm
        voltage, polynomial = (self._reverse_curve if low_pwm_branch
                               else self._forward_curve)
        percent = 100.0 * pwm
        force = polynomial[0] + (
            polynomial[1] * percent + polynomial[2] * percent * percent
        ) / 10.0
        exponent = 1.5 if self._pwm_conversion_mode == 1 else 2.0
        if voltage > 0.0 and self._motor_voltage > 0.0:
            force *= (self._motor_voltage / voltage) ** exponent
        positive = (low_pwm_branch == self._positive_thrust_below_neutral)
        return abs(force) if positive else -abs(force)

    def _thrust_to_pwm(self, thrust: float) -> float:
        if abs(thrust) <= 0.01:
            return self._neutral_pwm
        positive = thrust > 0.0
        low_pwm_branch = (positive == self._positive_thrust_below_neutral)
        edge_pwm = self._minimum_pwm if low_pwm_branch else self._maximum_pwm
        target_magnitude = min(abs(thrust), abs(self._force_from_pwm(edge_pwm)))
        lower_fraction = 0.0
        upper_fraction = 1.0
        for _ in range(50):
            middle_fraction = 0.5 * (lower_fraction + upper_fraction)
            middle_pwm = self._neutral_pwm + middle_fraction * (
                edge_pwm - self._neutral_pwm)
            if abs(self._force_from_pwm(middle_pwm)) < target_magnitude:
                lower_fraction = middle_fraction
            else:
                upper_fraction = middle_fraction
        fraction = 0.5 * (lower_fraction + upper_fraction)
        return self._neutral_pwm + fraction * (edge_pwm - self._neutral_pwm)

    def _wait_for_ros(self) -> None:
        services = [self._pressure_enable_name, self._reset_simulation_name]
        if self.synchronous_step:
            services.append(self._step_simulation_name)
        for service in services:
            try:
                rospy.wait_for_service(service, timeout=30.0)
            except rospy.ROSException as exc:
                raise RuntimeError(
                    f"Required Hugmy Gym service is unavailable: {service}. "
                    "Start the plant first in another terminal with "
                    "`roslaunch hugmy hugmy_gym_mujoco.launch gui:=false`."
                ) from exc
        # In synchronous mode MuJoCo starts with its gate closed. Permit one
        # interval so the initial odometry/contact/pressure samples exist.
        if self.synchronous_step:
            response = self._step_simulation()
            if not response.success:
                raise RuntimeError(response.message)
        deadline = time.monotonic() + 30.0
        with self._condition:
            while not self._sensors_ready() and time.monotonic() < deadline:
                self._condition.wait(timeout=0.1)
        if not self._sensors_ready():
            missing = []
            if self._odom is None:
                missing.append("ground_truth")
            if not self._grip_valid:
                missing.append("grip_state[36]")
            if not self._pressure_valid:
                missing.append("four arm pressures")
            if not self._bottom_valid:
                missing.append("bottom pressure")
            if not self._reset_count_valid:
                missing.append("reset_count")
            if not self._step_count_valid:
                missing.append("step_count")
            raise RuntimeError(
                "Timed out waiting for Hugmy MuJoCo observations: "
                + ", ".join(missing))

    def _sensors_ready(self) -> bool:
        return (self._odom is not None and self._grip_valid and self._pressure_valid
                and self._bottom_valid and self._reset_count_valid
                and self._step_count_valid)

    def _odom_callback(self, message: Odometry) -> None:
        with self._condition:
            self._odom = message
            self._odom_sequence += 1
            self._condition.notify_all()

    def _grip_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) < 36:
            return
        with self._condition:
            self._grip[:] = np.asarray(message.data[:36], dtype=np.float64)
            self._grip_valid = True
            self._condition.notify_all()

    def _pressure_callback(self, message: NeuronAdcStates) -> None:
        values = np.full(ARM_COUNT, np.nan, dtype=np.float64)
        for adc in message.adcs:
            if 1 <= adc.slave_id <= ARM_COUNT:
                values[adc.slave_id - 1] = adc.pressure
        if not np.all(np.isfinite(values)):
            return
        with self._condition:
            self._pressures[:] = values
            self._pressure_valid = True
            self._condition.notify_all()

    def _bottom_callback(self, message: Float32) -> None:
        with self._condition:
            self._bottom_pressure = float(message.data)
            self._bottom_valid = math.isfinite(self._bottom_pressure)
            self._condition.notify_all()

    def _reset_count_callback(self, message: UInt32) -> None:
        with self._condition:
            self._reset_count = int(message.data)
            self._reset_count_valid = True
            self._condition.notify_all()

    def _step_count_callback(self, message: UInt32) -> None:
        with self._condition:
            self._step_count = int(message.data)
            self._step_count_valid = True
            self._condition.notify_all()

    def set_curriculum_stage(self, stage: int) -> None:
        stages = self.curriculum.get("stages", [])
        if stages and not 0 <= int(stage) < len(stages):
            raise IndexError(f"curriculum stage {stage} is outside 0..{len(stages) - 1}")
        self.curriculum_stage = int(stage)

    def _apply_domain_randomization(self) -> Dict[str, float]:
        stages = self.curriculum.get("stages", [])
        if not stages:
            self._last_domain = {}
            return self._last_domain
        parameters = stages[self.curriculum_stage].get("parameters", {})
        sampled: Dict[str, float] = {}
        for name, bounds in parameters.items():
            if isinstance(bounds, (int, float)):
                value = float(bounds)
            else:
                value = float(self.np_random.uniform(float(bounds[0]), float(bounds[1])))
            sampled[name] = value
            rospy.set_param(
                self.robot_ns + "/simulation/pneumatic/" + name, value)
        if "human_arm_pitch" in sampled or "human_arm_yaw" in sampled:
            pitch = sampled.get("human_arm_pitch", 0.0)
            yaw = sampled.get("human_arm_yaw", 0.0)
            self.direction_xyz = np.asarray([
                math.cos(yaw) * math.cos(pitch),
                math.sin(yaw) * math.cos(pitch),
                -math.sin(pitch),
            ], dtype=np.float64)
        else:
            self.direction_xyz = self._base_direction_xyz.copy()
        self._last_domain = sampled
        return sampled

    def _publish_targets(self, pressures: np.ndarray, thrusts: np.ndarray,
                         bottom_pressure: float) -> None:
        pressure_message = Float32MultiArray()
        pressure_message.data = [float(value) for value in pressures]
        self._pressure_pub.publish(pressure_message)
        self._bottom_pressure_pub.publish(Float32(data=float(bottom_pressure)))
        direction = Vector3Stamped()
        direction.header.stamp = rospy.Time.now()
        direction.header.frame_id = self.robot_ns.strip("/") + "/root"
        horizontal = self.direction_xyz[:2]
        horizontal_norm = max(1.0e-9, float(np.linalg.norm(horizontal)))
        # The central chamber has no commanded rocking sign.  Keep publishing
        # the cylinder axis for compatibility with the MuJoCo diagnostics and
        # older plugins, but never reverse it from the bottom action.
        direction.vector.x = float(horizontal[0]) / horizontal_norm
        direction.vector.y = float(horizontal[1]) / horizontal_norm
        self._bottom_direction_pub.publish(direction)
        pwm = PwmTest()
        pwm.motor_index = list(range(ARM_COUNT))
        pwm.pwms = [float(self._thrust_to_pwm(value)) for value in thrusts]
        self._pwm_pub.publish(pwm)
        self._commanded_thrust[:] = thrusts

    def _physical_action(
        self, action: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        bounded = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        pressures = np.zeros(ARM_COUNT, dtype=np.float64)
        front_pressure = 0.5 * (bounded[0] + 1.0) * self.maximum_pressure_kpa
        rear_pressure = 0.5 * (bounded[1] + 1.0) * self.maximum_pressure_kpa
        pressures[list(FRONT_ARMS)] = front_pressure
        pressures[list(REAR_ARMS)] = rear_pressure

        def scale_thrust(value: float) -> float:
            return (value * self.maximum_forward_thrust_n if value >= 0.0
                    else value * self.maximum_reverse_thrust_n)

        thrusts = np.zeros(ARM_COUNT, dtype=np.float64)
        thrusts[list(FRONT_ARMS)] = scale_thrust(float(bounded[2]))
        thrusts[list(REAR_ARMS)] = scale_thrust(float(bounded[3]))
        bottom_pressure = (
            0.5 * (float(bounded[4]) + 1.0)
            * self.maximum_bottom_pressure_kpa)
        return pressures, thrusts, bottom_pressure

    def _rate_limited_thrust(self, requested: np.ndarray) -> np.ndarray:
        """Apply a hard slew-rate limit before a thrust reaches MuJoCo.

        Penalising thrust changes helps the policy choose smooth commands, but
        exploration is random at the beginning of training.  The physical
        limit also keeps those exploratory commands inside the actuator rate
        that will be allowed when the policy is evaluated on the robot.
        """
        maximum_delta = self.maximum_thrust_rate_n_s * self.control_dt
        delta = np.clip(
            requested - self._commanded_thrust, -maximum_delta, maximum_delta)
        return self._commanded_thrust + delta

    def _snapshot(self) -> Dict[str, Any]:
        with self._condition:
            if self._odom is None:
                raise RuntimeError("No ground-truth odometry")
            odom = self._odom
            position = np.asarray([
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ], dtype=np.float64)
            roll, pitch = quaternion_to_roll_pitch(odom.pose.pose.orientation)
            linear = odom.twist.twist.linear
            angular = odom.twist.twist.angular
            return {
                "position": position,
                "roll": roll,
                "pitch": pitch,
                "velocity_axis": linear.x * self.direction_xyz[0]
                                 + linear.y * self.direction_xyz[1]
                                 + linear.z * self.direction_xyz[2],
                "roll_rate": angular.x,
                "pitch_rate": angular.y,
                "grip": self._grip.copy(),
                "pressures": self._pressures.copy(),
                "bottom_pressure": self._bottom_pressure,
                "odom_sequence": self._odom_sequence,
                "stamp": odom.header.stamp.to_sec(),
            }

    def _observation(self, state: Mapping[str, Any]) -> np.ndarray:
        grip = state["grip"]
        values = np.concatenate((
            np.asarray([
                state["velocity_axis"], state["pitch"], state["pitch_rate"],
                state["roll"], state["roll_rate"],
            ]),
            grip[24:28], state["pressures"], self._commanded_thrust,
            grip[0:4], grip[28:32], grip[32:36],
            np.asarray([
                state["bottom_pressure"], self._target_direction,
                self._qualified_progress_m / self.stride_target_m,
                float(self._stride_recovery_phase),
            ]),
        )).astype(np.float32)
        return np.clip(values, self.observation_space.low,
                       self.observation_space.high).astype(np.float32)

    def _wait_sim_duration(
        self, duration: float, start_sequence: Optional[int] = None,
        start_stamp: Optional[float] = None,
    ) -> None:
        snapshot = self._snapshot()
        if start_stamp is None:
            start_stamp = float(snapshot["stamp"])
        sequence = snapshot["odom_sequence"] if start_sequence is None else start_sequence
        deadline = time.monotonic() + self.wall_step_timeout_sec
        with self._condition:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                stamp = self._odom.header.stamp.to_sec() if self._odom else start_stamp
                if self._odom_sequence > sequence and stamp >= start_stamp + duration:
                    return
                self._condition.wait(timeout=0.02)
        raise RuntimeError(f"MuJoCo did not advance {duration:.3f} simulation seconds")

    def _advance_policy_interval(self) -> None:
        before = self._snapshot()
        if self.synchronous_step:
            # Let pressure-controller and PwmTest callbacks latch every part
            # of the action before opening the integration gate.
            if self.action_latch_wall_sec > 0.0:
                time.sleep(self.action_latch_wall_sec)
            with self._condition:
                previous_step_count = self._step_count
            response = self._step_simulation()
            if not response.success:
                raise RuntimeError(response.message)
            deadline = time.monotonic() + self.wall_step_timeout_sec
            with self._condition:
                while (self._step_count <= previous_step_count
                       and time.monotonic() < deadline):
                    self._condition.wait(timeout=0.02)
            if self._step_count <= previous_step_count:
                raise RuntimeError("MuJoCo did not acknowledge the policy step")
            return
        self._wait_sim_duration(
            self.control_dt, int(before["odom_sequence"]), float(before["stamp"]))

    def reset(self, *, seed: Optional[int] = None,
              options: Optional[dict] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        if options and "curriculum_stage" in options:
            self.set_curriculum_stage(int(options["curriculum_stage"]))
        if options and "target_direction" in options:
            requested_direction = float(options["target_direction"])
            if requested_direction == 0.0:
                raise ValueError("target_direction must be nonzero")
            self._target_direction = 1.0 if requested_direction > 0.0 else -1.0
        elif self.target_direction_mode == "random":
            self._target_direction = (
                1.0 if int(self.np_random.integers(0, 2)) == 1 else -1.0)
        else:
            self._target_direction = (
                1.0 if self.target_direction_mode == "forward" else -1.0)
        domain = self._apply_domain_randomization()
        neutral_pressure = np.zeros(ARM_COUNT, dtype=np.float64)
        neutral_thrust = np.zeros(ARM_COUNT, dtype=np.float64)
        self._publish_targets(neutral_pressure, neutral_thrust, 0.0)
        try:
            self._pressure_enable(False)
        except rospy.ServiceException:
            pass
        with self._condition:
            previous_reset_count = self._reset_count
        response = self._reset_simulation()
        if not response.success:
            raise RuntimeError(response.message)
        deadline = time.monotonic() + self.wall_step_timeout_sec
        with self._condition:
            while self._reset_count <= previous_reset_count and time.monotonic() < deadline:
                self._condition.wait(timeout=0.02)
        if self._reset_count <= previous_reset_count:
            raise RuntimeError("MuJoCo reset request was not applied")

        initial_pressure = np.full(ARM_COUNT, self.reset_pressure_kpa, dtype=np.float64)
        self._publish_targets(initial_pressure, neutral_thrust, 0.0)
        enable_response = self._pressure_enable(True)
        if not enable_response.success:
            raise RuntimeError(enable_response.message)
        settle_intervals = max(1, int(math.ceil(
            self.reset_settle_sim_sec / self.control_dt)))
        for _ in range(settle_intervals):
            self._advance_policy_interval()
        state = self._snapshot()
        self._episode_start_position = state["position"].copy()
        self._stride_start_position = state["position"].copy()
        self._previous_position = state["position"].copy()
        self._episode_elapsed = 0.0
        self._detached_duration = 0.0
        self._qualified_progress_m = 0.0
        self._stride_recovery_phase = False
        self._stride_recovery_stable_duration = 0.0
        self._episode_stride_count = 0
        self._episode_completed_stride_distance_m = 0.0
        self._previous_bottom_action = 0.0
        self._previous_reward_thrust.fill(0.0)
        info = {
            "curriculum_stage": self.curriculum_stage,
            "domain_parameters": domain,
            "observation_names": self.observation_names,
            "target_direction": self._target_direction,
        }
        return self._observation(state), info

    def step(self, action: np.ndarray):
        if not self.action_space.contains(np.asarray(action, dtype=np.float32)):
            action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        pressures, requested_thrusts, bottom_pressure = self._physical_action(
            action)
        thrusts = self._rate_limited_thrust(requested_thrusts)
        self._publish_targets(pressures, thrusts, bottom_pressure)
        self._advance_policy_interval()
        state = self._snapshot()
        displacement = state["position"] - self._previous_position
        axis_progress = float(np.dot(displacement, self.direction_xyz))
        progress = self._target_direction * axis_progress
        total_progress = self._target_direction * float(np.dot(
            state["position"] - self._episode_start_position,
            self.direction_xyz))
        previous_stride_raw_progress = self._target_direction * float(np.dot(
            self._previous_position - self._stride_start_position,
            self.direction_xyz))
        stride_raw_progress = self._target_direction * float(np.dot(
            state["position"] - self._stride_start_position,
            self.direction_xyz))
        self._previous_position = state["position"].copy()
        self._episode_elapsed += self.control_dt
        contacts = state["grip"][0:4] >= 0.5
        contact_count = int(np.count_nonzero(contacts))
        self._detached_duration = (
            self._detached_duration + self.control_dt
            if contact_count == 0 else 0.0)

        total_abs_thrust = float(np.sum(np.abs(thrusts)))
        total_thrust_change = float(np.sum(np.abs(
            thrusts - self._previous_reward_thrust)))

        # Only motion made with at least two contacts advances the rewarded
        # stride coordinate.  Raw CoM motion is tracked separately so a launch
        # cannot hide behind the contact gate.
        previous_qualified_progress = self._qualified_progress_m
        qualified_progress = (
            progress if contact_count >= self.minimum_progress_contacts else 0.0)
        self._qualified_progress_m += qualified_progress

        # Progress shaping saturates at the nominal 20 mm target.  It therefore
        # encourages reaching the target but gives no additional reward for a
        # long slide.  Backsliding below the target removes the earlier reward.
        previous_progress_potential = min(
            previous_qualified_progress, self.stride_target_m)
        progress_potential = min(
            self._qualified_progress_m, self.stride_target_m)
        reward_progress = self.progress_weight * (
            progress_potential - previous_progress_potential)

        previous_stride_excess_m = max(
            0.0,
            max(previous_qualified_progress, previous_stride_raw_progress)
            - self.stride_preferred_maximum_m)
        stride_excess_m = max(
            0.0,
            max(self._qualified_progress_m, stride_raw_progress)
            - self.stride_preferred_maximum_m)
        # Crossing 30 mm is allowed; only the incremental excess gets a soft
        # cost.  Returning to a controlled stride length refunds that cost.
        reward_stride_overshoot = (
            -self.stride_overshoot_weight_per_m
            * (stride_excess_m - previous_stride_excess_m))
        stride_failure_excess_m = max(
            0.0,
            max(self._qualified_progress_m, stride_raw_progress)
            - self.stride_failure_maximum_m)
        stride_runaway = stride_failure_excess_m > 0.0

        # A stride is complete only after reaching the target and returning to
        # a low-speed, low-thrust, well-supported state.  This prevents steady
        # sliding from being counted as a sequence of stable strides.
        if (not self._stride_recovery_phase
                and self._qualified_progress_m >= self.stride_target_m
                and not stride_runaway):
            self._stride_recovery_phase = True
            self._stride_recovery_stable_duration = 0.0
        if (self._stride_recovery_phase
                and self._qualified_progress_m < self.stride_target_m):
            self._stride_recovery_phase = False
            self._stride_recovery_stable_duration = 0.0

        recovery_stable = bool(
            self._stride_recovery_phase
            and contact_count >= self.stride_recovery_contacts
            and abs(float(state["velocity_axis"]))
                <= self.stride_recovery_velocity_m_s
            and total_abs_thrust
                <= self.stride_recovery_max_total_thrust_n)
        self._stride_recovery_stable_duration = (
            self._stride_recovery_stable_duration + self.control_dt
            if recovery_stable else 0.0)
        stride_completed = bool(
            self._stride_recovery_phase
            and self._stride_recovery_stable_duration
                >= self.stride_recovery_duration_sec
            and not stride_runaway)
        completed_stride_m = 0.0
        if stride_completed:
            completed_stride_m = max(
                0.0, self._qualified_progress_m, stride_raw_progress)
            self._episode_stride_count += 1
            self._episode_completed_stride_distance_m += completed_stride_m
            self._stride_start_position = state["position"].copy()
            self._qualified_progress_m = 0.0
            self._stride_recovery_phase = False
            self._stride_recovery_stable_duration = 0.0

        roll_excess = max(0.0, abs(float(state["roll"])) - self.roll_safe_rad)
        pitch_excess = max(0.0, abs(float(state["pitch"])) - self.pitch_safe_rad)
        fallen = (
            abs(float(state["roll"])) >= self.fall_roll_rad
            or abs(float(state["pitch"])) >= self.fall_pitch_rad
            or abs(float(state["position"][2] - self._episode_start_position[2]))
               >= self.maximum_vertical_excursion_m
        )
        detached = self._detached_duration >= self.detached_grace_sec
        terminated = bool(stride_runaway or fallen or detached)
        truncated = bool(self._episode_elapsed >= self.episode_duration)
        reward_roll = -self.roll_weight * roll_excess * roll_excess
        reward_pitch = -self.pitch_weight * pitch_excess * pitch_excess
        # A fall at the natural end of the episode is acceptable.  An earlier
        # fall stops the remaining stride opportunities and receives a modest
        # terminal cost so one-stride-then-fall is not optimal.
        terminal_failure = bool((fallen or detached) and not truncated)
        reward_detach = -self.detach_penalty if terminal_failure else 0.0
        reward_stride_success = (
            self.stride_success_bonus if stride_completed else 0.0)
        reward_stride_failure = (
            -self.stride_failure_penalty if stride_runaway else 0.0)
        bottom_action = 0.5 * (float(np.clip(action[4], -1.0, 1.0)) + 1.0)
        bottom_change = abs(bottom_action - self._previous_bottom_action)
        reward_bottom_usage = -self.bottom_usage_weight * bottom_action
        reward_bottom_change = -self.bottom_change_weight * bottom_change
        reward_thrust_usage = (
            -self.thrust_usage_weight_per_n * total_abs_thrust)
        reward_thrust_change = (
            -self.thrust_change_weight_per_n * total_thrust_change)
        thrust_excess = np.maximum(
            0.0, np.abs(thrusts) - self.thrust_soft_limit_n)
        reward_thrust_excess = (
            -self.thrust_excess_weight_per_n2
            * float(np.sum(thrust_excess * thrust_excess)))
        self._previous_reward_thrust[:] = thrusts
        self._previous_bottom_action = bottom_action
        reward = (reward_progress + reward_roll + reward_pitch + reward_detach
                  + reward_stride_success + reward_stride_overshoot
                  + reward_stride_failure
                  + reward_bottom_usage + reward_bottom_change
                  + reward_thrust_usage + reward_thrust_change
                  + reward_thrust_excess)
        info = {
            "progress_m": progress,
            "qualified_progress_m": qualified_progress,
            "stride_progress_m": self._qualified_progress_m,
            "stride_target_m": self.stride_target_m,
            "stride_recovery_phase": self._stride_recovery_phase,
            "stride_recovery_stable": recovery_stable,
            "stride_completed": stride_completed,
            "completed_stride_m": completed_stride_m,
            "completed_stride_count": self._episode_stride_count,
            "mean_completed_stride_m": (
                self._episode_completed_stride_distance_m
                / self._episode_stride_count
                if self._episode_stride_count > 0 else 0.0),
            "stride_success": self._episode_stride_count > 0,
            "stride_overshoot": stride_excess_m > 0.0,
            "stride_overshoot_m": stride_excess_m,
            "stride_runaway": stride_runaway,
            "stride_failure_excess_m": stride_failure_excess_m,
            "axis_progress_m": axis_progress,
            "total_progress_m": total_progress,
            "target_direction": self._target_direction,
            "contact_count": contact_count,
            "fallen": fallen,
            "detached": detached,
            "reward_progress": reward_progress,
            "reward_roll": reward_roll,
            "reward_pitch": reward_pitch,
            "reward_detach": reward_detach,
            "reward_stride_success": reward_stride_success,
            "reward_stride_overshoot": reward_stride_overshoot,
            "reward_stride_failure": reward_stride_failure,
            "reward_bottom_usage": reward_bottom_usage,
            "reward_bottom_change": reward_bottom_change,
            "reward_thrust_usage": reward_thrust_usage,
            "reward_thrust_change": reward_thrust_change,
            "reward_thrust_excess": reward_thrust_excess,
            "total_abs_thrust_n": total_abs_thrust,
            "total_thrust_change_n": total_thrust_change,
            "physical_action": {
                "pressure_kpa": pressures.tolist(),
                "thrust_n": thrusts.tolist(),
                "requested_thrust_n": requested_thrusts.tolist(),
                "bottom_pressure_kpa": bottom_pressure,
                "bottom_pressure_fraction": bottom_action,
            },
            "curriculum_stage": self.curriculum_stage,
            "domain_parameters": self._last_domain,
        }
        return self._observation(state), float(reward), terminated, truncated, info

    def close(self) -> None:
        try:
            self._publish_targets(np.zeros(ARM_COUNT), np.zeros(ARM_COUNT), 0.0)
            self._pressure_enable(False)
        except (rospy.ServiceException, rospy.ROSException):
            pass
