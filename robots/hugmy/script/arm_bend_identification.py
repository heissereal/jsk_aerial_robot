#!/usr/bin/env python3

"""Automated pressure/PWM sweep for one configurable Hugmy arm."""

import csv
import math
import os
import threading
import time
from collections import deque

import rospy
from spinal.msg import Imu, NeuronAdcStates, NeuronImuStates, PwmTest
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def stddev(values):
    if not values:
        return float("nan")
    center = mean(values)
    return math.sqrt(sum((value - center) ** 2 for value in values) / len(values))


def vector_norm(vector):
    return math.sqrt(sum(value * value for value in vector))


class ArmBendIdentification:
    def __init__(self):
        self.neuron_id = int(rospy.get_param("~neuron_id", 4))
        self.motor_index = int(rospy.get_param("~motor_index", 3))
        self.pressure_arm_index = int(rospy.get_param("~pressure_arm_index", 3))
        self.pressure_values = [float(v) for v in rospy.get_param(
            "~pressure_values_kpa", [0, 10, 20, 30, 40, 50])]
        self.pwm_values = [float(v) for v in rospy.get_param(
            "~motor_pwm_values",
            [0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85])]
        self.repeats = int(rospy.get_param("~repeats", 3))
        self.bidirectional_pressure = bool(rospy.get_param("~bidirectional_pressure_sweep", True))
        self.bidirectional_pwm = bool(rospy.get_param("~bidirectional_pwm_sweep", True))
        self.command_rate_hz = float(rospy.get_param("~command_rate_hz", 20.0))
        self.minimum_settle_s = float(rospy.get_param("~minimum_settle_s", 0.5))
        self.stability_window_s = float(rospy.get_param("~stability_window_s", 0.5))
        self.record_duration_s = float(rospy.get_param("~record_duration_s", 0.5))
        self.imu_recovery_timeout_s = float(rospy.get_param(
            "~imu_recovery_timeout_s", 10.0))
        self.condition_timeout_s = float(rospy.get_param("~condition_timeout_s", 5.0))
        self.pressure_reach_timeout_s = float(rospy.get_param(
            "~pressure_reach_timeout_s", 20.0))
        self.require_stability = bool(rospy.get_param("~require_stability", False))
        self.pressure_tolerance_kpa = float(rospy.get_param("~pressure_tolerance_kpa", 1.0))
        self.pressure_std_limit_kpa = float(rospy.get_param("~pressure_std_limit_kpa", 0.5))
        self.gyro_rms_limit_rad_s = float(rospy.get_param("~gyro_rms_limit_rad_s", 0.20))
        self.bend_std_limit_deg = float(rospy.get_param("~bend_std_limit_deg", 1.0))
        self.acc_lpf_tau_s = float(rospy.get_param("~acc_lpf_tau_s", 0.15))
        self.max_pressure_kpa = float(rospy.get_param("~max_pressure_kpa", 55.0))
        self.idle_pwm = float(rospy.get_param("~idle_pwm", 0.50))
        self.max_motor_pwm = float(rospy.get_param("~max_motor_pwm", 0.85))
        self.arm_link_length_m = float(rospy.get_param("~arm_link_length_m", 0.023))
        self.rotor_offset_radial_m = float(rospy.get_param("~rotor_offset_radial_m", 0.020))
        self.rotor_offset_z_m = float(rospy.get_param("~rotor_offset_z_m", 0.0369))
        self.arm_base_xyz_m = [float(v) for v in rospy.get_param(
            "~arm_base_xyz_m", [0.04198, 0.04198, -0.0385])]
        self.arm_outward_yaw_rad = float(rospy.get_param(
            "~arm_outward_yaw_rad", -math.pi / 4.0))
        self.abort_on_timeout = bool(rospy.get_param("~abort_on_timeout", True))
        output_directory = os.path.expanduser(rospy.get_param(
            "~output_directory", "~/.ros/hugmy_arm_identification"))
        os.makedirs(output_directory, exist_ok=True)
        run_name = time.strftime("arm4_bend_%Y%m%d_%H%M%S")
        self.raw_path = os.path.join(output_directory, run_name + "_raw.csv")
        self.summary_path = os.path.join(output_directory, run_name + "_summary.csv")

        if not self.pressure_values or not self.pwm_values:
            raise ValueError("pressure_values_kpa and motor_pwm_values must not be empty")
        if self.pressure_arm_index < 0 or self.pressure_arm_index >= 4:
            raise ValueError("pressure_arm_index must be in 0..3")
        if min(self.pressure_values) < 0.0 or max(self.pressure_values) > self.max_pressure_kpa:
            raise ValueError("pressure sweep exceeds the configured safety range")
        if min(self.pwm_values) < self.idle_pwm or max(self.pwm_values) > self.max_motor_pwm:
            raise ValueError("motor PWM sweep exceeds the configured safety range")

        self.lock = threading.Lock()
        self.pressure_samples = deque(maxlen=2000)
        self.imu_samples = deque(maxlen=4000)
        self.root_imu = None
        self.recording = False
        self.current_condition = None
        self.current_records = []
        self.baseline_acc = None
        self.filtered_acc = None
        self.last_neuron_imu_time = None
        self.running = False
        self.abort_requested = False
        self.commanded_pressure_kpa = 0.0
        self.commanded_motor_pwm = self.idle_pwm

        pressure_target_topic = rospy.get_param(
            "~pressure_target_topic",
            "/quadrotor/independent_arm_pressure_controller/target_pressure")
        pressure_enable_service = rospy.get_param(
            "~pressure_enable_service",
            "/quadrotor/independent_arm_pressure_controller/enable")
        pwm_topic = rospy.get_param("~pwm_topic", "/pwm_test")
        neuron_imu_topic = rospy.get_param("~neuron_imu_topic", "/neuron/imu_states")
        neuron_adc_topic = rospy.get_param("~neuron_adc_topic", "/neuron/adc_states")
        root_imu_topic = rospy.get_param("~root_imu_topic", "/imu")

        self.pressure_target_pub = rospy.Publisher(
            pressure_target_topic, Float32MultiArray, queue_size=1, latch=True)
        self.pwm_pub = rospy.Publisher(pwm_topic, PwmTest, queue_size=1)
        self.pressure_enable = rospy.ServiceProxy(pressure_enable_service, SetBool)
        self.neuron_imu_sub = rospy.Subscriber(
            neuron_imu_topic, NeuronImuStates, self.neuron_imu_callback, queue_size=20)
        self.neuron_adc_sub = rospy.Subscriber(
            neuron_adc_topic, NeuronAdcStates, self.neuron_adc_callback, queue_size=20)
        self.root_imu_sub = rospy.Subscriber(root_imu_topic, Imu, self.root_imu_callback, queue_size=20)
        self.run_service = rospy.Service("~run", SetBool, self.run_callback)
        # Keep commands flowing independently of CSV writes and condition
        # transitions, so Spinal's bench-test watchdog does not see a gap.
        self.command_timer = rospy.Timer(
            rospy.Duration(1.0 / max(1.0, self.command_rate_hz)),
            self.command_timer_callback)
        rospy.on_shutdown(self.safe_stop)

        rospy.logwarn("Identification node is idle. Secure the vehicle and use ~run true to start.")
        rospy.loginfo("Measuring motor %d, Neuron %d, pressure arm index %d",
                      self.motor_index, self.neuron_id, self.pressure_arm_index)
        rospy.loginfo("Raw CSV: %s", self.raw_path)
        rospy.loginfo("Summary CSV: %s", self.summary_path)

    def neuron_adc_callback(self, msg):
        now = rospy.get_time()
        for adc in msg.adcs:
            if adc.slave_id == self.neuron_id:
                with self.lock:
                    self.pressure_samples.append((now, float(adc.pressure)))
                return

    def root_imu_callback(self, msg):
        with self.lock:
            self.root_imu = {
                "acc": list(msg.acc), "gyro": list(msg.gyro),
                "quaternion": list(msg.quaternion), "time": rospy.get_time()
            }

    def neuron_imu_callback(self, msg):
        now = rospy.get_time()
        for imu in msg.imus:
            if imu.slave_id != self.neuron_id:
                continue
            raw_acc = list(imu.acc)
            if self.filtered_acc is None or self.last_neuron_imu_time is None:
                self.filtered_acc = list(raw_acc)
            else:
                dt = max(0.0, min(0.2, now - self.last_neuron_imu_time))
                alpha = dt / (max(1.0e-3, self.acc_lpf_tau_s) + dt)
                self.filtered_acc = [previous + alpha * (current - previous)
                                     for previous, current in zip(self.filtered_acc, raw_acc)]
            self.last_neuron_imu_time = now
            sample = {"time": now, "acc": raw_acc, "acc_filtered": list(self.filtered_acc),
                      "gyro": list(imu.gyro)}
            with self.lock:
                self.imu_samples.append(sample)
                if self.recording and self.current_condition is not None:
                    record = dict(self.current_condition)
                    record.update(sample)
                    record["pressure_kpa"] = self.latest_pressure_unlocked()
                    record["root_imu"] = dict(self.root_imu) if self.root_imu else None
                    record["bend_deg"] = self.bend_angle_deg(sample["acc_filtered"])
                    (record["joint_each_deg"], record["rotor_x_m"],
                     record["rotor_y_m"], record["rotor_z_m"]) = \
                        self.equal_joint_kinematics(record["bend_deg"])
                    self.current_records.append(record)
            return

    def latest_pressure_unlocked(self):
        return self.pressure_samples[-1][1] if self.pressure_samples else float("nan")

    def bend_angle_deg(self, acc):
        if self.baseline_acc is None:
            return float("nan")
        norm_a = vector_norm(acc)
        norm_b = vector_norm(self.baseline_acc)
        if norm_a < 1.0e-6 or norm_b < 1.0e-6:
            return float("nan")
        cosine = sum(a * b for a, b in zip(acc, self.baseline_acc)) / (norm_a * norm_b)
        return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))

    def equal_joint_kinematics(self, bend_deg):
        """Estimate the selected rotor position using three equal joint angles."""
        if not math.isfinite(bend_deg):
            return (float("nan"),) * 4
        joint_rad = math.radians(bend_deg) / 3.0
        radial = (self.arm_link_length_m * math.cos(joint_rad) +
                  self.arm_link_length_m * math.cos(2.0 * joint_rad) +
                  self.rotor_offset_radial_m * math.cos(3.0 * joint_rad) +
                  self.rotor_offset_z_m * math.sin(3.0 * joint_rad))
        vertical = (-self.arm_link_length_m * math.sin(joint_rad) -
                    self.arm_link_length_m * math.sin(2.0 * joint_rad) -
                    self.rotor_offset_radial_m * math.sin(3.0 * joint_rad) +
                    self.rotor_offset_z_m * math.cos(3.0 * joint_rad))
        rotor_x = self.arm_base_xyz_m[0] + radial * math.cos(self.arm_outward_yaw_rad)
        rotor_y = self.arm_base_xyz_m[1] + radial * math.sin(self.arm_outward_yaw_rad)
        rotor_z = self.arm_base_xyz_m[2] + vertical
        return math.degrees(joint_rad), rotor_x, rotor_y, rotor_z

    def run_callback(self, request):
        if not request.data:
            self.abort_requested = True
            self.safe_stop()
            return SetBoolResponse(success=True, message="abort requested; motor set to idle")
        if self.running:
            return SetBoolResponse(success=False, message="identification is already running")
        self.abort_requested = False
        self.running = True
        thread = threading.Thread(target=self.run_experiment)
        thread.daemon = True
        thread.start()
        return SetBoolResponse(success=True, message="identification sweep started")

    def command_timer_callback(self, _event):
        if not self.running or self.abort_requested:
            return
        targets = [0.0, 0.0, 0.0, 0.0]
        targets[self.pressure_arm_index] = self.commanded_pressure_kpa
        self.pressure_target_pub.publish(Float32MultiArray(data=targets))
        self.pwm_pub.publish(PwmTest(
            motor_index=[self.motor_index], pwms=[self.commanded_motor_pwm]))

    @staticmethod
    def bidirectional(values, enabled):
        if not enabled or len(values) < 2:
            return [("up", value) for value in values]
        result = [("up", value) for value in values]
        result.extend(("down", value) for value in reversed(values[:-1]))
        return result

    def run_experiment(self):
        try:
            rospy.logwarn("Waiting for pressure controller service: %s",
                          self.pressure_enable.resolved_name)
            rospy.wait_for_service(self.pressure_enable.resolved_name, timeout=10.0)
            rospy.logwarn("Pressure controller service found; enabling pressure control")
            self.publish_pressure_target(0.0)
            response = self.pressure_enable(True)
            rospy.logwarn("Pressure controller enable response: success=%s message=%s",
                          response.success, response.message)
            if not response.success:
                raise RuntimeError(response.message)

            # Acquire the gravity reference with the rotor stopped even when
            # the requested measurement grid starts above idle PWM.  Using the
            # first 0.60 condition as zero would erase its real bending angle.
            baseline_condition = {
                "condition_id": 0, "repeat": 0,
                "pressure_direction": "baseline",
                "pwm_direction": "baseline",
                "target_pressure_kpa": min(self.pressure_values),
                "motor_pwm": self.idle_pwm,
            }
            rospy.logwarn("Acquiring bend-angle baseline at P=%.1f kPa PWM=%.4f",
                          min(self.pressure_values), self.idle_pwm)
            baseline_ready = self.wait_until_stable(
                min(self.pressure_values), self.idle_pwm)
            if not baseline_ready and self.abort_on_timeout:
                raise RuntimeError("baseline pressure timed out")
            baseline_records = self.record_condition(
                baseline_condition, min(self.pressure_values), self.idle_pwm)
            if not baseline_records:
                raise RuntimeError("Neuron IMU unavailable during baseline acquisition")
            self.set_baseline(baseline_records)

            pressure_sequence = self.bidirectional(
                self.pressure_values, self.bidirectional_pressure)
            pwm_sequence = self.bidirectional(self.pwm_values, self.bidirectional_pwm)
            rospy.logwarn("Starting sweep: %d pressure points x %d PWM points x %d repeats",
                          len(pressure_sequence), len(pwm_sequence), self.repeats)
            condition_id = 0
            for repeat in range(1, self.repeats + 1):
                for pressure_direction, pressure in pressure_sequence:
                    for pwm_direction, pwm in pwm_sequence:
                        if self.abort_requested or rospy.is_shutdown():
                            raise RuntimeError("experiment aborted")
                        condition_id += 1
                        condition = {
                            "condition_id": condition_id, "repeat": repeat,
                            "pressure_direction": pressure_direction,
                            "pwm_direction": pwm_direction,
                            "target_pressure_kpa": pressure, "motor_pwm": pwm,
                        }
                        rospy.logwarn("Condition %d: repeat=%d P=%.1f kPa PWM=%.4f",
                                      condition_id, repeat, pressure, pwm)
                        self.publish_pressure_target(pressure)
                        self.publish_motor_pwm(pwm)
                        ready = self.wait_until_stable(pressure, pwm)
                        if not ready and self.abort_on_timeout:
                            raise RuntimeError("condition timed out before becoming stable")
                        records = self.record_condition(condition, pressure, pwm)
                        if not records:
                            rospy.logerr("Skipping condition %d because Neuron %d IMU did not recover",
                                         condition_id, self.neuron_id)
                            continue
                        # When stability is not required, this flag remains a
                        # useful post-recording quality indicator only; it does
                        # not reject the measured mean.
                        stable = (ready if self.require_stability
                                  else self.is_stable(pressure))
                        if self.baseline_acc is None and pressure == min(self.pressure_values) and pwm == min(self.pwm_values):
                            self.set_baseline(records)
                            for record in records:
                                record["bend_deg"] = self.bend_angle_deg(record["acc_filtered"])
                                (record["joint_each_deg"], record["rotor_x_m"],
                                 record["rotor_y_m"], record["rotor_z_m"]) = \
                                    self.equal_joint_kinematics(record["bend_deg"])
                        self.append_csv(records, stable)
            rospy.logwarn("Identification sweep completed: %s", self.summary_path)
        except Exception as error:
            rospy.logerr("Identification stopped: %s", error)
        finally:
            self.running = False
            self.safe_stop()

    def wait_until_stable(self, target_pressure, pwm):
        start = rospy.get_time()
        pressure_reached_at = None
        rate = rospy.Rate(self.command_rate_hz)
        while not rospy.is_shutdown() and not self.abort_requested:
            self.publish_pressure_target(target_pressure)
            self.publish_motor_pwm(pwm)
            elapsed = rospy.get_time() - start

            pressure_cutoff = rospy.get_time() - self.stability_window_s
            with self.lock:
                recent_pressures = [
                    value for stamp, value in self.pressure_samples
                    if stamp >= pressure_cutoff and math.isfinite(value)]
                latest_pressure = self.latest_pressure_unlocked()
            mean_pressure = mean(recent_pressures)
            # Require a complete averaging window. A single 9--10 kPa ADC
            # spike must not advance a condition whose real pressure remains
            # near zero.
            pressure_window_ready = (
                len(recent_pressures) >= max(5, int(0.5 * self.command_rate_hz)))
            pressure_in_range = (
                pressure_window_ready and math.isfinite(mean_pressure) and
                abs(mean_pressure - target_pressure) <= self.pressure_tolerance_kpa)
            if pressure_in_range:
                if pressure_reached_at is None:
                    pressure_reached_at = rospy.get_time()
                    rospy.logwarn("Mean pressure reached target range at %.2f kPa (target %.1f kPa)",
                                  mean_pressure, target_pressure)
            elif self.require_stability:
                pressure_reached_at = None

            if pressure_reached_at is not None:
                settled_for = rospy.get_time() - pressure_reached_at
                if not self.require_stability and settled_for >= self.minimum_settle_s:
                    # Identification uses the mean and standard deviation of
                    # the recording window; perfectly static motion is not
                    # required. Pressure only has to enter the target band
                    # once before the fixed settling delay.
                    return True
                if (self.require_stability and elapsed >= self.minimum_settle_s and
                        self.is_stable(target_pressure)):
                    return True
            if pressure_reached_at is None and elapsed >= self.pressure_reach_timeout_s:
                rospy.logerr("Pressure reach timeout at P=%.1f kPa PWM=%.4f (latest %.2f kPa)",
                             target_pressure, pwm, latest_pressure)
                return False
            if (self.require_stability and pressure_reached_at is not None and
                    rospy.get_time() - pressure_reached_at >= self.condition_timeout_s):
                rospy.logerr("Stability timeout after pressure reached P=%.1f kPa PWM=%.4f",
                             target_pressure, pwm)
                return False
            rate.sleep()
        return False

    def is_stable(self, target_pressure):
        cutoff = rospy.get_time() - self.stability_window_s
        with self.lock:
            pressures = [value for stamp, value in self.pressure_samples if stamp >= cutoff and math.isfinite(value)]
            imus = [sample for sample in self.imu_samples if sample["time"] >= cutoff]
        if len(pressures) < 5 or len(imus) < 5:
            return False
        pressure_ok = (abs(mean(pressures) - target_pressure) <= self.pressure_tolerance_kpa and
                       stddev(pressures) <= self.pressure_std_limit_kpa)
        gyro_rms = math.sqrt(mean([vector_norm(sample["gyro"]) ** 2 for sample in imus]))
        if self.baseline_acc is None:
            # The first condition is motor-off and establishes the gravity
            # reference, so gyro is useful here.
            motion_ok = gyro_rms <= self.gyro_rms_limit_rad_s
        else:
            # A spinning propeller keeps raw gyro RMS high even at equilibrium.
            # Use the variation of LPF gravity-derived bend angle instead.
            bend_angles = [self.bend_angle_deg(sample["acc_filtered"]) for sample in imus]
            bend_angles = [angle for angle in bend_angles if math.isfinite(angle)]
            motion_ok = len(bend_angles) >= 5 and stddev(bend_angles) <= self.bend_std_limit_deg
        return pressure_ok and motion_ok

    def record_condition(self, condition, pressure, pwm):
        self.start_recording(condition)
        end = rospy.get_time() + self.record_duration_s
        recovery_deadline = None
        rate = rospy.Rate(self.command_rate_hz)
        while not rospy.is_shutdown() and not self.abort_requested:
            self.publish_pressure_target(pressure)
            self.publish_motor_pwm(pwm)
            now = rospy.get_time()
            with self.lock:
                sample_count = len(self.current_records)

            if recovery_deadline is None and now >= end:
                with self.lock:
                    last_sample_time = self.last_neuron_imu_time
                imu_fresh = (last_sample_time is not None and
                             now - last_sample_time <= 0.2)
                if sample_count >= 5 and imu_fresh:
                    break
                recovery_deadline = now + self.imu_recovery_timeout_s
                rospy.logwarn("Neuron %d IMU missing during condition %d; waiting up to %.1f s for recovery",
                              self.neuron_id, condition["condition_id"],
                              self.imu_recovery_timeout_s)
                # Remove an incomplete window so that only a genuinely new
                # IMU sample can trigger the recovery path below.
                self.start_recording(condition)
            elif recovery_deadline is not None:
                if sample_count > 0:
                    # Discard the first sample after the gap and collect a new,
                    # complete recording window for this same condition.
                    rospy.logwarn("Neuron %d IMU recovered during condition %d; restarting %.1f s recording",
                                  self.neuron_id, condition["condition_id"],
                                  self.record_duration_s)
                    self.start_recording(condition)
                    end = now + self.record_duration_s
                    recovery_deadline = None
                elif now >= recovery_deadline:
                    rospy.logerr("Neuron %d IMU recovery timeout during condition %d",
                                 self.neuron_id, condition["condition_id"])
                    break
                else:
                    rospy.logwarn_throttle(
                        1.0, "Waiting for Neuron %d IMU in condition %d" %
                        (self.neuron_id, condition["condition_id"]))
            rate.sleep()
        with self.lock:
            self.recording = False
            records = list(self.current_records)
            self.current_condition = None
        return records

    def start_recording(self, condition):
        with self.lock:
            self.current_condition = dict(condition)
            self.current_records = []
            self.recording = True

    def set_baseline(self, records):
        self.baseline_acc = [mean([record["acc_filtered"][axis] for record in records])
                             for axis in range(3)]
        rospy.logwarn("Baseline gravity vector set to [%.4f, %.4f, %.4f]",
                      *self.baseline_acc)

    def append_csv(self, records, stable):
        raw_exists = os.path.exists(self.raw_path)
        raw_fields = [
            "time", "condition_id", "repeat", "pressure_direction", "pwm_direction",
            "target_pressure_kpa", "motor_pwm", "pressure_kpa", "bend_deg",
            "joint_each_deg", "rotor_x_m", "rotor_y_m", "rotor_z_m",
            "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z",
            "root_acc_x", "root_acc_y", "root_acc_z", "root_gyro_x", "root_gyro_y",
            "root_gyro_z", "root_q_x", "root_q_y", "root_q_z", "root_q_w", "stable"
        ]
        with open(self.raw_path, "a", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=raw_fields)
            if not raw_exists:
                writer.writeheader()
            for record in records:
                root = record["root_imu"]
                row = {
                    "time": record["time"], "condition_id": record["condition_id"],
                    "repeat": record["repeat"], "pressure_direction": record["pressure_direction"],
                    "pwm_direction": record["pwm_direction"],
                    "target_pressure_kpa": record["target_pressure_kpa"],
                    "motor_pwm": record["motor_pwm"], "pressure_kpa": record["pressure_kpa"],
                    "bend_deg": record["bend_deg"], "stable": int(stable),
                    "joint_each_deg": record["joint_each_deg"],
                    "rotor_x_m": record["rotor_x_m"], "rotor_y_m": record["rotor_y_m"],
                    "rotor_z_m": record["rotor_z_m"],
                    "acc_x": record["acc"][0], "acc_y": record["acc"][1], "acc_z": record["acc"][2],
                    "gyro_x": record["gyro"][0], "gyro_y": record["gyro"][1], "gyro_z": record["gyro"][2],
                }
                for prefix, values in (("root_acc", root["acc"] if root else [float("nan")] * 3),
                                       ("root_gyro", root["gyro"] if root else [float("nan")] * 3)):
                    for axis, value in zip(("x", "y", "z"), values):
                        row[prefix + "_" + axis] = value
                quaternion = root["quaternion"] if root else [float("nan")] * 4
                for axis, value in zip(("x", "y", "z", "w"), quaternion):
                    row["root_q_" + axis] = value
                writer.writerow(row)

        summary_exists = os.path.exists(self.summary_path)
        summary_fields = [
            "condition_id", "repeat", "pressure_direction", "pwm_direction",
            "target_pressure_kpa", "motor_pwm", "sample_count", "stable",
            "pressure_mean_kpa", "pressure_std_kpa", "bend_mean_deg", "bend_std_deg",
            "joint_each_mean_deg", "rotor_x_mean_m", "rotor_y_mean_m", "rotor_z_mean_m",
            "gyro_rms_rad_s"
        ]
        finite_bends = [record["bend_deg"] for record in records if math.isfinite(record["bend_deg"])]
        pressures = [record["pressure_kpa"] for record in records if math.isfinite(record["pressure_kpa"])]
        gyro_rms = math.sqrt(mean([vector_norm(record["gyro"]) ** 2 for record in records]))
        mean_bend = mean(finite_bends)
        joint_deg, rotor_x, rotor_y, rotor_z = self.equal_joint_kinematics(mean_bend)
        first = records[0]
        summary = {
            "condition_id": first["condition_id"], "repeat": first["repeat"],
            "pressure_direction": first["pressure_direction"], "pwm_direction": first["pwm_direction"],
            "target_pressure_kpa": first["target_pressure_kpa"], "motor_pwm": first["motor_pwm"],
            "sample_count": len(records), "stable": int(stable),
            "pressure_mean_kpa": mean(pressures), "pressure_std_kpa": stddev(pressures),
            "bend_mean_deg": mean_bend, "bend_std_deg": stddev(finite_bends),
            "joint_each_mean_deg": joint_deg, "rotor_x_mean_m": rotor_x,
            "rotor_y_mean_m": rotor_y, "rotor_z_mean_m": rotor_z,
            "gyro_rms_rad_s": gyro_rms,
        }
        with open(self.summary_path, "a", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=summary_fields)
            if not summary_exists:
                writer.writeheader()
            writer.writerow(summary)

    def publish_pressure_target(self, pressure_kpa):
        self.commanded_pressure_kpa = pressure_kpa
        targets = [0.0, 0.0, 0.0, 0.0]
        targets[self.pressure_arm_index] = pressure_kpa
        self.pressure_target_pub.publish(Float32MultiArray(data=targets))

    def publish_motor_pwm(self, pwm):
        self.commanded_motor_pwm = pwm
        self.pwm_pub.publish(PwmTest(motor_index=[self.motor_index], pwms=[pwm]))

    def safe_stop(self):
        try:
            self.running = False
            self.commanded_motor_pwm = self.idle_pwm
            self.commanded_pressure_kpa = 0.0
            self.pwm_pub.publish(PwmTest(
                motor_index=[self.motor_index], pwms=[self.idle_pwm]))
            # An empty PwmTest exits Spinal's bench-test mode immediately.
            self.pwm_pub.publish(PwmTest())
            self.pressure_target_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("arm_bend_identification")
    ArmBendIdentification()
    rospy.spin()
