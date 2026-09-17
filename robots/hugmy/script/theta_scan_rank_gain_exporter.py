#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import yaml
import numpy as np
from std_msgs.msg import Float64MultiArray
from aerial_robot_msgs.msg import FourAxisGain
from sensor_msgs.msg import JointState

"""
機能:
  - theta を掃引し、/debug/q_matrix から rank(Q) を計算
  - /debug/four_axes/gain から LQI ゲインを回収
  - JointState を publish して robot_model に θ を反映（C++ の applyThetaToModel と同等の挙動）
  - rank >= 4 が保たれる連続区間から [theta_min, theta_max] を推定
  - 指定 thetas でのゲインを YAML に整形して保存

前提:
  - C++ 側で /debug/q_matrix (Float64MultiArray; row-major 4xM) が publish されている
  - /debug/four_axes/gain (aerial_robot_msgs/FourAxisGain) が publish されている
  - robot_model が publish された JointState を受けて内部状態を更新できる構成
"""

# ====== ユーザ調整パラメータ ======
NAMESPACE = "/quadrotor"
Q_TOPIC         = NAMESPACE + "/debug/q_matrix"
GAIN_TOPIC      = NAMESPACE + "/debug/four_axes/gain"

# ★ C++ の joint_reflect_theta_pub_ に合わせる（実際のトピック名に変えてください）
JOINT_THETA_TOPIC = NAMESPACE + "/joints_ctrl"

# 掃引レンジ
THETA_MIN = 0.0
THETA_MAX = 1.4
THETA_STEP = 0.02

# YAML に残す代表 θ
THETA_TABLE = [0.0, 0.3, 0.6, 0.9, 1.2, 1.4]

# 保存先
OUT_YAML = "/tmp/lqi_gains_theta_table.yaml"

# rank 判定の数値閾値
RANK_TOL = 1e-6

def np_rank(M, tol=RANK_TOL):
    u, s, vh = np.linalg.svd(M, full_matrices=False)
    return int(np.sum(s > tol))

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def build_jointstate_for_theta(theta):
    """
    C++ の applyThetaToModel と同じ関節命名/上限で JointState を組み立て。
    joint_1_1..joint_4_3 の 12関節に θ/3 を配分。
    joint_*_1 は 0.8727rad、joint_*_{2,3} は 1.047rad でクランプ。
    """
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = []
    js.position = []

    lim1 = 0.8727
    limN = 1.047
    d = float(theta) / 3.0

    for idx in range(1, 5):
        j1 = f"joint_{idx}_1"
        j2 = f"joint_{idx}_2"
        j3 = f"joint_{idx}_3"

        v1 = clamp(d, 0.0, lim1)
        v2 = clamp(d, 0.0, limN)
        v3 = clamp(d, 0.0, limN)

        js.name.extend([j1, j2, j3])
        js.position.extend([v1, v2, v3])

    return js

class GainSampler(object):
    def __init__(self):
        self.q_mat = None
        self.gain_msg = None
        self.q_sub = rospy.Subscriber(Q_TOPIC, Float64MultiArray, self.q_cb, queue_size=1)
        self.g_sub = rospy.Subscriber(GAIN_TOPIC, FourAxisGain, self.g_cb, queue_size=1)

        # θ反映: JointState を publish（C++ applyThetaToModel 相当）
        self.theta_pub = rospy.Publisher(JOINT_THETA_TOPIC, JointState, queue_size=1)

    def q_cb(self, msg):
        if len(msg.layout.dim) >= 2:
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size
        else:
            rows = 4
            cols = int(len(msg.data) / 4)
        arr = np.array(msg.data, dtype=float).reshape((rows, cols))
        self.q_mat = arr

    def g_cb(self, msg):
        self.gain_msg = msg

    def command_theta(self, theta):
        js = build_jointstate_for_theta(theta)
        self.theta_pub.publish(js)

    def wait_for_q(self, timeout=1.0):
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.q_mat is not None:
                return True
            if rospy.Time.now().to_sec() - t0 > timeout:
                return False
            rospy.sleep(0.01)

    def wait_for_gain(self, timeout=1.0):
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.gain_msg is not None:
                return True
            if rospy.Time.now().to_sec() - t0 > timeout:
                return False
            rospy.sleep(0.01)

    def get_rank(self):
        if self.q_mat is None:
            return None
        return np_rank(self.q_mat)

    def get_gain_arrays(self):
        g = self.gain_msg
        if g is None:
            return None

        def pack3(p, i, d):
            return [[float(p[k]), float(i[k]), float(d[k])] for k in range(len(p))]

        roll = pack3(g.roll_p_gain,  g.roll_i_gain,  g.roll_d_gain)
        pitch= pack3(g.pitch_p_gain, g.pitch_i_gain, g.pitch_d_gain)
        yaw  = pack3(g.yaw_p_gain,   g.yaw_i_gain,   g.yaw_d_gain)
        zed  = pack3(g.z_p_gain,     g.z_i_gain,     g.z_d_gain)
        return roll, pitch, yaw, zed


def main():
    rospy.init_node("theta_scan_rank_gain_exporter")
    sampler = GainSampler()
    rospy.loginfo("theta_scan_rank_gain_exporter: start")

    thetas = np.arange(THETA_MIN, THETA_MAX + 1e-9, THETA_STEP)
    ranks  = []
    table_gains = {th: None for th in THETA_TABLE}

    for th in thetas:
        sampler.command_theta(float(th))
        rospy.sleep(0.25)  # モデル更新 & LQI再計算待ち（環境に合わせて調整）

        ok_q = sampler.wait_for_q(timeout=1.0)
        ok_g = sampler.wait_for_gain(timeout=1.0)
        if not ok_q:
            rospy.logwarn("Q not received at theta=%.3f" % th)
            ranks.append(None)
            continue

        r = sampler.get_rank()
        ranks.append(r)
        rospy.loginfo("theta=%.3f -> rank(Q)=%s" % (th, str(r)))

        for tref in THETA_TABLE:
            if table_gains[tref] is None and abs(th - tref) < (THETA_STEP/2.0 + 1e-6):
                if ok_g:
                    g = sampler.get_gain_arrays()
                    if g is not None:
                        table_gains[tref] = g
                        rospy.loginfo("  captured gains for theta=%.2f" % tref)

    theta_valid = []
    for th, r in zip(thetas, ranks):
        if r is not None and r >= 4:
            theta_valid.append(float(th))

    if len(theta_valid) == 0:
        th_min, th_max = None, None
        rospy.logwarn("No theta with rank(Q) >= 4 found")
    else:
        th_min, th_max = min(theta_valid), max(theta_valid)
        rospy.loginfo("Estimated stable theta range (rank>=4): [%.3f, %.3f]" % (th_min, th_max))

    yaml_out = {
        "controller": {
            "lqi": {
                "thetas": THETA_TABLE,
                "roll_gains": [],
                "pitch_gains": [],
                "yaw_gains": [],
                "z_gains": [],
            }
        }
    }

    last_good = None
    for tref in THETA_TABLE:
        g = table_gains[tref]
        if g is None:
            if last_good is None:
                rospy.logwarn("No gains captured at theta=%.2f, using zeros" % tref)
                zeros = [[0.0,0.0,0.0] for _ in range(4)]  # モータ数は必要に応じて変更
                g = (zeros, zeros, zeros, zeros)
            else:
                rospy.logwarn("No gains captured at theta=%.2f, reusing previous" % tref)
                g = last_good
        roll, pitch, yaw, zed = g
        yaml_out["controller"]["lqi"]["roll_gains"].append(roll)
        yaml_out["controller"]["lqi"]["pitch_gains"].append(pitch)
        yaml_out["controller"]["lqi"]["yaw_gains"].append(yaw)
        yaml_out["controller"]["lqi"]["z_gains"].append(zed)
        last_good = g

    with open(OUT_YAML, "w") as f:
        yaml.safe_dump(yaml_out, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    rospy.loginfo("Wrote YAML to %s" % OUT_YAML)
    if theta_valid:
        rospy.loginfo("Stable theta range (rank>=4): [%.3f, %.3f]" % (th_min, th_max))

if __name__ == "__main__":
    main()
