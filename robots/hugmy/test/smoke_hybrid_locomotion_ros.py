#!/usr/bin/env python3
"""Isolated ROS wiring test, synthetic sensors only; never uses an existing master.

This is NOT a physics or real-robot performance test. Invoke after sourcing devel.
"""
import math
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exploration', action='store_true',
                        help='test thrust growth then stale stop; default tests >3-s continuous drive and averaged goal')
    parser.add_argument('--continuation', action='store_true',
                        help='force prediction timeout, check automatic retry and a synthetic goal arrival')
    parser.add_argument('--backward', action='store_true', help='use the negative-x goal and rear pair')
    parser.add_argument('--bottom', action='store_true', help='exercise bottom assistance without interrupting thrust')
    parser.add_argument('--pressure-guards', action='store_true',
                        help='preparation ripple, transient spike hold, then persistent overpressure stop')
    parser.add_argument('--energy-probe', action='store_true',
                        help='no-motion effort cutoff, automatic m change, then synthetic retained goal')
    args = parser.parse_args()
    if args.energy_probe or args.continuation or args.bottom:
        parser.error('legacy repeated MPC/bottom search was replaced by single-pass motion; use default, --exploration or --pressure-guards')
    exploration, continuation = args.exploration, args.continuation
    direction = -1. if args.backward else 1.
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="hugmy_hybrid_ros_") as directory:
        env = os.environ.copy()
        env.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port,
                   ROS_HOSTNAME="127.0.0.1", ROS_LOG_DIR=directory)
        os.environ.update(env)
        children = []
        stop = threading.Event()
        log_path = Path(directory) / "controller.log"
        with log_path.open("w") as log:
            try:
                children.append(subprocess.Popen(["roscore", "-p", str(port)], env=env,
                                                 stdout=log, stderr=subprocess.STDOUT))
                import rospy
                from geometry_msgs.msg import PoseStamped, Vector3Stamped
                from spinal.msg import Imu, NeuronImu, NeuronImuStates, PerchingThrustCommand
                from std_msgs.msg import Float32, Float32MultiArray, UInt8
                from std_srvs.srv import SetBool, SetBoolResponse
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    try:
                        rospy.get_master().getPid()
                        break
                    except Exception:
                        time.sleep(.1)
                rospy.init_node("hybrid_fixture", disable_signals=True)
                ns = "/hybrid_fixture_robot"
                node = ns + "/attitude_pressure_controller"
                root = Path(__file__).resolve().parents[1]
                config = yaml.safe_load((root / "config/attitude_pressure_controller.yaml").read_text())
                config.update(inchworm_action_c_enabled=True, use_pwm_test_for_correction=False,
                              base_pressure_kpa=30., inchworm_max_cycles=10,
                              hybrid_locomotion_outer_enabled=False,
                              hybrid_locomotion_initial_bottom_kpa=0.)
                # These sensors have fixed bend and no real thrust dynamics.
                # Test the full regrip at a low fixture cap separately from
                # breakaway growth; do not claim high-force physics validation.
                if not exploration:
                    config['action_c_max_thrust_n'] = 1.
                if args.energy_probe:
                    config.update(action_c_max_thrust_n=2.,hybrid_locomotion_outer_enabled=True)
                if continuation:
                    config.update(hybrid_locomotion_outer_enabled=True,
                                  hybrid_locomotion_budget_ms=1e-12)
                if args.bottom:
                    config['hybrid_locomotion_outer_enabled']=True
                rospy.set_param(node, config)
                rospy.Service(ns + "/independent_arm_pressure_controller/enable", SetBool,
                              lambda request: SetBoolResponse(True, "synthetic pressure plant"))
                command = [30.] * 4
                pressure = [30.] * 4
                bottom_command = [0.]
                bottom = [3.]  # residual pressure at a zero target (cd_1 regression)
                applied = [0.] * 4
                states, plans, thrusts = [], [], []
                bottoms=[]
                powered_since = [None]
                rospy.Subscriber(ns + "/independent_arm_pressure_controller/target_pressure",
                                 Float32MultiArray, lambda m: command.__setitem__(slice(None), m.data))
                def bottom_callback(message):
                    bottom_command[0]=message.data
                    bottoms.append((time.monotonic(),message.data))
                rospy.Subscriber(ns + "/independent_arm_pressure_controller/bottom_target_pressure",
                                 Float32, bottom_callback)
                def thrust_callback(message):
                    applied[:] = message.thrust
                    thrusts.append((time.monotonic(), list(message.thrust)))
                rospy.Subscriber(ns + "/perching_correction/thrust_command", PerchingThrustCommand, thrust_callback)
                def state_callback(message):
                    if not states or states[-1] != message.data:
                        states.append(message.data)
                    if message.data==16 and powered_since[0] is None:
                        powered_since[0]=time.monotonic()
                rospy.Subscriber(node + "/state", UInt8, state_callback)
                rospy.Subscriber(node + "/hybrid_locomotion", Float32MultiArray, lambda m: plans.append(list(m.data)))
                imu = rospy.Publisher(ns + "/imu", Imu, queue_size=1)
                neurons = rospy.Publisher(ns + "/neuron/imu_states", NeuronImuStates, queue_size=1)
                pressures = rospy.Publisher(ns + "/independent_arm_pressure_controller/pressure", Float32MultiArray, queue_size=1)
                bottom_pub = rospy.Publisher(ns + "/independent_arm_pressure_controller/bottom_pressure", Float32, queue_size=1)
                pose_pub = rospy.Publisher(ns + "/mocap/pose", PoseStamped, queue_size=1)
                target = rospy.Publisher(node + "/target_attitude", Vector3Stamped, queue_size=1, latch=True)
                move = rospy.Publisher(node + "/inchworm_command", Vector3Stamped, queue_size=1)
                pause_pressure = threading.Event()
                def sensors():
                    counter=0
                    while not stop.is_set():
                        counter+=1
                        for j in range(4):
                            pressure[j] += max(-.30, min(.22, command[j] - pressure[j]))
                        bottom[0] += max(-.30, min(.22, max(3., bottom_command[0]) - bottom[0]))
                        imu.publish(Imu(quaternion=[0., 0., 0., 1.], gyro=[0., 0., 0.]))
                        neurons.publish(NeuronImuStates(imus=[NeuronImu(slave_id=j+1,
                            acc=[-9.81*math.sin(1.4), 0., 9.81*math.cos(1.4)], gyro=[0.,0.,0.]) for j in range(4)]))
                        if not pause_pressure.is_set():
                            sample=list(pressure)
                            if args.pressure_guards:
                                if powered_since[0] is None and counter%20==10:
                                    sample[2]+=4. # strict +/-2 never stays ready for 1.2 s
                                if powered_since[0] is not None:
                                    elapsed=time.monotonic()-powered_since[0]
                                    if 1. <= elapsed < 1.03 or elapsed>=3.:
                                        sample[0]=50. # <60 hard limit, >45 soft limit
                            pressures.publish(Float32MultiArray(data=sample))
                        bottom_pub.publish(Float32(data=bottom[0]))
                        pose = PoseStamped(); pose.header.stamp = rospy.Time.now(); pose.pose.orientation.w = 1.
                        # Only the continuation test injects a known goal
                        # crossing; this tests task ownership, not locomotion.
                        if (continuation or args.energy_probe) and states.count(16)>=2:
                            pose.pose.position.x=direction*.021
                        elif not exploration and not continuation and not args.energy_probe and powered_since[0] is not None and time.monotonic()-powered_since[0]>(10. if args.bottom else 5.):
                            pose.pose.position.x=direction*.021
                        pose_pub.publish(pose)
                        time.sleep(.01)
                thread = threading.Thread(target=sensors, daemon=True)
                thread.start()
                children.append(subprocess.Popen(["rosrun", "hugmy", "attitude_pressure_controller_node",
                    "__ns:=" + ns, "__name:=attitude_pressure_controller"], env=env, stdout=log, stderr=subprocess.STDOUT))
                rospy.wait_for_service(node + "/enable", timeout=15)
                connected = [imu, neurons, pressures, bottom_pub, pose_pub, target, move]
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline and any(p.get_num_connections()==0 for p in connected):
                    time.sleep(.05)
                assert all(p.get_num_connections()>0 for p in connected), "synthetic sensor connections not ready"
                time.sleep(.5)
                target.publish(Vector3Stamped())
                assert rospy.ServiceProxy(node + "/enable", SetBool)(True).success
                time.sleep(.5)
                request = Vector3Stamped();request.vector.x=direction;request.vector.z=.02
                move.publish(request)
                deadline = time.monotonic() + 25
                def finished():
                    if args.pressure_guards:
                        return 16 in states and states[-1]==0
                    if args.energy_probe:
                        return states.count(16)>=2 and states[-1]==0
                    if continuation:
                        return states.count(16)>=2 and states[-1]==0
                    if exploration:
                        return any(max(v) >= 1.5 for _, v in thrusts)
                    return 18 in states and states[-1]==0
                while time.monotonic() < deadline and not finished():
                    time.sleep(.05)
                if not continuation:
                    assert plans and any(p[0] == 2 and p[1] == 1 for p in plans), "no single-pass motion output"
                    assert any(max(abs(t) for t in v) > .01 for _, v in thrusts), "no actuator output"
                assert 16 in states, "powered phase not reached"
                if args.pressure_guards:
                    assert finished(), 'sustained overpressure did not stop'
                    assert states.count(16)==1, 'pressure ripple interrupted/restarted task'
                    pending=[i for i,p in enumerate(plans) if len(p)>=26 and p[24]>0.5]
                    assert pending, 'transient was not held'
                    i=pending[0]
                    assert any(p[1]==1 and p[24]<.5 for p in plans[i+1:]), 'no continuation after transient'
                    for i in pending:
                        if i>0 and plans[i-1][1]==1:
                            assert all(abs(plans[i][j]-plans[i-1][j])<1e-6 for j in (16,17)), 'thrust changed during pressure confirmation'
                    log.flush()
                    assert 'sustained overpressure' in log_path.read_text(), 'wrong stop reason'
                    assert max(abs(t) for t in thrusts[-1][1])<.01, 'fault did not zero thrust'
                elif args.energy_probe:
                    assert finished(), 'no automatic mix retry/goal completion'
                    assert any(p[8]<1. for p in plans), 'mix remained stuck at 1'
                    mixed=[p for p in plans if p[8]<1. and p[1]==1]
                    assert mixed and max(p[39] for p in mixed)-min(p[39] for p in mixed)<.01, 'mix trial changed bottom at the same time'
                    log.flush()
                    assert 'unproductive powered probe' in log_path.read_text(), 'no effort-limited termination'
                    assert states.count(16)==2, 'extra retry after synthetic goal'
                elif continuation:
                    assert finished(), "no automatic retry/goal completion after one command"
                    assert all(p[7]<.1 for p in plans), "prediction failure must not scan bottom/freeze thrust"
                    assert states.count(18)>=2, "prediction failure did not regrip before retry"
                    time.sleep(.6)
                    assert states[-1]==0 and states.count(16)==2, "restarted after goal arrival"
                elif exploration:
                    assert finished(), "no increase beyond the 1-N initial search"
                else:
                    assert finished(), "stable goal not confirmed at the 1-N fixture cap"
                    assert states.count(16)==1, "unnecessary timed interruption"
                    powered=[t for t,v in thrusts if max(v)>.1]
                    assert powered[-1]-powered[0]>4., "did not drive continuously beyond the old 3-s limit"
                    if args.bottom:
                        assert max(p for _,p in bottoms)>3., 'no bottom assistance during continuous run'
                        assert bottom_command[0]==0., 'endpoint was not measured at zero bottom target'
                cap = 2. if args.energy_probe else 6. if exploration else 1.
                assert all(max(abs(t) for t in v) <= cap+.001 for _, v in thrusts), "thrust cap exceeded"
                pause_pressure.set()
                time.sleep(.6)
                assert thrusts and max(abs(t) for t in thrusts[-1][1]) < .01, "stale-pressure stop failed"
                attempts = states.count(16)
                pause_pressure.clear()
                time.sleep(.5)
                assert states.count(16)==attempts and max(abs(t) for t in thrusts[-1][1])<.01, "sensor fault incorrectly auto-retried"
                checked = ('switched m after an effort-limited stalled probe and reached synthetic goal' if args.energy_probe else 'passed ripple/transient hold and persistent-overpressure stop' if args.pressure_guards else 'automatically retried without frozen bottom probes and confirmed synthetic goal' if continuation
                           else 'increased thrust beyond 1.5 N' if exploration else 'drove >3 s continuously and confirmed low-bottom mean goal')
                print("PASS: isolated ROS accepted 3-kPa residual at zero bottom target, " + checked + ", and stopped on stale pressure", flush=True)
            except Exception:
                log.flush()
                print(log_path.read_text()[-14000:], flush=True)
                raise
            finally:
                stop.set()
                for process in reversed(children):
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill();process.wait()


if __name__ == "__main__":
    main()
