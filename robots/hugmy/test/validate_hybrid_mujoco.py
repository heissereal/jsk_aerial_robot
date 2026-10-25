#!/usr/bin/env python3
"""Run the actual MuJoCo/ROS plant on an isolated master, with no hardware IO.

No synthetic state/pose is published. Outputs/logs are kept for inspection.
"""
import argparse
import json
import math
import os
import re
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

SCRIPT_DIRECTORY = Path(__file__).resolve().parents[1] / 'script'
sys.path.insert(0, str(SCRIPT_DIRECTORY))
from finalize_mujoco_model import (  # noqa: E402
    AIST_FOREARM_DISTAL_DEPTH_M,
    AIST_FOREARM_DISTAL_WIDTH_M,
    AIST_FOREARM_LENGTH_M,
    AIST_FOREARM_MEAN_DEPTH_M,
    AIST_FOREARM_MEAN_WIDTH_M,
    AIST_FOREARM_PROXIMAL_DEPTH_M,
    AIST_FOREARM_PROXIMAL_WIDTH_M,
    tapered_elliptic_forearm_mesh,
)


def settled_x_summary(poses, end_time, duration=3.):
    """Same small-window principle as PositionWindow, not raw noisy extrema."""
    tail=[(t,p[0]) for t,p in poses if end_time-duration<=t<=end_time]
    if len(tail)<8 or tail[0][0]>end_time-duration+.25 or tail[-1][0]<end_time-.25:
        return float('nan'),float('inf'),False
    bins=[[] for _ in range(4)]
    for t,x in tail:
        bins[min(3,int(4*(t-end_time+duration)/duration))].append(x)
    if any(not b for b in bins):
        return float('nan'),float('inf'),False
    means=[sum(b)/len(b) for b in bins]
    mean=sum(x for _,x in tail)/len(tail)
    span=max(means)-min(means)
    stable=span<=.002 and abs(means[-1]-means[0])/(.75*duration)<=.002 and abs(tail[-1][1]-mean)<=.004
    return mean,span,stable


def forward_cycle_summary(logs):
    """Count retained progress, never powered displacement or phase transitions."""
    progress=[]; consecutive=0; longest=0
    for _,msg in logs:
        match=re.search(r'Hybrid C/D result: completed=1 safe=1 retained ([-+0-9.]+) mm',msg)
        if match:
            value=float(match.group(1));progress.append(value)
            consecutive=consecutive+1 if value>=1. else 0
            longest=max(longest,consecutive)
        elif 'Hybrid C/D result:' in msg:
            consecutive=0
    return progress,longest


def contact_regrasped(pressure, grip, pressure_floor=15., tolerance=1.):
    """Accept the controller's measured load-bearing pressure equilibrium.

    Recovery may deliberately retain one pair below nominal 30 kPa when
    further flexion reverses its roll moment.  Regrasp therefore means at
    least three measured contacts and every chamber above the configured
    15-kPa support floor, not equality with the nominal pressure.
    """
    return bool(pressure and grip and len(pressure) >= 4 and len(grip) >= 4 and
                all(math.isfinite(p) and p >= pressure_floor-tolerance
                    for p in pressure[:4]) and
                sum(1 for value in grip[:4] if value > .5) >= 3)


def terminate_run_processes(output_path):
    """Terminate daemonized processes that belong to this validation run.

    mujoco_node creates a new session on some systems and can therefore
    outlive both roslaunch and its original process group.  Every process of
    an isolated validation run receives a ROS log path below output_path, so
    use that exact path as the ownership token rather than matching generic
    ROS or MuJoCo process names.
    """
    token=str(output_path).encode()
    owned=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        pid=int(entry.name)
        if pid==os.getpid():
            continue
        try:
            command=(entry/'cmdline').read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if token in command:
            owned.append(pid)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in owned:
            try:
                os.kill(pid,sig)
            except ProcessLookupError:
                pass
        if sig==signal.SIGTERM:
            time.sleep(.5)
            owned=[pid for pid in owned if Path('/proc/%d' % pid).exists()]
            if not owned:
                break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--distance', type=float, default=.02)
    parser.add_argument('--lateral-limit', type=float, default=.03,
                        help='final lateral diagnostic limit in m; forward motion may include transverse travel')
    parser.add_argument('--bottom', type=float, default=30.)
    parser.add_argument('--mix', type=float, default=1., help='legacy option: only 1 is accepted; single-pass motion derives m from its measured trajectory')
    parser.add_argument('--contact-z', type=float, default=None, help='controller support pivot in body coordinates; does not change physics')
    parser.add_argument('--rear-pressure', type=float, default=0., help='residual rear grasp during sliding; controller only, 0..6 kPa')
    parser.add_argument('--pressure-limit', type=float, default=40., help='arm pressure command limit, <=50 kPa; no change to hard pressure protection')
    parser.add_argument('--retain-grasp', action='store_true', help='retain front regrip pressure at the settled endpoint instead of re-extending to 30 kPa')
    parser.add_argument('--extension-limit-deg', type=float, default=20., help='simulation comparison only, 10..35 deg; physical hinge stops unchanged')
    parser.add_argument('--predictive-recovery', action='store_true', help='execute existing recovery MPC (not shadow) and use the same support-pivot assumption')
    parser.add_argument('--body-z', type=float, default=None,
                        help='legacy MJCF z before height offset; omit to use production perching placement')
    parser.add_argument('--initial-pressure', type=float, default=30.)
    parser.add_argument('--grip-stiffness', type=float, default=1200., help='tangential N/m; conservative human-arm proxy default')
    parser.add_argument('--grip-friction', type=float, default=4.5, help='pneumatic grip Coulomb coefficient; conservative proxy default')
    parser.add_argument('--lift-pressure-gain', type=float, default=20., help='deprecated compatibility option; early lift remains at zero front pressure, with bounded predictive prefill on an adverse slope')
    parser.add_argument('--pull-min-sec', type=float, default=.5, help='minimum powered pull before front catch, .5..4 s; hard guards unchanged')
    parser.add_argument('--pull-seed-min', type=float, default=2.5,
                        help='pair-level horizontal pull seed after an ineffective slide, 1..6 N')
    parser.add_argument('--pull-seed-max', type=float, default=2.55,
                        help='pair-level horizontal pull seed ceiling, seed-min..8 N')
    parser.add_argument('--lift-rate', type=float, default=.7, help='lift thrust search rate, (0,1] N/s')
    parser.add_argument('--lift-seconds', type=float, default=15., help='powered lift deadline after front venting, 1..20 s')
    parser.add_argument('--probe-extension-deg', type=float, default=8., help='desired extension, 3..15 degrees; protection remains separate')
    parser.add_argument('--identification-bend-deg', type=float, default=45.,
                        help='deepest bounded motion-2 search pose, 5..60 degrees; not a completion condition')
    parser.add_argument('--identification-extension-limit-deg', type=float, default=99.,
                        help='independent relative Motion-2 excursion guard, 20..120 degrees')
    parser.add_argument('--unload-min-extension-deg', type=float, default=3.,
                        help='minimum measured extension required on both front arms, 1..10 degrees')
    parser.add_argument('--unload-pitch-deg', type=float, default=4.,
                        help='commanded-direction pitch change accepted as unloading, 0.5..10 degrees')
    parser.add_argument('--unload-height-mm', type=float, default=2.,
                        help='body-height increase accepted as unloading, 0.5..10 mm')
    parser.add_argument('--unload-confirm-sec', type=float, default=.06,
                        help='continuous unloading evidence required for handoff, 0.06..0.5 s')
    parser.add_argument('--unload-prefill-kpa', type=float, default=3.,
                        help='bounded predictive pilot fill after the first measured event, 1..10 kPa')
    parser.add_argument('--trajectory-min-extension-deg', type=float, default=15.,
                        help='minimum paired Motion-2 travel before measured unloading can start Motion 3, 5..45 deg')
    parser.add_argument('--trajectory-extension-rate', type=float, default=.70,
                        help='minimum-jerk Motion-2 bend-reference rate, .2..2 rad/s')
    parser.add_argument('--windup-pitch-deg', type=float, default=17.,
                        help='body-pitch target after load relief and before pressure flexion, 2..17 deg')
    parser.add_argument('--windup-pitch-kp', type=float, default=3.,
                        help='common front-thrust pitch feedback gain, 0..20 N/rad')
    parser.add_argument('--windup-pitch-kd', type=float, default=1.,
                        help='common front-thrust pitch-rate feedback gain, 0..10 N/(rad/s)')
    parser.add_argument('--windup-bottom-kpa', type=float, default=30.,
                        help='bottom target during rear-pivot windup, 0..initial bottom kPa')
    parser.add_argument('--windup-rear-reverse', type=float, default=1.5,
                        help='rear-pair reverse-thrust limit during windup, 0..3 N')
    parser.add_argument('--trajectory-flex-rate', type=float, default=1.2,
                        help='minimum-jerk Motion-3 bend-reference rate, .2..4 rad/s')
    parser.add_argument('--trajectory-pressure-lead', type=float, default=.60,
                        help='maximum learned pneumatic feed-forward lead, .05..1.2 s')
    parser.add_argument('--cylinder-radius', type=float, default=None,
                        help='legacy compatibility: make a uniform circular fixture with this radius')
    parser.add_argument('--forearm-length', type=float, default=AIST_FOREARM_LENGTH_M,
                        help='tapered forearm length in m')
    parser.add_argument('--forearm-proximal-width', type=float,
                        default=AIST_FOREARM_MEAN_WIDTH_M)
    parser.add_argument('--forearm-proximal-depth', type=float,
                        default=AIST_FOREARM_MEAN_DEPTH_M)
    parser.add_argument('--forearm-distal-width', type=float,
                        default=AIST_FOREARM_MEAN_WIDTH_M)
    parser.add_argument('--forearm-distal-depth', type=float,
                        default=AIST_FOREARM_MEAN_DEPTH_M)
    parser.add_argument('--cylinder-pitch-deg', type=float, default=0.,
                        help='fixture pitch in degrees for longitudinal-slope validation, -15..15')
    parser.add_argument('--surface-friction', type=float, default=4.0, help='MuJoCo sliding friction; conservative human-arm proxy default')
    parser.add_argument('--slip-min-progress', type=float, default=.002,
                        help='test routing threshold in m; production default is .002')
    parser.add_argument('--slip-thrust-max', type=float, default=5.,
                        help='sliding-probe rotor ceiling in N; production default is 5')
    parser.add_argument('--require-cycles', type=int, default=0, help='minimum completed and measured locomotion strokes')
    parser.add_argument('--require-forward-cycles', type=int, default=0, help='minimum consecutive settled strokes retaining at least 1 mm each')
    parser.add_argument('--require-unload-events', type=int, default=0,
                        help='minimum measured two-arm/body unloading events')
    parser.add_argument('--require-optimizer-observations', type=int, default=0,
                        help='minimum completed outer trajectory evaluations')
    parser.add_argument('--require-no-slip', action='store_true', help='also require absolute initial sliding probe displacement <=2 mm and no sustained sliding')
    parser.add_argument('--grasp-only', action='store_true', help='negative control: no locomotion command')
    parser.add_argument('--recovery-only', action='store_true',
                        help='test only the shared roll-posture recovery; do not send a locomotion command')
    parser.add_argument('--target-roll-deg', type=float, default=15.,
                        help='absolute body-roll target for --recovery-only, -30..30 deg and at least 5 deg from zero')
    parser.add_argument('--target-roll-offset-deg', type=float, default=None,
                        help='instead target this signed roll change from the measured initial equilibrium, -30..30 deg')
    parser.add_argument('--recovery-tolerance-deg', type=float, default=3.,
                        help='settled roll-error tolerance for --recovery-only, 1..5 deg')
    args = parser.parse_args()
    if args.mix != 1.:
        parser.error('--mix is no longer a commanded search variable; omit it for single-pass motion')
    if args.grasp_only and args.recovery_only:
        parser.error('--grasp-only and --recovery-only are mutually exclusive')
    if not (math.isfinite(args.seconds) and 0.<args.seconds<=300. and
            math.isfinite(args.distance) and 0.<args.distance<=.1 and
            math.isfinite(args.lateral_limit) and .01<=args.lateral_limit<=.1 and
            math.isfinite(args.bottom) and 0.<=args.bottom<=30. and
            math.isfinite(args.mix) and 0.<=args.mix<=1. and
            math.isfinite(args.extension_limit_deg) and 10.<=args.extension_limit_deg<=35. and
            math.isfinite(args.rear_pressure) and 0.<=args.rear_pressure<=6. and
            math.isfinite(args.grip_friction) and 0.<=args.grip_friction<=8. and
            math.isfinite(args.surface_friction) and 0.<=args.surface_friction<=8. and
            math.isfinite(args.slip_min_progress) and .001<=args.slip_min_progress<=.02 and
            math.isfinite(args.slip_thrust_max) and 0.<args.slip_thrust_max<=5. and
            (args.cylinder_radius is None or
             (math.isfinite(args.cylinder_radius) and .025<=args.cylinder_radius<=.15)) and
            math.isfinite(args.forearm_length) and .15<=args.forearm_length<=.40 and
            math.isfinite(args.forearm_proximal_width) and .05<=args.forearm_proximal_width<=.14 and
            math.isfinite(args.forearm_proximal_depth) and .04<=args.forearm_proximal_depth<=.12 and
            math.isfinite(args.forearm_distal_width) and .04<=args.forearm_distal_width<=.12 and
            math.isfinite(args.forearm_distal_depth) and .03<=args.forearm_distal_depth<=.10 and
            math.isfinite(args.cylinder_pitch_deg) and -15.<=args.cylinder_pitch_deg<=15. and
            math.isfinite(args.grip_stiffness) and 0.<=args.grip_stiffness<=5000. and
            math.isfinite(args.lift_pressure_gain) and 0.<args.lift_pressure_gain<=150. and
            math.isfinite(args.pull_min_sec) and .5<=args.pull_min_sec<=4. and
            math.isfinite(args.pull_seed_min) and 1.<=args.pull_seed_min<=6. and
            math.isfinite(args.pull_seed_max) and
              args.pull_seed_min<=args.pull_seed_max<=8. and
            math.isfinite(args.lift_rate) and 0.<args.lift_rate<=1. and
            math.isfinite(args.lift_seconds) and 1.<=args.lift_seconds<=20. and
            math.isfinite(args.probe_extension_deg) and 3.<=args.probe_extension_deg<=15. and
            math.isfinite(args.identification_bend_deg) and 5.<=args.identification_bend_deg<=60. and
            math.isfinite(args.identification_extension_limit_deg) and
              20.<=args.identification_extension_limit_deg<=120. and
            math.isfinite(args.unload_min_extension_deg) and 1.<=args.unload_min_extension_deg<=10. and
            math.isfinite(args.unload_pitch_deg) and .5<=args.unload_pitch_deg<=10. and
            math.isfinite(args.unload_height_mm) and .5<=args.unload_height_mm<=10. and
            math.isfinite(args.unload_confirm_sec) and .06<=args.unload_confirm_sec<=.5 and
            math.isfinite(args.unload_prefill_kpa) and 1.<=args.unload_prefill_kpa<=10. and
            math.isfinite(args.trajectory_min_extension_deg) and
              5.<=args.trajectory_min_extension_deg<=45. and
            math.isfinite(args.trajectory_extension_rate) and
              .2<=args.trajectory_extension_rate<=2. and
            math.isfinite(args.windup_pitch_deg) and
              2.<=args.windup_pitch_deg<=17. and
            math.isfinite(args.windup_pitch_kp) and
              0.<=args.windup_pitch_kp<=20. and
            math.isfinite(args.windup_pitch_kd) and
              0.<=args.windup_pitch_kd<=10. and
            math.isfinite(args.windup_bottom_kpa) and
              0.<=args.windup_bottom_kpa<=args.bottom and
            math.isfinite(args.windup_rear_reverse) and
              0.<=args.windup_rear_reverse<=3. and
            math.isfinite(args.trajectory_flex_rate) and
              .2<=args.trajectory_flex_rate<=4. and
            math.isfinite(args.trajectory_pressure_lead) and
              .05<=args.trajectory_pressure_lead<=1.2 and
            math.isfinite(args.target_roll_deg) and -30.<=args.target_roll_deg<=30. and
            (args.target_roll_offset_deg is None or
             (math.isfinite(args.target_roll_offset_deg) and
              -30.<=args.target_roll_offset_deg<=30.)) and
            math.isfinite(args.recovery_tolerance_deg) and
              1.<=args.recovery_tolerance_deg<=5. and
            0<=args.require_cycles<=10 and
            0<=args.require_forward_cycles<=10 and
            0<=args.require_unload_events<=20 and
            0<=args.require_optimizer_observations<=20 and
            math.isfinite(args.pressure_limit) and 30.<=args.pressure_limit<=50.):
        parser.error('require seconds in (0,300], distance in (0,0.1] m, bottom 0..30, rear 0..6, arm limit 30..50 kPa')
    requested_recovery_step=(args.target_roll_offset_deg
        if args.target_roll_offset_deg is not None else args.target_roll_deg)
    if args.recovery_only and abs(requested_recovery_step) < 5.:
        parser.error('--recovery-only requires a target/offset magnitude >= 5 deg')
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    # spawn_z belongs to Gazebo in bringup; make the MuJoCo initial placement
    # explicit without changing its shapes, masses, contacts or spring model.
    model_dir=Path(__file__).resolve().parents[1]/'mujoco/hugmy2'
    model=ET.parse(str(model_dir/'robot.xml'))
    if args.cylinder_radius is not None:
        args.forearm_proximal_width = args.forearm_proximal_depth = 2. * args.cylinder_radius
        args.forearm_distal_width = args.forearm_distal_depth = 2. * args.cylinder_radius
    forearm_mesh=model.find('./asset/mesh[@name="hugmy_aist_male_forearm"]')
    if forearm_mesh is None:
        raise RuntimeError('generated model does not contain the tapered AIST forearm mesh; rebuild hugmy')
    vertices,faces=tapered_elliptic_forearm_mesh(
        args.forearm_length,args.forearm_proximal_width,args.forearm_proximal_depth,
        args.forearm_distal_width,args.forearm_distal_depth)
    forearm_mesh.set('vertex',vertices)
    forearm_mesh.set('face',faces)
    if args.body_z is not None:
        model.find('./worldbody/body[@name="main_body"]').set('pos','0 0 '+str(args.body_z))
    for mesh in model.findall('./asset/mesh'):
        if mesh.get('file'):
            mesh.set('file',os.path.relpath(str(model_dir/mesh.get('file')),str(out)))
    for include in model.findall('.//include'):
        include.set('file',os.path.relpath(str((model_dir/include.get('file')).resolve()),str(out)))
    model.write(str(out/'robot.xml'))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_HOSTNAME='127.0.0.1', ROS_LOG_DIR=str(out / 'roslog'))
    os.environ.update(env)
    processes = []
    samples = {'pose': [], 'physical_roll': [], 'rpy': [], 'thrust': [], 'pwm': [],
               'actuator': [], 'pressure': [], 'bottom': [], 'grip': [],
               'plan': [], 'state': [], 'bend': [], 'circle': [], 'logs': []}
    with (out / 'launch.log').open('w') as log:
        try:
            # Give each launcher its own process group. roslaunch plugins may
            # fork a simulator which outlives the parent process; signalling
            # only Popen.pid left old MuJoCo servers consuming CPU and sharing
            # node names with later validation runs.
            processes.append(subprocess.Popen(['roscore', '-p', str(port)], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            import rospy
            from geometry_msgs.msg import PoseStamped, Vector3Stamped
            from std_msgs.msg import Float32MultiArray, Float32, UInt8
            from spinal.msg import PerchingThrustCommand, PwmTest
            from rosgraph_msgs.msg import Log
            from std_srvs.srv import SetBool
            deadline = time.monotonic() + 15.
            while time.monotonic() < deadline:
                try:
                    rospy.get_master().getPid()
                    break
                except Exception:
                    time.sleep(.1)
            rospy.set_param('/use_sim_time', True)
            rospy.init_node('validate_hybrid_mujoco', disable_signals=True)
            ns = '/quadrotor'
            node = ns + '/attitude_pressure_controller'
            def record(key, value):
                samples[key].append([rospy.Time.now().to_sec(), value])
            def pose_callback(m):
                record('pose', [m.pose.position.x, m.pose.position.y, m.pose.position.z])
                q=m.pose.orientation
                record('physical_roll',math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)))
            rospy.Subscriber(ns+'/mocap/pose', PoseStamped, pose_callback)
            rospy.Subscriber(ns+'/perching_correction/thrust_command', PerchingThrustCommand, lambda m: record('thrust', list(m.thrust)))
            rospy.Subscriber(ns+'/pwm_test', PwmTest,
                lambda m: record('pwm', [list(m.motor_index),list(m.pwms)]))
            rospy.Subscriber(ns+'/independent_arm_pressure_controller/pressure', Float32MultiArray, lambda m: record('pressure', list(m.data)))
            rospy.Subscriber(ns+'/independent_arm_pressure_controller/bottom_pressure', Float32, lambda m: record('bottom', m.data))
            rospy.Subscriber(ns+'/pneumatic/grip_state', Float32MultiArray, lambda m: record('grip', list(m.data)))
            rospy.Subscriber(ns+'/pneumatic/actuator_state', Float32MultiArray, lambda m: record('actuator', list(m.data)))
            rospy.Subscriber(node+'/hybrid_locomotion', Float32MultiArray, lambda m: record('plan', list(m.data)))
            rospy.Subscriber(node+'/state', UInt8, lambda m: record('state', m.data))
            rospy.Subscriber(node+'/current_rpy', Vector3Stamped, lambda m: record('rpy', [m.vector.x,m.vector.y,m.vector.z]))
            rospy.Subscriber(node+'/arm_bend_angle_deg', Float32MultiArray,
                             lambda m: record('bend', list(m.data)))
            rospy.Subscriber(node+'/recovery_circle_geometry', Float32MultiArray,
                             lambda m: record('circle', list(m.data)))
            rospy.Subscriber('/rosout', Log, lambda m: record('logs', m.msg) if m.name.endswith('attitude_pressure_controller') else None)
            launch_args=[ 'simulation:=true', 'real_machine:=false',
                'mujoco:=true', 'headless:=true', 'fixed_arms:=false', 'spawn_human:=true',
                'start_attitude_allocator:='+str(not args.grasp_only).lower(),
                'start_pressure_controller:=true', 'inchworm_action_c:=true',
                'mujoco_initial_arm_pressure:='+str(args.initial_pressure),
                'mujoco_initial_bottom_pressure:=0.0',
                'mujoco_forearm_half_length:='+str(.5*args.forearm_length),
                'mujoco_forearm_proximal_width:='+str(args.forearm_proximal_width),
                'mujoco_forearm_proximal_depth:='+str(args.forearm_proximal_depth),
                'mujoco_forearm_distal_width:='+str(args.forearm_distal_width),
                'mujoco_forearm_distal_depth:='+str(args.forearm_distal_depth),
                'mujoco_cylinder_pitch:='+str(math.radians(args.cylinder_pitch_deg)),
                'mujoco_grip_friction:='+str(args.grip_friction),
                'mujoco_surface_friction:='+str(args.surface_friction),
                'mujoco_tangential_stiffness:='+str(args.grip_stiffness),
                'hybrid_locomotion_initial_bottom_kpa:='+str(args.bottom),
                'hybrid_locomotion_initial_m:='+str(args.mix),
                'hybrid_rear_low_kpa:='+str(args.rear_pressure),
                'hybrid_lift_pressure_gain_kpa_s_rad:='+str(args.lift_pressure_gain),
                'hybrid_slip_min_progress_m:='+str(args.slip_min_progress),
                'hybrid_slip_probe_max_thrust_n:='+str(args.slip_thrust_max),
                'hybrid_pull_min_sec:='+str(args.pull_min_sec),
                'hybrid_lift_search_rate_n_s:='+str(args.lift_rate),
                'hybrid_lift_probe_sec:='+str(args.lift_seconds),
                'hybrid_probe_extension_rad:='+str(math.radians(args.probe_extension_deg)),
                'hybrid_lift_identification_bend_rad:='+str(math.radians(args.identification_bend_deg)),
                'hybrid_lift_identification_extension_limit_rad:='+str(math.radians(args.identification_extension_limit_deg)),
                'hybrid_lift_unload_min_extension_rad:='+str(math.radians(args.unload_min_extension_deg)),
                'hybrid_lift_unload_pitch_rad:='+str(math.radians(args.unload_pitch_deg)),
                'hybrid_lift_unload_height_m:='+str(.001*args.unload_height_mm),
                'hybrid_lift_unload_confirm_sec:='+str(args.unload_confirm_sec),
                'hybrid_lift_unload_prefill_kpa:='+str(args.unload_prefill_kpa),
                'hybrid_trajectory_min_extension_rad:='+str(math.radians(args.trajectory_min_extension_deg)),
                'hybrid_trajectory_extension_rate_rad_s:='+str(args.trajectory_extension_rate),
                'hybrid_trajectory_windup_pitch_rad:='+str(math.radians(args.windup_pitch_deg)),
                'hybrid_trajectory_windup_pitch_kp_n_rad:='+str(args.windup_pitch_kp),
                'hybrid_trajectory_windup_pitch_kd_n_s_rad:='+str(args.windup_pitch_kd),
                'hybrid_trajectory_windup_bottom_kpa:='+str(args.windup_bottom_kpa),
                'hybrid_trajectory_windup_rear_reverse_max_n:='+str(args.windup_rear_reverse),
                'hybrid_locomotion_pitch_change_limit_rad:='+str(math.radians(20.)),
                'hybrid_trajectory_flex_rate_rad_s:='+str(args.trajectory_flex_rate),
                'hybrid_trajectory_pressure_lead_max_sec:='+str(args.trajectory_pressure_lead),
                'maximum_pressure_kpa:='+str(args.pressure_limit),
                'hybrid_locomotion_retain_grasp_pressure:='+str(args.retain_grasp).lower(),
                'hybrid_locomotion_regrip_timeout_sec:=45.0',
                'recovery_predictive_shadow:='+str(not args.predictive_recovery).lower(),
                'mujoco_model:='+str(out/'robot.xml')]
            launch=ET.Element('launch')
            include=ET.SubElement(launch,'include',file='$(find hugmy)/launch/bringup.launch')
            for argument in launch_args:
                name,value=argument.split(':=',1)
                ET.SubElement(include,'arg',name=name,value=value)
            if args.contact_z is not None:
                ET.SubElement(launch,'rosparam',param=node+'/hybrid_contact_position_main_body_m').text='[0.0, 0.0, %s]' % args.contact_z
                if args.predictive_recovery:
                    ET.SubElement(launch,'rosparam',param=node+'/contact_position_main_body_m').text='[0.0, 0.0, %s]' % args.contact_z
            for name in ('hybrid_locomotion_extension_limit_rad','action_c_max_extension_rad'):
                ET.SubElement(launch,'param',name=node+'/'+name,value=str(math.radians(args.extension_limit_deg)))
            if args.recovery_only:
                # This is a commanded recovery experiment, not the ordinary
                # gait's transient-roll detector.  Lower only this isolated
                # launch's entry thresholds so a static target step is an
                # explicit recovery request. Production parameters and the
                # +/-3-deg completion band remain unchanged.
                ET.SubElement(launch,'param',name=node+'/correction_trigger_angle_rad',
                              value=str(math.radians(4.0)))
                ET.SubElement(launch,'param',name=node+'/action_c_posture_recovery_trigger_rad',
                              value=str(math.radians(4.0)))
                ET.SubElement(launch,'param',name=node+'/action_c_posture_hard_roll_rad',
                              value=str(math.radians(5.0)))
            ET.SubElement(launch,'param',name=node+'/hybrid_slip_seed_pull_min_n',
                          value=str(args.pull_seed_min))
            ET.SubElement(launch,'param',name=node+'/hybrid_slip_seed_pull_max_n',
                          value=str(args.pull_seed_max))
            if args.body_z is not None:
                ET.SubElement(launch,'param',name=ns+'/simulation/pneumatic/perching_initial_clearance',value='0')
            ET.ElementTree(launch).write(str(out/'validation.launch'))
            processes.append(subprocess.Popen(['roslaunch',str(out/'validation.launch')], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            pressure_enable_service=ns+'/independent_arm_pressure_controller/enable'
            controller_enable_service=node+'/enable'
            # A recovery-only case must establish the mechanical grasp without
            # letting the attitude controller correct its natural fixture-
            # dependent equilibrium before the commanded target step.
            enable_service=(pressure_enable_service
                if args.grasp_only or args.recovery_only else
                controller_enable_service)
            rospy.wait_for_service(enable_service, timeout=60.)
            if args.recovery_only:
                rospy.wait_for_service(controller_enable_service, timeout=60.)
            target = rospy.Publisher(node+'/target_attitude', Vector3Stamped, queue_size=1, latch=True)
            move = rospy.Publisher(node+'/inchworm_command', Vector3Stamped, queue_size=1)
            deadline = time.monotonic()+30.
            while time.monotonic()<deadline and (not samples['pose'] or not samples['grip'] or
                    (not args.grasp_only and not args.recovery_only and
                     move.get_num_connections()==0)):
                time.sleep(.1)
            if not samples['pose'] or not samples['grip']:
                raise RuntimeError('MuJoCo pose/grip telemetry missing')
            target.publish(Vector3Stamped())
            grasp_target=rospy.Publisher(ns+'/independent_arm_pressure_controller/target_pressure', Float32MultiArray, queue_size=1, latch=True)
            if args.grasp_only or args.recovery_only:
                grasp_target.publish(Float32MultiArray(data=[30.]*4))
            deadline=time.monotonic()+15.
            response = rospy.ServiceProxy(enable_service, SetBool)(True)
            while not response.success and time.monotonic()<deadline:
                time.sleep(.2)
                response=rospy.ServiceProxy(enable_service, SetBool)(True)
            if not response.success:
                raise RuntimeError(response.message)
            # Let the real pressure controller establish a grasp before move.
            start = rospy.Time.now().to_sec()
            deadline=time.monotonic()+120.
            while time.monotonic()<deadline:
                if (rospy.Time.now().to_sec()-start>=3. and samples['pressure'] and
                    all(abs(p-30.)<=2. for p in samples['pressure'][-1][1])):
                    break
                time.sleep(.1)
            else:
                raise RuntimeError('MuJoCo grasp did not reach 30 +/- 2 kPa')
            if args.recovery_only:
                # Define the current passive grasp as the zero-error boundary,
                # then enable posture control.  The following target step is
                # therefore the only recovery event in the recorded case.
                passive_roll=(samples['rpy'][-1][1][0] if samples['rpy'] else
                              samples['physical_roll'][-1][1])
                hold_command=Vector3Stamped()
                hold_command.header.stamp=rospy.Time.now()
                hold_command.vector.x=passive_roll
                target.publish(hold_command)
                response=rospy.ServiceProxy(controller_enable_service, SetBool)(True)
                if not response.success:
                    raise RuntimeError(response.message)
                hold_start=rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec()-hold_start<.5:
                    time.sleep(.05)
            start_x = samples['pose'][-1][1][0]
            initial_window_end=rospy.Time.now().to_sec()
            initial_x_samples=[p[0] for t,p in samples['pose'] if t>=initial_window_end-.8]
            initial_x_mean=sum(initial_x_samples)/len(initial_x_samples)
            initial_control_roll=(samples['rpy'][-1][1][0] if samples['rpy'] else
                                  samples['physical_roll'][-1][1])
            command = Vector3Stamped(); command.vector.x=1.; command.vector.z=args.distance
            recovery_target_roll=(initial_control_roll+
                math.radians(args.target_roll_offset_deg)
                if args.target_roll_offset_deg is not None else
                math.radians(args.target_roll_deg))
            command_start=rospy.Time.now().to_sec()
            if args.recovery_only:
                target_command=Vector3Stamped()
                target_command.header.stamp=rospy.Time.from_sec(command_start)
                target_command.vector.x=recovery_target_roll
                target.publish(target_command)
            elif not args.grasp_only:
                move.publish(command)
            initial_pose=list(samples['pose'][-1][1])
            initial_roll=samples['physical_roll'][-1][1]
            initial_pitch=(samples['rpy'][-1][1][1] if samples['rpy'] else 0.)
            # A target callback may enter PREPARE in the same simulator tick.
            # Anchor recovery traces before publishing the step so that a
            # short fault/success is not lost from the state/log history.
            start = command_start if args.recovery_only else rospy.Time.now().to_sec()
            deadline = time.monotonic()+args.seconds*8.+60.
            last_report=-1.
            recovery_settle_start=None
            recovery_settle_time=None
            while rospy.Time.now().to_sec()-start<args.seconds and time.monotonic()<deadline:
                elapsed=rospy.Time.now().to_sec()-start
                if elapsed-last_report>=5.:
                    last_report=elapsed
                    latest=samples['plan'][-1][1] if samples['plan'] else []
                    report_roll=(samples['rpy'][-1][1][0] if samples['rpy'] else
                                 samples['physical_roll'][-1][1])
                    print('t=%.1f dx=%.2f mm roll=%.1f err=%.1f T=%s P=%s state=%s plan=%s' % (elapsed,1000*(samples['pose'][-1][1][0]-start_x),
                        report_roll*180/math.pi,
                        (math.atan2(math.sin(recovery_target_roll-report_roll),
                                    math.cos(recovery_target_roll-report_roll))*180/math.pi
                         if args.recovery_only else 0.0),
                        [round(v,2) for v in samples['actuator'][-1][1][4:8]],
                        [round(v,1) for v in samples['pressure'][-1][1]],
                        samples['state'][-1][1] if samples['state'] else None, latest[:5]), flush=True)
                if (abs(samples['physical_roll'][-1][1])>math.radians(40.) or
                    initial_pose[2]-samples['pose'][-1][1][2]>.05):
                    print('Physical fall detected; ending the failed trial and shutting down this isolated simulator.',flush=True)
                    break
                if args.recovery_only:
                    control_roll=(samples['rpy'][-1][1][0] if samples['rpy'] else
                                  samples['physical_roll'][-1][1])
                    error=abs(math.atan2(math.sin(recovery_target_roll-control_roll),
                                         math.cos(recovery_target_roll-control_roll)))
                    thrust_off=(samples['actuator'] and
                        max(abs(v) for v in samples['actuator'][-1][1][4:8])<.05)
                    regrasped=contact_regrasped(
                        samples['pressure'][-1][1] if samples['pressure'] else [],
                        samples['grip'][-1][1] if samples['grip'] else [])
                    in_grasp=bool(samples['state'] and samples['state'][-1][1]==0)
                    recovery_seen=(any(v in (1,2,3) for t,v in samples['state'] if t>=start) or
                        any(t>=start and 'Posture recovery accepted:' in message
                            for t,message in samples['logs']))
                    recovery_failed=next((t for t,message in samples['logs']
                        if t>=start and
                        'Inchworm stopped and returned directly to GRASP:' in message),None)
                    settled=recovery_seen and error<=math.radians(args.recovery_tolerance_deg) and \
                        thrust_off and regrasped and in_grasp
                    if settled:
                        if recovery_settle_start is None:
                            recovery_settle_start=rospy.Time.now().to_sec()
                        elif rospy.Time.now().to_sec()-recovery_settle_start>=1.0:
                            recovery_settle_time=rospy.Time.now().to_sec()-start
                            break
                    else:
                        recovery_settle_start=None
                    if (recovery_failed is not None and
                            rospy.Time.now().to_sec()-recovery_failed>=1.0 and
                            thrust_off and regrasped and in_grasp):
                        break
                    time.sleep(.1)
                    continue
                terminal=next((t for t,msg in samples['logs'] if t>=start and
                    'Inchworm stopped and returned directly to GRASP:' in msg),None)
                terminal_actuation_off=(samples['actuator'] and
                    max(abs(v) for v in samples['actuator'][-1][1][4:8])<.05)
                terminal_regrasped=(samples['pressure'] and
                    all(p>=25. for p in samples['pressure'][-1][1]))
                terminal_roll_settled=(samples['physical_roll'] and
                    abs(samples['physical_roll'][-1][1])<math.radians(8.))
                if (terminal is not None and
                    rospy.Time.now().to_sec()-terminal>=5. and
                    terminal_actuation_off and terminal_regrasped and
                    terminal_roll_settled):
                    # A locomotion endpoint may immediately request posture
                    # recovery.  Do not terminate the simulator merely
                    # because the gait emitted its GRASP message; wait for the
                    # real recovery/regrasp boundary as well.
                    break
                time.sleep(.1)
            result={'start_x':start_x, 'end_x':samples['pose'][-1][1][0],
                    'dx_mm':1000*(samples['pose'][-1][1][0]-start_x), 'sim_seconds':rospy.Time.now().to_sec()-start,
                    'max_applied_thrust':max((max(abs(t) for t in v[4:8]) for _,v in samples['actuator']), default=0.),
                    'powered_phase_seen':any(v==16 for _,v in samples['state'])}
            final_pose=samples['pose'][-1][1]
            result.update(body_z_drop=initial_pose[2]-final_pose[2],
                          lateral_displacement=final_pose[1]-initial_pose[1],
                          final_grips=samples['grip'][-1][1][:4],
                          goal_confirmed=any('hybrid target reached' in message for _,message in samples['logs']))
            result['max_roll_change_deg']=max((abs(math.atan2(math.sin(r-initial_roll),math.cos(r-initial_roll)))
                for t,r in samples['physical_roll'] if t>=start),default=0.)*180/math.pi
            result['max_pitch_change_deg']=max((abs(math.atan2(
                math.sin(v[1]-initial_pitch),math.cos(v[1]-initial_pitch)))
                for t,v in samples['rpy'] if t>=start),default=0.)*180/math.pi
            result['success']=bool(result['powered_phase_seen'] and result['goal_confirmed'] and
                                   result['body_z_drop']<.04 and sum(result['final_grips'])>=1)
            # Reaching x while falling or retaining thrust is not a completed
            # stride. Require the final settled grasp to retain the progress.
            end_time=rospy.Time.now().to_sec()
            tail=[p for t,p in samples['pose'] if t>=end_time-3.]
            tail_span=max(p[0] for p in tail)-min(p[0] for p in tail)
            end_x_mean,mean_span,tail_stable=settled_x_summary(samples['pose'],end_time)
            final_roll=samples['physical_roll'][-1][1]*180/math.pi
            final_thrust=samples['actuator'][-1][1][4:8]
            result.update(final_roll_deg=final_roll,final_thrust=final_thrust,tail_x_span_mm=1000*tail_span)
            result['final_bottom_kpa']=samples['bottom'][-1][1]
            # With continuous inflated support and no requested PPG dwell, the
            # production controller deliberately ends at the same 30-kPa
            # four-arm boundary used between strokes.  The legacy bottom-zero
            # endpoint remains valid when a biosignal dwell is requested.
            inflated_terminal=abs(result['final_bottom_kpa']-args.bottom)<=3.
            empty_terminal=abs(result['final_bottom_kpa'])<=3.
            terminal_roll_limit=15. if inflated_terminal else 8.
            peak_roll_limit=20. if inflated_terminal else 15.
            # Diagnostic schema 3 is the current C/D single-pass controller;
            # retain schema 2 so archived validation traces remain readable.
            is_hybrid_plan=lambda v: len(v)>=26 and int(v[0]) in (2,3)
            inflated_start=next((v[22] for _,v in samples['plan'] if is_hybrid_plan(v) and v[23]>0.5),initial_x_mean)
            result.update(preinflation_mean_dx_mm=1000*(end_x_mean-initial_x_mean),
                          retained_mean_dx_mm=1000*(end_x_mean-inflated_start),tail_mean_span_mm=1000*mean_span,tail_stable=tail_stable)
            # When both ends use the same inflated support state, the command-
            # start mean is already comparable with the final mean.  The old
            # `inflated_start` subtraction was needed only for a 0 -> 30 kPa
            # preparation and under-reported a run which started inflated.
            result['validation_progress_mm']=(result['preinflation_mean_dx_mm']
                if inflated_terminal else result['retained_mean_dx_mm'])
            result['probe_counts']=([max(v[3] for _,v in samples['plan'] if is_hybrid_plan(v)),
                                     max(v[4] for _,v in samples['plan'] if is_hybrid_plan(v))]
                                    if any(is_hybrid_plan(v) for _,v in samples['plan']) else None)
            adaptation=[v for _,v in samples['plan'] if len(v)>=34 and int(v[0]) in (2,3)]
            result['extension_onset_samples']=[max(v[26+j] for v in adaptation) for j in range(2)] if adaptation else None
            result['fast_approach_samples']=[sum(v[32+j]>.5 for v in adaptation) for j in range(2)] if adaptation else None
            result['front_vent_seen']=any(v[1]==1 and v[2]==6 for v in adaptation)
            result['lift_calibrated']=any(len(v)>=42 and v[34]>.5 for v in adaptation)
            result['trajectory_candidates']=sorted(set(int(v[38]) for v in adaptation if len(v)>=42 and v[34]>.5))
            # Schema 3 fields 89--93 record the actual bounded candidate and
            # instantaneous rear assist.  Retaining these values makes it
            # possible to verify that a run really explored unloading depth
            # or reverse assist instead of relying on an opaque candidate id.
            result['trajectory_settings']=sorted(set(
                (int(v[38]),round(v[89],3),round(v[90],3),round(v[91],3),round(v[92],3))
                for v in adaptation if len(v)>=94 and v[34]>.5))
            result['max_rear_reverse_thrust_n']=max(
                (v[93] for v in adaptation if len(v)>=94),default=0.)
            # The Motion-3 rear positive handoff pulse is appended at schema
            # index 128 so all established diagnostic indices remain stable.
            result['max_rear_positive_thrust_n']=max(
                (v[128] for v in adaptation if len(v)>=129),default=0.)
            result['rear_positive_started']=any(
                v[132]>.5 for v in adaptation if len(v)>=133)
            result['paired_extension_memory_seen']=[any(
                v[133+j]>0. for v in adaptation if len(v)>=137)
                for j in range(2)]
            result['max_unload_pair_mismatch_deg']=max(
                (math.degrees(v[135]) for v in adaptation if len(v)>=137),
                default=0.)
            # Confirm the command after the N->PWM conversion actually sent to
            # Spinal. Positive thrust for MotorInfo_3D_V is below the 0.75
            # neutral duty. Correlate PWM samples with the diagnostic rear-
            # positive interval rather than counting front-only commands.
            rear_positive_plan=[(t,v[128]) for t,v in samples['plan']
                                if len(v)>=129]
            rear_positive_pwm=[]
            plan_index=0
            for pwm_time,pwm_message in samples['pwm']:
                while (plan_index+1<len(rear_positive_plan) and
                       rear_positive_plan[plan_index+1][0]<=pwm_time):
                    plan_index+=1
                candidates=rear_positive_plan[plan_index:plan_index+2]
                if not candidates:
                    continue
                plan_time,rear_command=min(candidates,
                    key=lambda sample:abs(sample[0]-pwm_time))
                if rear_command<=.01 or abs(plan_time-pwm_time)>.05:
                    continue
                pwm_by_motor=dict(zip(pwm_message[0],pwm_message[1]))
                if 1 in pwm_by_motor and 2 in pwm_by_motor:
                    rear_positive_pwm.append(
                        [pwm_by_motor[1],pwm_by_motor[2]])
            result['rear_positive_pwm_confirmed']=any(
                pair[0]<.75-1e-6 and pair[1]<.75-1e-6
                for pair in rear_positive_pwm)
            result['min_rear_positive_pwm']=(
                [min(pair[j] for pair in rear_positive_pwm) for j in range(2)]
                if rear_positive_pwm else None)
            result['trajectory_observations']=sum('Hybrid measured trajectory optimization:' in m for _,m in samples['logs'])
            result['unloading_events']=sum('Hybrid measured unloading event:' in m for _,m in samples['logs'])
            result['load_relief_events']=sum('Hybrid load relief detected:' in m for _,m in samples['logs'])
            result['motion2_to_3_seen']=any(
                'Hybrid single-pass trajectory phase 2 -> 3' in message
                for _,message in samples['logs'])
            result['windup_event_pitch_deg']=[float(match.group(1)) for _,message in samples['logs']
                for match in [re.search(r'Hybrid measured unloading event:.*pitch ([-+0-9.]+) deg',message)]
                if match]
            result['completed_cycles']=sum('Hybrid C/D result: completed=1 safe=1' in msg for _,msg in samples['logs'])
            result['cycle_retained_mm'],result['consecutive_forward_cycles']=forward_cycle_summary(samples['logs'])
            slip=[t for t,v in samples['plan'] if is_hybrid_plan(v) and v[1]==1 and v[2]==0]
            slip_tail=[p[0] for t,p in samples['pose'] if slip and slip[-1]-.5<=t<=slip[-1]]
            result['initial_slip_dx_mm']=1000*(sum(slip_tail)/len(slip_tail)-inflated_start) if slip_tail else None
            result['slide_hold_seen']=any(is_hybrid_plan(v) and v[1]==1 and v[2]==1 for _,v in samples['plan'])
            uniform_ellipse=(abs(args.forearm_proximal_width-args.forearm_distal_width)<1e-9 and
                abs(args.forearm_proximal_depth-args.forearm_distal_depth)<1e-9)
            result['fixture']={'shape':('uniform_elliptic_forearm' if uniform_ellipse else
                                        'tapered_elliptic_forearm'),
                'length_m':args.forearm_length,
                'proximal_width_m':args.forearm_proximal_width,
                'proximal_depth_m':args.forearm_proximal_depth,
                'distal_width_m':args.forearm_distal_width,
                'distal_depth_m':args.forearm_distal_depth,
                'pitch_deg':args.cylinder_pitch_deg,'grip_mu':args.grip_friction,
                'surface_mu':args.surface_friction,'tangential_stiffness_n_m':args.grip_stiffness,
                'lift_pressure_gain_kpa_s_rad':args.lift_pressure_gain,
                'pull_min_sec':args.pull_min_sec,'lift_rate_n_s':args.lift_rate,
                'identification_extension_limit_deg':args.identification_extension_limit_deg,
                'lift_seconds':args.lift_seconds,
                'probe_extension_deg':args.probe_extension_deg,
                'unload_min_extension_deg':args.unload_min_extension_deg,
                'unload_pitch_deg':args.unload_pitch_deg,
                'unload_height_mm':args.unload_height_mm,
                'unload_confirm_sec':args.unload_confirm_sec,
                'unload_prefill_kpa':args.unload_prefill_kpa,
                'trajectory_min_extension_deg':args.trajectory_min_extension_deg,
                'trajectory_extension_rate_rad_s':args.trajectory_extension_rate,
                'windup_pitch_deg':args.windup_pitch_deg,
                'windup_pitch_kp':args.windup_pitch_kp,
                'windup_pitch_kd':args.windup_pitch_kd,
                'windup_bottom_kpa':args.windup_bottom_kpa,
                'windup_rear_reverse_n':args.windup_rear_reverse,
                'trajectory_flex_rate_rad_s':args.trajectory_flex_rate,
                'trajectory_pressure_lead_max_sec':args.trajectory_pressure_lead,
                'lateral_limit_m':args.lateral_limit}
            result['success']=bool(result['success'] and result['validation_progress_mm']>=1000*args.distance-2. and
                abs(result['body_z_drop'])<.02 and abs(result['lateral_displacement'])<args.lateral_limit and
                sum(result['final_grips'])>=3 and abs(final_roll)<terminal_roll_limit and
                result['max_roll_change_deg']<peak_roll_limit and
                max(abs(t) for t in final_thrust)<.05 and all(p>=25. for p in samples['pressure'][-1][1]) and
                (inflated_terminal or empty_terminal) and tail_stable)
            result['success']=bool(result['success'] and result['completed_cycles']>=args.require_cycles and
                result['consecutive_forward_cycles']>=args.require_forward_cycles and
                result['unloading_events']>=args.require_unload_events and
                result['trajectory_observations']>=args.require_optimizer_observations and
                (not args.require_no_slip or (result['initial_slip_dx_mm'] is not None and
                 abs(result['initial_slip_dx_mm'])<=2. and not result['slide_hold_seen'])))
            if args.grasp_only:
                result['success']=bool(abs(result['body_z_drop'])<.02 and
                    abs(result['lateral_displacement'])<.02 and abs(result['dx_mm'])<10. and
                    result['max_roll_change_deg']<5. and sum(result['final_grips'])>=3)
            if args.recovery_only:
                roll_trace=[(t,v[0]) for t,v in samples['rpy'] if t>=start]
                if not roll_trace:
                    roll_trace=[(t,r) for t,r in samples['physical_roll'] if t>=start]
                final_control_roll=roll_trace[-1][1]
                final_error=math.atan2(
                    math.sin(recovery_target_roll-final_control_roll),
                    math.cos(recovery_target_roll-final_control_roll))
                direction=1. if recovery_target_roll>=initial_control_roll else -1.
                overshoot=max((direction*(roll-recovery_target_roll)
                               for _,roll in roll_trace), default=0.)
                active_states=[(t,v) for t,v in samples['state']
                               if t>=start and v in (1,2,3)]
                accepted_times=[t for t,message in samples['logs']
                                if t>=start and 'Posture recovery accepted:' in message]
                stop_messages=[message for t,message in samples['logs']
                               if t>=start and
                               'Inchworm stopped and returned directly to GRASP:' in message]
                post_actuator=[v[4:8] for t,v in samples['actuator'] if t>=start]
                final_thrust_off=bool(samples['actuator'] and
                    max(abs(v) for v in samples['actuator'][-1][1][4:8])<.05)
                final_regrasped=contact_regrasped(
                    samples['pressure'][-1][1] if samples['pressure'] else [],
                    samples['grip'][-1][1] if samples['grip'] else [])
                final_in_grasp=bool(samples['state'] and
                    samples['state'][-1][1]==0)
                circle=[v for t,v in samples['circle']
                        if t>=start and len(v)>=14 and v[0]>.5 and
                        math.isfinite(v[1])]
                radii=sorted(v[1] for v in circle)
                estimated_radius=(radii[len(radii)//2] if radii else None)
                pair_sequence=[]
                for value in circle:
                    selection=(int(round(value[12])), bool(value[13]>.5))
                    if selection[0]>=0 and (not pair_sequence or
                                            pair_sequence[-1]!=selection):
                        pair_sequence.append(selection)
                result.update(
                    mode='recovery_only',
                    target_roll_deg=recovery_target_roll*180./math.pi,
                    target_roll_offset_deg=(
                        recovery_target_roll-initial_control_roll)*180./math.pi,
                    target_mode=('relative_to_initial' if
                                 args.target_roll_offset_deg is not None else
                                 'absolute'),
                    initial_control_roll_deg=initial_control_roll*180./math.pi,
                    final_control_roll_deg=final_control_roll*180./math.pi,
                    final_error_deg=final_error*180./math.pi,
                    peak_overshoot_deg=max(0.,overshoot*180./math.pi),
                    settling_time_sec=recovery_settle_time,
                    recovery_started=bool(active_states or accepted_times),
                    recovery_start_sec=(min(
                        ([t for t,_ in active_states]+accepted_times))-start
                        if active_states or accepted_times else None),
                    recovery_states=sorted(set(v for _,v in active_states)),
                    recovery_stop_reason=(stop_messages[-1] if stop_messages else None),
                    peak_recovery_thrust_n=max(
                        (max(abs(x) for x in v) for v in post_actuator),
                        default=0.),
                    final_thrust_off=final_thrust_off,
                    final_regrasped=final_regrasped,
                    final_in_grasp=final_in_grasp,
                    estimated_circle_radius_m=estimated_radius,
                    circle_valid_samples=len(circle),
                    recovery_pair_sequence=[
                        {'pair':('01' if pair==0 else '23'),
                         'mode':('released' if release else 'flexed')}
                        for pair,release in pair_sequence])
                result['success']=bool(
                    result['recovery_started'] and recovery_settle_time is not None and
                    abs(result['final_error_deg'])<=args.recovery_tolerance_deg and
                    final_thrust_off and final_regrasped and final_in_grasp and
                    result['body_z_drop']<.04 and
                    abs(result['final_roll_deg'])<40.)
            print(json.dumps(result), flush=True)
            (out/'result.json').write_text(json.dumps(result, indent=2))
        finally:
            (out/'samples.json').write_text(json.dumps(samples))
            # Ask every node on this run's isolated ROS master to shut down
            # before stopping roslaunch/roscore. mujoco_node daemonizes into
            # its own process group on this system, so killing only the
            # roslaunch group cannot reach it after it has been reparented.
            try:
                subprocess.run(['rosnode','kill','-a'], env=env,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=5., check=False)
            except (subprocess.TimeoutExpired, OSError):
                pass
            for p in reversed(processes):
                if p.poll() is None:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    try:
                        p.wait(timeout=10.)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        p.wait()
            # Clean up simulator/rosmaster children which daemonized out of
            # the roslaunch process group.  The exact output directory keeps
            # this isolated from other ROS sessions on the machine.
            terminate_run_processes(out)
    if not result['success']:
        raise SystemExit('FAIL: physical validation did not meet the retained-goal/grasp criteria; inspect '+str(out))


if __name__ == '__main__':
    main()
