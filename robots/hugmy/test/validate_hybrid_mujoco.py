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
import time
import xml.etree.ElementTree as ET


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
    parser.add_argument('--grip-stiffness', type=float, default=180., help='tangential N/m; 0 for legacy comparison')
    parser.add_argument('--grip-friction', type=float, default=1.2, help='pneumatic grip Coulomb coefficient')
    parser.add_argument('--lift-pressure-gain', type=float, default=20., help='deprecated compatibility option; early lift remains at zero front pressure, with bounded predictive prefill on an adverse slope')
    parser.add_argument('--pull-min-sec', type=float, default=.5, help='minimum powered pull before front catch, .5..4 s; hard guards unchanged')
    parser.add_argument('--lift-rate', type=float, default=.7, help='lift thrust search rate, (0,1] N/s')
    parser.add_argument('--lift-seconds', type=float, default=10., help='powered lift deadline after front venting, 1..10 s')
    parser.add_argument('--probe-extension-deg', type=float, default=8., help='desired extension, 3..15 degrees; protection remains separate')
    parser.add_argument('--identification-bend-deg', type=float, default=20.,
                        help='one-time motion=2 absolute bend target, 5..30 degrees')
    parser.add_argument('--cylinder-radius', type=float, default=.05, help='fixture radius in m, robot placement follows cylinder top')
    parser.add_argument('--cylinder-pitch-deg', type=float, default=0.,
                        help='fixture pitch in degrees for longitudinal-slope validation, -15..15')
    parser.add_argument('--surface-friction', type=float, default=2.5, help='MuJoCo cylinder sliding friction, torsional/rolling coefficients unchanged')
    parser.add_argument('--require-cycles', type=int, default=0, help='minimum completed and measured locomotion strokes')
    parser.add_argument('--require-forward-cycles', type=int, default=0, help='minimum consecutive settled strokes retaining at least 1 mm each')
    parser.add_argument('--require-no-slip', action='store_true', help='also require absolute initial sliding probe displacement <=2 mm and no sustained sliding')
    parser.add_argument('--grasp-only', action='store_true', help='negative control: no locomotion command')
    args = parser.parse_args()
    if args.mix != 1.:
        parser.error('--mix is no longer a commanded search variable; omit it for single-pass motion')
    if not (math.isfinite(args.seconds) and 0.<args.seconds<=300. and
            math.isfinite(args.distance) and 0.<args.distance<=.1 and
            math.isfinite(args.lateral_limit) and .01<=args.lateral_limit<=.1 and
            math.isfinite(args.bottom) and 0.<=args.bottom<=30. and
            math.isfinite(args.mix) and 0.<=args.mix<=1. and
            math.isfinite(args.extension_limit_deg) and 10.<=args.extension_limit_deg<=35. and
            math.isfinite(args.rear_pressure) and 0.<=args.rear_pressure<=6. and
            math.isfinite(args.grip_friction) and 0.<=args.grip_friction<=8. and
            math.isfinite(args.surface_friction) and 0.<=args.surface_friction<=8. and
            math.isfinite(args.cylinder_radius) and .025<=args.cylinder_radius<=.15 and
            math.isfinite(args.cylinder_pitch_deg) and -15.<=args.cylinder_pitch_deg<=15. and
            math.isfinite(args.grip_stiffness) and 0.<=args.grip_stiffness<=5000. and
            math.isfinite(args.lift_pressure_gain) and 0.<args.lift_pressure_gain<=150. and
            math.isfinite(args.pull_min_sec) and .5<=args.pull_min_sec<=4. and
            math.isfinite(args.lift_rate) and 0.<args.lift_rate<=1. and
            math.isfinite(args.lift_seconds) and 1.<=args.lift_seconds<=10. and
            math.isfinite(args.probe_extension_deg) and 3.<=args.probe_extension_deg<=15. and
            math.isfinite(args.identification_bend_deg) and 5.<=args.identification_bend_deg<=30. and
            0<=args.require_cycles<=10 and
            0<=args.require_forward_cycles<=10 and
            math.isfinite(args.pressure_limit) and 30.<=args.pressure_limit<=50.):
        parser.error('require seconds in (0,300], distance in (0,0.1] m, bottom 0..30, rear 0..6, arm limit 30..50 kPa')
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    # spawn_z belongs to Gazebo in bringup; make the MuJoCo initial placement
    # explicit without changing its shapes, masses, contacts or spring model.
    model_dir=Path(__file__).resolve().parents[1]/'mujoco/hugmy2'
    model=ET.parse(str(model_dir/'robot.xml'))
    if args.body_z is not None:
        model.find('./worldbody/body[@name="main_body"]').set('pos','0 0 '+str(args.body_z))
    for mesh in model.findall('./asset/mesh'):
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
    samples = {'pose': [], 'physical_roll': [], 'rpy': [], 'thrust': [], 'actuator': [], 'pressure': [], 'bottom': [], 'grip': [], 'plan': [], 'state': [], 'logs': []}
    with (out / 'launch.log').open('w') as log:
        try:
            processes.append(subprocess.Popen(['roscore', '-p', str(port)], env=env, stdout=log, stderr=subprocess.STDOUT))
            import rospy
            from geometry_msgs.msg import PoseStamped, Vector3Stamped
            from std_msgs.msg import Float32MultiArray, Float32, UInt8
            from spinal.msg import PerchingThrustCommand
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
            rospy.Subscriber(ns+'/independent_arm_pressure_controller/pressure', Float32MultiArray, lambda m: record('pressure', list(m.data)))
            rospy.Subscriber(ns+'/independent_arm_pressure_controller/bottom_pressure', Float32, lambda m: record('bottom', m.data))
            rospy.Subscriber(ns+'/pneumatic/grip_state', Float32MultiArray, lambda m: record('grip', list(m.data)))
            rospy.Subscriber(ns+'/pneumatic/actuator_state', Float32MultiArray, lambda m: record('actuator', list(m.data)))
            rospy.Subscriber(node+'/hybrid_locomotion', Float32MultiArray, lambda m: record('plan', list(m.data)))
            rospy.Subscriber(node+'/state', UInt8, lambda m: record('state', m.data))
            rospy.Subscriber(node+'/current_rpy', Vector3Stamped, lambda m: record('rpy', [m.vector.x,m.vector.y,m.vector.z]))
            rospy.Subscriber('/rosout', Log, lambda m: record('logs', m.msg) if m.name.endswith('attitude_pressure_controller') else None)
            launch_args=[ 'simulation:=true', 'real_machine:=false',
                'mujoco:=true', 'headless:=true', 'fixed_arms:=false', 'spawn_human:=true',
                'start_attitude_allocator:='+str(not args.grasp_only).lower(),
                'start_pressure_controller:=true', 'inchworm_action_c:=true',
                'mujoco_initial_arm_pressure:='+str(args.initial_pressure),
                'mujoco_initial_bottom_pressure:=0.0',
                'mujoco_cylinder_radius:='+str(args.cylinder_radius),
                'mujoco_cylinder_pitch:='+str(math.radians(args.cylinder_pitch_deg)),
                'mujoco_grip_friction:='+str(args.grip_friction),
                'mujoco_surface_friction:='+str(args.surface_friction),
                'mujoco_tangential_stiffness:='+str(args.grip_stiffness),
                'hybrid_locomotion_initial_bottom_kpa:='+str(args.bottom),
                'hybrid_locomotion_initial_m:='+str(args.mix),
                'hybrid_rear_low_kpa:='+str(args.rear_pressure),
                'hybrid_lift_pressure_gain_kpa_s_rad:='+str(args.lift_pressure_gain),
                'hybrid_pull_min_sec:='+str(args.pull_min_sec),
                'hybrid_lift_search_rate_n_s:='+str(args.lift_rate),
                'hybrid_lift_probe_sec:='+str(args.lift_seconds),
                'hybrid_probe_extension_rad:='+str(math.radians(args.probe_extension_deg)),
                'hybrid_lift_identification_bend_rad:='+str(math.radians(args.identification_bend_deg)),
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
            if args.body_z is not None:
                ET.SubElement(launch,'param',name=ns+'/simulation/pneumatic/perching_initial_clearance',value='0')
            ET.ElementTree(launch).write(str(out/'validation.launch'))
            processes.append(subprocess.Popen(['roslaunch',str(out/'validation.launch')], env=env, stdout=log, stderr=subprocess.STDOUT))
            enable_service=(ns+'/independent_arm_pressure_controller/enable' if args.grasp_only else node+'/enable')
            rospy.wait_for_service(enable_service, timeout=60.)
            target = rospy.Publisher(node+'/target_attitude', Vector3Stamped, queue_size=1, latch=True)
            move = rospy.Publisher(node+'/inchworm_command', Vector3Stamped, queue_size=1)
            deadline = time.monotonic()+30.
            while time.monotonic()<deadline and (not samples['pose'] or not samples['grip'] or (not args.grasp_only and move.get_num_connections()==0)):
                time.sleep(.1)
            if not samples['pose'] or not samples['grip']:
                raise RuntimeError('MuJoCo pose/grip telemetry missing')
            target.publish(Vector3Stamped())
            grasp_target=rospy.Publisher(ns+'/independent_arm_pressure_controller/target_pressure', Float32MultiArray, queue_size=1, latch=True)
            if args.grasp_only:
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
            start_x = samples['pose'][-1][1][0]
            initial_window_end=rospy.Time.now().to_sec()
            initial_x_samples=[p[0] for t,p in samples['pose'] if t>=initial_window_end-.8]
            initial_x_mean=sum(initial_x_samples)/len(initial_x_samples)
            command = Vector3Stamped(); command.vector.x=1.; command.vector.z=args.distance
            if not args.grasp_only:
                move.publish(command)
            initial_pose=list(samples['pose'][-1][1])
            initial_roll=samples['physical_roll'][-1][1]
            start = rospy.Time.now().to_sec()
            deadline = time.monotonic()+args.seconds*8.+60.
            last_report=-1.
            while rospy.Time.now().to_sec()-start<args.seconds and time.monotonic()<deadline:
                elapsed=rospy.Time.now().to_sec()-start
                if elapsed-last_report>=5.:
                    last_report=elapsed
                    latest=samples['plan'][-1][1] if samples['plan'] else []
                    print('t=%.1f dx=%.2f mm roll=%.1f T=%s P=%s state=%s plan=%s' % (elapsed,1000*(samples['pose'][-1][1][0]-start_x),
                        samples['physical_roll'][-1][1]*180/math.pi,
                        [round(v,2) for v in samples['actuator'][-1][1][4:8]],
                        [round(v,1) for v in samples['pressure'][-1][1]],
                        samples['state'][-1][1] if samples['state'] else None, latest[:5]), flush=True)
                if (abs(samples['physical_roll'][-1][1])>math.radians(40.) or
                    initial_pose[2]-samples['pose'][-1][1][2]>.05):
                    print('Physical fall detected; ending the failed trial and shutting down this isolated simulator.',flush=True)
                    break
                terminal=next((t for t,msg in samples['logs'] if t>=start and
                    'Inchworm stopped and returned directly to GRASP:' in msg),None)
                if terminal is not None and rospy.Time.now().to_sec()-terminal>=5.:
                    # Always include a real thrust-off settling interval, on
                    # both success and failure; do not burn the full time idle.
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
            result['max_roll_change_deg']=max(abs(math.atan2(math.sin(r-initial_roll),math.cos(r-initial_roll)))
                for t,r in samples['physical_roll'] if t>=start)*180/math.pi
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
            inflated_start=next((v[22] for _,v in samples['plan'] if len(v)>=26 and v[0]==2 and v[23]>0.5),initial_x_mean)
            result.update(preinflation_mean_dx_mm=1000*(end_x_mean-initial_x_mean),
                          retained_mean_dx_mm=1000*(end_x_mean-inflated_start),tail_mean_span_mm=1000*mean_span,tail_stable=tail_stable)
            result['probe_counts']=([max(v[3] for _,v in samples['plan'] if len(v)>=26 and v[0]==2),
                                     max(v[4] for _,v in samples['plan'] if len(v)>=26 and v[0]==2)]
                                    if any(len(v)>=26 and v[0]==2 for _,v in samples['plan']) else None)
            adaptation=[v for _,v in samples['plan'] if len(v)>=34 and v[0]==2]
            result['extension_onset_samples']=[max(v[26+j] for v in adaptation) for j in range(2)] if adaptation else None
            result['fast_approach_samples']=[sum(v[32+j]>.5 for v in adaptation) for j in range(2)] if adaptation else None
            result['front_vent_seen']=any(v[1]==1 and v[2]==6 for v in adaptation)
            result['lift_calibrated']=any(len(v)>=42 and v[34]>.5 for v in adaptation)
            result['trajectory_candidates']=sorted(set(int(v[38]) for v in adaptation if len(v)>=42 and v[34]>.5))
            result['trajectory_observations']=sum('Hybrid measured trajectory optimization:' in m for _,m in samples['logs'])
            result['completed_cycles']=sum('Hybrid C/D result: completed=1 safe=1' in msg for _,msg in samples['logs'])
            result['cycle_retained_mm'],result['consecutive_forward_cycles']=forward_cycle_summary(samples['logs'])
            slip=[t for t,v in samples['plan'] if len(v)>=26 and v[0]==2 and v[1]==1 and v[2]==0]
            slip_tail=[p[0] for t,p in samples['pose'] if slip and slip[-1]-.5<=t<=slip[-1]]
            result['initial_slip_dx_mm']=1000*(sum(slip_tail)/len(slip_tail)-inflated_start) if slip_tail else None
            result['slide_hold_seen']=any(len(v)>=26 and v[0]==2 and v[1]==1 and v[2]==1 for _,v in samples['plan'])
            result['fixture']={'radius_m':args.cylinder_radius,
                'pitch_deg':args.cylinder_pitch_deg,'grip_mu':args.grip_friction,
                'surface_mu':args.surface_friction,'tangential_stiffness_n_m':args.grip_stiffness,
                'lift_pressure_gain_kpa_s_rad':args.lift_pressure_gain,
                'pull_min_sec':args.pull_min_sec,'lift_rate_n_s':args.lift_rate,
                'lift_seconds':args.lift_seconds,
                'probe_extension_deg':args.probe_extension_deg,
                'lateral_limit_m':args.lateral_limit}
            result['success']=bool(result['success'] and result['retained_mean_dx_mm']>=1000*args.distance-2. and
                abs(result['body_z_drop'])<.02 and abs(result['lateral_displacement'])<args.lateral_limit and
                sum(result['final_grips'])>=3 and abs(final_roll)<8. and result['max_roll_change_deg']<15. and
                max(abs(t) for t in final_thrust)<.05 and all(p>=25. for p in samples['pressure'][-1][1]) and
                abs(samples['bottom'][-1][1]-args.bottom)<=3. and tail_stable)
            result['success']=bool(result['success'] and result['completed_cycles']>=args.require_cycles and
                result['consecutive_forward_cycles']>=args.require_forward_cycles and
                (not args.require_no_slip or (result['initial_slip_dx_mm'] is not None and
                 abs(result['initial_slip_dx_mm'])<=2. and not result['slide_hold_seen'])))
            if args.grasp_only:
                result['success']=bool(abs(result['body_z_drop'])<.02 and
                    abs(result['lateral_displacement'])<.02 and abs(result['dx_mm'])<10. and
                    result['max_roll_change_deg']<5. and sum(result['final_grips'])>=3)
            print(json.dumps(result), flush=True)
            (out/'result.json').write_text(json.dumps(result, indent=2))
        finally:
            (out/'samples.json').write_text(json.dumps(samples))
            for p in reversed(processes):
                if p.poll() is None:
                    p.send_signal(signal.SIGINT)
                    try:
                        p.wait(timeout=10.)
                    except subprocess.TimeoutExpired:
                        p.kill();p.wait()
    if not result['success']:
        raise SystemExit('FAIL: physical validation did not meet the retained-goal/grasp criteria; inspect '+str(out))


if __name__ == '__main__':
    main()
