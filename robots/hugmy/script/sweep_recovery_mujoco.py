#!/usr/bin/env python3
"""Sweep circular grasp diameters and roll targets in recovery-only MuJoCo runs.

Each case is launched on an isolated ROS master by validate_hybrid_mujoco.py.
The sweep intentionally continues after a failed case so the resulting CSV
describes the recovery controller's operating region rather than hiding it.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys


def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True,
                        help='new directory in which all case results are stored')
    parser.add_argument('--diameters-mm', type=float, nargs='+',
                        default=[70., 100., 130.],
                        help='circular fixture diameters (default: 70 100 130)')
    parser.add_argument('--roll-offset-deg', '--target-roll-deg',
                        dest='roll_offset_deg', type=float, nargs='+',
                        default=[-20., -12., 12., 20.],
                        help='target changes from each diameter-specific initial roll (default: -20 -12 12 20)')
    parser.add_argument('--seconds', type=float, default=60.,
                        help='maximum simulated seconds per case')
    parser.add_argument('--predictive-recovery', action='store_true',
                        help='run the existing predictive recovery in active rather than shadow mode')
    parser.add_argument('--surface-friction', type=float, default=4.0)
    parser.add_argument('--grip-friction', type=float, default=4.5)
    args = parser.parse_args()

    if (not args.diameters_mm or
            any(not math.isfinite(v) or v < 50. or v > 300.
                for v in args.diameters_mm)):
        parser.error('diameters must be finite and in [50, 300] mm')
    if (not args.roll_offset_deg or
            any(not math.isfinite(v) or abs(v) < 5. or abs(v) > 30.
                for v in args.roll_offset_deg)):
        parser.error('roll offsets must have magnitude in [5, 30] deg')
    if not math.isfinite(args.seconds) or not 10. <= args.seconds <= 300.:
        parser.error('--seconds must be in [10, 300]')

    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    validator = Path(__file__).resolve().parents[1] / 'test' / 'validate_hybrid_mujoco.py'
    if not validator.exists():
        raise RuntimeError('cannot find recovery validator: ' + str(validator))

    rows = []
    total = len(args.diameters_mm) * len(args.roll_offset_deg)
    index = 0
    for diameter in args.diameters_mm:
        for target in args.roll_offset_deg:
            index += 1
            sign = 'p' if target >= 0. else 'm'
            case_name = 'd%03d_roll_%s%02d' % (
                int(round(diameter)), sign, int(round(abs(target))))
            case_dir = output / case_name
            command = [
                sys.executable, str(validator),
                '--output', str(case_dir),
                '--recovery-only',
                '--cylinder-radius', str(diameter / 2000.),
                '--target-roll-offset-deg', str(target),
                '--seconds', str(args.seconds),
                '--surface-friction', str(args.surface_friction),
                '--grip-friction', str(args.grip_friction),
            ]
            if args.predictive_recovery:
                command.append('--predictive-recovery')
            print('[%d/%d] diameter %.1f mm, roll offset %+.1f deg' %
                  (index, total, diameter, target), flush=True)
            completed = subprocess.run(command, check=False)
            result_path = case_dir / 'result.json'
            if result_path.exists():
                result = json.loads(result_path.read_text())
            else:
                result = {'success': False}
            row = {
                'case': case_name,
                'diameter_mm': diameter,
                'roll_offset_deg': target,
                'target_roll_deg': finite_float(result.get('target_roll_deg')),
                'success': bool(result.get('success', False)),
                'return_code': completed.returncode,
                'initial_roll_deg': finite_float(result.get('initial_control_roll_deg')),
                'final_roll_deg': finite_float(result.get('final_control_roll_deg')),
                'final_error_deg': finite_float(result.get('final_error_deg')),
                'settling_time_sec': finite_float(result.get('settling_time_sec')),
                'peak_overshoot_deg': finite_float(result.get('peak_overshoot_deg')),
                'peak_thrust_n': finite_float(result.get('peak_recovery_thrust_n')),
                'estimated_diameter_mm': (2000. * result['estimated_circle_radius_m']
                                          if finite_float(result.get('estimated_circle_radius_m')) is not None
                                          else None),
                'pair_sequence': ','.join(
                    '%s:%s' % (v.get('pair', '?'), v.get('mode', '?'))
                    for v in result.get('recovery_pair_sequence', [])),
                'body_z_drop_mm': (1000. * finite_float(result.get('body_z_drop'))
                                   if finite_float(result.get('body_z_drop')) is not None
                                   else None),
            }
            rows.append(row)
            print('  success=%s, final error=%s deg, settle=%s s, pair=%s' %
                  (row['success'], row['final_error_deg'],
                   row['settling_time_sec'], row['pair_sequence']), flush=True)

    fields = list(rows[0].keys())
    with (output / 'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        'cases': rows,
        'success_count': sum(row['success'] for row in rows),
        'case_count': len(rows),
        'success_rate': sum(row['success'] for row in rows) / float(len(rows)),
        'settings': {
            'diameters_mm': args.diameters_mm,
            'roll_offset_deg': args.roll_offset_deg,
            'seconds': args.seconds,
            'predictive_recovery': args.predictive_recovery,
            'surface_friction': args.surface_friction,
            'grip_friction': args.grip_friction,
        },
    }
    (output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print('completed: %d/%d cases succeeded; summary: %s' %
          (summary['success_count'], summary['case_count'],
           output / 'summary.csv'), flush=True)


if __name__ == '__main__':
    main()
