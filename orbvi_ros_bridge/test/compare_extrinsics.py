#!/usr/bin/env python3
"""Independently compare a device calibration snapshot, export and ROS reports.

Test-only dependency: numpy. Device calibration stays outside the repository.
Raw rounded calibration matrices use a separately reported tolerance; broadcast
and export values must agree to 1e-12.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def matrix(rotation, translation):
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation).reshape(3, 3)
    out[:3, 3] = translation
    return out


def quaternion_matrix(q, t):
    x, y, z, w = q
    return matrix([
        1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
        2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
        2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)], t)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calibration', required=True)
    p.add_argument('--export', dest='export_file', required=True)
    p.add_argument('--ros-report', action='append', default=[])
    p.add_argument('--matrix-tolerance', type=float, default=2e-6)
    p.add_argument('--output', required=True)
    p.add_argument('--device-frame', default='orbvi_link')
    args = p.parse_args()
    calibration = json.loads(Path(args.calibration).read_text())
    exported = json.loads(Path(args.export_file).read_text())
    cameras = calibration['extrinsics']
    front = next(c for c in cameras if c['role'] == 'front')
    anchor = next(e for e in calibration['lidar_extrinsics'] if e['camera_id'] == front['camera_id'])
    imu = calibration['lidar_imu_extrinsic']
    axis = matrix([0,0,1,-1,0,0,0,-1,0], [0,0,0])
    front_rig = matrix(front['rotation_matrix'], front['translation_m'])
    front_lidar = matrix(anchor['rotation_matrix_3x3'], anchor['translation_m'])
    imu_lidar = matrix(imu['rotation_matrix'], imu['translation_m'])
    expected = {
        ('aft_mapped', 'mid360_imu'): np.eye(4),
        ('aft_mapped', 'livox_frame'): imu_lidar,
        ('aft_mapped', args.device_frame): imu_lidar @ np.linalg.inv(axis @ front_rig @ front_lidar),
    }
    for camera in cameras:
        expected[(args.device_frame, 'orbvi/camera/%d' % camera['camera_id'])] = axis @ matrix(camera['rotation_matrix'], camera['translation_m'])
    values = {(e['parent'], e['child']): e['translation_m']+e['quaternion_xyzw'] for e in exported['transforms']}
    errors = {}
    failures = []
    if set(values) != set(expected): failures.append('unexpected_export_edges')
    for edge in set(values) & set(expected):
        v = values[edge]
        error = float(np.max(np.abs(quaternion_matrix(v[3:], v[:3])-expected[edge])))
        errors['<-'.join(edge)] = error
        if error > args.matrix_tolerance: failures.append('raw_matrix_mismatch:' + edge[1])
    if exported['camera_calibration_hash'] != calibration['calibration_hash']: failures.append('camera_hash_mismatch')
    if exported['lidar_calibration_hash'] != calibration['lidar_calibration']['lidar_calibration_hash']: failures.append('lidar_hash_mismatch')
    broadcasts = {}
    for filename in args.ros_report:
        report = json.loads(Path(filename).read_text())
        actual = {(e['parent'], e['child']): e['values'] for e in report['static_transforms']}
        if set(actual) != set(values): failures.append('broadcast_edges_mismatch:' + filename)
        error = max((max(abs(a-b) for a,b in zip(actual[edge], values[edge])) for edge in actual.keys() & values.keys()), default=float('inf'))
        broadcasts[filename] = error
        if error > 1e-12: failures.append('broadcast_values_mismatch:' + filename)
    rotation = front_lidar[:3, :3]
    result = dict(passed=not failures, errors=failures, raw_matrix_errors=errors,
                  raw_matrix_tolerance=args.matrix_tolerance,
                  anchor_orthogonality_error=float(np.max(np.abs(rotation.T@rotation-np.eye(3)))),
                  export_broadcast_max_errors=broadcasts)
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
