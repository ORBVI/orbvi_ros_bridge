#!/usr/bin/env python3
"""Read-only board acceptance through an isolated, locally launched ROS bridge.

Run ROS1 with a dedicated ROS_MASTER_URI/master, ROS2 with a dedicated domain.
This tool never starts/stops board services. It writes a bounded JSON summary.
"""
import argparse
import collections
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ros', type=int, choices=(1, 2), required=True)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--control-port', type=int, default=18088)
    p.add_argument('--duration', type=float, default=30)
    p.add_argument('--mode', choices=('full', 'sensors', 'disabled', 'static-only', 'dynamic-only', 'no-tf-sensors'), default='full')
    p.add_argument('--bridge-executable', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--include-lidar', action='store_true')
    p.add_argument('--device-frame', default='orbvi_link')
    p.add_argument('--tf-mode', choices=('device', 'robot'), default='device')
    p.add_argument('--mounting-config')
    p.add_argument('--publish-mounting-tf', action='store_true')
    p.add_argument('--mounting-reference', help='Test-only installation YAML used by the simulated URDF publisher')
    p.add_argument('--external-mount-publisher', action='store_true',
                   help='Use an already running robot_state_publisher instead of simulating the installation edge')
    p.add_argument('--publish-odom-tf', action='store_true')
    p.add_argument('--mount-conflict', choices=('none','before','after'), default='none')
    p.add_argument('--delay-mount-seconds', type=float, default=0.0)
    p.add_argument('--robot-reference', help='Device-mode extrinsics JSON for independent robot pose comparison')
    p.add_argument('--include-ipm', action='store_true')
    p.add_argument('--include-depth', action='store_true',
                   help='Also require ROS_BODY depth clouds to use the device frame')
    p.add_argument('--sensor-reader-best-effort', action='store_true',
                   help='ROS2 sensor subscriber QoS for comparing older bridge builds')
    args = p.parse_args()
    if args.include_depth and args.mode == "sensors":
        p.error("--include-depth requires a mode with image/disparity streams")
    dynamic = args.mode not in ('sensors', 'disabled', 'static-only', 'no-tf-sensors')
    static = args.mode not in ('disabled', 'dynamic-only', 'no-tf-sensors')
    if args.publish_odom_tf and (args.tf_mode != 'robot' or args.mode != 'full'):
        p.error('--publish-odom-tf requires robot/full mode')
    if args.mount_conflict != 'none' and (not args.publish_mounting_tf or args.tf_mode != 'robot' or args.mode != 'full'):
        p.error('mount conflicts require robot/full mode with bridge ownership')
    if args.external_mount_publisher and (args.tf_mode != 'robot' or args.publish_mounting_tf or not static or args.delay_mount_seconds):
        p.error('external publisher requires static-enabled URDF mode without simulated delay')
    mounting = None
    correction = None
    if args.tf_mode == 'robot':
        import yaml
        import numpy as np
        if not args.mounting_config: p.error('robot mode requires --mounting-config')
        mounting = yaml.safe_load(Path(args.mounting_config).read_text())['orbvi']['mounting']
        if not args.publish_mounting_tf:
            if not args.mounting_reference: p.error('URDF tests require --mounting-reference; production config must have no xyz/rpy')
            reference_mount = yaml.safe_load(Path(args.mounting_reference).read_text())['orbvi']['mounting']
            if any(mounting[k] != reference_mount[k] for k in ('parent_frame','child_frame')):
                p.error('reference and URDF frame names differ')
            if 'xyz' in mounting or 'rpy' in mounting: p.error('URDF config must not contain xyz/rpy')
            mounting.update(xyz=reference_mount['xyz'],rpy=reference_mount['rpy'])
        args.device_frame = mounting['child_frame']
        roll,pitch,yaw = mounting['rpy']
        sr,cr,sp,cp,sy,cy = math.sin(roll),math.cos(roll),math.sin(pitch),math.cos(pitch),math.sin(yaw),math.cos(yaw)
        mount_matrix = np.eye(4)
        mount_matrix[:3,:3] = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                               [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
        mount_matrix[:3,3] = mounting['xyz']
        def matrix(value):
            x,y,z,w = value[3:]
            out = np.eye(4)
            out[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
            out[:3,3] = value[:3]
            return out
        if dynamic:
            if not args.robot_reference: p.error('robot dynamic tests require --robot-reference')
            reference = json.loads(Path(args.robot_reference).read_text())
            anchor = next(t for t in reference['transforms'] if t['parent']=='aft_mapped' and t['child']==args.device_frame)
            correction = matrix(anchor['translation_m']+anchor['quaternion_xyzw']) @ np.linalg.inv(mount_matrix)
    elif args.include_ipm:
        p.error('--include-ipm currently tests the robot-ground projection; use --tf-mode robot')
    if args.ros == 1:
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, CompressedImage, PointCloud2, Image
        from tf2_msgs.msg import TFMessage
        rospy.init_node('orbvi_tf_acceptance', anonymous=True)
        subscribe = lambda topic, typ, cb: rospy.Subscriber(topic, typ, cb, queue_size=50)
        spin = lambda: time.sleep(0.01)
        ns = lambda stamp: stamp.to_nsec()
    else:
        import rclpy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, CompressedImage, PointCloud2, Image
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        rclpy.init()
        node = rclpy.create_node('orbvi_tf_acceptance')
        def subscribe(topic, typ, cb):
            qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
            if args.sensor_reader_best_effort and topic not in ('/tf', '/tf_static', '/orbvi/vio/odometry'):
                qos.reliability = ReliabilityPolicy.BEST_EFFORT
            if topic == '/tf_static':
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            return node.create_subscription(typ, topic, cb, qos)
        spin = lambda: rclpy.spin_once(node, timeout_sec=0.01)
        ns = lambda stamp: stamp.sec * 10**9 + stamp.nanosec

    import tf2_ros
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer) if args.ros == 1 else tf2_ros.TransformListener(buffer, node)
    counts = collections.Counter()
    poses = collections.OrderedDict()
    transforms = collections.OrderedDict()
    edges = {}
    late_edges = set()
    image_frames = set()
    imu_frames = set()
    owners = set()
    samples = []
    errors = []
    source_times = []
    arrival_times = []
    dynamic_times = []
    compared = set()
    def compare(stamp):
        if stamp in poses and stamp in transforms and stamp not in compared:
            if len(compared) >= 256:
                compared.clear()
            compared.add(stamp)
            a, b = poses[stamp], transforms[stamp]
            error = (float(np.max(np.abs(matrix(a) @ correction - matrix(b))))
                     if mounting else max(abs(x-y) for x,y in zip(a,b)))
            counts['matched_pose_tf'] += 1
            if error > 1e-9:
                errors.append('odometry_tf_mismatch')
    def remember(mapping, stamp, value):
        mapping[stamp] = value
        while len(mapping) > 256:
            mapping.popitem(last=False)
        compare(stamp)
    def values(pose):
        return [pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    def odometry(message):
        counts['odometry'] += 1
        source_times.append(ns(message.header.stamp))
        arrival_times.append(time.monotonic())
        if len(source_times) > 16384:
            source_times.pop(0)
        remember(poses, ns(message.header.stamp), values(message.pose.pose))
    def tf(message):
        for t in message.transforms:
            if t.header.frame_id != 'camera_init' or t.child_frame_id != (mounting['parent_frame'] if mounting else 'aft_mapped'):
                errors.append('unexpected_dynamic_edge')
                continue
            counts['dynamic_tf'] += 1
            dynamic_times.append(ns(t.header.stamp))
            v = t.transform
            remember(transforms, ns(t.header.stamp), [v.translation.x,v.translation.y,v.translation.z,
                     v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w])
        if args.ros == 1 and hasattr(message, '_connection_header'):
            owners.add(message._connection_header.get('callerid', ''))
    def tf_static(message):
        counts['static_messages'] += 1
        for t in message.transforms:
            key = (t.header.frame_id,t.child_frame_id)
            v = t.transform
            edges[key] = [v.translation.x,v.translation.y,v.translation.z,
                          v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w]
    def late_static(message):
        for t in message.transforms:
            late_edges.add((t.header.frame_id,t.child_frame_id))
    def image(message):
        counts['raw_images'] += 1
        image_frames.add(message.header.frame_id)
    ipm_frames = set()
    ipm_samples = []
    def ipm(message):
        counts['ipm_images'] += 1
        ipm_frames.add(message.header.frame_id)
        if message.encoding != 'bgr8' or message.width != 80 or message.height != 80 or len(message.data) != 80*80*3:
            errors.append('ipm_image_format_or_size')
        if ns(message.header.stamp) == 0: errors.append('ipm_zero_stamp')
        if len(ipm_samples) < 128:
            ipm_samples.append(dict(stamp_ns=ns(message.header.stamp),sha256=hashlib.sha256(bytes(message.data)).hexdigest(),nonzero=any(message.data)))
    depth_frames = set()
    def depth(message):
        counts['depth_clouds'] += 1
        depth_frames.add(message.header.frame_id)
    def imu(message):
        counts['lidar_imu'] += 1
        imu_frames.add(message.header.frame_id)
    subs = [subscribe('/tf', TFMessage, tf), subscribe('/tf_static', TFMessage, tf_static),
            subscribe('/orbvi/vio/odometry', Odometry, odometry),
            subscribe('/orbvi/lidar/imu', Imu, imu)]
    for camera_id in (3,2,4,5):
        subs.append(subscribe('/orbvi/raw/camera_%d/image/compressed' % camera_id, CompressedImage, image))
    if args.include_depth:
        subs.append(subscribe('/orbvi/depth/points', PointCloud2, depth))
    if args.include_ipm:
        subs.append(subscribe('/orbvi/ipm/image', Image, ipm))
    streams = 'lidar_imu' if args.mode == 'sensors' else 'raw,lidar_imu'
    if dynamic or args.mode == 'disabled':
        streams += ',vio'
    if args.include_lidar:
        streams += ',lidar'
    if args.include_depth:
        streams += ',disparity,depth'
    params = dict(host=args.host, control_port=args.control_port, streams=streams,
                  publish_vio_tf=dynamic, publish_sensor_tf=static,
                  publish_decoded_images=False, publish_depth_pointcloud=args.include_depth,
                  publish_depth_viz=False, publish_depth_panorama=False,
                  publish_vio_path=False)
    if mounting:
        params.update(tf_mode='robot',mounting_config=args.mounting_config,
                      publish_mounting_tf=args.publish_mounting_tf,publish_odom_tf=args.publish_odom_tf)
    if args.include_ipm:
        params.update(publish_ipm=True,ipm_extent_m=2.0,ipm_resolution_m_per_pixel=0.05,
                      ipm_max_publish_rate_hz=0.0)
    if args.device_frame != 'orbvi_link':
        params['depth_pointcloud_frame_id'] = args.device_frame
    external_mount_broadcaster = None
    if mounting and static and not args.external_mount_publisher and (not args.publish_mounting_tf or args.mount_conflict != 'none'):
        from geometry_msgs.msg import TransformStamped
        external_mount_broadcaster = (tf2_ros.StaticTransformBroadcaster() if args.ros == 1
                                      else tf2_ros.StaticTransformBroadcaster(node))
        t = TransformStamped()
        t.header.frame_id = mounting['parent_frame']; t.child_frame_id = mounting['child_frame']
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z = mounting['xyz']
        r,p,y = (v/2 for v in mounting['rpy'])
        t.transform.rotation.x = math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y)
        t.transform.rotation.y = math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y)
        t.transform.rotation.z = math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y)
        t.transform.rotation.w = math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y)
        if args.delay_mount_seconds <= 0 and args.mount_conflict != "after":
            external_mount_broadcaster.sendTransform(t)
    cmd = [args.bridge_executable]
    if args.ros == 1:
        # This namespace belongs to the isolated probe. ROS1 private parameters
        # survive node exit; clear them before switching device/robot test modes.
        if rospy.has_param('/orbvi_tf_test_bridge'):
            rospy.delete_param('/orbvi_tf_test_bridge')
        cmd += ['__name:=orbvi_tf_test_bridge']
        cmd += ['_%s:=%s' % (k,str(v).lower() if isinstance(v,bool) else v) for k,v in params.items()]
    else:
        cmd += ['--ros-args','-r','__node:=orbvi_tf_test_bridge']
        for k,v in params.items():
            cmd += ['-p','%s:=%s' % (k,str(v).lower() if isinstance(v,bool) else v)]
    output = Path(args.output)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.with_suffix('.bridge.log').open('w') as log:
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        started = time.monotonic()
        try:
            late = False
            sampled = 0
            mount_sent = args.delay_mount_seconds <= 0 and args.mount_conflict != "after"
            paused_counts = None
            before_mount_counts = None
            while time.monotonic()-started < args.duration:
                spin()
                elapsed = time.monotonic()-started
                injection_time = 3.0 if args.mount_conflict == 'after' else args.delay_mount_seconds
                if external_mount_broadcaster and not mount_sent:
                    before_mount_counts = dict(counts)
                    if elapsed >= injection_time:
                        external_mount_broadcaster.sendTransform(t); mount_sent = True
                if args.mount_conflict == 'after' and elapsed > injection_time + 1.5 and paused_counts is None:
                    paused_counts = dict(counts)
                if elapsed > 3 and not late:
                    subs.append(subscribe('/tf_static', TFMessage, late_static)); late = True
                if elapsed-sampled >= 1:
                    if args.ros == 2:
                        owners.update(info.node_name for info in node.get_publishers_info_by_topic('/tf'))
                    sampled = elapsed
                    try:
                        fields = Path('/proc/%d/stat' % process.pid).read_text().split()
                        status = Path('/proc/%d/status' % process.pid).read_text().splitlines()
                        rss = next(int(line.split()[1]) for line in status if line.startswith('VmRSS:'))
                        samples.append(dict(elapsed=elapsed,cpu_ticks=int(fields[13])+int(fields[14]),rss_kib=rss))
                    except (OSError,StopIteration):
                        pass
                if process.poll() is not None:
                    errors.append('bridge_exited:%d' % process.returncode)
                    break
        finally:
            os.killpg(process.pid, signal.SIGINT) if process.poll() is None else None
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
                errors.append('bridge_shutdown_timeout')
    if args.ros == 2:
        owners.update(info.node_name for info in node.get_publishers_info_by_topic('/tf'))
    if dynamic and args.mount_conflict != 'before':
        if counts['matched_pose_tf'] < 5: errors.append('insufficient_matched_dynamic_samples')
        if any(b <= a for a,b in zip(dynamic_times,dynamic_times[1:])):
            errors.append('nonincreasing_dynamic_timestamp')
    elif counts['dynamic_tf']:
        errors.append('dynamic_tf_not_disabled')
    if args.mount_conflict == 'before' and counts['ipm_images']:
        errors.append('ipm_not_paused_for_mount_conflict')
    if args.mount_conflict == 'after':
        if paused_counts is None: errors.append('missing_conflict_observation_window')
        elif any(counts[k] != paused_counts.get(k,0) for k in ('dynamic_tf','ipm_images')):
            errors.append('robot_outputs_continued_after_mount_conflict')
    if args.delay_mount_seconds > 0 and before_mount_counts:
        if any(before_mount_counts.get(k,0) for k in ('dynamic_tf','ipm_images')):
            errors.append('robot_outputs_before_installation_tf')
    if static:
        if len(edges) != ((8 if mounting else 7) + int(args.publish_odom_tf)): errors.append('static_edge_count:%d' % len(edges))
        if set(edges) != late_edges: errors.append('late_subscriber_missing_static_tf')
    elif edges or late_edges:
        errors.append('static_tf_not_disabled')
    if dynamic or args.mode == 'disabled':
        if counts['odometry'] < 5: errors.append('missing_odometry')
    if args.mode != 'sensors':
        if image_frames != {'orbvi/camera/%d' % i for i in (3,2,4,5)}:
            errors.append('raw_camera_frame_mismatch')
    if imu_frames != {'mid360_imu'}: errors.append('lidar_imu_frame_mismatch')
    if args.include_depth and depth_frames != {args.device_frame}:
        errors.append('depth_cloud_frame_mismatch')
    if args.include_ipm and args.mount_conflict != 'before' and ipm_frames != {mounting['parent_frame']}:
        errors.append('robot_ipm_frame_mismatch_or_missing')
    if static:
        parents = {}
        for parent,child in edges:
            if child in parents: errors.append('multiple_tf_parents:'+child)
            parents[child] = parent
        for child in parents:
            seen = set()
            while child in parents:
                if child in seen:
                    errors.append('tf_cycle'); break
                seen.add(child);child = parents[child]
    lookups = []
    if static:
        root_frame = ('odom' if args.publish_odom_tf else 'camera_init') if dynamic and args.mount_conflict != 'before' else (mounting['parent_frame'] if mounting else args.device_frame)
        targets = ['orbvi/camera/%d' % i for i in (3,2,4,5)]
        targets += ['livox_frame', 'mid360_imu', args.device_frame]
        if mounting: targets += ['aft_mapped',mounting['parent_frame']]
        for target in targets:
            try:
                stamp = rospy.Time(0) if args.ros == 1 else rclpy.time.Time()
                buffer.lookup_transform(root_frame, target, stamp)
                lookups.append(root_frame + '<-' + target)
                if dynamic and args.mount_conflict == 'none' and transforms:
                    sample = max(transforms)
                    stamp = rospy.Time(sample//10**9,sample%10**9) if args.ros == 1 else rclpy.time.Time(nanoseconds=sample)
                    buffer.lookup_transform(root_frame, target, stamp)
            except Exception as error:
                errors.append('tf_lookup_failed:' + target + ':' + str(error))
    report = dict(ros=args.ros,mode=args.mode,tf_mode=args.tf_mode,mount_conflict=args.mount_conflict,delay_mount_seconds=args.delay_mount_seconds,publish_odom_tf=args.publish_odom_tf,publish_mounting_tf=args.publish_mounting_tf,duration_s=time.monotonic()-started,
                  passed=not errors,errors=sorted(set(errors)),counts=dict(counts),tf_lookups=lookups,
                  static_transforms=[dict(parent=k[0],child=k[1],values=v) for k,v in edges.items()],
                  late_static_edges=len(late_edges),raw_camera_frames=sorted(image_frames),
                  lidar_imu_frames=sorted(imu_frames),depth_cloud_frames=sorted(depth_frames),ipm_frames=sorted(ipm_frames),ipm_samples=ipm_samples,tf_owners=sorted(owners),resource_samples=samples)
    if len(source_times)>1:
        report['source_pose_hz']=(len(source_times)-1)*1e9/(source_times[-1]-source_times[0])
        report['max_source_gap_s']=max(b-a for a,b in zip(source_times,source_times[1:]))/1e9
        report['max_arrival_gap_s']=max(b-a for a,b in zip(arrival_times,arrival_times[1:]))
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('resource_samples','static_transforms')},indent=2))
    if args.ros == 2:
        node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
