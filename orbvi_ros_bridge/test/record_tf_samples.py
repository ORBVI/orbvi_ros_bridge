#!/usr/bin/env python3
"""Bounded acceptance bag: all TF/poses, each sensor topic at most once/second."""
import argparse
import json
from pathlib import Path
import signal
import threading
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--ros',type=int,choices=(1,2),required=True)
p.add_argument('--duration',type=float,default=660)
p.add_argument('--output',required=True)
p.add_argument('--max-bytes',type=int,default=1500*1024*1024)
a=p.parse_args()
output=Path(a.output)
if output.exists():raise SystemExit('Output already exists')
counts={};last={};budget=0
stopping=threading.Event()
write_lock=threading.Lock()
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage,Imu
from livox_ros_driver2.msg import CustomMsg
if a.ros==1:
 import rospy,rosbag
 rospy.init_node('orbvi_tf_sample_recorder',anonymous=True)
 writer=rosbag.Bag(str(output),'w',compression='lz4')
 def write(topic,message):writer.write(topic,message,t=rospy.Time.now())
 def subscribe(topic,kind,callback):return rospy.Subscriber(topic,kind,callback,queue_size=10)
 def spin():time.sleep(.01)
else:
 import rclpy,rosbag2_py
 from rclpy.serialization import serialize_message
 from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
 rclpy.init();node=rclpy.create_node('orbvi_tf_sample_recorder')
 writer=rosbag2_py.SequentialWriter()
 writer.open(rosbag2_py.StorageOptions(uri=str(output),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
 def write(topic,message):writer.write(topic,serialize_message(message),time.time_ns())
 def subscribe(topic,kind,callback):
  qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
  if topic=='/tf_static':qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
  writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=kind.__module__.split('.')[0]+'/msg/'+kind.__name__,serialization_format='cdr'))
  return node.create_subscription(kind,topic,callback,qos)
 def spin():rclpy.spin_once(node,timeout_sec=.01)

def callback(topic,sensor=False):
 def receive(message):
  global budget
  with write_lock:
   if stopping.is_set():return
   now=time.monotonic()
   if sensor and now-last.get(topic,0)<1:return
   last[topic]=now
   estimate=len(getattr(message,'data',[]))+len(getattr(message,'points',[]))*24+1024
   if budget+estimate>a.max_bytes:return
   write(topic,message);budget+=estimate;counts[topic]=counts.get(topic,0)+1
 return receive
subs=[]
for topic,kind,sensor in [('/tf',TFMessage,False),('/tf_static',TFMessage,False),('/orbvi/vio/odometry',Odometry,False),('/orbvi/lidar/custom',CustomMsg,True),('/orbvi/lidar/imu',Imu,True)]+[('/orbvi/raw/camera_%d/image/compressed'%i,CompressedImage,True)for i in(3,2,4,5)]:
 subs.append(subscribe(topic,kind,callback(topic,sensor)))
started=time.monotonic()
started_utc_ns=time.time_ns()
failure=''
for sig in (signal.SIGINT,signal.SIGTERM):
 signal.signal(sig,lambda *_:stopping.set())
try:
 while time.monotonic()-started<a.duration and not stopping.is_set():spin()
except Exception as error:
 failure=str(error)
 raise
finally:
 interrupted=stopping.is_set()
 stopping.set()
 with write_lock:
  if a.ros==1:writer.close()
  else:
   del writer;node.destroy_node();rclpy.shutdown()
 output.with_suffix('.summary.json').write_text(json.dumps(dict(ros=a.ros,duration_s=time.monotonic()-started,started_utc_ns=started_utc_ns,interrupted=interrupted,error=failure,estimated_bytes=budget,max_bytes=a.max_bytes,counts=counts),indent=2)+'\n')
