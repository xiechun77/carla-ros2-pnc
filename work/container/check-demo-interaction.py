import json
import math
import time
from collections import Counter
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path,Odometry
from sensor_msgs.msg import Image,PointCloud2
from rosgraph_msgs.msg import Clock
from carla_msgs.msg import CarlaControl,CarlaEgoVehicleControl

rclpy.init()
node=rclpy.create_node('demo_interface_check')
counts=Counter();last={};subscriptions=[]
def sub(key,typ,topic,qos=10):
 def callback(msg):counts[key]+=1;last[key]=msg
 subscriptions.append(node.create_subscription(typ,topic,callback,qos))
sub('clock',Clock,'/clock')
sub('path',Path,'/carla/ego_vehicle/waypoints',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
sub('odometry',Odometry,'/carla/ego_vehicle/odometry')
sub('image',Image,'/carla/ego_vehicle/rgb_view/image')
sub('lidar',PointCloud2,'/carla/ego_vehicle/lidar')
sub('control',CarlaEgoVehicleControl,'/carla/ego_vehicle/vehicle_control_cmd')
control=node.create_publisher(CarlaControl,'/carla/control',10)
goal=node.create_publisher(PoseStamped,'/carla/ego_vehicle/goal',10)
def spin(seconds,predicate=None):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=0.1)
  if predicate and predicate():return True
 return False
def stamp():
 c=last['clock'].clock;return c.sec+c.nanosec/1e9
result={}
try:
 result['data_ready']=spin(45,lambda:counts['clock']>=5 and counts['image']>=3 and counts['lidar']>=3 and counts['control']>=5 and 'path'in last and len(last['path'].poses)>10 and 'odometry'in last)
 if not result['data_ready']:raise RuntimeError('Demo topic readiness failed')
 control.publish(CarlaControl(command=CarlaControl.PAUSE));spin(1.)
 before=stamp();spin(0.6);result['pause_stops_clock']=abs(stamp()-before)<1e-8
 before=stamp();control.publish(CarlaControl(command=CarlaControl.STEP_ONCE));spin(2.,lambda:stamp()>before);spin(0.5)
 result['step_sim_seconds']=stamp()-before
 result['single_step_passed']=abs(result['step_sim_seconds']-0.05)<1e-5
 old=last['path'];p=last['odometry'].pose.pose.position
 index=min(range(len(old.poses)),key=lambda i:math.hypot(old.poses[i].pose.position.x-p.x,old.poses[i].pose.position.y-p.y))
 destination=old.poses[min(index+35,len(old.poses)-1)].pose
 old_count=counts['path'];msg=PoseStamped();msg.header.frame_id='map';msg.pose=destination
 result['goal_subscribers']=node.count_subscribers('/carla/ego_vehicle/goal')
 goal.publish(msg);result['new_route_received']=spin(10,lambda:counts['path']>old_count)
 result['old_path_points']=len(old.poses);result['new_path_points']=len(last['path'].poses)
 result['requested_goal_ros_xy']=[destination.position.x,destination.position.y]
 if last['path'].poses:
  end=last['path'].poses[-1].pose.position
  result['new_route_endpoint_distance_to_requested_goal_m']=math.hypot(end.x-destination.position.x,end.y-destination.position.y)
 before=stamp();control.publish(CarlaControl(command=CarlaControl.PLAY));result['play_resumes_clock']=spin(5,lambda:stamp()>before+0.1)
 control.publish(CarlaControl(command=CarlaControl.PAUSE));spin(1.)
 result['passed']=all(result.get(k,False) for k in ['data_ready','pause_stops_clock','single_step_passed','new_route_received','play_resumes_clock']) and result.get('new_route_endpoint_distance_to_requested_goal_m',100)<3.
except Exception as exc:
 result['passed']=False;result['error']=str(exc)
finally:
 control.publish(CarlaControl(command=CarlaControl.PAUSE));spin(0.5)
 result['topic_counts']=dict(counts)
 print(json.dumps(result,indent=2),flush=True)
 node.destroy_node();rclpy.shutdown()
if not result['passed']:raise SystemExit(1)
