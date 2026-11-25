"""Reproducible same-lane lead-vehicle diagnostic. No planner/controller edits."""
import csv,json,math,os,signal,subprocess,time,hashlib
from pathlib import Path
import carla
from agents.navigation.controller import VehiclePIDController
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import Bool
from carla_msgs.msg import CarlaEgoVehicleControl,CarlaControl,CarlaCollisionEvent
from derived_object_msgs.msg import ObjectArray
from nav_msgs.msg import Odometry
from pnc_msgs.msg import Trajectory

out=Path('/results');cfg=json.loads(Path('/scenario/scenario.json').read_text())
rclpy.init();node=rclpy.create_node('following_diagnostic')
latest={};counts={};subs=[];processes=[];rows=[];collisions=[];ego=None;lead=None
stop=[False];signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True));signal.signal(signal.SIGINT,lambda *_:stop.__setitem__(0,True))
summary={'completed':False,'purpose':'diagnose original same-lane response, not an acceptance test for safe following','scenario':cfg}
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
override=node.create_publisher(Bool,'/carla/ego_vehicle/vehicle_control_manual_override',latched)
manual=node.create_publisher(CarlaEgoVehicleControl,'/carla/ego_vehicle/vehicle_control_cmd_manual',10)
control=node.create_publisher(CarlaControl,'/carla/control',10)
holding=[True]
def hold():
 override.publish(Bool(data=holding[0]))
 if holding[0]:manual.publish(CarlaEgoVehicleControl(brake=1.0))
timer=node.create_timer(0.05,hold)
def subscribe(key,typ,topic):
 def cb(msg):
  latest[key]=msg;counts[key]=counts.get(key,0)+1
  if key=='collision':collisions.append({'other_actor_id':msg.other_actor_id})
 subs.append(node.create_subscription(typ,topic,cb,10))
subscribe('odom',Odometry,'/carla/ego_vehicle/odometry')
subscribe('trajectory',Trajectory,'/carla/ego_vehicle/planning_trajectory')
subscribe('objects',ObjectArray,'/carla/ego_vehicle/objects')
subscribe('command',CarlaEgoVehicleControl,'/carla/ego_vehicle/vehicle_control_cmd')
subscribe('collision',CarlaCollisionEvent,'/carla/ego_vehicle/collision')
def spin(seconds=.1):
 until=time.monotonic()+seconds
 while time.monotonic()<until and not stop[0]:rclpy.spin_once(node,timeout_sec=.02)
 if stop[0]:raise RuntimeError('interrupted')
 failed=[(name,p.returncode) for name,p,_ in processes if p.poll() is not None]
 if failed:raise RuntimeError('Child exited '+str(failed))
def launch(name,args):
 f=(out/(name+'.log')).open('w');p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,p,f))
 print('START',name,flush=True)
def wait(label,predicate,timeout=40):
 deadline=time.monotonic()+timeout
 while time.monotonic()<deadline:
  spin()
  if predicate():print('READY',label,flush=True);return
 raise RuntimeError('Timeout '+label+' '+str(counts))
def speed(v):return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
client=carla.Client('127.0.0.1',2000);client.set_timeout(10)
try:
 launch('bridge',['ros2','launch','carla_ros_bridge','carla_ros_bridge.launch.py','host:=127.0.0.1','town:=Town01','fixed_delta_seconds:=0.05'])
 wait('bridge control subscription',lambda:node.count_subscribers('/carla/control')>0)
 hold();spin(.5)
 launch('pnc',['ros2','launch','my_pnc','my_pnc_launch.py'])
 launch('waypoints',['ros2','launch','carla_waypoint_publisher','carla_waypoint_publisher.launch.py','host:=127.0.0.1'])
 launch('spawn',['ros2','run','carla_spawn_objects','carla_spawn_objects','--ros-args','-p','objects_definition_file:=/scenario/objects.json','-p','spawn_point_ego_vehicle:='+cfg['ego_spawn_ros']])
 wait('state, objects and trajectory',lambda:all(counts.get(k,0)>3 for k in ['odom','objects','trajectory']),60)
 world=client.get_world();world.wait_for_tick(10)
 actors=world.get_actors().filter('vehicle.*')
 ego=next(a for a in actors if a.attributes.get('role_name')=='ego_vehicle')
 lead=next(a for a in actors if a.attributes.get('role_name')=='vehicle_2')
 control.publish(CarlaControl(command=1));spin(.5)
 # Use the published pause command, never add a second world ticker.
 f1=world.get_snapshot().frame;spin(.3);f2=world.get_snapshot().frame
 if f1!=f2:raise RuntimeError('World pause not confirmed')
 ego.set_transform(carla.Transform(carla.Location(x=92.387,y=190,z=.6),carla.Rotation(yaw=-90)))
 lead.set_transform(carla.Transform(carla.Location(x=92.387,y=160,z=.6),carla.Rotation(yaw=-90)))
 for actor in [ego,lead]:
  actor.set_target_velocity(carla.Vector3D());actor.set_target_angular_velocity(carla.Vector3D())
 lead.apply_control(carla.VehicleControl(brake=0,hand_brake=False))
 dt=world.get_settings().fixed_delta_seconds or .05
 pid=VehiclePIDController(lead,args_lateral=dict(K_P=1.95,K_I=.05,K_D=.2,dt=dt),args_longitudinal=dict(K_P=1.,K_I=.05,K_D=0.,dt=dt),max_throttle=.5,max_brake=1,max_steering=.8)
 if os.environ.get('FOLLOW_GUI')=='1':
  launch('rviz',['ros2','run','rviz2','rviz2','-d','/local-path.rviz'])
  launch('camera',['ros2','launch','carla_manual_control','carla_manual_control.launch.py'])
  spin(2)
  # The keyboard window may publish override=False on startup; reassert our hold.
  hold();spin(.2)
 start=world.get_snapshot().timestamp.elapsed_seconds
 frame=world.get_snapshot().frame;last_report=-1;released=False
 control.publish(CarlaControl(command=0))
 deadline=time.monotonic()+90
 reason='wall_timeout'
 while time.monotonic()<deadline:
  spin(.005)
  snap=world.get_snapshot()
  if snap.frame==frame:continue
  frame=snap.frame;t=snap.timestamp.elapsed_seconds-start
  wp=world.get_map().get_waypoint(lead.get_location());candidates=wp.next(6)
  if not candidates:reason='lead_road_end';break
  target=sorted(candidates,key=lambda p:(p.road_id,p.section_id,p.lane_id,p.s))[0]
  lead.apply_control(pid.run_step(cfg['lead_speed_kmh'],target))
  if t>=cfg['lead_warmup_sim_s'] and not released:
   holding[0]=False;hold();released=True;print('RELEASE ego',round(t,2),flush=True)
  ep=ego.get_location();lp=lead.get_location();eh=-math.radians(ego.get_transform().rotation.yaw)
  dx=lp.x-ep.x;dy=-lp.y+ep.y
  longitudinal=dx*math.cos(eh)+dy*math.sin(eh);lateral=-dx*math.sin(eh)+dy*math.cos(eh)
  trajectory=latest.get('trajectory');near=min(trajectory.points,key=lambda p:(p.x-ep.x)**2+(p.y+ep.y)**2) if trajectory and trajectory.points else None
  objects=latest.get('objects');obj=next((o for o in objects.objects if o.id==lead.id),None) if objects else None
  obj_lateral_velocity=(-obj.twist.linear.x*math.sin(eh)+obj.twist.linear.y*math.cos(eh)) if obj else None
  cmd=latest.get('command')
  row=dict(sim_s=round(t,3),phase='released' if released else 'lead_warmup',ego_x=ep.x,ego_y=-ep.y,lead_x=lp.x,lead_y=-lp.y,center_longitudinal_m=longitudinal,lateral_m=lateral,ego_speed_mps=speed(ego.get_velocity()),lead_speed_mps=speed(lead.get_velocity()),nearest_plan_speed_mps=near.v if near else None,objects_contains_lead=obj is not None,lead_lateral_speed_ego_frame=obj_lateral_velocity,throttle=cmd.throttle if cmd else None,brake=cmd.brake if cmd else None)
  rows.append(row)
  if t-last_report>=1:
   print('t=%.1f gap(center)=%.1f ego=%.2f lead=%.2f plan=%s objects=%s'%(t,longitudinal,row['ego_speed_mps'],row['lead_speed_mps'],row['nearest_plan_speed_mps'],obj is not None),flush=True);last_report=t
  if collisions:reason='collision';break
  if longitudinal<cfg['collision_guard_gap_m'] and abs(lateral)<3:reason='diagnostic_gap_guard';break
  if -lp.y>cfg['end_before_ros_y']:reason='straight_segment_end';break
  if t>=cfg['observation_sim_s']+cfg['lead_warmup_sim_s']:reason='window_complete';break
 summary.update(completed=True,stop_reason=reason,topic_counts=counts,collision_events=collisions,rows=len(rows),guard_is_not_vehicle_bbox_clearance=True)
 if rows:
  summary.update(initial_center_gap_m=rows[0]['center_longitudinal_m'],final_center_gap_m=rows[-1]['center_longitudinal_m'],minimum_center_gap_m=min(r['center_longitudinal_m'] for r in rows),max_ego_speed_mps=max(r['ego_speed_mps'] for r in rows),max_lead_speed_mps=max(r['lead_speed_mps'] for r in rows),object_lead_visible_samples=sum(r['objects_contains_lead'] for r in rows))
 print(json.dumps(summary,indent=2),flush=True)
except Exception as e:
 summary['error']=str(e);print('ERROR',str(e),flush=True)
finally:
 # Brake and pause before terminating subscribers/controllers. Stop child groups only.
 holding[0]=True
 try:
  hold()
  if lead and lead.is_alive:lead.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
  control.publish(CarlaControl(command=1))
  for _ in range(10):rclpy.spin_once(node,timeout_sec=.02)
 except Exception:pass
 for name,p,f in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  f.close()
 if rows:
  with (out/'samples.csv').open('w') as f:
   writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
 (out/'summary.json').write_text(json.dumps(summary,indent=2))
 node.destroy_node();rclpy.shutdown()
if not summary['completed']:raise SystemExit(1)
