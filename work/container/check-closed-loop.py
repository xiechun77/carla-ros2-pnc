import csv
import json
import math
import os
import signal
import subprocess
import time
from collections import Counter
from pathlib import Path

import carla
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from derived_object_msgs.msg import ObjectArray
from carla_msgs.msg import CarlaEgoVehicleInfo, CarlaEgoVehicleStatus, CarlaEgoVehicleControl, CarlaCollisionEvent
from pnc_msgs.msg import Trajectory
from ament_index_python.packages import get_package_share_directory

out = Path('/results')
out.mkdir(exist_ok=True)
rclpy.init()
node = rclpy.create_node('reproduction_closed_loop_check')
counts = Counter()
processes = []
subscriptions = []
rows = []
route = []
route_s = []
last = {}
phase = 'startup'
start = time.monotonic()
collisions = []
invalid_trajectories = 0
invalid_commands = 0
empty_trajectories = 0
trajectory_sizes = []
commands = []
command_violations = []
nonfinite_commands = 0
severe_command_violations = 0
last_image = None
last_image_time = 0

def subscribe(key, msg_type, topic, callback=None, qos=10):
    def receive(msg):
        counts[key] += 1
        last[key] = msg
        if callback:
            callback(msg)
    subscriptions.append(node.create_subscription(msg_type, topic, receive, qos))

def route_cb(msg):
    route[:] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    route_s[:] = [0.]
    for a,b in zip(route,route[1:]):
        route_s.append(route_s[-1]+math.hypot(b[0]-a[0],b[1]-a[1]))

def distance_to_route(x, y):
    best = float('inf')
    along = None
    for i,((ax, ay), (bx, by)) in enumerate(zip(route, route[1:])):
        dx, dy = bx-ax, by-ay
        u = max(0., min(1., ((x-ax)*dx+(y-ay)*dy)/max(dx*dx+dy*dy, 1e-12)))
        distance = math.hypot(x-ax-u*dx, y-ay-u*dy)
        if distance < best:
            best = distance
            along = route_s[i]+u*math.hypot(dx,dy)
    return (best, along) if math.isfinite(best) else (None,None)

def odom_cb(msg):
    p = msg.pose.pose.position
    v = msg.twist.twist.linear
    q = msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    c = last.get('command')
    clock = last.get('clock')
    offset, along = distance_to_route(p.x,p.y)
    rows.append({'wall_s': time.monotonic()-start, 'sim_s': clock.clock.sec+clock.clock.nanosec/1e9 if clock else 0.,
        'phase': phase, 'x': p.x, 'y': p.y, 'z': p.z,
        'speed_mps': math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
        'heading_rad':yaw,
        'route_distance_m': offset, 'route_s_m': along,
        'route_remaining_m': route_s[-1]-along if along is not None else None,
        'goal_distance_m': math.hypot(p.x-route[-1][0],p.y-route[-1][1]) if route else None,
        'throttle': c.throttle if c else 0., 'brake': c.brake if c else 0., 'steer': c.steer if c else 0.})

def trajectory_cb(msg):
    global invalid_trajectories, empty_trajectories
    trajectory_sizes.append(len(msg.points))
    if not msg.points:
        empty_trajectories += 1
    if any(not all(math.isfinite(getattr(p, k)) for k in ['x','y','heading','kappa','v','ax','ay','a_tau','time_stamped']) for p in msg.points):
        invalid_trajectories += 1

def command_cb(msg):
    global invalid_commands, nonfinite_commands, severe_command_violations
    record = {'wall_s':time.monotonic()-start,'phase':phase,'throttle':msg.throttle,'brake':msg.brake,'steer':msg.steer}
    commands.append(record)
    finite = all(math.isfinite(x) for x in [msg.throttle,msg.brake,msg.steer])
    if not finite:
        nonfinite_commands += 1
    if not finite or not (-0.01 <= msg.throttle <= 1.01 and -0.01 <= msg.brake <= 1.01 and -1.01 <= msg.steer <= 1.01):
        severe_command_violations += 1
    if not finite or not (0 <= msg.throttle <= 1 and 0 <= msg.brake <= 1 and -1 <= msg.steer <= 1):
        invalid_commands += 1
        if len(command_violations)<30:
            command_violations.append(record)
    if abs(msg.steer)>=0.99 and 'trajectory' in last and not (out/'steering-event-trajectory.json').exists():
        (out/'steering-event-trajectory.json').write_text(json.dumps({
            'command':record,
            'trajectory':[{'x':p.x,'y':p.y,'heading':p.heading,'kappa':p.kappa,'v':p.v,'time_stamped':p.time_stamped} for p in last['trajectory'].points]}))

def collision_cb(msg):
    i = msg.normal_impulse
    collisions.append({'wall_s': time.monotonic()-start, 'other_actor_id': msg.other_actor_id,
        'impulse': math.sqrt(i.x*i.x+i.y*i.y+i.z*i.z)})

def image_cb(msg):
    global last_image, last_image_time
    if time.monotonic()-last_image_time > 0.5:
        last_image = msg
        last_image_time = time.monotonic()

prefix = '/carla/ego_vehicle/'
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscribe('clock', Clock, '/clock')
subscribe('odometry', Odometry, prefix+'odometry', odom_cb)
subscribe('imu', Imu, prefix+'imu')
subscribe('vehicle_info', CarlaEgoVehicleInfo, prefix+'vehicle_info', qos=latched)
subscribe('vehicle_status', CarlaEgoVehicleStatus, prefix+'vehicle_status')
subscribe('objects', ObjectArray, prefix+'objects')
subscribe('waypoints', RosPath, prefix+'waypoints', route_cb, latched)
subscribe('trajectory', Trajectory, prefix+'planning_trajectory', trajectory_cb)
subscribe('command', CarlaEgoVehicleControl, prefix+'vehicle_control_cmd', command_cb)
subscribe('collision', CarlaCollisionEvent, prefix+'collision', collision_cb)
subscribe('image', Image, prefix+'rgb_view/image', image_cb)
target = node.create_publisher(Float64, prefix+'target_speed', latched)
timer = node.create_timer(1., lambda: target.publish(Float64(data=6.33)))

def launch(name, args):
    log = open(out/(name+'.log'), 'w')
    p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((name, p, log))
    print('START', name, flush=True)

def wait_for(label, predicate, seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        failed = [(n,p.returncode) for n,p,_ in processes if p.poll() is not None]
        if failed:
            raise RuntimeError('Processes exited: '+str(failed))
        if predicate():
            print('PASS', label, dict(counts), flush=True)
            return
    raise RuntimeError('Timeout: '+label+'; counts='+str(dict(counts)))

full_route = os.environ.get('FULL_ROUTE') == '1'
obstacle_s = float(os.environ.get('STATIC_OBSTACLE_S','0'))
max_sim = float(os.environ.get('MAX_SIM_S', '180' if full_route else '12'))
max_wall = float(os.environ.get('MAX_WALL_S', '300' if full_route else '45'))
summary = {'passed': False, 'source_modified': os.environ.get('SOURCE_MODIFIED') == '1', 'pnc_image': os.environ.get('PNC_IMAGE','unspecified'), 'requested_observation_sim_s': max_sim,
    'mode': 'static_obstacle' if obstacle_s else ('full_route' if full_route else 'short_window'),
    'arrival_distance_tolerance_m':3.,'arrival_speed_tolerance_mps':0.5,'arrival_hold_sim_s':2.}
try:
    launch('bridge', ['ros2','launch','carla_ros_bridge','carla_ros_bridge.launch.py','host:=127.0.0.1','town:=Town01','synchronous_mode:=True','fixed_delta_seconds:=0.05'])
    wait_for('bridge clock', lambda: counts['clock'] >= 5, 35)
    launch('pnc', ['ros2','launch','my_pnc','my_pnc_launch.py'])
    wait_for('pnc vehicle_info subscribers', lambda: node.count_subscribers(prefix+'vehicle_info') >= 3, 10)
    if os.environ.get('FIXED_BACKGROUND') == '1':
        setup_client = carla.Client('127.0.0.1',2000)
        setup_client.set_timeout(10.)
        setup_world = setup_client.get_world()
        candidates = setup_world.get_map().get_spawn_points()
        selected = []
        for t in candidates:
            if math.hypot(t.location.x-127.4,t.location.y-195.4) < 60:
                continue
            if any(t.location.distance(p.location)<15 for p in selected):
                continue
            selected.append(t)
            if len(selected)==9:
                break
        if len(selected)!=9:
            raise RuntimeError('Not enough separated background spawn points')
        args = ['ros2','run','carla_spawn_objects','carla_spawn_objects','--ros-args',
            '-p','objects_definition_file:='+get_package_share_directory('carla_spawn_objects')+'/config/objects.json',
            '-p','spawn_point_ego_vehicle:=127.4,-195.4,2,0,0,180']
        positions = {}
        for i,t in enumerate(selected,1):
            value = ','.join(f'{v:.6f}' for v in [t.location.x,-t.location.y,t.location.z+0.3,t.rotation.roll,-t.rotation.pitch,-t.rotation.yaw])
            args.extend(['-p',f'spawn_point_vehicle_{i}:={value}'])
            positions[f'vehicle_{i}'] = value
        summary['background_mode'] = 'fixed_separated_at_least_60m_from_initial_ego'
        summary['background_spawn_parameters'] = positions
        launch('spawn', args)
    else:
        summary['background_mode'] = 'original_random'
        launch('spawn', ['ros2','launch','carla_spawn_objects','carla_spawn_objects.launch.py','spawn_point_ego_vehicle:=127.4,-195.4,2,0,0,180'])
    phase = 'vehicle_gate'
    wait_for('vehicle state/IMU/info/objects', lambda: counts['odometry']>=10 and counts['imu']>=10 and counts['vehicle_info']>=1 and counts['objects']>=3 and counts['vehicle_status']>=3, 50)
    summary['vehicle_gate_counts'] = dict(counts)
    client = carla.Client('127.0.0.1',2000)
    client.set_timeout(3.)
    world = client.get_world()
    world.wait_for_tick(3.)
    summary['vehicles'] = [{'id':a.id,'role':a.attributes.get('role_name',''),'x':a.get_location().x,'y':a.get_location().y} for a in world.get_actors().filter('vehicle.*')]
    launch('ad_agent', ['ros2','run','carla_ad_agent','ad_agent'])
    launch('waypoints', ['ros2','launch','carla_waypoint_publisher','carla_waypoint_publisher.launch.py','host:=127.0.0.1','timeout:=10'])
    phase = 'planning_gate'
    wait_for('route and nonempty trajectory', lambda: len(route)>=20 and counts['trajectory']>=3 and trajectory_sizes[-1]>0, 35)
    if obstacle_s:
        index = next(i for i,s in enumerate(route_s) if s>=obstacle_s)
        ox,oy = route[index]
        before,after = route[max(0,index-1)],route[min(len(route)-1,index+1)]
        obstacle_yaw = math.atan2(after[1]-before[1],after[0]-before[0])
        obstacle_world = client.get_world()
        obstacle_world.wait_for_tick(3.)
        actors = list(obstacle_world.get_actors().filter('vehicle.*'))
        obstacle = next(a for a in actors if a.attributes.get('role_name')=='vehicle_1')
        ego_actor = next(a for a in actors if a.attributes.get('role_name')=='ego_vehicle')
        waypoint = obstacle_world.get_map().get_waypoint(carla.Location(x=ox,y=-oy,z=0.))
        transform = carla.Transform(carla.Location(x=ox,y=-oy,z=waypoint.transform.location.z+0.4),carla.Rotation(yaw=-math.degrees(obstacle_yaw)))
        obstacle.set_transform(transform)
        obstacle.apply_control(carla.VehicleControl(brake=1.))
        obstacle_world.wait_for_tick(3.)
        summary['obstacle'] = {'id':obstacle.id,'route_s_m':route_s[index],'ros_x':ox,'ros_y':oy,'ros_heading_rad':obstacle_yaw,
            'bbox_extent_x':obstacle.bounding_box.extent.x,'bbox_extent_y':obstacle.bounding_box.extent.y,
            'bbox_offset_x':obstacle.bounding_box.location.x,'bbox_offset_y':-obstacle.bounding_box.location.y}
        summary['ego_bbox'] = {'extent_x':ego_actor.bounding_box.extent.x,'extent_y':ego_actor.bounding_box.extent.y,
            'offset_x':ego_actor.bounding_box.location.x,'offset_y':-ego_actor.bounding_box.location.y}
        print('OBSTACLE',json.dumps(summary['obstacle']),flush=True)
    phase = 'closed_loop'
    observation_start = time.monotonic()
    sim_start = rows[-1]['sim_s']
    begin_counts = dict(counts)
    next_progress = observation_start
    arrival_start = None
    departure_start = None
    while time.monotonic()-observation_start < max_wall:
        rclpy.spin_once(node, timeout_sec=0.1)
        if any(p.poll() is not None for _,p,_ in processes):
            raise RuntimeError('A process exited during closed loop')
        if collisions:
            summary['stop_reason'] = 'collision_event'
            break
        if severe_command_violations or invalid_trajectories or (invalid_commands and os.environ.get('STRICT_COMMAND_BOUNDS','1')=='1'):
            summary['stop_reason'] = 'nonfinite_or_out_of_range_output'
            break
        r = rows[-1]
        elapsed_sim = r['sim_s']-sim_start
        if obstacle_s and r['route_s_m'] > summary['obstacle']['route_s_m']+25 and r['route_distance_m']<1.:
            summary['stop_reason'] = 'obstacle_passed_and_rejoined'
            break
        if time.monotonic() >= next_progress:
            print('PROGRESS', json.dumps({'sim_s':round(elapsed_sim,2),'route_s_m':r['route_s_m'],
                'route_length_m':route_s[-1], 'goal_distance_m':r['goal_distance_m'],
                'speed_mps':r['speed_mps'],'route_distance_m':r['route_distance_m']}),flush=True)
            if last_image is not None:
                cv2.imwrite(str(out/f'frame-{int(elapsed_sim):03d}.png'),CvBridge().imgmsg_to_cv2(last_image,'bgr8'))
            next_progress = time.monotonic()+10
        if full_route:
            if r['goal_distance_m'] < 3. and r['speed_mps'] < 0.5:
                if arrival_start is None:
                    arrival_start = r['sim_s']
                if r['sim_s']-arrival_start >= 2.:
                    summary['stop_reason'] = 'goal_reached_and_stopped'
                    break
            else:
                arrival_start = None
            if r['route_distance_m'] > 8.:
                if departure_start is None:
                    departure_start = r['sim_s']
                if r['sim_s']-departure_start > 1.:
                    summary['stop_reason'] = 'route_departure_over_8m'
                    break
            else:
                departure_start = None
        if elapsed_sim >= max_sim:
            summary['stop_reason'] = 'simulation_time_limit' if full_route else 'observation_window_complete'
            break
    else:
        summary['stop_reason'] = 'wall_time_limit'
    active = [r for r in rows if r['phase']=='closed_loop']
    travelled = sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(active,active[1:]))
    offsets = [r['route_distance_m'] for r in active if r['route_distance_m'] is not None]
    summary.update({'observation_wall_s':time.monotonic()-observation_start,
        'observation_sim_s':active[-1]['sim_s']-active[0]['sim_s'] if active else 0,
        'distance_travelled_m':travelled, 'speed_max_mps':max((r['speed_mps'] for r in active),default=0),
        'route_distance_max_m':max(offsets,default=None), 'route_distance_rms_m':math.sqrt(sum(x*x for x in offsets)/len(offsets)) if offsets else None,
        'observation_topic_counts':{k:counts[k]-begin_counts.get(k,0) for k in counts}, 'route_points':len(route)})
    summary.update({'route_length_m':route_s[-1], 'route_progress_max_m':max((r['route_s_m'] for r in active),default=0),
        'goal_distance_min_m':min((r['goal_distance_m'] for r in active),default=None),
        'goal_distance_final_m':active[-1]['goal_distance_m'] if active else None,
        'speed_final_mps':active[-1]['speed_mps'] if active else None})
    success_reason = 'obstacle_passed_and_rejoined' if obstacle_s else ('goal_reached_and_stopped' if full_route else 'observation_window_complete')
    summary['task_completed'] = summary['stop_reason']==success_reason
    if obstacle_s:
        summary['obstacle_min_center_distance_m'] = min(math.hypot(r['x']-summary['obstacle']['ros_x'],r['y']-summary['obstacle']['ros_y']) for r in active)
    summary['passed'] = travelled>5 and counts['trajectory']>5 and counts['command']>20 and not collisions and not invalid_commands and not invalid_trajectories and summary['stop_reason']==success_reason
except Exception as exc:
    summary['error'] = str(exc)
    print('FAIL', exc, flush=True)
finally:
    phase = 'cleanup'
    # Stop route, helper and controller before destroying actors; keep Bridge alive for spawn cleanup services.
    cleanup_order = {'pnc':0,'waypoints':1,'ad_agent':2,'spawn':3,'bridge':4}
    for name,p,log in sorted(processes, key=lambda item: cleanup_order[item[0]]):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL)
                p.wait(timeout=5)
        log.close()
        if name == 'pnc' and 'vehicle_info' in last:
            try:
                cleanup_client = carla.Client('127.0.0.1',2000)
                cleanup_client.set_timeout(3.)
                cleanup_world = cleanup_client.get_world()
                cleanup_world.wait_for_tick(3.)
                ego = cleanup_world.get_actor(last['vehicle_info'].id)
                if ego is not None:
                    ego.apply_control(carla.VehicleControl(throttle=0.,brake=1.))
                    cleanup_world.wait_for_tick(3.)
                    summary['cleanup_brake_applied'] = True
            except Exception as exc:
                summary['cleanup_brake_error'] = str(exc)
    summary.update({'topic_counts':dict(counts),'invalid_trajectories':invalid_trajectories,
        'empty_trajectories':empty_trajectories,'invalid_commands':invalid_commands,'collisions':collisions,
        'nonfinite_commands':nonfinite_commands,'severe_command_violations':severe_command_violations,
        'command_violations_sample':command_violations,
        'trajectory_point_count_min':min(trajectory_sizes,default=0),'trajectory_point_count_max':max(trajectory_sizes,default=0)})
    if last_image is not None:
        try:
            cv2.imwrite(str(out/'last-camera-frame.png'),CvBridge().imgmsg_to_cv2(last_image,'bgr8'))
        except Exception as exc:
            summary['image_error'] = str(exc)
    if rows:
        with open(out/'telemetry.csv','w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    if commands:
        with open(out/'commands.csv','w') as f:
            writer=csv.DictWriter(f,fieldnames=list(commands[0]));writer.writeheader();writer.writerows(commands)
    if route:
        (out/'route.json').write_text(json.dumps(route))
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    node.destroy_node()
    rclpy.shutdown()
if not summary['passed']:
    raise SystemExit(1)
