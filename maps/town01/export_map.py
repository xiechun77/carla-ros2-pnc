import carla,json,csv,math
from pathlib import Path
from agents.navigation.global_route_planner import GlobalRoutePlanner
out=Path('/out')
c=carla.Client('127.0.0.1',2000); c.set_timeout(20)
w=c.get_world(); m=w.get_map()
if not m.name.endswith('Town01'):
    raise RuntimeError('Expected Town01, got '+m.name)
def pose(t):
    return dict(x=round(t.location.x,4),y=round(-t.location.y,4),z=round(t.location.z,4),yaw=round(-t.rotation.yaw,4))
lanes={}
points=[]
for wp in m.generate_waypoints(1.0):
    p=pose(wp.transform)
    p.update(road=wp.road_id,lane=wp.lane_id,section=wp.section_id,s=wp.s,width=wp.lane_width)
    points.append(p)
    lanes.setdefault((wp.road_id,wp.section_id,wp.lane_id),[]).append(p)
lines=[sorted(v,key=lambda p:p['s']) for v in lanes.values()]
start=carla.Location(x=127.4,y=195.4,z=2)
goal=m.get_spawn_points()[150]
route=GlobalRoutePlanner(m,sampling_resolution=1).trace_route(start,goal.location)
rp=[]; dist=0
for i,(wp,option) in enumerate(route):
    p=pose(wp.transform)
    if rp: dist+=math.hypot(p['x']-rp[-1]['x'],p['y']-rp[-1]['y'])
    p.update(index=i,distance_m=round(dist,3),road=wp.road_id,lane=wp.lane_id)
    rp.append(p)
config=json.loads(Path('/objects.json').read_text())
vehicles=[dict(id=o['id'],**o['spawn_point']) for o in config['objects'] if o.get('type','').startswith('vehicle.') and 'spawn_point' in o]
data=dict(map=m.name,coordinate_system='ROS: x=CARLA.x, y=-CARLA.y, yaw=-CARLA.yaw (degrees)',route_source='Reconstructed default route; not live ROS capture',start=dict(x=127.4,y=-195.4,z=2,yaw=180),goal=pose(goal),lanes=lines,route=rp,spawns=[dict(index=i,**pose(t)) for i,t in enumerate(m.get_spawn_points())],vehicles=vehicles)
(out/'map-data.json').write_text(json.dumps(data))
for name,rows in [('global-route.csv',rp),('spawn-points.csv',data['spawns']),('lane-points.csv',points)]:
    with (out/name).open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
print(json.dumps(dict(map=m.name,lane_points=len(points),route_points=len(rp),length_m=dist,spawn_points=len(data['spawns']))))
