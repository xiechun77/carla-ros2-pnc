#!/usr/bin/env python3
"""Spawn a crossing pedestrian near vehicle_8 to test EM planner ST speed planning.

ROS Bridge remains the sole world ticker (this client never calls world.tick()).
Coordinate note: ROS (x, y) <-> CARLA (x, -y).
Along the Town01 straight near vehicle_8:
  - Ego lane center is ROS x ~= 158.05, heading +90 deg (+y in ROS, -y in CARLA).
  - vehicle_8 is parked at ROS (153.76, -28.9), i.e. left offset l ~= +4.3 m.
  - Pedestrian spawns at ROS y = -25.0 (just past vehicle_8 to avoid overlap),
    and walks in +x direction (from left l > 0 to right l < 0) when ego approaches.
"""

import argparse
import math
import signal
import sys
import carla


PRESETS = {
    # 鬼探头让行：起步靠车道较近 (l=4.2m)，26m 处触发，1.8 m/s 横穿 -> ST 图中必须减速让行
    'yield': dict(start_l=4.2, trigger_dist=26.0, speed=1.8, cross_y=-25.0),
    # 远侧起步抢行：起步在左侧人行道边 (l=6.5m)，18m 处触发，1.5 m/s 横穿 -> ST 图倾向加速通过
    'pass': dict(start_l=6.5, trigger_dist=18.0, speed=1.5, cross_y=-25.0),
}

EGO_LANE_X = 158.05


def find_actor_by_role(world, pattern, role_name):
    matches = [a for a in world.get_actors().filter(pattern)
               if a.attributes.get('role_name') == role_name]
    return matches[0] if matches else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['yield', 'pass', 'status'], default='yield', nargs='?')
    parser.add_argument('--start-l', type=float, default=None, help='起步在自车车道左侧的距离 (m)')
    parser.add_argument('--trigger-dist', type=float, default=None, help='自车距横穿断面的触发距离 (m)')
    parser.add_argument('--speed', type=float, default=None, help='行人横穿速度 (m/s)')
    parser.add_argument('--cross-y', type=float, default=None, help='横穿断面的 ROS y 坐标 (默认 -25.0)')
    parser.add_argument('--end-l', type=float, default=-4.5, help='横穿结束时的右侧偏移 (m)')
    args = parser.parse_args()

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    try:
        world.wait_for_tick(10.0)
    except RuntimeError as exc:
        raise RuntimeError('No simulation tick: resume the demo, then retry.') from exc

    ego = find_actor_by_role(world, 'vehicle.*', 'ego_vehicle')
    if ego is None:
        raise RuntimeError('ego_vehicle not found. Wait for scene spawning.')

    if args.mode == 'status':
        loc = ego.get_location()
        vel = ego.get_velocity()
        v = math.hypot(vel.x, vel.y)
        walkers = list(world.get_actors().filter('walker.pedestrian.*'))
        print('ego ROS=(%.2f, %.2f) v=%.2f m/s | active walkers=%d' % (loc.x, -loc.y, v, len(walkers)), flush=True)
        for w in walkers:
            wl = w.get_location()
            wv = w.get_velocity()
            print('  walker id=%d ROS=(%.2f, %.2f) v=%.2f m/s' % (w.id, wl.x, -wl.y, math.hypot(wv.x, wv.y)), flush=True)
        return

    cfg = dict(PRESETS[args.mode])
    if args.start_l is not None:
        cfg['start_l'] = args.start_l
    if args.trigger_dist is not None:
        cfg['trigger_dist'] = args.trigger_dist
    if args.speed is not None:
        cfg['speed'] = args.speed
    if args.cross_y is not None:
        cfg['cross_y'] = args.cross_y

    start_x = EGO_LANE_X - cfg['start_l']
    end_x = EGO_LANE_X - args.end_l
    ros_y = cfg['cross_y']
    carla_y = -ros_y

    bp_lib = world.get_blueprint_library()
    walker_bps = bp_lib.filter('walker.pedestrian.*')
    if not walker_bps:
        raise RuntimeError('No walker.pedestrian.* blueprint available in CARLA.')
    walker_bp = walker_bps[0]
    if walker_bp.has_attribute('is_invincible'):
        walker_bp.set_attribute('is_invincible', 'false')

    spawn_tf = carla.Transform(
        carla.Location(x=start_x, y=carla_y, z=1.0),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )
    walker = world.try_spawn_actor(walker_bp, spawn_tf)
    if walker is None:
        # 稍微抬高 z 再试一次，避免路沿轻微干涉
        spawn_tf.location.z = 1.5
        walker = world.try_spawn_actor(walker_bp, spawn_tf)
    if walker is None:
        raise RuntimeError('Failed to spawn crossing walker at ROS (%.2f, %.2f).' % (start_x, ros_y))

    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))

    print(
        'Crossing walker spawned (id=%d) mode=%s | ROS start=(%.2f, %.2f) l=+%.1fm -> end_x=%.2f | '
        'trigger_dist=%.1fm speed=%.2f m/s' % (
            walker.id, args.mode, start_x, ros_y, cfg['start_l'], end_x, cfg['trigger_dist'], cfg['speed']
        ),
        flush=True,
    )
    print('Waiting for ego_vehicle to approach (Ctrl+C to stop and destroy walker)...', flush=True)

    state = 'waiting'  # waiting -> crossing -> done
    last_report = 0.0
    try:
        while not stop[0]:
            try:
                snapshot = world.wait_for_tick(1.0)
            except RuntimeError:
                continue
            if not walker.is_alive or not ego.is_alive:
                break

            ego_loc = ego.get_location()
            ego_ros_x, ego_ros_y = ego_loc.x, -ego_loc.y
            w_loc = walker.get_location()
            w_ros_x, w_ros_y = w_loc.x, -w_loc.y
            longitudinal_gap = ros_y - ego_ros_y  # 自车朝 +y 驶向横穿断面
            lateral_to_lane = abs(ego_ros_x - EGO_LANE_X)

            if state == 'waiting':
                walker.apply_control(carla.WalkerControl(direction=carla.Vector3D(1.0, 0.0, 0.0), speed=0.0))
                if lateral_to_lane < 6.0 and 0.0 < longitudinal_gap <= cfg['trigger_dist']:
                    state = 'crossing'
                    print('TRIGGER crossing! ego ROS=(%.2f, %.2f) gap=%.2fm sim=%.2fs' % (
                        ego_ros_x, ego_ros_y, longitudinal_gap, snapshot.timestamp.elapsed_seconds
                    ), flush=True)
            elif state == 'crossing':
                if w_ros_x >= end_x:
                    state = 'done'
                    walker.apply_control(carla.WalkerControl(direction=carla.Vector3D(1.0, 0.0, 0.0), speed=0.0))
                    print('Walker finished crossing at ROS x=%.2f (sim=%.2fs).' % (
                        w_ros_x, snapshot.timestamp.elapsed_seconds
                    ), flush=True)
                else:
                    walker.apply_control(carla.WalkerControl(
                        direction=carla.Vector3D(1.0, 0.0, 0.0),
                        speed=cfg['speed'],
                        jump=False,
                    ))
            else:
                walker.apply_control(carla.WalkerControl(direction=carla.Vector3D(1.0, 0.0, 0.0), speed=0.0))

            if snapshot.timestamp.elapsed_seconds - last_report >= 1.0:
                w_vel = walker.get_velocity()
                e_vel = ego.get_velocity()
                print('[%s] ego=(%.1f, %.1f) v=%.2f | walker=(%.1f, %.1f) l=%.2f v=%.2f | gap=%.1fm' % (
                    state, ego_ros_x, ego_ros_y, math.hypot(e_vel.x, e_vel.y),
                    w_ros_x, w_ros_y, EGO_LANE_X - w_ros_x, math.hypot(w_vel.x, w_vel.y),
                    longitudinal_gap,
                ), flush=True)
                last_report = snapshot.timestamp.elapsed_seconds
    finally:
        if walker.is_alive:
            walker.destroy()
            print('Crossing walker destroyed.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('ERROR:', exc, file=sys.stderr)
        sys.exit(1)
