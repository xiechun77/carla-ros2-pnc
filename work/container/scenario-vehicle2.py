#!/usr/bin/env python3
"""Inspect or drive vehicle_2 only. ROS Bridge remains the sole world ticker."""
import argparse
import math
import signal
import time
import carla
from agents.navigation.controller import VehiclePIDController


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'run'])
    parser.add_argument('--speed-kmh', type=float, default=10.0)
    args = parser.parse_args()
    if not 0 < args.speed_kmh <= 20:
        parser.error('--speed-kmh must be in (0, 20] for this learning scenario')
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    # Wait for the first streamed snapshot to populate the client actor cache.
    try:
        world.wait_for_tick(10.0)
    except RuntimeError as exc:
        raise RuntimeError("No simulation tick: resume the demo, then retry.") from exc
    # Match the role_name from the scene JSON, not CARLA transient actor ID.
    matches = [a for a in world.get_actors().filter('vehicle.*')
               if a.attributes.get('role_name') == 'vehicle_2']
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one vehicle_2; found %d. Wait for scene spawning.' % len(matches))
    vehicle = matches[0]

    def report():
        loc = vehicle.get_location()
        vel = vehicle.get_velocity()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        print('vehicle_2 actor=%d | ROS x=%.3f y=%.3f | speed=%.3f m/s | sim=%.2f s'
              % (vehicle.id, loc.x, -loc.y, speed, world.get_snapshot().timestamp.elapsed_seconds), flush=True)

    report()
    if args.action == 'status':
        return
    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    settings = world.get_settings()
    dt = settings.fixed_delta_seconds or 0.05
    controller = VehiclePIDController(
        vehicle,
        args_lateral=dict(K_P=1.95, K_I=0.05, K_D=0.2, dt=dt),
        args_longitudinal=dict(K_P=1.0, K_I=0.05, K_D=0.0, dt=dt),
        max_throttle=0.5, max_brake=1.0, max_steering=0.8)
    road_map = world.get_map()
    last_report = 0.0
    print('Lane-following PID drives vehicle_2 only. Target %.1f km/h.' % args.speed_kmh, flush=True)
    print('Learning scenario: no traffic-light or obstacle avoidance logic for background car.', flush=True)
    print('Ctrl+C applies brake. ROS Bridge remains the only world ticker.', flush=True)
    try:
        while not stop[0]:
            try:
                snapshot = world.wait_for_tick(1.0)
            except RuntimeError:
                continue  # Paused world: remain interruptible without advancing it.
            current = road_map.get_waypoint(vehicle.get_location())
            candidates = current.next(6.0)
            if not candidates:
                print('Road ends; stopping background vehicle.', flush=True)
                break
            # Stable branch choice for this simple learning scenario.
            target = sorted(candidates, key=lambda wp: (wp.road_id, wp.section_id, wp.lane_id, wp.s))[0]
            vehicle.apply_control(controller.run_step(args.speed_kmh, target))
            if snapshot.timestamp.elapsed_seconds - last_report >= 1.0:
                report()
                last_report = snapshot.timestamp.elapsed_seconds
    finally:
        if vehicle.is_alive:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
            print('vehicle_2 script stopped; brake applied.', flush=True)


if __name__ == '__main__':
    main()
