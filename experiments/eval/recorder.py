#!/usr/bin/env python3
"""Unified closed-loop PnC evaluation recorder for CARLA + ROS2 Foxy.

Subscribes to standard ego/obstacle topics plus /planning_debug and /control_debug,
writes samples.csv, config.json, and computes summary.json on completion.
"""

import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from derived_object_msgs.msg import ObjectArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

try:
    from carla_msgs.msg import CarlaCollisionEvent
except ImportError:
    CarlaCollisionEvent = None


def percentile(values, p):
    if not values:
        return None
    ordered = sorted(values)
    k = (len(ordered) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return ordered[int(k)]
    return ordered[f] * (c - k) + ordered[c] * (k - f)


def rmse(values):
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))


def safe_round(val, digits=4):
    if val is None or not math.isfinite(val):
        return None
    return round(val, digits)


class EvalRecorder(Node):
    def __init__(self, args):
        super().__init__("pnc_eval_recorder")
        self.args = args
        self.out_dir = os.path.abspath(args.output_dir)
        os.makedirs(self.out_dir, exist_ok=True)

        self.samples = []
        self.planning_times = []
        self.control_times = []
        self.collision_count = 0

        self.ego_x = None
        self.ego_y = None
        self.ego_v = None
        self.ego_ax = None
        self.ego_ay = None
        self.ego_id = None
        self.prev_x = None
        self.prev_y = None
        self.travel_distance = 0.0

        self.min_obs_dist_latest = None
        self.min_obs_dist_overall = math.inf
        self.latest_plan_ms = None
        self.latest_path_qp_ok = None
        self.latest_speed_qp_ok = None

        self.start_wall = time.monotonic()
        self.first_sample_wall = None
        self.first_sample_t = None
        self.goal_stopped_wall = None
        self.done = False

        role = args.role_name
        self.create_subscription(Odometry, f"/carla/{role}/odometry", self._odom_cb, 20)
        self.create_subscription(Imu, f"/carla/{role}/imu", self._imu_cb, 20)
        self.create_subscription(ObjectArray, f"/carla/{role}/objects", self._objects_cb, 10)
        self.create_subscription(String, f"/carla/{role}/planning_debug", self._plan_debug_cb, 10)
        self.create_subscription(String, f"/carla/{role}/control_debug", self._ctrl_debug_cb, 50)
        if CarlaCollisionEvent is not None:
            self.create_subscription(
                CarlaCollisionEvent, f"/carla/{role}/collision", self._collision_cb, 10
            )

        self.create_timer(0.2, self._check_stop)

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = math.sqrt(
            msg.twist.twist.linear.x ** 2
            + msg.twist.twist.linear.y ** 2
            + msg.twist.twist.linear.z ** 2
        )
        if self.prev_x is not None:
            step = math.hypot(x - self.prev_x, y - self.prev_y)
            if step < 5.0:  # guard against respawn jump
                self.travel_distance += step
        self.prev_x = x
        self.prev_y = y
        self.ego_x = x
        self.ego_y = y
        self.ego_v = v

    def _imu_cb(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        if math.isfinite(ax) and abs(ax) < 20.0:
            self.ego_ax = ax
        if math.isfinite(ay) and abs(ay) < 20.0:
            self.ego_ay = ay

    def _objects_cb(self, msg):
        if self.ego_x is None:
            return
        min_d = math.inf
        for obj in msg.objects:
            dx = obj.pose.position.x - self.ego_x
            dy = obj.pose.position.y - self.ego_y
            d = math.hypot(dx, dy)
            # Exclude ego vehicle itself (distance ~0)
            if d < 0.25:
                continue
            if d < min_d:
                min_d = d
        if math.isfinite(min_d):
            self.min_obs_dist_latest = min_d
            if min_d < self.min_obs_dist_overall:
                self.min_obs_dist_overall = min_d

    def _collision_cb(self, _msg):
        self.collision_count += 1

    def _plan_debug_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        ct = data.get("compute_ms")
        if ct is None:
            ct = data.get("compute_time_ms")
        if isinstance(ct, (int, float)) and math.isfinite(ct):
            self.latest_plan_ms = float(ct)
            self.planning_times.append(float(ct))
        self.latest_path_qp_ok = data.get("path_qp_ok")
        self.latest_speed_qp_ok = data.get("speed_qp_ok")

    def _ctrl_debug_cb(self, msg):
        if self.done:
            return
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        t = data.get("t")
        if not isinstance(t, (int, float)) or not math.isfinite(t):
            return
        if self.first_sample_t is None:
            self.first_sample_t = float(t)
            self.first_sample_wall = time.monotonic()

        solve_ms = data.get("solve_time_ms")
        if isinstance(solve_ms, (int, float)) and math.isfinite(solve_ms):
            self.control_times.append(float(solve_ms))

        raw_ax = data.get("ego_ax", self.ego_ax)
        raw_ay = data.get("ego_ay", self.ego_ay)
        clean_ax = raw_ax if isinstance(raw_ax, (int, float)) and math.isfinite(raw_ax) and abs(raw_ax) < 15.0 else None
        clean_ay = raw_ay if isinstance(raw_ay, (int, float)) and math.isfinite(raw_ay) and abs(raw_ay) < 15.0 else None

        row = {
            "t_rel": float(t) - self.first_sample_t,
            "t_sim": float(t),
            "x": self.ego_x,
            "y": self.ego_y,
            "ego_v": data.get("ego_v"),
            "target_v": data.get("target_v"),
            "lateral_error": data.get("lateral_error"),
            "heading_error": data.get("heading_error"),
            "speed_error": data.get("speed_error"),
            "ego_ax": clean_ax,
            "ego_ay": clean_ay,
            "throttle": data.get("throttle"),
            "brake": data.get("brake"),
            "steer": data.get("steer"),
            "control_ms": solve_ms,
            "planning_ms": self.latest_plan_ms,
            "min_obs_dist": self.min_obs_dist_latest,
            "route_remaining": data.get("route_remaining"),
            "goal_stopped": bool(data.get("goal_stopped", False)),
        }
        self.samples.append(row)

        if row["goal_stopped"] and self.goal_stopped_wall is None:
            self.goal_stopped_wall = time.monotonic()

    def _check_stop(self):
        now = time.monotonic()
        if self.goal_stopped_wall is not None and (now - self.goal_stopped_wall) >= 1.0:
            self.get_logger().info("Goal reached; finishing evaluation recording.")
            self.done = True
        elif self.first_sample_wall is not None and (now - self.first_sample_wall) >= self.args.duration:
            self.get_logger().info(f"Reached active duration limit ({self.args.duration}s); finishing.")
            self.done = True
        elif self.first_sample_wall is None and (now - self.start_wall) >= (self.args.duration + 35.0):
            self.get_logger().warn("Timed out waiting for control_debug samples; finishing.")
            self.done = True

    def save_outputs(self):
        # 1. config.json
        config = {
            "created_at": datetime.now(timezone.utc).isoformat(),
            "scenario": self.args.scenario,
            "planner": self.args.planner,
            "controller": self.args.controller,
            "duration_limit_s": self.args.duration,
            "role_name": self.args.role_name,
        }
        with open(os.path.join(self.out_dir, "config.json"), "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)

        # 2. samples.csv
        fieldnames = [
            "t_rel", "t_sim", "x", "y", "ego_v", "target_v",
            "lateral_error", "heading_error", "speed_error",
            "ego_ax", "ego_ay", "throttle", "brake", "steer",
            "control_ms", "planning_ms", "min_obs_dist",
            "route_remaining", "goal_stopped",
        ]
        csv_path = os.path.join(self.out_dir, "samples.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.samples:
                writer.writerow(row)

        # 3. summary.json
        lat_errs = [r["lateral_error"] for r in self.samples if isinstance(r["lateral_error"], (int, float))]
        head_errs_deg = [
            math.degrees(r["heading_error"])
            for r in self.samples
            if isinstance(r["heading_error"], (int, float))
        ]
        spd_errs = [r["speed_error"] for r in self.samples if isinstance(r["speed_error"], (int, float))]
        ax_vals = [r["ego_ax"] for r in self.samples if isinstance(r["ego_ax"], (int, float))]

        # Compute longitudinal jerk & steer rate via finite differences
        lon_jerks = []
        steer_rates = []
        for i in range(1, len(self.samples)):
            dt = self.samples[i]["t_rel"] - self.samples[i - 1]["t_rel"]
            if dt <= 1e-4 or dt > 0.5:
                continue
            ax0, ax1 = self.samples[i - 1]["ego_ax"], self.samples[i]["ego_ax"]
            if isinstance(ax0, (int, float)) and isinstance(ax1, (int, float)):
                lon_jerks.append((ax1 - ax0) / dt)
            s0, s1 = self.samples[i - 1]["steer"], self.samples[i]["steer"]
            if isinstance(s0, (int, float)) and isinstance(s1, (int, float)):
                steer_rates.append((s1 - s0) / dt)

        duration_s = self.samples[-1]["t_rel"] if self.samples else 0.0
        goal_reached = any(r["goal_stopped"] for r in self.samples)

        summary = {
            "scenario": self.args.scenario,
            "planner": self.args.planner,
            "controller": self.args.controller,
            "sample_count": len(self.samples),
            "duration_s": safe_round(duration_s, 3),
            "travel_distance_m": safe_round(self.travel_distance, 3),
            "goal_reached": goal_reached,
            "collision_count": self.collision_count,
            "min_obstacle_dist_m": safe_round(
                self.min_obs_dist_overall if math.isfinite(self.min_obs_dist_overall) else None, 3
            ),
            "lateral_error_rmse_m": safe_round(rmse(lat_errs), 4),
            "lateral_error_max_m": safe_round(max((abs(v) for v in lat_errs), default=None), 4),
            "heading_error_rmse_deg": safe_round(rmse(head_errs_deg), 3),
            "heading_error_max_deg": safe_round(max((abs(v) for v in head_errs_deg), default=None), 3),
            "speed_error_rmse_mps": safe_round(rmse(spd_errs), 4),
            "max_accel_mps2": safe_round(max(ax_vals, default=None), 3),
            "max_decel_mps2": safe_round(min(ax_vals, default=None), 3),
            "lon_jerk_rms_mps3": safe_round(rmse(lon_jerks), 3),
            "steer_rate_rms_per_s": safe_round(rmse(steer_rates), 4),
            "planning_time_mean_ms": safe_round(
                sum(self.planning_times) / len(self.planning_times) if self.planning_times else None, 3
            ),
            "planning_time_p95_ms": safe_round(percentile(self.planning_times, 95), 3),
            "planning_time_max_ms": safe_round(max(self.planning_times, default=None), 3),
            "control_time_mean_ms": safe_round(
                sum(self.control_times) / len(self.control_times) if self.control_times else None, 3
            ),
            "control_time_p95_ms": safe_round(percentile(self.control_times, 95), 3),
            "control_time_max_ms": safe_round(max(self.control_times, default=None), 3),
        }
        summary_path = os.path.join(self.out_dir, "summary.json")
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        print(json.dumps(summary, indent=2, ensure_ascii=False))
        return summary_path


def parse_args():
    parser = argparse.ArgumentParser(description="CARLA+ROS2 PnC closed-loop evaluation recorder")
    parser.add_argument("--output-dir", required=True, help="Directory to store config/samples/summary")
    parser.add_argument("--scenario", default="cruise", help="Scenario label")
    parser.add_argument("--planner", default="em", help="Planner name")
    parser.add_argument("--controller", default="mpc", help="Controller name")
    parser.add_argument("--duration", type=float, default=60.0, help="Max recording duration (seconds)")
    parser.add_argument("--role-name", default="ego_vehicle", help="CARLA ego vehicle role name")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = EvalRecorder(args)

    def handle_signal(_sig, _frame):
        node.done = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.save_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
