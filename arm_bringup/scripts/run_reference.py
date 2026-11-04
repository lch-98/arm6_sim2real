#!/usr/bin/env python3
"""
기준 궤적 세트(config/reference_trajectories.yaml)를 컨트롤러로 보내고, 필요하면 rosbag으로 기록한다.

  # mock / 실물 (벽시계 시간)
  ros2 run arm_bringup run_reference.py --set all --bag ~/arm_logs/mock_baseline
  # Gazebo (시뮬레이션 시간)
  ros2 run arm_bringup run_reference.py --set all --sim-time --bag ~/arm_logs/gazebo_baseline

  --set   all | sweeps | static | pick | home     (여러 개는 쉼표로: --set sweeps,static)
  --only  특정 관절 스윕만 (예: --only joint2,joint3)
  --speed 시간 배율. 0.5 → 모든 동작을 2배 느리게 (실물 첫 구동용)

구간마다 /reference_events (std_msgs/String, JSON)에 시작·끝 표시를 남긴다.
분석 스크립트(analyze_baseline.py)는 이 표시로 기록을 구간별로 나눈다.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

RECORD_TOPICS = [
    "/joint_states", "/arm_controller/state", "/gripper_controller/state",
    "/reference_events", "/robot_description", "/clock",
]


def to_duration(t):
    sec = int(math.floor(t))
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def quintic(q0, q1, T, dt):
    """정지 → 정지 5차 다항식 보간. (시간, 위치, 속도, 가속도) 목록"""
    pts = []
    n = max(1, int(math.ceil(T / dt)))
    for k in range(1, n + 1):
        t = min(k * dt, T)
        s = t / T
        p = 10 * s**3 - 15 * s**4 + 6 * s**5
        v = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
        a = (60 * s - 180 * s**2 + 120 * s**3) / T**2
        pts.append((t, [a0 + (b0 - a0) * p for a0, b0 in zip(q0, q1)],
                    [(b0 - a0) * v for a0, b0 in zip(q0, q1)],
                    [(b0 - a0) * a for a0, b0 in zip(q0, q1)]))
    return pts


def sine_sweep(q_center, idx, A, T, cycles, dt):
    """q = q_c + A·w(t)·sin(2πt/T). w: 처음·마지막 반주기 동안 코사인 램프 (속도·가속도 연속)"""
    total = T * cycles
    ramp = T / 2
    pts = []
    n = int(math.ceil(total / dt))
    w_ = 2 * math.pi / T
    for k in range(1, n + 1):
        t = min(k * dt, total)
        if t < ramp:
            w, dw = 0.5 * (1 - math.cos(math.pi * t / ramp)), 0.5 * math.pi / ramp * math.sin(math.pi * t / ramp)
        elif t > total - ramp:
            u = total - t
            w, dw = 0.5 * (1 - math.cos(math.pi * u / ramp)), -0.5 * math.pi / ramp * math.sin(math.pi * u / ramp)
        else:
            w, dw = 1.0, 0.0
        q = list(q_center)
        v = [0.0] * len(q)
        q[idx] += A * w * math.sin(w_ * t)
        v[idx] = A * (dw * math.sin(w_ * t) + w * w_ * math.cos(w_ * t))
        pts.append((t, q, v, None))
    return pts


class Runner(Node):
    def __init__(self, args, cfg):
        overrides = [Parameter("use_sim_time", value=True)] if args.sim_time else []
        super().__init__("reference_runner", parameter_overrides=overrides)
        self.cfg = cfg
        self.speed = args.speed
        self.dt = 1.0 / cfg["sample_rate_hz"]
        self.joints = cfg["joints"]
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ev_pub = self.create_publisher(String, "/reference_events", 10)
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")

    # ---------- 공통 ----------
    def _js(self, msg):
        self.js = msg

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, segment, phase, **extra):
        d = dict(segment=segment, phase=phase, t=self.now(), **extra)
        self.ev_pub.publish(String(data=json.dumps(d)))
        self.get_logger().info(f"[{phase:5s}] {segment}")

    def wait(self, seconds):
        """노드 시계(시뮬레이션 시간 포함) 기준으로 기다림"""
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def current_q(self):
        while rclpy.ok() and self.js is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        m = dict(zip(self.js.name, self.js.position))
        return [m[j] for j in self.joints]

    def send(self, client, joints, pts, label):
        if not client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(f"액션 서버 없음: {label} (컨트롤러가 켜져 있는지 확인)")
        traj = JointTrajectory(joint_names=joints)
        for t, q, v, a in pts:
            p = JointTrajectoryPoint(positions=[float(x) for x in q], time_from_start=to_duration(t / self.speed))
            if v is not None:
                p.velocities = [float(x) * self.speed for x in v]
            if a is not None:
                p.accelerations = [float(x) * self.speed**2 for x in a]
            traj.points.append(p)
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            raise RuntimeError(f"궤적 거부됨: {label}")
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        if res.error_code != 0:
            raise RuntimeError(f"궤적 실패: {label} (error_code={res.error_code} {res.error_string})")

    def move_to(self, q_target, label, T=None):
        T = T or self.cfg["move_time"]
        self.send(self.arm, self.joints, quintic(self.current_q(), q_target, T, self.dt), label)

    def gripper(self, value, label):
        g = self.cfg["gripper"]
        target = g[value] if isinstance(value, str) else float(value)
        if self.js is not None and "finger_left_joint" in self.js.name:
            cur = dict(zip(self.js.name, self.js.position))["finger_left_joint"]
        else:
            cur = 0.0
        # 두 핑거에 같은 값 (실물은 모터 1개)
        self.send(self.grip, ["finger_left_joint", "finger_right_joint"],
                  quintic([cur, cur], [target, target], g["move_time"], self.dt), label)

    def pose(self, q):
        return self.cfg["ready_pose"] if q == "ready_pose" else (self.cfg["home_pose"] if q == "home_pose" else q)

    # ---------- 세트 ----------
    def run_home(self):
        self.event("home", "start")
        self.move_to(self.cfg["home_pose"], "home")
        self.event("home", "end")

    def run_sweeps(self, only):
        ready = self.cfg["ready_pose"]
        self.move_to(ready, "ready")
        for sw in self.cfg["sweeps"]:
            j = sw["joint"]
            if only and j not in only:
                continue
            seg = f"sweep/{j}"
            pts = sine_sweep(ready, self.joints.index(j), sw["amplitude"], sw["period"], sw["cycles"], self.dt)
            self.event(seg, "start", amplitude=sw["amplitude"], period=sw["period"] / self.speed)
            self.send(self.arm, self.joints, pts, seg)
            self.event(seg, "end")
            self.wait(0.5)

    def run_static(self):
        st = self.cfg["static_poses"]
        ready = self.cfg["ready_pose"]
        for k, q in enumerate(st["poses"]):
            seg = f"static/{k:02d}"
            self.move_to(q, f"{seg} 이동")
            self.event(seg, "start", q=q, average_last=st["average_last"])
            self.wait(st["hold_time"])
            self.event(seg, "end")
            self.move_to(ready, "ready")

    def run_pick(self):
        wps = self.cfg["pick_place"]["waypoints"]
        has_gripper = self.grip.wait_for_server(timeout_sec=5.0)
        self.event("pick", "start")
        last_grip = None
        prev_q = None
        for wp in wps:
            q = self.pose(wp["q"])
            if q != prev_q:                       # 같은 자세가 이어지면(집기·놓기) 이동 생략
                self.move_to(q, f"pick/{wp['name']}")
                prev_q = q
            if has_gripper and wp.get("gripper") and wp["gripper"] != last_grip:
                self.gripper(wp["gripper"], f"pick/{wp['name']} gripper {wp['gripper']}")
                last_grip = wp["gripper"]
            self.event(f"pick/{wp['name']}", "mark")
        self.event("pick", "end")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", default="all")
    ap.add_argument("--only", default="")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--sim-time", action="store_true", help="Gazebo 사용 시 지정 (/clock 기준)")
    ap.add_argument("--bag", default="", help="이 경로로 ros2 bag 기록 (폴더가 이미 있으면 실패)")
    ap.add_argument("--config", default="", help="기준 궤적 yaml (기본: arm_bringup/config/reference_trajectories.yaml)")
    args, ros_args = ap.parse_known_args()
    if not 0.05 <= args.speed <= 1.0:
        sys.exit("--speed 는 0.05 ~ 1.0 사이로 지정")

    path = args.config or os.path.join(get_package_share_directory("arm_bringup"), "config", "reference_trajectories.yaml")
    with open(path) as f:
        cfg = yaml.safe_load(f)
    sets = ["home", "sweeps", "static", "pick"] if args.set == "all" else args.set.split(",")
    only = [s for s in args.only.split(",") if s]

    bag = None
    if args.bag:
        out = os.path.expanduser(args.bag)
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        cmd = ["ros2", "bag", "record", "-o", out] + RECORD_TOPICS
        if args.sim_time:
            cmd += ["--use-sim-time"]
        bag = subprocess.Popen(cmd, start_new_session=True)
        time.sleep(3.0)  # 기록기가 토픽을 구독할 시간

    rclpy.init(args=ros_args)
    node = Runner(args, cfg)
    if bag is not None:
        # 기록기가 /reference_events를 구독하기 전에 보낸 표시는 기록되지 않으므로 연결될 때까지 기다림
        t_end = time.time() + 15.0
        while node.ev_pub.get_subscription_count() == 0 and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.ev_pub.get_subscription_count() == 0:
            node.get_logger().warn("기록기가 /reference_events를 구독하지 않음 — 구간 표시가 빠질 수 있음")
        time.sleep(0.5)
    try:
        node.event("run", "start", sets=sets, speed=args.speed, config_version=cfg.get("version"))
        for s in sets:
            if s == "home":
                node.run_home()
            elif s == "sweeps":
                node.run_sweeps(only)
            elif s == "static":
                node.run_static()
            elif s == "pick":
                node.run_pick()
            else:
                raise ValueError(f"알 수 없는 세트: {s}")
        node.run_home()
        node.event("run", "end")
        node.wait(1.0)
    except KeyboardInterrupt:
        node.get_logger().warn("중단됨 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if bag is not None:
            time.sleep(1.0)
            os.killpg(bag.pid, signal.SIGINT)
            bag.wait(timeout=20)
            print(f"기록 완료: {os.path.expanduser(args.bag)}")


if __name__ == "__main__":
    main()
