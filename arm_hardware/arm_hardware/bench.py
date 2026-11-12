"""
1단계 모터 단품 벤치 도구 (docs/05_motor_bench.md).

    ros2 run arm_hardware dm_bench [연결 옵션] <명령> [명령 옵션]

연결 옵션
    --iface socketcan --channel can0        USB-CAN이 can0으로 잡힐 때 (기본)
    --iface damiao-serial --channel /dev/ttyACM0   Damiao USB-CAN 보드가 시리얼로 잡힐 때
    --fake                                  가상 모터로 실행 (하드웨어 없이 연습)

모터 지정: --joint joint2 (config/motors.yaml) 또는 --id 0x02 --mst 0x12 --model DM-J4340P-2EC

명령
    scan       ID 범위를 훑어 응답하는 모터와 주요 레지스터 표시
    info       한 모터의 레지스터·변환 범위·해상도
    set-id     ESC_ID / MST_ID 변경 후 플래시 저장 (한 대만 연결한 상태에서)
    set-mode   제어 모드를 MIT로 바꾸고 저장
    zero       현재 위치를 0으로 저장
    clear      오류 해제
    monitor    토크 없이(게인 0) 위치·속도·토크·온도 표시 — 손으로 돌려 보기, 절대 위치 확인
    hold       현재 위치 유지하며 토크 기록 — 레버+추 토크 보정
    move       부드러운 궤적으로 목표 위치까지 이동 후 유지
    friction   여러 속도로 등속 회전하며 평균 토크 → 쿨롱·점성 마찰
    latency    명령 → 응답 왕복 시간 통계
    sweep      기준 궤적의 1축 사인 스윕을 실물에 실행 → 추종 오차 (Gazebo 기준선과 비교)

모든 동작 명령은 Ctrl+C, 응답 끊김, 오류 상태, 토크·속도·범위 초과, 과열에서 즉시 모터를 사용 안 함으로 바꾼다.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import time
from contextlib import contextmanager
from datetime import datetime

from . import dm_protocol as dp
from .motor import DamiaoBus, Motor
from .transport import CanTransport, open_transport

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # ROS 없이 실행할 때
    get_package_share_directory = None

HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
def default_config_path():
    if get_package_share_directory is not None:
        try:
            return os.path.join(get_package_share_directory("arm_hardware"), "config", "motors.yaml")
        except Exception:
            pass
    return os.path.join(HERE, "..", "config", "motors.yaml")


def load_config(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


def find_reference_yaml():
    if get_package_share_directory is not None:
        try:
            return os.path.join(get_package_share_directory("arm_bringup"), "config", "reference_trajectories.yaml")
        except Exception:
            pass
    return os.path.join(HERE, "..", "..", "arm_bringup", "config", "reference_trajectories.yaml")


class Abort(Exception):
    pass


class Rate:
    def __init__(self, hz):
        self.dt = 1.0 / hz
        self.next = time.perf_counter()

    def sleep(self):
        self.next += self.dt
        d = self.next - time.perf_counter()
        if d > 0:
            time.sleep(d)
        else:
            self.next = time.perf_counter()


# ---------------------------------------------------------------------------
# 연결·안전
# ---------------------------------------------------------------------------
class Bench:
    def __init__(self, a):
        self.a = a
        self.cfg = load_config(a.config)
        self.fake_srv = None
        if a.fake:
            from .fake_motor import FakeMotor, FakeMotorServer
            ch = f"fake{os.getpid()}"
            motors = [FakeMotor(v["esc_id"], v["mst_id"], v["model"]) for v in self.cfg["motors"].values()]
            if a.fake_lever:
                mass, length = (float(x) for x in a.fake_lever.split(","))
                for fm in motors:
                    fm.lever_m, fm.lever_L = mass, length
            self.fake_srv = FakeMotorServer(CanTransport("virtual", ch), motors).start()
            self.bus = DamiaoBus(CanTransport("virtual", ch))
        else:
            self.bus = DamiaoBus(open_transport(a.iface, a.channel, a.bitrate))
        self.motor = None

    def close(self):
        self.bus.close()
        if self.fake_srv:
            self.fake_srv.stop()

    def select(self, read_limits=True) -> Motor:
        a = self.a
        if a.joint:
            v = self.cfg["motors"][a.joint]
            esc, mst, model = v["esc_id"], v["mst_id"], v["model"]
        else:
            esc, mst, model = a.id, a.mst, a.model
        if esc is None:
            sys.exit("모터를 지정하세요: --joint joint2 또는 --id 0x02 --mst 0x12 --model DM-J4340P-2EC")
        m = self.bus.add(esc, mst if mst is not None else esc + 0x10, model)
        if read_limits:
            if self.bus.read_limits(m):
                print(f"[{m.name}] 변환 범위(모터 레지스터): PMAX {m.limits.pmax:g}, VMAX {m.limits.vmax:g}, "
                      f"TMAX {m.limits.tmax:g}")
            else:
                print(f"[{m.name}] 레지스터 응답 없음 → 기본 변환 범위 사용 {m.limits}")
        self.motor = m
        return m

    def gains(self, m):
        g = self.cfg["bench"][m.model]
        kp = self.a.kp if self.a.kp is not None else g["kp"]
        kd = self.a.kd if self.a.kd is not None else g["kd"]
        return kp, kd

    @contextmanager
    def active(self, m: Motor):
        """사용 → (블록) → 반드시 사용 안 함"""
        g = self.cfg["bench"][m.model]
        self.tau_abort = self.a.tau_abort if self.a.tau_abort is not None else g["tau_abort"]
        self.vel_abort = g["vel_abort"]
        self.temp_stop = self.cfg["bench"]["temp_stop"]
        self.range_abort = self.a.range_abort if self.a.range_abort is not None else self.cfg["bench"]["range_abort"]
        fb = self.bus.enable(m)
        if fb is None:
            raise Abort("사용(enable) 응답 없음 — 배선, 전원, ID, 비트레이트 확인")
        if fb.is_fault:
            raise Abort(f"오류 상태: {fb.status_text} → `clear` 후 다시 시도")
        self.q_start = fb.q
        self.misses = 0
        try:
            yield fb
        except KeyboardInterrupt:
            print("\nCtrl+C — 정지")
        finally:
            for _ in range(3):
                if self.bus.disable(m) is not None:
                    break
            print(f"[{m.name}] 사용 안 함")

    def check(self, m: Motor, fb):
        """매 주기 안전 확인. 문제가 있으면 Abort"""
        if fb is None:
            self.misses += 1
            if self.misses >= 5:
                raise Abort("응답이 5번 연속 없음")
            return False
        self.misses = 0
        if fb.is_fault:
            raise Abort(f"모터 오류: {fb.status_text}")
        if abs(fb.tau) > self.tau_abort:
            raise Abort(f"토크 {fb.tau:+.2f} N·m 가 한계 {self.tau_abort} 초과")
        if abs(fb.dq) > self.vel_abort:
            raise Abort(f"속도 {fb.dq:+.2f} rad/s 가 한계 {self.vel_abort} 초과")
        if abs(fb.q - self.q_start) > self.range_abort:
            raise Abort(f"시작 위치에서 {fb.q - self.q_start:+.2f} rad 벗어남 (한계 {self.range_abort})")
        if max(fb.t_mos, fb.t_rotor) > self.temp_stop:
            raise Abort(f"온도 {max(fb.t_mos, fb.t_rotor)} °C 가 {self.temp_stop} °C 초과")
        return True


def out_path(a, kind, m):
    os.makedirs(a.out, exist_ok=True)
    return os.path.join(a.out, f"{datetime.now():%Y%m%d_%H%M%S}_{kind}_0x{m.esc_id:02X}.csv")


def confirm(msg, a):
    if a.yes:
        return True
    return input(f"{msg} [y/N] ").strip().lower() == "y"


def min_jerk(q0, q1, T, t):
    s = min(max(t / T, 0.0), 1.0)
    p = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    v = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T
    return q0 + (q1 - q0) * p, (q1 - q0) * v


def sine_sweep_point(A, T, cycles, t):
    """arm_bringup/scripts/run_reference.py 의 sine_sweep 과 같은 식 (중심 기준 변위, 속도)"""
    total, ramp, w_ = T * cycles, T / 2, 2 * math.pi / T
    t = min(max(t, 0.0), total)
    if t < ramp:
        w, dw = 0.5 * (1 - math.cos(math.pi * t / ramp)), 0.5 * math.pi / ramp * math.sin(math.pi * t / ramp)
    elif t > total - ramp:
        u = total - t
        w, dw = 0.5 * (1 - math.cos(math.pi * u / ramp)), -0.5 * math.pi / ramp * math.sin(math.pi * u / ramp)
    else:
        w, dw = 1.0, 0.0
    return A * w * math.sin(w_ * t), A * (dw * math.sin(w_ * t) + w * w_ * math.cos(w_ * t))


# ---------------------------------------------------------------------------
# 명령
# ---------------------------------------------------------------------------
REG_SHOW = [("ESC_ID", dp.Rid.ESC_ID, "0x{:02X}"), ("MST_ID", dp.Rid.MST_ID, "0x{:02X}"),
            ("CTRL_MODE", dp.Rid.CTRL_MODE, "{}"), ("PMAX", dp.Rid.PMAX, "{:g}"), ("VMAX", dp.Rid.VMAX, "{:g}"),
            ("TMAX", dp.Rid.TMAX, "{:g}"), ("감속비", dp.Rid.GR, "{:g}"), ("SW", dp.Rid.SW_VER, "{}"),
            ("TIMEOUT", dp.Rid.TIMEOUT, "{}")]


def fmt(v, f):
    if v is None:
        return "-"
    try:
        return f.format(int(v) if "X" in f or f == "{}" else v)
    except (ValueError, TypeError):
        return str(v)


def cmd_scan(b: Bench, a):
    lo, hi = (int(x, 0) for x in a.ids.split("-"))
    print("ESC_ID 범위 0x%02X~0x%02X 확인 중..." % (lo, hi))
    found = 0
    print(" | ".join(n for n, _, _ in REG_SHOW))
    for esc in range(lo, hi + 1):
        if b.bus.read_reg(esc, dp.Rid.ESC_ID, timeout=0.05, retries=1) is None:
            continue
        found += 1
        vals = [b.bus.read_reg(esc, rid, timeout=0.05) for _, rid, _ in REG_SHOW]
        print(" | ".join(fmt(v, f) for v, (_, _, f) in zip(vals, REG_SHOW)))
    print(f"응답한 모터 {found}대" if found else
          "응답 없음 — 전원, CAN_H/L 배선, 종단저항, 비트레이트(1 Mbps), 인터페이스 종류를 확인")
    mode_hint = {1: "MIT", 2: "위치-속도", 3: "속도", 4: "힘-위치"}
    print("CTRL_MODE: " + ", ".join(f"{k} {v}" for k, v in mode_hint.items()))


def cmd_info(b: Bench, a):
    m = b.select()
    for n, rid, f in REG_SHOW + [("저전압 보호", dp.Rid.UV_VALUE, "{:g}"), ("과전압 보호", dp.Rid.OV_VALUE, "{:g}"),
                                 ("과열 보호", dp.Rid.OT_VALUE, "{:g}")]:
        print(f"  {n:10s} {fmt(b.bus.read_reg(m, rid), f)}")
    r = dp.resolution(m.limits)
    print(f"  해상도: 위치 {r['q'] * 1e3:.3f} mrad, 속도 {r['dq']:.4f} rad/s, 토크 {r['tau']:.4f} N·m")
    fb = b.bus.mit(m, 0, 0, 0, 0, 0)
    if fb:
        print(f"  상태 {fb.status_text}, 위치 {fb.q:+.4f} rad, 온도 드라이버 {fb.t_mos} / 코일 {fb.t_rotor} °C")


def cmd_set_id(b: Bench, a):
    old, new, mst = a.id, a.new_id, a.new_mst
    if old is None or new is None or mst is None:
        sys.exit("--id(현재) --new-id --new-mst 가 필요합니다")
    print("버스에 모터를 **한 대만** 연결한 상태여야 합니다.")
    if not confirm(f"ESC_ID 0x{old:02X} → 0x{new:02X}, MST_ID → 0x{mst:02X} 로 바꾸고 플래시에 저장할까요?", a):
        return
    b.bus.tr.send(old, dp.sys_cmd(dp.SysCmd.DISABLE))
    time.sleep(0.05)
    r1 = b.bus.write_reg(old, dp.Rid.MST_ID, mst)
    r2 = b.bus.write_reg(old, dp.Rid.ESC_ID, new)
    print(f"  쓰기 응답: MST_ID {fmt(r1, '0x{:02X}')}, ESC_ID {fmt(r2, '0x{:02X}')}")
    b.bus.save_params(old)
    b.bus.save_params(new)
    print("저장 명령을 보냈습니다. 전원을 껐다 켠 뒤 `scan`으로 바뀐 ID를 확인하세요.")
    print("(바뀌지 않았으면 Damiao 디버깅 도구로 변경 — docs/05 참고)")


def cmd_set_mode(b: Bench, a):
    m = b.select(read_limits=False)
    b.bus.disable(m)
    r = b.bus.write_reg(m, dp.Rid.CTRL_MODE, int(dp.CtrlMode.MIT))
    print(f"CTRL_MODE 쓰기 응답: {fmt(r, '{}')} (1 = MIT)")
    if a.save and confirm("플래시에 저장할까요?", a):
        b.bus.save_params(m)
        print("저장 완료")


def cmd_zero(b: Bench, a):
    m = b.select()
    b.bus.disable(m)
    fb = b.bus.mit(m, 0, 0, 0, 0, 0)
    if fb:
        print(f"현재 위치 {fb.q:+.4f} rad")
    if confirm("현재 위치를 0으로 저장할까요?", a):
        fb = b.bus.save_zero(m)
        time.sleep(0.1)
        fb = b.bus.mit(m, 0, 0, 0, 0, 0)
        print(f"저장 후 위치 {fb.q:+.4f} rad" if fb else "응답 없음")


def cmd_clear(b: Bench, a):
    m = b.select(read_limits=False)
    fb = b.bus.clear_error(m)
    print(f"상태: {fb.status_text}" if fb else "응답 없음")


def cmd_monitor(b: Bench, a):
    m = b.select()
    print("게인 0 (토크 없음). 손으로 돌려 보세요. Ctrl+C로 종료")
    rate = Rate(a.hz)
    with b.active(m):
        t0, k = time.perf_counter(), 0
        while a.seconds <= 0 or time.perf_counter() - t0 < a.seconds:
            fb = b.bus.mit(m, 0, 0, 0, 0, 0)
            if fb and k % max(1, int(a.hz / 10)) == 0:
                print(f"\r q {fb.q:+8.4f} rad ({math.degrees(fb.q):+8.2f}°)  dq {fb.dq:+7.3f}  "
                      f"tau {fb.tau:+6.3f} N·m  T {fb.t_mos}/{fb.t_rotor} °C  {fb.status_text}   ", end="")
            if fb and fb.is_fault:
                raise Abort(fb.status_text)
            k += 1
            rate.sleep()
    print()


def run_loop(b, m, a, duration, target_fn, writer=None, label=""):
    """target_fn(t) -> (q_des, dq_des). 반환: 기록 리스트 [(t, q_des, q, dq, tau, T)]"""
    kp, kd = b.gains(m)
    rate = Rate(a.hz)
    t0 = time.perf_counter()
    rows = []
    while True:
        t = time.perf_counter() - t0
        if t > duration:
            break
        qd, dqd = target_fn(t)
        fb = b.bus.mit(m, qd, dqd, kp, kd, 0.0)
        if b.check(m, fb):
            row = (t, qd, fb.q, fb.dq, fb.tau, max(fb.t_mos, fb.t_rotor))
            rows.append(row)
            if writer:
                writer.writerow([label] + [f"{x:.6f}" if isinstance(x, float) else x for x in row])
        rate.sleep()
    return rows


def cmd_hold(b: Bench, a):
    m = b.select()
    kp, kd = b.gains(m)
    path = out_path(a, "hold", m)
    with open(path, "w", newline="") as f, b.active(m) as fb0:
        w = csv.writer(f)
        w.writerow(["label", "t", "q_des", "q", "dq", "tau", "temp"])
        q0 = fb0.q
        print(f"위치 {q0:+.4f} rad 유지 (kp {kp}, kd {kd}) {a.seconds:g} s")
        rows = run_loop(b, m, a, a.seconds, lambda t: (q0, 0.0), w, "hold")
    if rows:
        tail = [r for r in rows if r[0] > rows[-1][0] / 2]
        taus = [r[4] for r in tail]
        err = [r[1] - r[2] for r in tail]
        print(f"후반 평균 토크 {statistics.mean(taus):+.4f} N·m (표준편차 {statistics.pstdev(taus):.4f}), "
              f"위치 오차 평균 {statistics.mean(err) * 1e3:+.2f} mrad")
        if a.lever:
            mass, length = (float(x) for x in a.lever.split(","))
            q_rel = statistics.mean(r[2] for r in tail)
            print(f"  레버 기대 토크 m·g·L·cos(q) = {mass * 9.81 * length * math.cos(q_rel):+.4f} N·m "
                  f"(q = 0 이 레버 수평이 되도록 영점을 잡은 경우)")
    print("기록:", path)


def cmd_move(b: Bench, a):
    m = b.select()
    with b.active(m) as fb0:
        q0 = fb0.q
        q1 = a.to if not a.relative else q0 + a.to
        T = max(abs(q1 - q0) * 1.875 / a.vmax, 0.5)     # 최소 저크 최고 속도 = 1.875·Δq/T
        print(f"{q0:+.4f} → {q1:+.4f} rad, {T:.2f} s (최고 {a.vmax} rad/s), 이후 {a.hold:g} s 유지")
        rows = run_loop(b, m, a, T + a.hold, lambda t: min_jerk(q0, q1, T, t))
    if rows:
        print(f"최종 위치 {rows[-1][2]:+.4f} rad, 오차 {(q1 - rows[-1][2]) * 1e3:+.2f} mrad, 토크 {rows[-1][4]:+.3f} N·m")


def cmd_friction(b: Bench, a):
    m = b.select()
    speeds = [float(s) for s in a.speeds.split(",")]
    path = out_path(a, "friction", m)
    kd = a.kd if a.kd is not None else 1.0
    print(f"속도 제어(kp 0, kd {kd})로 각 속도 {a.seconds_per:g} s, 양방향. 출력축이 자유롭게 돌 수 있어야 합니다")
    result = []
    with open(path, "w", newline="") as f, b.active(m):
        w = csv.writer(f)
        w.writerow(["label", "t", "dq_des", "q", "dq", "tau", "temp"])
        b.range_abort = 1e9                         # 연속 회전이라 위치 범위 검사 끔
        for v in speeds:
            for sgn in (1, -1):
                vd = sgn * v
                rate, t0, rows = Rate(a.hz), time.perf_counter(), []
                while time.perf_counter() - t0 < a.seconds_per:
                    t = time.perf_counter() - t0
                    ramp = min(t / 0.5, 1.0)
                    fb = b.bus.mit(m, 0.0, vd * ramp, 0.0, kd, 0.0)
                    if b.check(m, fb):
                        rows.append((t, vd, fb.q, fb.dq, fb.tau))
                        w.writerow([f"v{vd:+.2f}", f"{t:.4f}", vd, f"{fb.q:.5f}", f"{fb.dq:.5f}", f"{fb.tau:.5f}",
                                    max(fb.t_mos, fb.t_rotor)])
                    rate.sleep()
                steady = [r for r in rows if r[0] > a.seconds_per * 0.4]
                if steady:
                    dq_m = statistics.mean(r[3] for r in steady)
                    tau_m = statistics.mean(r[4] for r in steady)
                    result.append((vd, dq_m, tau_m))
                    print(f"  목표 {vd:+5.2f} rad/s → 실제 {dq_m:+6.3f} rad/s, 평균 토크 {tau_m:+.4f} N·m")
    if len(result) >= 4:
        # tau = fc·sign(dq) + b·dq  최소제곱
        X = [(math.copysign(1, r[1]), r[1]) for r in result]
        y = [r[2] for r in result]
        s11 = sum(x[0] * x[0] for x in X); s12 = sum(x[0] * x[1] for x in X); s22 = sum(x[1] * x[1] for x in X)
        t1 = sum(x[0] * yy for x, yy in zip(X, y)); t2 = sum(x[1] * yy for x, yy in zip(X, y))
        det = s11 * s22 - s12 * s12
        if abs(det) > 1e-9:
            fc = (t1 * s22 - t2 * s12) / det
            bv = (s11 * t2 - s12 * t1) / det
            print(f"추정: 쿨롱 마찰 {fc:.4f} N·m, 점성 마찰 {bv:.4f} N·m·s/rad → URDF <dynamics friction damping>")
    print("기록:", path)


def cmd_latency(b: Bench, a):
    m = b.select()
    kp, kd = b.gains(m)
    dts, miss = [], 0
    with b.active(m) as fb0:
        rate = Rate(a.hz)
        for _ in range(a.n):
            t = time.perf_counter()
            fb = b.bus.mit(m, fb0.q, 0.0, kp, kd, 0.0)
            if fb is None:
                miss += 1
            else:
                dts.append((time.perf_counter() - t) * 1e3)
                b.check(m, fb)
            rate.sleep()
    if dts:
        s = sorted(dts)
        print(f"왕복 {len(dts)}회 (응답 없음 {miss}): 평균 {statistics.mean(s):.3f} ms, 중앙 {s[len(s) // 2]:.3f}, "
              f"99% {s[int(len(s) * 0.99) - 1]:.3f}, 최대 {s[-1]:.3f} ms")
        print("  PC 쪽 시간(USB·OS 지연 포함). 버스 위 시간은 `candump -td can0` 로 따로 확인")


def cmd_sweep(b: Bench, a):
    import yaml
    m = b.select()
    ref = yaml.safe_load(open(a.reference))
    sw = next((s for s in ref["sweeps"] if s["joint"] == a.ref_joint), None)
    if sw is None:
        sys.exit(f"{a.reference} 에 {a.ref_joint} 스윕이 없습니다")
    A, T, cyc = sw["amplitude"] * a.scale, sw["period"], sw["cycles"]
    path = out_path(a, f"sweep_{a.ref_joint}", m)
    print(f"{a.ref_joint} 스윕: 진폭 {A:.3f} rad, 주기 {T} s × {cyc}, 최고 속도 약 {A * 2 * math.pi / T:.2f} rad/s")
    with open(path, "w", newline="") as f, b.active(m) as fb0:
        w = csv.writer(f)
        w.writerow(["label", "t", "q_des", "q", "dq", "tau", "temp"])
        qc = fb0.q
        rows = run_loop(b, m, a, T * cyc + 1.0, lambda t: (qc + sine_sweep_point(A, T, cyc, t)[0],
                                                           sine_sweep_point(A, T, cyc, t)[1]), w, "sweep")
    if rows:
        err = [(r[1] - r[2]) * 1e3 for r in rows]
        rms = math.sqrt(sum(e * e for e in err) / len(err))
        print(f"추종 오차 RMS {rms:.1f} mrad, 최대 {max(abs(e) for e in err):.1f} mrad "
              f"(Gazebo 기준선 {a.ref_joint}: docs/results/00_sim_baseline/gazebo_summary.md)")
    print("기록:", path)


# ---------------------------------------------------------------------------
def build_parser():
    ap = argparse.ArgumentParser(prog="dm_bench", description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--iface", default="socketcan", help="socketcan | damiao-serial | slcan | gs_usb")
    ap.add_argument("--channel", default="can0", help="can0, vcan0, /dev/ttyACM0 ...")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--fake", action="store_true", help="가상 모터로 실행 (하드웨어 없이)")
    ap.add_argument("--fake-lever", help="가상 모터에 레버 추 'kg,m' (예 0.5,0.1)")
    ap.add_argument("--config", default=default_config_path())
    ap.add_argument("--joint", help="config/motors.yaml 의 이름 (joint1 ... gripper)")
    ap.add_argument("--id", type=lambda s: int(s, 0))
    ap.add_argument("--mst", type=lambda s: int(s, 0))
    ap.add_argument("--model", default="DM-J4340P-2EC")
    ap.add_argument("--kp", type=float)
    ap.add_argument("--kd", type=float)
    ap.add_argument("--tau-abort", type=float, help="이 토크를 넘으면 정지 [N·m]")
    ap.add_argument("--range-abort", type=float, help="시작 위치에서 이만큼 벗어나면 정지 [rad]")
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--out", default=os.path.expanduser("~/arm_bench_logs"))
    ap.add_argument("-y", "--yes", action="store_true", help="확인 질문 생략")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("scan"); p.add_argument("--ids", default="0x01-0x10")
    sub.add_parser("info")
    p = sub.add_parser("set-id"); p.add_argument("--new-id", type=lambda s: int(s, 0)); \
        p.add_argument("--new-mst", type=lambda s: int(s, 0))
    p = sub.add_parser("set-mode"); p.add_argument("--save", action="store_true")
    sub.add_parser("zero")
    sub.add_parser("clear")
    p = sub.add_parser("monitor"); p.add_argument("--seconds", type=float, default=0.0)
    p = sub.add_parser("hold"); p.add_argument("--seconds", type=float, default=10.0); \
        p.add_argument("--lever", help="레버 추 'kg,m' — 기대 토크를 같이 출력")
    p = sub.add_parser("move"); p.add_argument("--to", type=float, required=True); \
        p.add_argument("--relative", action="store_true"); p.add_argument("--vmax", type=float, default=0.5); \
        p.add_argument("--hold", type=float, default=1.0)
    p = sub.add_parser("friction"); p.add_argument("--speeds", default="0.2,0.5,1.0,2.0"); \
        p.add_argument("--seconds-per", type=float, default=3.0)
    p = sub.add_parser("latency"); p.add_argument("--n", type=int, default=2000)
    p = sub.add_parser("sweep"); p.add_argument("--ref-joint", required=True, help="기준 궤적의 관절 이름"); \
        p.add_argument("--scale", type=float, default=1.0, help="진폭 배율 (처음엔 0.3 정도로)"); \
        p.add_argument("--reference", default=find_reference_yaml())
    return ap


COMMANDS = {"scan": cmd_scan, "info": cmd_info, "set-id": cmd_set_id, "set-mode": cmd_set_mode, "zero": cmd_zero,
            "clear": cmd_clear, "monitor": cmd_monitor, "hold": cmd_hold, "move": cmd_move,
            "friction": cmd_friction, "latency": cmd_latency, "sweep": cmd_sweep}


def main(argv=None):
    a = build_parser().parse_args(argv)
    b = Bench(a)
    try:
        COMMANDS[a.cmd](b, a)
    except Abort as e:
        print(f"\n정지: {e}")
        sys.exit(2)
    finally:
        b.close()


if __name__ == "__main__":
    main()
