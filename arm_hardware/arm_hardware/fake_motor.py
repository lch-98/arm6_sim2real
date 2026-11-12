"""
가상 Damiao 모터. 실물 없이 벤치 도구와 하드웨어 인터페이스를 시험하기 위한 것.

같은 CAN 프로토콜로 명령을 받고 응답한다. 동역학은 단순 모델이다.
    J·ddq = tau_motor − b·dq − fc·sign(dq) − m·g·L·cos(q)      (q = 0 이 레버 수평)
    tau_motor = clamp(kp·(q_des − q) + kd·(dq_des − dq) + tau_ff, ±피크 토크)
J, b, fc 등은 **가상값**이다. 실제 모터 특성은 1단계 측정으로 얻는다.

단독 실행 (리눅스 가상 CAN):
    sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
    ros2 run arm_hardware dm_fake_motor --channel vcan0 --motor 1:0x11:DM-J4340P-2EC --motor 2:0x12:DM-J4310-2EC
"""
from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass, field

from . import dm_protocol as dp
from .transport import Transport, open_transport

PEAK = {"DM-J4340P-2EC": 27.0, "DM-J4310-2EC": 7.0, "DM-H3510": 0.45}
GEAR = {"DM-J4340P-2EC": 40.0, "DM-J4310-2EC": 10.0, "DM-H3510": 1.0}


@dataclass
class FakeMotor:
    esc_id: int
    mst_id: int
    model: str
    J: float = 0.004          # 출력축 기준 관성 [kg·m²] (가상값)
    b: float = 0.05           # 점성 마찰 [N·m·s/rad] (가상값)
    fc: float = 0.10          # 쿨롱 마찰 [N·m] (가상값)
    lever_m: float = 0.0      # 레버 끝 추 질량 [kg]
    lever_L: float = 0.0      # 레버 길이 [m]
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    enabled: bool = False
    status: int = dp.Status.DISABLED
    t_rotor: float = 30.0
    cmd: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)
    last_cmd_time: float = 0.0
    regs: dict = field(default_factory=dict)

    def __post_init__(self):
        lim = dp.DEFAULT_LIMITS[self.model]
        self.regs = {dp.Rid.ESC_ID: self.esc_id, dp.Rid.MST_ID: self.mst_id, dp.Rid.CTRL_MODE: int(dp.CtrlMode.MIT),
                     dp.Rid.PMAX: lim.pmax, dp.Rid.VMAX: lim.vmax, dp.Rid.TMAX: lim.tmax,
                     dp.Rid.SW_VER: 0, dp.Rid.HW_VER: 0, dp.Rid.GR: GEAR[self.model], dp.Rid.TIMEOUT: 0,
                     dp.Rid.UV_VALUE: 15.0, dp.Rid.OV_VALUE: 32.0, dp.Rid.OT_VALUE: 80.0}

    @property
    def limits(self) -> dp.Limits:
        return dp.Limits(self.regs[dp.Rid.PMAX], self.regs[dp.Rid.VMAX], self.regs[dp.Rid.TMAX])

    def step(self, dt: float):
        if self.enabled:
            qd, dqd, kp, kd, tff = self.cmd
            t = kp * (qd - self.q) + kd * (dqd - self.dq) + tff
            pk = min(PEAK[self.model], self.limits.tmax)
            self.tau = max(-pk, min(pk, t))
        else:
            self.tau = 0.0
        grav = self.lever_m * 9.81 * self.lever_L * math.cos(self.q)
        drive = self.tau - self.b * self.dq - grav
        # 쿨롱 마찰: 정지 중에는 구동 토크가 fc를 넘을 때만 움직임
        if abs(self.dq) < 1e-4 and abs(drive) <= self.fc:
            self.dq, acc = 0.0, 0.0
        else:
            acc = (drive - self.fc * math.copysign(1.0, self.dq if abs(self.dq) >= 1e-4 else drive)) / self.J
        self.dq += acc * dt
        self.q += self.dq * dt
        # 코일 온도 (가상 1차 모델): 정격 토크에서 약 60 °C로 수렴
        rated = PEAK[self.model] / 3.0
        target = 25.0 + 35.0 * (self.tau / rated) ** 2
        self.t_rotor += (target - self.t_rotor) * dt / 60.0
        if self.t_rotor > self.regs[dp.Rid.OT_VALUE] and self.enabled:
            self.enabled, self.status = False, dp.Status.ROTOR_OVER_TEMP
        to = self.regs[dp.Rid.TIMEOUT]
        if to and self.enabled and time.perf_counter() - self.last_cmd_time > to * 50e-6:
            self.enabled, self.status = False, dp.Status.LOST_COMM

    def feedback(self) -> bytes:
        return dp.pack_feedback(self.esc_id, self.status, self.q, self.dq, self.tau,
                                int(self.t_rotor - 3), int(self.t_rotor), self.limits)


class FakeMotorServer:
    def __init__(self, transport: Transport, motors, dt: float = 0.0005):
        self.tr = transport
        self.motors = {m.esc_id: m for m in motors}
        self.dt = dt
        self._stop = threading.Event()
        self._thread = None

    def _reply(self, m: FakeMotor, data: bytes):
        self.tr.send(m.mst_id, data)

    def handle(self, arb_id: int, data: bytes):
        now = time.perf_counter()
        if arb_id == dp.REGISTER_ID:
            esc = data[0] | (data[1] << 8)
            m = self.motors.get(esc)
            if m is None:
                return
            op, rid = data[2], data[3]
            if op == dp.RegOp.READ and rid in m.regs:
                self._reply(m, dp.pack_reg(esc, dp.RegOp.READ, rid, m.regs[rid]))
            elif op == dp.RegOp.WRITE:
                _, _, val = dp.unpack_reg_reply(data)
                m.regs[rid] = val
                self._reply(m, dp.pack_reg(esc, dp.RegOp.WRITE, rid, val))
            return
        base, off = arb_id & 0x0FF, arb_id & 0xF00
        m = self.motors.get(base)
        if m is None:
            return
        m.last_cmd_time = now
        if off == 0 and data[:7] == b"\xff" * 7 and data[7] in (0xFC, 0xFD, 0xFE, 0xFB):
            c = data[7]
            if c == dp.SysCmd.ENABLE and m.status in (dp.Status.DISABLED, dp.Status.ENABLED):
                m.enabled, m.status = True, dp.Status.ENABLED
                m.cmd = (m.q, 0.0, 0.0, 0.0, 0.0)
            elif c == dp.SysCmd.DISABLE:
                m.enabled, m.status = False, dp.Status.DISABLED if m.status == dp.Status.ENABLED else m.status
            elif c == dp.SysCmd.SAVE_ZERO and not m.enabled:
                m.q = 0.0
            elif c == dp.SysCmd.CLEAR_ERROR:
                m.status = dp.Status.DISABLED
                m.enabled = False
            self._reply(m, m.feedback())
            return
        if off == 0:
            m.cmd = dp.unpack_mit(data, m.limits)
        self._reply(m, m.feedback())

    def run(self):
        last = time.perf_counter()
        while not self._stop.is_set():
            fr = self.tr.recv(self.dt)
            now = time.perf_counter()
            while now - last >= self.dt:
                for m in self.motors.values():
                    m.step(self.dt)
                last += self.dt
            if fr is not None:
                self.handle(*fr)

    def start(self):
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self.tr.close()


def parse_motor(spec: str) -> FakeMotor:
    """'1:0x11:DM-J4340P-2EC[:lever_m:lever_L]'"""
    parts = spec.split(":")
    m = FakeMotor(int(parts[0], 0), int(parts[1], 0), parts[2])
    if len(parts) >= 5:
        m.lever_m, m.lever_L = float(parts[3]), float(parts[4])
    return m


def main(argv=None):
    ap = argparse.ArgumentParser(description="가상 Damiao 모터 (실물 없이 시험)")
    ap.add_argument("--iface", default="socketcan", help="socketcan | virtual | ...")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--motor", action="append", default=[],
                    help="ESC_ID:MST_ID:모델[:레버추kg:레버길이m]  (여러 번 지정 가능)")
    a = ap.parse_args(argv)
    specs = a.motor or ["1:0x11:DM-J4340P-2EC", "2:0x12:DM-J4310-2EC"]
    srv = FakeMotorServer(open_transport(a.iface, a.channel), [parse_motor(s) for s in specs])
    print(f"가상 모터 {len(specs)}개 실행 중 ({a.iface}:{a.channel}). Ctrl+C로 종료")
    for s in specs:
        print("  ", s)
    try:
        srv.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
