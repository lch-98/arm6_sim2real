"""
Damiao 모터 여러 개를 한 CAN 버스에서 다루는 클래스.

    bus = DamiaoBus(open_transport("socketcan", "can0"))
    m = bus.add(esc_id=0x01, mst_id=0x11, model="DM-J4340P-2EC")
    bus.read_limits(m)           # PMAX/VMAX/TMAX를 모터에서 읽어 변환 범위로 사용
    bus.enable(m)
    fb = bus.mit(m, q=0.0, dq=0.0, kp=10.0, kd=0.5, tau=0.0)   # 명령 1프레임 → 응답 1프레임
    bus.disable(m)

모터는 명령을 받을 때마다 응답 1프레임을 보낸다(요청-응답). 응답이 오지 않으면 None.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, Optional

from . import dm_protocol as dp
from .transport import Transport


@dataclass
class Motor:
    esc_id: int
    mst_id: int
    model: str
    limits: dp.Limits
    last: Optional[dp.Feedback] = None
    last_rx_time: float = 0.0
    params: Dict[int, float] = field(default_factory=dict)

    @property
    def name(self) -> str:
        return f"0x{self.esc_id:02X}({self.model})"


class DamiaoBus:
    def __init__(self, transport: Transport, reply_timeout: float = 0.01):
        self.tr = transport
        self.reply_timeout = reply_timeout
        self.motors: Dict[int, Motor] = {}

    # ------------------------------------------------------------------
    def add(self, esc_id: int, mst_id: int, model: str) -> Motor:
        m = Motor(esc_id, mst_id, model, dp.DEFAULT_LIMITS.get(model, dp.Limits(12.5, 30.0, 10.0)))
        self.motors[esc_id] = m
        return m

    def _owner(self, arb_id: int, data: bytes) -> Optional[Motor]:
        for m in self.motors.values():
            if m.mst_id != 0 and arb_id == m.mst_id:
                return m
        if arb_id == 0:   # MST_ID 0: 데이터 하위 4비트로 구분
            for m in self.motors.values():
                if (m.esc_id & 0xF) == (data[0] & 0xF):
                    return m
        return None

    def _wait_feedback(self, m: Motor, timeout: float) -> Optional[dp.Feedback]:
        end = time.perf_counter() + timeout
        while True:
            left = end - time.perf_counter()
            if left <= 0:
                return None
            fr = self.tr.recv(left)
            if fr is None:
                return None
            arb_id, data = fr
            owner = self._owner(arb_id, data)
            if owner is None or len(data) != 8:
                continue
            fb = dp.unpack_feedback(data, owner.limits)
            owner.last, owner.last_rx_time = fb, time.perf_counter()
            if owner is m:
                return fb

    # ------------------------------------------------------------------
    # 시스템 명령
    def _sys(self, m: Motor, cmd: dp.SysCmd, wait: float = 0.05) -> Optional[dp.Feedback]:
        self.tr.drain()
        self.tr.send(m.esc_id, dp.sys_cmd(cmd))
        return self._wait_feedback(m, wait)

    def enable(self, m: Motor):
        return self._sys(m, dp.SysCmd.ENABLE)

    def disable(self, m: Motor):
        return self._sys(m, dp.SysCmd.DISABLE)

    def save_zero(self, m: Motor):
        """현재 위치를 0으로 저장 (모터를 사용 안 함 상태에서)"""
        return self._sys(m, dp.SysCmd.SAVE_ZERO)

    def clear_error(self, m: Motor):
        return self._sys(m, dp.SysCmd.CLEAR_ERROR)

    # ------------------------------------------------------------------
    # 제어 명령
    def mit(self, m: Motor, q: float, dq: float, kp: float, kd: float, tau: float) -> Optional[dp.Feedback]:
        self.tr.send(m.esc_id, dp.pack_mit(q, dq, kp, kd, tau, m.limits))
        return self._wait_feedback(m, self.reply_timeout)

    def pos_vel(self, m: Motor, q: float, v_limit: float):
        self.tr.send(dp.POS_VEL_OFFSET + m.esc_id, dp.pack_pos_vel(q, v_limit))
        return self._wait_feedback(m, self.reply_timeout)

    def vel(self, m: Motor, dq: float):
        self.tr.send(dp.VEL_OFFSET + m.esc_id, dp.pack_vel(dq))
        return self._wait_feedback(m, self.reply_timeout)

    # ------------------------------------------------------------------
    # 레지스터
    def read_reg(self, m_or_id, rid: int, timeout: float = 0.1, retries: int = 3):
        esc_id = m_or_id.esc_id if isinstance(m_or_id, Motor) else int(m_or_id)
        for _ in range(retries):
            self.tr.drain()
            self.tr.send(dp.REGISTER_ID, dp.pack_reg(esc_id, dp.RegOp.READ, rid))
            end = time.perf_counter() + timeout
            while time.perf_counter() < end:
                fr = self.tr.recv(max(0.0, end - time.perf_counter()))
                if fr is None:
                    break
                _, data = fr
                if len(data) == 8 and data[2] == dp.RegOp.READ and data[3] == rid and \
                        (data[0] | (data[1] << 8)) == esc_id:
                    _, _, value = dp.unpack_reg_reply(data)
                    if isinstance(m_or_id, Motor):
                        m_or_id.params[rid] = value
                    return value
        return None

    def write_reg(self, m_or_id, rid: int, value, timeout: float = 0.1):
        """쓰기 후 응답(0x55)의 값을 돌려준다. 플래시 저장은 save_params()로 따로."""
        esc_id = m_or_id.esc_id if isinstance(m_or_id, Motor) else int(m_or_id)
        self.tr.drain()
        self.tr.send(dp.REGISTER_ID, dp.pack_reg(esc_id, dp.RegOp.WRITE, rid, value))
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            fr = self.tr.recv(max(0.0, end - time.perf_counter()))
            if fr is None:
                break
            _, data = fr
            if len(data) == 8 and data[2] == dp.RegOp.WRITE and data[3] == rid:
                return dp.unpack_reg_reply(data)[2]
        return None

    def save_params(self, m_or_id):
        """RAM의 레지스터를 플래시에 저장 (모터를 먼저 사용 안 함으로)"""
        esc_id = m_or_id.esc_id if isinstance(m_or_id, Motor) else int(m_or_id)
        self.tr.send(dp.REGISTER_ID, dp.pack_reg(esc_id, dp.RegOp.SAVE, 0x01))
        time.sleep(0.05)

    def read_limits(self, m: Motor) -> bool:
        """모터의 PMAX/VMAX/TMAX를 읽어 변환 범위로 사용. 실패하면 기본값 유지"""
        vals = [self.read_reg(m, rid) for rid in (dp.Rid.PMAX, dp.Rid.VMAX, dp.Rid.TMAX)]
        if all(v is not None and v > 0 for v in vals):
            m.limits = dp.Limits(*vals)
            return True
        return False

    def close(self):
        self.tr.close()
