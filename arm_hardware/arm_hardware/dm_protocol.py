"""
Damiao 액추에이터 CAN 프로토콜 (ROS 의존 없음, 순수 함수).

프레임 정리 (CAN 2.0 표준 ID, 8바이트):
  명령 ID
    MIT 제어            : ESC_ID            (모터 CAN ID, 예 0x01)
    위치-속도 제어       : 0x100 + ESC_ID    (float32 위치, float32 속도 상한, 리틀엔디언)
    속도 제어            : 0x200 + ESC_ID    (float32 속도)
    시스템 명령          : ESC_ID            (FF×7 + FC 사용 / FD 해제 / FE 영점 저장 / FB 오류 해제)
    레지스터 읽기·쓰기    : 0x7FF             ([id_l, id_h, 0x33 읽기 | 0x55 쓰기 | 0xAA 플래시 저장, RID, 값 4바이트])
  응답 ID
    MST_ID (마스터 ID, 예 0x11). 데이터 0번 바이트 하위 4비트 = ESC_ID 하위 4비트, 상위 4비트 = 상태 코드

MIT 명령 비트 배치: 위치 16비트, 속도·kp·kd·토크 각 12비트
  [p15:8][p7:0][v11:4][v3:0|kp11:8][kp7:0][kd11:4][kd3:0|t11:8][t7:0]
응답 비트 배치: [상태|id][p15:8][p7:0][v11:4][v3:0|t11:8][t7:0][T_mos][T_rotor]

정수 ↔ 실수 변환 범위: 위치 ±PMAX, 속도 ±VMAX, 토크 ±TMAX (모터 레지스터 21·22·23),
kp 0~500, kd 0~5.

참고: Damiao 공개 자료와 DM_Control_Python(MIT License, cmjang)의 프레임 정의.
"""
from __future__ import annotations

import enum
import struct
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# 상수
# ---------------------------------------------------------------------------
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

POS_VEL_OFFSET = 0x100
VEL_OFFSET = 0x200
REGISTER_ID = 0x7FF


class SysCmd(enum.IntEnum):
    ENABLE = 0xFC
    DISABLE = 0xFD
    SAVE_ZERO = 0xFE
    CLEAR_ERROR = 0xFB


class RegOp(enum.IntEnum):
    READ = 0x33
    WRITE = 0x55
    SAVE = 0xAA


class Rid(enum.IntEnum):
    """레지스터 번호 (일부). 7~10, 13~16, 35~36은 uint32, 나머지는 float32."""
    UV_VALUE = 0      # 저전압 보호 [V]
    KT_VALUE = 1
    OT_VALUE = 2      # 과열 보호 [°C]
    OC_VALUE = 3      # 과전류 보호
    MAX_SPD = 6
    MST_ID = 7        # 응답(마스터) ID
    ESC_ID = 8        # 모터 CAN ID
    TIMEOUT = 9       # 통신 끊김 보호 (1 = 50 µs, 0 = 끔)
    CTRL_MODE = 10    # 1 MIT, 2 위치-속도, 3 속도, 4 힘-위치
    HW_VER = 13
    SW_VER = 14
    SN = 15
    GR = 20           # 감속비
    PMAX = 21
    VMAX = 22
    TMAX = 23
    OV_VALUE = 29     # 과전압 보호 [V]
    CAN_BR = 35


UINT32_RIDS = set(range(7, 11)) | set(range(13, 17)) | {35, 36}


class CtrlMode(enum.IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    TORQUE_POS = 4


class Status(enum.IntEnum):
    DISABLED = 0x0
    ENABLED = 0x1
    OVER_VOLTAGE = 0x8
    UNDER_VOLTAGE = 0x9
    OVER_CURRENT = 0xA
    MOS_OVER_TEMP = 0xB
    ROTOR_OVER_TEMP = 0xC
    LOST_COMM = 0xD
    OVERLOAD = 0xE


STATUS_TEXT = {
    Status.DISABLED: "사용 안 함",
    Status.ENABLED: "사용 중",
    Status.OVER_VOLTAGE: "과전압",
    Status.UNDER_VOLTAGE: "저전압",
    Status.OVER_CURRENT: "과전류",
    Status.MOS_OVER_TEMP: "드라이버(MOS) 과열",
    Status.ROTOR_OVER_TEMP: "모터 코일 과열",
    Status.LOST_COMM: "통신 끊김",
    Status.OVERLOAD: "과부하",
}


@dataclass(frozen=True)
class Limits:
    """MIT 정수 변환 범위. 모터 레지스터 PMAX/VMAX/TMAX와 같아야 한다."""
    pmax: float
    vmax: float
    tmax: float


# 공장 기본값 (Seeed DAMIAO 43 시리즈 자료, DM_Control_Python). 실제 값은 레지스터를 읽어 확인한다.
DEFAULT_LIMITS = {
    "DM-J4310-2EC": Limits(12.5, 30.0, 10.0),
    "DM-J4340P-2EC": Limits(12.5, 8.0, 28.0),
    "DM-H3510": Limits(12.5, 280.0, 1.0),
}


# ---------------------------------------------------------------------------
# 정수 ↔ 실수
# ---------------------------------------------------------------------------
def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = min(max(x, x_min), x_max)
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def uint_to_float(u: int, x_min: float, x_max: float, bits: int) -> float:
    return u / ((1 << bits) - 1) * (x_max - x_min) + x_min


# ---------------------------------------------------------------------------
# 명령 프레임
# ---------------------------------------------------------------------------
def sys_cmd(cmd: SysCmd) -> bytes:
    return bytes([0xFF] * 7 + [int(cmd)])


def pack_mit(q: float, dq: float, kp: float, kd: float, tau: float, lim: Limits) -> bytes:
    p = float_to_uint(q, -lim.pmax, lim.pmax, 16)
    v = float_to_uint(dq, -lim.vmax, lim.vmax, 12)
    kp_u = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_u = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t = float_to_uint(tau, -lim.tmax, lim.tmax, 12)
    return bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF, ((v & 0xF) << 4) | ((kp_u >> 8) & 0xF),
        kp_u & 0xFF,
        (kd_u >> 4) & 0xFF, ((kd_u & 0xF) << 4) | ((t >> 8) & 0xF),
        t & 0xFF,
    ])


def unpack_mit(data: bytes, lim: Limits):
    """명령 프레임 해석 (가상 모터용). 반환: q, dq, kp, kd, tau"""
    p = (data[0] << 8) | data[1]
    v = (data[2] << 4) | (data[3] >> 4)
    kp_u = ((data[3] & 0xF) << 8) | data[4]
    kd_u = (data[5] << 4) | (data[6] >> 4)
    t = ((data[6] & 0xF) << 8) | data[7]
    return (uint_to_float(p, -lim.pmax, lim.pmax, 16), uint_to_float(v, -lim.vmax, lim.vmax, 12),
            uint_to_float(kp_u, KP_MIN, KP_MAX, 12), uint_to_float(kd_u, KD_MIN, KD_MAX, 12),
            uint_to_float(t, -lim.tmax, lim.tmax, 12))


def pack_pos_vel(q: float, v_limit: float) -> bytes:
    return struct.pack("<ff", q, v_limit)


def pack_vel(dq: float) -> bytes:
    return struct.pack("<f", dq) + bytes(4)


def pack_reg(esc_id: int, op: RegOp, rid: int = 0, value=None) -> bytes:
    head = bytes([esc_id & 0xFF, (esc_id >> 8) & 0xFF, int(op), rid & 0xFF])
    if value is None:
        return head + bytes(4)
    if rid in UINT32_RIDS:
        return head + struct.pack("<I", int(value))
    return head + struct.pack("<f", float(value))


# ---------------------------------------------------------------------------
# 응답 프레임
# ---------------------------------------------------------------------------
@dataclass
class Feedback:
    id_low4: int        # ESC_ID 하위 4비트
    status: int         # Status 값
    q: float            # 위치 [rad]
    dq: float           # 속도 [rad/s]
    tau: float          # 토크 [N·m] (모터 출력축 기준, 드라이버 추정값)
    t_mos: int          # 드라이버 온도 [°C]
    t_rotor: int        # 코일 온도 [°C]

    @property
    def status_text(self) -> str:
        try:
            return STATUS_TEXT[Status(self.status)]
        except ValueError:
            return f"알 수 없음(0x{self.status:X})"

    @property
    def is_fault(self) -> bool:
        return self.status not in (Status.DISABLED, Status.ENABLED)


def is_register_reply(data: bytes) -> bool:
    return len(data) == 8 and data[2] in (RegOp.READ, RegOp.WRITE)


def unpack_feedback(data: bytes, lim: Limits) -> Feedback:
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0xF) << 8) | data[5]
    return Feedback(
        id_low4=data[0] & 0x0F, status=(data[0] >> 4) & 0x0F,
        q=uint_to_float(p, -lim.pmax, lim.pmax, 16),
        dq=uint_to_float(v, -lim.vmax, lim.vmax, 12),
        tau=uint_to_float(t, -lim.tmax, lim.tmax, 12),
        t_mos=data[6], t_rotor=data[7],
    )


def pack_feedback(esc_id: int, status: int, q: float, dq: float, tau: float, t_mos: int, t_rotor: int,
                  lim: Limits) -> bytes:
    """응답 프레임 생성 (가상 모터용)"""
    p = float_to_uint(q, -lim.pmax, lim.pmax, 16)
    v = float_to_uint(dq, -lim.vmax, lim.vmax, 12)
    t = float_to_uint(tau, -lim.tmax, lim.tmax, 12)
    return bytes([((status & 0xF) << 4) | (esc_id & 0xF), (p >> 8) & 0xFF, p & 0xFF, (v >> 4) & 0xFF,
                  ((v & 0xF) << 4) | ((t >> 8) & 0xF), t & 0xFF, int(t_mos) & 0xFF, int(t_rotor) & 0xFF])


def unpack_reg_reply(data: bytes):
    """반환: (esc_id, rid, value)"""
    esc_id = data[0] | (data[1] << 8)
    rid = data[3]
    raw = bytes(data[4:8])
    value = struct.unpack("<I", raw)[0] if rid in UINT32_RIDS else struct.unpack("<f", raw)[0]
    return esc_id, rid, value


def resolution(lim: Limits) -> dict:
    """정수 변환 1단위의 크기 (측정 해상도 확인용)"""
    return dict(q=2 * lim.pmax / 65535, dq=2 * lim.vmax / 4095, tau=2 * lim.tmax / 4095)
