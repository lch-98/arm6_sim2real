"""프로토콜 변환 시험 (하드웨어 없이)"""
import math
import struct

import pytest

from arm_hardware import dm_protocol as dp

LIM = dp.DEFAULT_LIMITS["DM-J4340P-2EC"]


def test_float_uint_ends():
    assert dp.float_to_uint(-12.5, -12.5, 12.5, 16) == 0
    assert dp.float_to_uint(12.5, -12.5, 12.5, 16) == 65535
    assert dp.float_to_uint(99.0, -12.5, 12.5, 16) == 65535      # 범위 밖은 잘림
    assert dp.uint_to_float(0, -1.0, 1.0, 12) == -1.0


@pytest.mark.parametrize("q,dq,kp,kd,tau", [(0.0, 0.0, 0.0, 0.0, 0.0), (1.234, -3.2, 25.0, 1.1, 5.5),
                                             (-12.0, 7.9, 499.0, 4.9, -27.0)])
def test_mit_roundtrip(q, dq, kp, kd, tau):
    data = dp.pack_mit(q, dq, kp, kd, tau, LIM)
    assert len(data) == 8
    r = dp.resolution(LIM)
    q2, dq2, kp2, kd2, t2 = dp.unpack_mit(data, LIM)
    assert abs(q2 - q) <= r["q"]
    assert abs(dq2 - dq) <= r["dq"]
    assert abs(kp2 - kp) <= 500 / 4095
    assert abs(kd2 - kd) <= 5 / 4095
    assert abs(t2 - tau) <= r["tau"]


def test_mit_byte_layout():
    # 위치 0 → 0x7FFF, 속도·토크 0 → 0x7FF, kp 0, kd 0
    d = dp.pack_mit(0.0, 0.0, 0.0, 0.0, 0.0, LIM)
    assert d == bytes([0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF])


def test_feedback_roundtrip():
    data = dp.pack_feedback(0x02, dp.Status.ENABLED, 0.5, -1.0, 3.0, 40, 45, LIM)
    fb = dp.unpack_feedback(data, LIM)
    assert fb.id_low4 == 2 and fb.status == dp.Status.ENABLED and not fb.is_fault
    assert math.isclose(fb.q, 0.5, abs_tol=1e-3)
    assert math.isclose(fb.dq, -1.0, abs_tol=0.01)
    assert math.isclose(fb.tau, 3.0, abs_tol=0.02)
    assert (fb.t_mos, fb.t_rotor) == (40, 45)
    assert dp.unpack_feedback(dp.pack_feedback(1, 0xC, 0, 0, 0, 0, 0, LIM), LIM).is_fault


def test_sys_and_register_frames():
    assert dp.sys_cmd(dp.SysCmd.ENABLE) == b"\xff" * 7 + b"\xfc"
    assert dp.pack_reg(0x02, dp.RegOp.READ, dp.Rid.PMAX) == bytes([0x02, 0x00, 0x33, 21, 0, 0, 0, 0])
    w = dp.pack_reg(0x02, dp.RegOp.WRITE, dp.Rid.MST_ID, 0x12)
    assert w[4:] == struct.pack("<I", 0x12)
    w = dp.pack_reg(0x02, dp.RegOp.WRITE, dp.Rid.TMAX, 28.0)
    assert dp.unpack_reg_reply(w) == (0x02, 23, 28.0)
    assert dp.pack_pos_vel(1.0, 2.0) == struct.pack("<ff", 1.0, 2.0)


class _FakeSerial:
    def __init__(self, rx=b""):
        self.rx = bytearray(rx)
        self.tx = bytearray()

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n):
        out = bytes(self.rx[:n]); del self.rx[:n]
        return out

    def write(self, b):
        self.tx += b

    def close(self):
        pass


def test_damiao_serial_framing():
    from arm_hardware.transport import DamiaoSerialTransport
    t = DamiaoSerialTransport.__new__(DamiaoSerialTransport)
    t.buf = bytearray()
    # 수신: 쓰레기 바이트 + 정상 프레임 (CMD 0x11, ID 0x12)
    frame = bytes([0xAA, 0x11, 0x08, 0x12, 0x00, 0x00, 0x00]) + bytes(range(8)) + bytes([0x55])
    t.ser = _FakeSerial(b"\x01\x02" + frame)
    assert t.recv(0.01) == (0x12, bytes(range(8)))
    assert t.recv(0.0) is None
    # 송신: ID와 데이터 위치
    t.send(0x102, bytes([9] * 8))
    tx = t.ser.tx
    assert len(tx) == 30 and tx[:2] == b"\x55\xaa" and tx[13] == 0x02 and tx[14] == 0x01 and tx[21:29] == bytes([9] * 8)
