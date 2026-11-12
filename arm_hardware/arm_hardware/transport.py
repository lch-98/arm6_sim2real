"""
CAN 송수신 계층. 같은 인터페이스(send / recv / close)로 세 가지 연결을 다룬다.

  CanTransport          : python-can (SocketCAN can0·vcan0, slcan, gs_usb, 프로세스 내부 virtual 버스)
  DamiaoSerialTransport : Damiao USB-CAN 보드 (USB 시리얼 /dev/ttyACM*, 921600 bps, 전용 프레임)

키트의 USB-CAN 보드가 리눅스에서 can0으로 잡히면 CanTransport, /dev/ttyACM0 같은 시리얼로 잡히면
DamiaoSerialTransport를 쓴다 (docs/05_motor_bench.md 2-2-1).
"""
from __future__ import annotations

import time
from typing import Optional, Tuple

Frame = Tuple[int, bytes]          # (CAN ID, 데이터 8바이트)


class Transport:
    def send(self, arb_id: int, data: bytes) -> None:
        raise NotImplementedError

    def recv(self, timeout: float) -> Optional[Frame]:
        raise NotImplementedError

    def close(self) -> None:
        pass

    def drain(self) -> None:
        """수신 버퍼에 남은 프레임 비우기"""
        while self.recv(0.0) is not None:
            pass


class CanTransport(Transport):
    def __init__(self, interface: str = "socketcan", channel: str = "can0", bitrate: int = 1_000_000, **kw):
        import can  # python-can
        args = dict(interface=interface, channel=channel, receive_own_messages=False, **kw)
        if interface not in ("socketcan", "virtual"):
            args["bitrate"] = bitrate          # SocketCAN 비트레이트는 ip link 로 설정
        self.bus = can.Bus(**args)
        self._can = can

    def send(self, arb_id, data):
        self.bus.send(self._can.Message(arbitration_id=arb_id, data=bytes(data), is_extended_id=False))

    def recv(self, timeout):
        msg = self.bus.recv(timeout)
        if msg is None or msg.is_error_frame or msg.is_remote_frame:
            return None
        return msg.arbitration_id, bytes(msg.data)

    def close(self):
        self.bus.shutdown()


class DamiaoSerialTransport(Transport):
    """
    Damiao USB-CAN 보드 시리얼 프레임 (DM_Control_Python의 정의를 따름)
      송신 30바이트: 55 AA 1E 03 01 00 00 00 0A 00 00 00 00 [ID_L ID_H] 00 00 00 08 00 00 [데이터 8] 00
      수신 16바이트: AA [CMD] [?] [ID 4바이트, 리틀엔디언] [데이터 8] 55   (CMD 0x11 = CAN 수신 프레임)
    """
    TX_TEMPLATE = bytes([0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0, 0,
                         0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00])
    RX_LEN = 16

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 921600):
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0)
        self.buf = bytearray()

    def send(self, arb_id, data):
        f = bytearray(self.TX_TEMPLATE)
        f[13], f[14] = arb_id & 0xFF, (arb_id >> 8) & 0xFF
        f[21:29] = bytes(data)
        self.ser.write(f)

    def _parse(self) -> Optional[Frame]:
        b = self.buf
        while len(b) >= self.RX_LEN:
            if b[0] == 0xAA and b[self.RX_LEN - 1] == 0x55:
                cmd = b[1]
                arb_id = b[3] | (b[4] << 8) | (b[5] << 16) | (b[6] << 24)
                data = bytes(b[7:15])
                del b[:self.RX_LEN]
                if cmd == 0x11:
                    return arb_id, data
            else:
                del b[0]
        return None

    def recv(self, timeout):
        end = time.perf_counter() + timeout
        while True:
            fr = self._parse()
            if fr is not None:
                return fr
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
                continue
            if time.perf_counter() >= end:
                return None
            time.sleep(0.0002)

    def close(self):
        self.ser.close()


def open_transport(kind: str, channel: str, bitrate: int = 1_000_000) -> Transport:
    """kind: socketcan | damiao-serial | slcan | gs_usb | virtual"""
    if kind == "damiao-serial":
        return DamiaoSerialTransport(channel)
    return CanTransport(kind, channel, bitrate)
