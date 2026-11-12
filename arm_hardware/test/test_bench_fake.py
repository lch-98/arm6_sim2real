"""가상 모터와 버스 클래스 통합 시험 (python-can virtual 버스, 하드웨어 없이)"""
import math
import os
import time

import pytest

can = pytest.importorskip("can")

from arm_hardware import dm_protocol as dp            # noqa: E402
from arm_hardware.bench import main as bench_main      # noqa: E402
from arm_hardware.fake_motor import FakeMotor, FakeMotorServer  # noqa: E402
from arm_hardware.motor import DamiaoBus               # noqa: E402
from arm_hardware.transport import CanTransport        # noqa: E402


@pytest.fixture
def rig():
    ch = f"t{os.getpid()}{time.perf_counter_ns()}"
    fm = FakeMotor(0x02, 0x12, "DM-J4340P-2EC")
    srv = FakeMotorServer(CanTransport("virtual", ch), [fm]).start()
    bus = DamiaoBus(CanTransport("virtual", ch))
    m = bus.add(0x02, 0x12, "DM-J4340P-2EC")
    yield bus, m, fm
    srv.stop()
    bus.close()


def test_registers(rig):
    bus, m, _ = rig
    assert bus.read_reg(m, dp.Rid.ESC_ID) == 0x02
    assert bus.read_reg(m, dp.Rid.MST_ID) == 0x12
    assert bus.read_limits(m) and m.limits.tmax == 28.0
    assert bus.write_reg(m, dp.Rid.TIMEOUT, 0) == 0


def test_enable_hold_disable(rig):
    bus, m, fm = rig
    fb = bus.enable(m)
    assert fb is not None and fb.status == dp.Status.ENABLED
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 1.0:
        fb = bus.mit(m, 0.5, 0.0, 20.0, 1.0, 0.0)
        time.sleep(0.005)
    assert fb is not None and abs(fb.q - 0.5) < 0.05
    fb = bus.disable(m)
    assert fb.status == dp.Status.DISABLED


def test_cli_scan_and_move(capsys, tmp_path):
    bench_main(["--fake", "scan", "--ids", "0x01-0x07"])
    out = capsys.readouterr().out
    assert "응답한 모터 7대" in out
    bench_main(["--fake", "--joint", "joint2", "--out", str(tmp_path), "move", "--to", "0.4", "--vmax", "1.0",
                "--hold", "0.5"])
    out = capsys.readouterr().out
    assert "최종 위치" in out


def test_cli_friction_estimates(capsys, tmp_path):
    bench_main(["--fake", "--joint", "joint4", "--out", str(tmp_path), "friction", "--speeds", "0.5,1.0,2.0",
                "--seconds-per", "1.5"])
    out = capsys.readouterr().out
    assert "쿨롱 마찰" in out
    # 가상 모터: fc 0.10, b 0.05
    line = [l for l in out.splitlines() if l.startswith("추정")][0]
    fc = float(line.split("쿨롱 마찰 ")[1].split(" ")[0])
    bv = float(line.split("점성 마찰 ")[1].split(" ")[0])
    assert abs(fc - 0.10) < 0.03 and abs(bv - 0.05) < 0.02


def test_cli_hold_lever(capsys, tmp_path):
    bench_main(["--fake", "--fake-lever", "0.5,0.1", "--joint", "joint2", "--out", str(tmp_path), "hold",
                "--seconds", "2", "--lever", "0.5,0.1"])
    out = capsys.readouterr().out
    mean_tau = float(out.split("후반 평균 토크 ")[1].split(" ")[0])
    expected = float(out.split("m·g·L·cos(q) = ")[1].split(" ")[0])
    # 정지 마찰(가상 fc 0.10 N·m)만큼은 차이가 날 수 있다
    assert abs(mean_tau - expected) <= 0.11


def test_cli_sweep_and_latency(capsys, tmp_path):
    ref = os.path.join(os.path.dirname(__file__), "..", "..", "arm_bringup", "config", "reference_trajectories.yaml")
    bench_main(["--fake", "--joint", "joint5", "--out", str(tmp_path), "sweep", "--ref-joint", "joint5", "--scale",
                "0.3", "--reference", ref])
    out = capsys.readouterr().out
    assert "추종 오차 RMS" in out
    bench_main(["--fake", "--joint", "joint5", "latency", "--n", "200"])
    assert "왕복 200회" in capsys.readouterr().out


def test_safety_abort_on_torque(tmp_path):
    with pytest.raises(SystemExit) as e:
        bench_main(["--fake", "--fake-lever", "3.0,0.2", "--joint", "joint4", "--out", str(tmp_path), "hold",
                    "--seconds", "2"])
    assert e.value.code == 2
