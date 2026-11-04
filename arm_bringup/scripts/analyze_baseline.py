#!/usr/bin/env python3
"""
run_reference.py로 기록한 rosbag을 분석한다.

  ros2 run arm_bringup analyze_baseline.py ~/arm_logs/gazebo_baseline
  ros2 run arm_bringup analyze_baseline.py ~/arm_logs/gazebo_baseline --compare ~/arm_logs/mock_baseline

출력 (기본: <bag>/analysis/)
  joint_states.csv      시간, 관절별 위치·속도·토크
  tracking.csv          시간, 관절별 목표·실제·오차 (/arm_controller/state)
  summary.md            구간별 추종 오차(RMS·최대), 정지 자세별 토크 vs 모델 중력 토크
  *.png                 그래프 (matplotlib이 있을 때)

모델 중력 토크: 기록 안의 /robot_description(URDF)에서 질량·무게중심을 읽어
"그 자세를 버티려면 모터가 내야 하는 토크"를 계산한다. 실물 단계에서 질량 식별의 기준이 된다.
"""
import argparse
import csv
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
G = np.array([0.0, 0.0, -9.81])


# ---------------------------------------------------------------- bag 읽기
def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {k: [] for k in types}
    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        out[topic].append(deserialize_message(data, get_message(types[topic])))
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# ---------------------------------------------------------------- URDF 중력 모델
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    o = el.find("origin") if el is not None else None
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_rot(axis, q):
    a = np.asarray(axis) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


class GravityModel:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.links = {}
        for l in root.findall("link"):
            ine = l.find("inertial")
            if ine is not None:
                m = float(ine.find("mass").get("value"))
                self.links[l.get("name")] = (m, _origin(ine)[:3, 3])
        self.joints = []
        for j in root.findall("joint"):
            ax = j.find("axis")
            mim = j.find("mimic")
            self.joints.append(dict(
                name=j.get("name"), type=j.get("type"),
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                T=_origin(j), axis=[float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()],
                mimic=(mim.get("joint"), float(mim.get("multiplier", "1"))) if mim is not None else None))
        self.children = {}
        for j in self.joints:
            self.children.setdefault(j["parent"], []).append(j)
        self.root = ({j["parent"] for j in self.joints} - {j["child"] for j in self.joints}).pop()

    def hold_torque(self, q):
        """q: {관절이름: 값}. 각 관절이 그 자세를 버티려면 내야 하는 토크 {관절: N·m}"""
        frames, jframes = {self.root: np.eye(4)}, {}
        stack = [self.root]
        while stack:
            p = stack.pop()
            for j in self.children.get(p, []):
                v = q.get(j["name"], 0.0)
                if j["mimic"]:
                    v = q.get(j["mimic"][0], 0.0) * j["mimic"][1]
                Tj = frames[p] @ j["T"]
                jframes[j["name"]] = (Tj, j)
                if j["type"] in ("revolute", "continuous"):
                    Tc = Tj @ _axis_rot(j["axis"], v)
                elif j["type"] == "prismatic":
                    Tc = Tj.copy()
                    Tc[:3, 3] += Tj[:3, :3] @ (np.asarray(j["axis"]) * v)
                else:
                    Tc = Tj
                frames[j["child"]] = Tc
                stack.append(j["child"])

        def subtree(link):
            out = [link]
            for j in self.children.get(link, []):
                out += subtree(j["child"])
            return out

        tau = {}
        for name, (Tj, j) in jframes.items():
            if j["type"] not in ("revolute", "continuous"):
                continue
            p = Tj[:3, 3]
            a = Tj[:3, :3] @ (np.asarray(j["axis"]) / np.linalg.norm(j["axis"]))
            tg = 0.0
            for l in subtree(j["child"]):
                if l in self.links:
                    m, c = self.links[l]
                    pc = (frames[l] @ np.append(c, 1.0))[:3]
                    tg += np.cross(pc - p, m * G) @ a
            tau[name] = -tg          # 중력 토크를 상쇄하는 토크 = 모터가 내야 하는 토크
        return tau


# ---------------------------------------------------------------- 분석
def segments(events):
    seg, out = {}, []
    for e in events:
        d = json.loads(e.data)
        if d["phase"] == "start":
            seg[d["segment"]] = d
        elif d["phase"] == "end" and d["segment"] in seg:
            s = seg.pop(d["segment"])
            out.append(dict(name=d["segment"], t0=s["t"], t1=d["t"], info=s))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default="")
    ap.add_argument("--compare", default="", help="다른 기록(예: mock)과 정지 자세 토크·추종 오차를 나란히 비교")
    args = ap.parse_args()
    bag = os.path.expanduser(args.bag)
    out = os.path.expanduser(args.out) if args.out else os.path.join(bag, "analysis")
    os.makedirs(out, exist_ok=True)

    res = analyze(bag, out)
    other = analyze(os.path.expanduser(args.compare), None) if args.compare else None
    write_summary(res, other, out, bag, args.compare)
    try:
        plots(res, out)
    except ImportError:
        print("matplotlib 없음 → 그래프 생략 (sudo apt install python3-matplotlib)")
    print(f"분석 완료: {out}")


def analyze(bag, out):
    data = read_bag(bag)
    js = data.get("/joint_states", [])
    st = data.get("/arm_controller/state", [])
    ev = data.get("/reference_events", [])
    if not js or not ev:
        raise SystemExit(f"{bag}: /joint_states 또는 /reference_events 기록이 없음")
    names = list(js[0].name)
    t_js = np.array([stamp(m) for m in js])
    P = np.array([[dict(zip(m.name, m.position)).get(n, np.nan) for n in names] for m in js])
    E = np.array([[dict(zip(m.name, m.effort or [np.nan] * len(m.name))).get(n, np.nan) for n in names] for m in js])
    Vv = np.array([[dict(zip(m.name, m.velocity or [np.nan] * len(m.name))).get(n, np.nan) for n in names] for m in js])

    # 추종 오차 (/arm_controller/state). Humble: reference/feedback 또는 desired/actual
    t_st, ref, fb = [], [], []
    st_names = list(st[0].joint_names) if st else []
    for m in st:
        r = getattr(m, "reference", None)
        f = getattr(m, "feedback", None)
        if r is None or len(r.positions) == 0:
            r, f = m.desired, m.actual
        t_st.append(stamp(m))
        ref.append(list(r.positions))
        fb.append(list(f.positions))
    t_st, ref, fb = np.array(t_st), np.array(ref), np.array(fb)

    if out:
        with open(os.path.join(out, "joint_states.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + [f"{n}_pos" for n in names] + [f"{n}_vel" for n in names] + [f"{n}_eff" for n in names])
            for i in range(len(t_js)):
                w.writerow([f"{t_js[i]:.4f}"] + [f"{x:.6f}" for x in np.concatenate([P[i], Vv[i], E[i]])])
        if len(t_st):
            with open(os.path.join(out, "tracking.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t"] + [f"{n}_ref" for n in st_names] + [f"{n}_act" for n in st_names] + [f"{n}_err" for n in st_names])
                for i in range(len(t_st)):
                    w.writerow([f"{t_st[i]:.4f}"] + [f"{x:.6f}" for x in np.concatenate([ref[i], fb[i], ref[i] - fb[i]])])

    model = None
    if data.get("/robot_description"):
        model = GravityModel(data["/robot_description"][-1].data)

    segs = segments(ev)
    track, static = [], []
    for s in segs:
        if s["name"].startswith(("sweep/", "pick", "home")) and len(t_st):
            m = (t_st >= s["t0"]) & (t_st <= s["t1"])
            if m.sum() < 2:
                continue
            err = ref[m] - fb[m]
            track.append(dict(name=s["name"], rms=np.sqrt(np.mean(err**2, axis=0)), mx=np.max(np.abs(err), axis=0),
                              t=t_st[m] - s["t0"], err=err, ref=ref[m], act=fb[m]))
        if s["name"].startswith("static/"):
            avg = s["info"].get("average_last", 1.5)
            m = (t_js >= s["t1"] - avg) & (t_js <= s["t1"])
            q_meas = dict(zip(names, np.nanmean(P[m], axis=0)))
            e_meas = dict(zip(names, np.nanmean(E[m], axis=0)))
            tau = model.hold_torque(q_meas) if model else {}
            static.append(dict(name=s["name"], q=[q_meas.get(j, np.nan) for j in ARM],
                               eff=[e_meas.get(j, np.nan) for j in ARM], model=[tau.get(j, np.nan) for j in ARM]))
    return dict(names=st_names or ARM, track=track, static=static, duration=t_js[-1] - t_js[0],
                n_js=len(t_js), rate=len(t_js) / max(t_js[-1] - t_js[0], 1e-9))


def write_summary(res, other, out, bag, other_path):
    L = []
    L.append(f"# 기준선 분석: `{os.path.basename(bag.rstrip('/'))}`\n")
    L.append(f"- 기록 길이 {res['duration']:.1f} s, /joint_states {res['n_js']}개 (평균 {res['rate']:.0f} Hz)")
    if other:
        L.append(f"- 비교 대상: `{os.path.basename(other_path.rstrip('/'))}`")
    L.append("\n## 1. 구간별 추종 오차 (목표 − 실제) [mrad]\n")
    L.append("mock은 컨트롤러 보간 오차만, Gazebo는 시뮬레이터 위치 루프(position_proportional_gain)의 지연까지 포함한다.")
    L.append("실물 모터의 추종 오차와 직접 같지 않으며, 실물 단계에서 같은 표를 만들어 비교한다.\n")
    L.append("| 구간 | " + " | ".join(f"{n} RMS / 최대" for n in res["names"]) + " |")
    L.append("|---|" + "---|" * len(res["names"]))
    for tr in res["track"]:
        L.append(f"| {tr['name']} | " + " | ".join(f"{r * 1e3:.1f} / {m * 1e3:.1f}" for r, m in zip(tr["rms"], tr["mx"])) + " |")
    L.append("\n## 2. 정지 자세 토크 [N·m]\n")
    L.append("측정 = 기록된 effort (hold 마지막 구간 평균), 모델 = URDF 질량·무게중심으로 계산한 유지 토크.")
    L.append("mock 하드웨어는 effort를 계산하지 않으므로 측정값이 0 또는 nan으로 나온다.\n")
    L.append("| 자세 | q2, q3, q5 [rad] | J2 측정 / 모델 | J3 측정 / 모델 | J5 측정 / 모델 |")
    L.append("|---|---|---|---|---|")
    errs = []
    for s in res["static"]:
        c = []
        for i in (1, 2, 4):
            c.append(f"{s['eff'][i]:+.3f} / {s['model'][i]:+.3f}")
            if not (math.isnan(s["eff"][i]) or math.isnan(s["model"][i])):
                errs.append(s["eff"][i] - s["model"][i])
        L.append(f"| {s['name']} | {s['q'][1]:+.2f}, {s['q'][2]:+.2f}, {s['q'][4]:+.2f} | " + " | ".join(c) + " |")
    meas = [s["eff"][i] for s in res["static"] for i in (1, 2, 4)]
    if meas and all(abs(v) < 1e-9 for v in meas if not math.isnan(v)):
        L.append("\n측정 토크가 모두 0 → 토크를 계산하지 않는 하드웨어(mock). 토크 비교는 생략")
    elif errs:
        e = np.array(errs)
        L.append(f"\n측정 − 모델: 평균 {e.mean():+.3f} N·m, RMS {np.sqrt((e**2).mean()):.3f} N·m, 최대 {np.abs(e).max():.3f} N·m")
    if other:
        L.append("\n## 3. 비교 기록의 추종 오차 RMS [mrad]\n")
        L.append("| 구간 | " + " | ".join(res["names"]) + " |")
        L.append("|---|" + "---|" * len(res["names"]))
        for tr in other["track"]:
            L.append(f"| {tr['name']} | " + " | ".join(f"{r * 1e3:.1f}" for r in tr["rms"]) + " |")
    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))


def plots(res, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sweeps = [t for t in res["track"] if t["name"].startswith("sweep/")]
    if sweeps:
        fig, axs = plt.subplots(len(sweeps), 1, figsize=(9, 2.2 * len(sweeps)), squeeze=False)
        for ax, tr in zip(axs[:, 0], sweeps):
            j = res["names"].index(tr["name"].split("/")[1])
            ax.plot(tr["t"], tr["ref"][:, j], label="ref", lw=1.2)
            ax.plot(tr["t"], tr["act"][:, j], label="actual", lw=1.0, ls="--")
            ax2 = ax.twinx()
            ax2.plot(tr["t"], tr["err"][:, j] * 1e3, color="tab:red", lw=0.8, alpha=0.7)
            ax2.set_ylabel("err [mrad]", color="tab:red")
            ax.set_title(tr["name"], fontsize=9)
            ax.set_ylabel("rad")
        axs[0, 0].legend(loc="upper right", fontsize=8)
        axs[-1, 0].set_xlabel("t [s]")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "sweeps.png"), dpi=110)
    if res["static"]:
        fig, axs = plt.subplots(1, 3, figsize=(12, 3.4))
        x = np.arange(len(res["static"]))
        for ax, i, n in zip(axs, (1, 2, 4), ("joint2", "joint3", "joint5")):
            ax.bar(x - 0.2, [s["model"][i] for s in res["static"]], 0.4, label="model")
            ax.bar(x + 0.2, [s["eff"][i] for s in res["static"]], 0.4, label="measured")
            ax.set_title(n)
            ax.set_xlabel("static pose #")
            ax.set_ylabel("N·m")
        axs[0].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "static_torque.png"), dpi=110)


if __name__ == "__main__":
    main()
