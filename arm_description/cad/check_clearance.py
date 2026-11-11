#!/usr/bin/env python3
"""
이웃한 링크끼리 겹치는지 검사 (CadQuery 불리언, 부피 기준).

URDF 충돌 검사(MoveIt 등)는 관절로 바로 이어진 두 링크를 검사하지 않는다.
그래서 요크 안에서 도는 허브, 모터 캔처럼 "붙어 있는 링크끼리"의 간섭은 이 스크립트로 따로 본다.
각 관절을 한계 범위 안에서 1° 간격으로 돌리며 부모 링크(본체+부품+모터)와 자식 링크의 겹친 부피를 계산한다.

    python3 cad/check_clearance.py

관절 범위는 urdf/arm_macro.xacro 의 limit 와 같게 유지한다.
"""
import os
import sys

import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_cad as gc  # noqa: E402

# (부모, 자식, 회전축(홈 자세), 하한°, 상한°)
PAIRS = [
    ("base_link", "link1", (0, 0, 1), -170, 170),
    ("link1", "link2", (0, 1, 0), -110, 110),
    ("link2", "link3", (0, 1, 0), -150, 32),
    ("link3", "link4", (1, 0, 0), 0, 0),      # 롤 관절: 축 대칭이라 0°만
    ("link4", "link5", (0, 1, 0), -110, 110),
    ("link5", "link6", (1, 0, 0), 0, 0),
]
TOL = 0.5  # mm^3 이하는 맞닿음으로 보고 무시


def solids(name):
    p = gc.LINKS[name]()
    s = [p["body"]] + [x for x, _ in p["extras"].values()]
    if p["motor"] is not None:
        s.append(p["motor"])
    return s


def overlap(A, B, axis, ang, off):
    v = 0.0
    for b in B:
        bb = b.rotate(cq.Vector(0, 0, 0), cq.Vector(*axis), ang).translate(cq.Vector(*off))
        for a in A:
            v += a.intersect(bb).Volume()
    return v


def main():
    home = gc.home_positions()
    ok = True
    for par, ch, axis, lo, hi in PAIRS:
        A, B = solids(par), solids(ch)
        off = [home[ch][i] - home[par][i] for i in range(3)]
        hits = [a for a in range(lo, hi + 1) if overlap(A, B, axis, a, off) > TOL]
        if hits:
            ok = False
            print(f"{par:9s} - {ch:6s}: 간섭 {min(hits)}° ~ {max(hits)}° ({len(hits)}개 각도)")
        else:
            print(f"{par:9s} - {ch:6s}: 간섭 없음 ({lo}° ~ {hi}°)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
