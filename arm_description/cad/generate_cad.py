#!/usr/bin/env python3
"""
arm 형상 생성 스크립트 (CadQuery) — 소형, 전 부품 3D 프린팅 + Damiao CAN(1 Mbps) 액추에이터 기준

- 모든 치수는 mm 단위, 각 링크는 "자기 관절 좌표계" 기준으로 모델링한다.
  (URDF에서 visual origin을 0으로 두면 그대로 맞물리도록)
- 출력
    cad/step/<link>.step              : 링크별 프린팅 부품 STEP
    cad/arm_assembly.step             : 홈 자세 조립 STEP (액추에이터·베어링 외형 포함)
    cad/arm_exploded.step             : 분해도 STEP (체결 부품을 조립 방향으로 빼 놓은 상태)
    meshes/<link>_body.stl            : 프린팅 본체 (흰색) ← 슬라이서에 넣을 파일
    meshes/<link>_accent.stl          : 프린팅 캡/패드, 구매 피니언 (주황)
    meshes/<link>_motor.stl           : 액추에이터 외형 (회색, 표시용 — 프린팅 안 함)
    meshes/<link>_collision.stl       : 위 셋을 합친 충돌 형상
    urdf/generated/arm_properties.xacro : 관절 오프셋 + 질량/관성

치수를 바꾸고 싶으면 P / ACT / JOINT_ACT만 수정한 뒤 다시 실행한다.
    python3 cad/generate_cad.py

주의: 액추에이터는 판매처 사양표의 외경 x 높이로 만든 "외형 원기둥"이다.
      하우징 체결 볼트, 출력 플랜지 볼트 패턴은 아직 반영되지 않았다.
      공식 도면(STEP)으로 확인한 뒤 프린팅 전에 구멍을 추가해야 한다.
"""
import math
import os

import cadquery as cq
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# ---------------------------------------------------------------------------
# 구매 부품
# ---------------------------------------------------------------------------
# 액추에이터 외형: 외경 D x 높이 H [mm], 질량 [kg] (판매처 사양표 값)
# 모두 CAN 2.0 1 Mbps 기본, 드라이버 일체형 (Damiao)
# 구매: Seeed reBot Arm B601-DM Body Motor Kit (4340P ×3, 4310 ×3) + 그리퍼용 H3510 ×1
# D, H: 하우징 외경, 전체 높이(출력면 포함) / out_d, out_h: 출력 플랜지(회전부) 지름과 하우징 앞면에서 튀어나온 길이
# 4340P·4310 치수는 판매처(디바이스마트 키트 페이지) 도면 기준. 4340P out_d는 도면에 수치가 없어 도면 축척으로 잰 값 → 실측 확인
# 질량은 Seeed DAMIAO 43 시리즈 자료값(약). 모터를 받으면 저울로 재서 바꾼다 (docs/05 3절)
ACT = {
    "DM-J4340P-2EC": dict(D=57.0, H=56.5, out_d=38.0, out_h=1.5, mass=0.375),  # 정격 9 / 피크 27 N·m, 36 rpm, 40:1
    "DM-J4310-2EC": dict(D=57.0, H=46.0, out_d=35.0, out_h=1.0, mass=0.300),  # 정격 3 / 피크 7 N·m, 120 rpm, 10:1
    "DM-H3510":     dict(D=42.0, H=31.5, out_d=25.2, out_h=3.0, mass=0.116),  # 0.18 / 0.45 N·m, 직결. 도면 미확보(추정 형상)
}
JOINT_ACT = dict(
    J1="DM-J4340P-2EC",
    J2="DM-J4340P-2EC",
    J3="DM-J4340P-2EC",
    J4="DM-J4310-2EC",
    J5="DM-J4310-2EC",
    J6="DM-J4310-2EC",
    GRIP="DM-H3510",
)
# 반대편 지지 베어링 (내경 d, 외경 D, 폭 B) [mm]
BRG = {
    "61804": (20.0, 32.0, 7.0),
    "61802": (15.0, 24.0, 5.0),
}
JOINT_BRG = dict(J2="61804", J3="61802", J5="61802")
PINION_R = 10.0   # 그리퍼 피니언: 모듈 1, 20T → 피치원 반경 10 mm (구매)
RACK_M = 1.0      # 랙 모듈 (프린팅)

# 손목 카메라: Intel RealSense D435 (공식 사양 90 x 25 x 25 mm, 약 75 g)
#   w: 가로(y), d: 깊이(x, 렌즈 방향), h: 높이(z)
#   x, z: link6 좌표계에서 카메라 본체 중심 위치 [mm], pitch_deg: 아래로 숙인 각도
CAM = dict(w=90.0, d=25.0, h=25.0, mass=0.075, x=20.0, z=54.0, pitch_deg=25.0)
M3_INSERT_D = 4.0   # M3 열압입 인서트 구멍 지름 (사용할 인서트 규격에 맞춰 조정)
M3_CLEAR_D = 3.4    # M3 통과 구멍

# 그리퍼 랙 안내 구조 (link6 좌표 x는 grip_body_len 기준, 핑거 좌표와 같은 기준)
#   가이드판(뒤) ─ 랙(핑거 x -10~-2) ─ 앞 덮개 ─ 핑거 연결판·패드
#   랙은 가이드판 바깥 벽(z)과 덮개 슬롯(스템)으로 붙잡혀 y 방향으로만 미끄러진다
RACK_BACK = 19.0               # 랙 바깥면 |z| (피치선 10, 이끝 9)
GRIP_STEM_Z = (12.0, 15.0)     # 랙 → 덮개 슬롯을 지나는 스템의 |z| 범위 (피니언 이끝원 r 11 바깥)
GRIP_COVER_T = 3.0             # 앞 덮개 두께
GRIP_WALL_Z = 25.0             # 가이드판·벽·덮개 바깥 |z|
SLOT_CLR = 0.4                 # 스템 ↔ 슬롯 공차 (한쪽)
M3_PILOT_D = 2.6    # M3 셀프태핑용 밑구멍

CLR = 0.3          # 프린팅 공차 (반경 방향) — 프린터에 맞춰 조정
PRINT_FILL = 0.5   # 프린팅 부품 실효 충진율 가정 (벽 3~4줄 + 내부 20~30% 기준 대략값)
PLA_DENSITY = 1240.0  # kg/m^3

# ---------------------------------------------------------------------------
# 설계 파라미터 [mm]
# ---------------------------------------------------------------------------
P = dict(
    # 베이스 (바닥면 볼트 체결) — J1 액추에이터가 기둥 안에 들어감
    base_plate=120.0,      # 팔각 플랜지 맞변 거리
    base_plate_t=6.0,
    base_hole_pcd=98.0,    # 4-M5 볼트 피치원 지름 (45° 방향 배치)
    base_h=66.0,           # 바닥 ~ J1 출력면 (4340P 높이 56.5 + 바닥판 6 + 여유)

    # link1 : 터렛 + 어깨 요크 (J2가 -y 바깥쪽, +y는 베어링)
    j2_z=68.0,
    yoke2_gap=24.0,        # 요크 안쪽 반폭
    yoke2_t=10.0,

    # link2 : 상완 + 팔꿈치 요크
    L2=170.0,
    yoke3_gap=20.0,
    yoke3_t=8.0,

    # link3 : 팔꿈치 허브 + J4 액추에이터 캔
    L3=115.0,

    # link4 : 전완 + 손목 요크
    L4=100.0,
    yoke5_gap=18.0,
    yoke5_t=8.0,

    # link5 : 손목 허브 + J6 액추에이터 캔
    #   J6 캔(4310, Ø57)이 손목 요크판(J5 축 반경 33.5)과 J5 모터 바깥에 오도록 95 mm.
    #   70 mm일 때 캔이 요크판·J5 모터와 겹쳤음. 95 mm에서 J5 ±119°까지 간섭 없음 (한계 ±110°)
    L5=95.0,

    # link6 : 그리퍼 바디 (그리퍼 액추에이터 + 랙·피니언)
    grip_body_len=56.0,    # J6 출력면 ~ 핑거 원점
    finger_len=40.0,
    finger_stroke=20.0,    # 핑거 한쪽 이동량 → 최대 벌림 약 44 mm
)


# ---------------------------------------------------------------------------
# 형상 헬퍼
# ---------------------------------------------------------------------------
def V(*a):
    return cq.Vector(*a)


def cyl(r, p0, p1):
    """p0 -> p1 축을 갖는 원기둥"""
    p0, p1 = V(*p0), V(*p1)
    d = p1 - p0
    return cq.Solid.makeCylinder(r, d.Length, p0, d.normalized())


def box(x0, x1, y0, y1, z0, z1, fillet=0.0, axis="Z"):
    w = (
        cq.Workplane("XY")
        .box(x1 - x0, y1 - y0, z1 - z0)
        .translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    )
    if fillet > 0:
        try:
            w = w.edges("|" + axis).fillet(fillet)
        except Exception:
            pass
    return w.val()


def union(*shapes):
    s = shapes[0]
    for o in shapes[1:]:
        s = s.fuse(o)
    return s.clean()


def _convex_hull(pts):
    pts = sorted(set(pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def hull_cheek_y(y0, y1, circle, base_pts, n=48):
    """
    y방향 두께의 요크 판. 관절 원(cx, cz, r)과 받침점들을 감싸는
    볼록 껍질 → 관절 쪽으로 좁아지는 사다리꼴/물방울 형태
    """
    cx, cz, r = circle
    pts = [(cx + r * math.cos(2 * math.pi * i / n), cz + r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    pts += [tuple(p) for p in base_pts]
    hull = _convex_hull([(round(x, 4), round(z, 4)) for x, z in pts])
    # XZ 평면 법선은 -y → y1에서 시작해 (y1-y0)만큼 -y로 돌출
    return cq.Workplane("XZ", origin=(0, y1, 0)).polyline(hull).close().extrude(y1 - y0).val()


def hollow_beam_z(x, y, z0, z1, wall, r_out):
    """z축 방향 속이 빈 둥근 사각 빔 (내부 = 배선 통로)"""
    outer = box(-x, x, -y, y, z0, z1, fillet=r_out, axis="Z")
    inner = box(-x + wall, x - wall, -y + wall, y - wall, z0 - 1, z1 + 1, fillet=max(r_out - wall, 1.0), axis="Z")
    return outer.cut(inner)


def hollow_beam_x(x0, x1, y, z, wall, r_out):
    outer = box(x0, x1, -y, y, -z, z, fillet=r_out, axis="X")
    inner = box(x0 - 1, x1 + 1, -y + wall, y - wall, -z + wall, z - wall, fillet=max(r_out - wall, 1.0), axis="X")
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# 구매 부품 형상 (표시용)
# ---------------------------------------------------------------------------
def act_r(j):
    return ACT[JOINT_ACT[j]]["D"] / 2


def act_out_r(j):
    """출력 플랜지(회전부) 반경"""
    return ACT[JOINT_ACT[j]]["out_d"] / 2


def act_out_h(j):
    """출력 플랜지가 하우징 앞면에서 튀어나온 길이"""
    return ACT[JOINT_ACT[j]]["out_h"]


def actuator(j, face, inward):
    """
    액추에이터 외형. face = 하우징 앞면(출력 쪽) 중심, inward = 하우징이 뻗는 방향 단위벡터.
    하우징 (H - out_h) + 출력 플랜지 out_h (face에서 바깥쪽으로).
    """
    a = ACT[JOINT_ACT[j]]
    f = V(*face)
    d = V(*inward)
    housing = cyl(a["D"] / 2, f.toTuple(), (f + d * (a["H"] - a["out_h"])).toTuple())
    out = cyl(act_out_r(j), f.toTuple(), (f - d * a["out_h"]).toTuple())
    return union(housing, out)


def bearing_seat_y(body, brg, y_outer, sign):
    """
    요크판에 베어링 자리를 판다. y_outer = 판 바깥면 y, sign = +1 (+y쪽 판).
    베어링은 바깥쪽에서 압입, 안쪽 1.5 mm 이상은 턱으로 남긴다.
    """
    d, D, B = brg
    y_in = y_outer - sign * B
    seat = cyl(D / 2 + CLR * 0.3, (0, y_in, 0), (0, y_outer + sign * 1, 0))
    through = cyl(d / 2 + 1.5, (0, y_outer - sign * 30, 0), (0, y_outer + sign * 1, 0))
    return body.cut(seat).cut(through)


def rack(side, y0, y1, x0=-10.0, x1=-2.0, z_back=17.0):
    """
    모듈 RACK_M 랙 (압력각 20°). side=+1: 피니언 위쪽(+z), 이빨이 아래(-z)를 향함.
    피치선 |z| = PINION_R
    """
    m = RACK_M
    p = math.pi * m
    tan20 = math.tan(math.radians(20))
    zp = PINION_R
    z_tip, z_root = zp - m, zp + 1.25 * m
    w_pitch = p / 4
    w_tip = w_pitch - m * tan20
    w_root = w_pitch + 1.25 * m * tan20
    pts = [(y0, z_back), (y0, z_root)]
    # 이 사이 중심을 y = 0(피니언 바로 위·아래)에 맞춤 → 위·아래 랙이 같은 위상으로 피니언과 물림
    yc = p / 2 + p * math.ceil((y0 + w_root - p / 2) / p)
    while yc + w_root <= y1:
        pts += [(yc - w_root, z_root), (yc - w_tip, z_tip), (yc + w_tip, z_tip), (yc + w_root, z_root)]
        yc += p
    pts += [(y1, z_root), (y1, z_back)]
    if side < 0:
        pts = [(u, -v) for u, v in pts]
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()


def spur_gear(z, m, x0, x1, alpha_deg=20.0, n_inv=8):
    """
    인벌류트 평기어 (축 = x, x0~x1 두께). 이빨 하나가 +z(위) 방향에 오도록 배치.
    표시·간섭 확인용. 구매품 피니언(모듈 m, 잇수 z)과 같은 치형.
    """
    a = math.radians(alpha_deg)
    rp, rb = z * m / 2, z * m / 2 * math.cos(a)
    ra, rf = rp + m, rp - 1.25 * m
    inv = lambda r: math.tan(math.acos(rb / r)) - math.acos(rb / r)
    half = math.pi / (2 * z) + inv(rp)          # 기초원에서 이빨 반각 (인벌류트 시작점)
    flank = [rb + (ra - rb) * (i / n_inv) ** 1.5 for i in range(n_inv + 1)]
    pts = []
    for k in range(z):
        c = math.pi / 2 + k * 2 * math.pi / z
        r_start = max(rf, 0.0)
        side_r = [(r, c - (half - inv(r))) for r in flank]          # 오른쪽 이면 (기초원 → 이끝)
        side_l = [(r, c + (half - inv(r))) for r in reversed(flank)]
        tooth = [(r_start, c - half)] + side_r + side_l + [(r_start, c + half)]
        pts += [(r * math.cos(t), r * math.sin(t)) for r, t in tooth]
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()


def pitch_hub_parts(j, gap, t, brg, r_hub, center=(0, 0, 0), axis_pts=None):
    """
    피치 관절 자식 링크의 허브 + 분리 부품.
      - 허브: 요크 안쪽(±(gap-1))에 들어가는 원판. +y쪽에 핀 구멍
      - adapter: 액추에이터 출력부와 허브 사이 원판 (−y 요크판 구멍을 통해 바깥에서 삽입)
      - pin: +y 베어링 내륜에 끼워져 허브까지 들어가는 축 (바깥에서 삽입)
    → 일체형 요크에 허브를 넣은 뒤 양쪽에서 조립할 수 있음
    """
    cx, cy, cz = center
    d = brg[0]
    hub = cyl(r_hub, (cx, -(gap - 1), cz), (cx, gap - 1, cz))
    pin_depth = 12.0
    hub = hub.cut(cyl(d / 2 + CLR, (cx, gap - 1 - pin_depth, cz), (cx, gap, cz)))
    adapter = cyl(act_out_r(j), (cx, -(gap - 1), cz), (cx, -(gap + t) + act_out_h(j), cz))
    pin = cyl(d / 2, (cx, gap - 1 - pin_depth + 0.5, cz), (cx, gap + t, cz))
    return hub, adapter, pin


# ---------------------------------------------------------------------------
# 링크 형상 (각 링크 자기 좌표계)
#   → dict(body, extras={이름: (형상, 종류)}, motor)
#     종류: "print" 프린팅(PLA/PETG), "tpu" 프린팅(TPU), "buy" 구매품(표시용)
# ---------------------------------------------------------------------------
def r_motor_cheek(j):
    """액추에이터 쪽 요크판 관절 원 반경"""
    return act_r(j) + 5


def r_brg_cheek(j):
    """베어링 쪽 요크판 관절 원 반경"""
    return BRG[JOINT_BRG[j]][1] / 2 + 6


def brg_cap(j, center, y_outer):
    cx, _, cz = center
    r = BRG[JOINT_BRG[j]][1] / 2 + 3
    return cyl(r, (cx, y_outer, cz), (cx, y_outer + 3, cz))


def can_parts(j, L, r_extra=4.0):
    """
    롤 관절용 액추에이터 캔 (x축). 하우징 앞면 = L - out_h, 출력면 = L.
    반환: (can 솔리드, 포켓 컷, 고정 링, 액추에이터 외형, can 시작 x)
    """
    a = ACT[JOINT_ACT[j]]
    x_face = L - a["out_h"]
    x_can0 = L - a["H"] - 6
    r_can = a["D"] / 2 + CLR + r_extra
    can = cyl(r_can, (x_can0, 0, 0), (x_face, 0, 0))
    pocket = cyl(a["D"] / 2 + CLR, (x_face - (a["H"] - a["out_h"]) - 0.5, 0, 0), (x_face + 1, 0, 0))
    ring = cyl(r_can + 0.5, (x_face - 4, 0, 0), (x_face, 0, 0)).cut(
        cyl(a["D"] / 2 + CLR + 0.5, (x_face - 5, 0, 0), (x_face + 1, 0, 0)))
    motor = actuator(j, (x_face, 0, 0), (-1, 0, 0))
    return can, pocket, ring, motor, x_can0, r_can


def make_base():
    # 팔각 플랜지 + 원뿔대 기둥(J1 액추에이터 수납) + 뒤쪽 케이블 출구
    t = P["base_plate_t"]
    zh = P["base_h"]
    a = ACT[JOINT_ACT["J1"]]
    R = P["base_plate"] / 2 / math.cos(math.pi / 8)
    plate = (
        cq.Workplane("XY").polygon(8, 2 * R).extrude(t)
        .rotate((0, 0, 0), (0, 0, 1), 22.5)
        .edges("|Z").fillet(4.0)
        .val()
    )
    rh = P["base_hole_pcd"] / 2
    for k in range(4):
        ang = math.radians(45 + 90 * k)
        hx, hy = rh * math.cos(ang), rh * math.sin(ang)
        plate = plate.cut(cyl(2.75, (hx, hy, -1), (hx, hy, t + 1)))
    z_face = zh - a["out_h"]
    z_bot = zh - a["H"]
    r_pocket = a["D"] / 2 + CLR
    column = cq.Solid.makeCone(r_pocket + 12, r_pocket + 6, z_face - t, V(0, 0, t), V(0, 0, 1))
    body = union(plate, column)
    body = body.cut(cyl(r_pocket, (0, 0, z_bot - 0.5), (0, 0, z_face + 1)))
    body = body.cut(box(-80, -10, -7, 7, z_bot + 1, z_bot + 13, fillet=3.0, axis="X"))  # 케이블 출구
    band = cyl(r_pocket + 8, (0, 0, z_face - 4), (0, 0, z_face)).cut(
        cyl(r_pocket + 0.5, (0, 0, z_face - 5), (0, 0, z_face + 1)))
    motor = actuator("J1", (0, 0, z_face), (0, 0, -1))
    return dict(body=body, extras={"base_band": (band, "print")}, motor=motor)


def make_link1():
    # 터렛 + 비대칭 요크: -y판은 J2 액추에이터, +y판은 베어링
    z2 = P["j2_z"]
    g, t = P["yoke2_gap"], P["yoke2_t"]
    rm = r_motor_cheek("J2")
    px = max(rm, act_r("J1") + 6) + 8
    z_ped = 20
    disk = cyl(act_r("J1") + 6, (0, 0, 0), (0, 0, 8))
    pedestal = box(-px, px, -(g + t), g + t, 8, z_ped)   # 판과 모서리를 맞추기 위해 각진 받침
    base_pts = [(-px, z_ped), (px, z_ped)]
    ch_m = hull_cheek_y(-(g + t), -g, (0, z2, rm), base_pts)
    ch_b = hull_cheek_y(g, g + t, (0, z2, r_brg_cheek("J2")), base_pts)
    body = union(disk, pedestal, ch_m, ch_b)
    body = body.cut(cyl(act_out_r("J2") + 1, (0, -(g + t) - 1, z2), (0, -g + 1, z2)))
    seat = bearing_seat_y(cq.Workplane().add(body).translate((0, 0, -z2)).val(), BRG[JOINT_BRG["J2"]], g + t, +1)
    body = cq.Workplane().add(seat).translate((0, 0, z2)).val()
    body = body.cut(box(-9, 9, -(g - 4), g - 4, -1, z_ped + 1, fillet=3.0))           # 배선 통과 홀
    cap = brg_cap("J2", (0, 0, z2), g + t)
    motor = actuator("J2", (0, -(g + t), z2), (0, -1, 0))
    return dict(body=body, extras={"j2_bearing_cap": (cap, "print")}, motor=motor)


def make_link2():
    # 상완: 속 빈 둥근 사각 빔(배선 통로) + 팔꿈치 비대칭 요크
    L = P["L2"]
    g2, t2 = P["yoke2_gap"], P["yoke2_t"]
    g, t = P["yoke3_gap"], P["yoke3_t"]
    rm = r_motor_cheek("J3")
    hub, adapter, pin = pitch_hub_parts("J2", g2, t2, BRG[JOINT_BRG["J2"]], act_out_r("J2") + 8)
    bx = g - 4
    z_br0, z_br1 = L - rm - 18, L - rm - 6
    beam = hollow_beam_z(bx, g2 - 6, 12, z_br0 + 2, 2.5, 7.0)
    pts = [(-(bx + 4), z_br0), (bx + 4, z_br0)]
    # 요크 = 하나의 윤곽을 전체 폭으로 돌출한 뒤 가운데 홈만 파냄
    #  → 양쪽 판과 브릿지가 이음새 없이 같은 면을 공유 (판·브릿지 단차 없음)
    yoke = hull_cheek_y(-(g + t), g + t, (0, L, rm), pts).cut(box(-80, 80, -g, g, z_br1, L + 80))
    body = union(hub, beam, yoke)
    body = body.cut(cyl(act_out_r("J3") + 1, (0, -(g + t) - 1, L), (0, -g + 1, L)))
    seat = bearing_seat_y(cq.Workplane().add(body).translate((0, 0, -L)).val(), BRG[JOINT_BRG["J3"]], g + t, +1)
    body = cq.Workplane().add(seat).translate((0, 0, L)).val()
    body = body.cut(cyl(5, (-bx - 3, 0, 40), (-bx + 3, 0, 40))).cut(cyl(5, (-bx - 3, 0, z_br0 - 10), (-bx + 3, 0, z_br0 - 10)))
    cap = brg_cap("J3", (0, 0, L), g + t)
    motor = actuator("J3", (0, -(g + t), L), (0, -1, 0))
    return dict(body=body, motor=motor, extras={
        "j2_adapter": (adapter, "print"), "j2_pin": (pin, "print"), "j3_bearing_cap": (cap, "print")})


def make_link3():
    # 팔꿈치 허브 + 속 빈 목 + J4 액추에이터 캔
    L = P["L3"]
    g3, t3 = P["yoke3_gap"], P["yoke3_t"]
    hub, adapter, pin = pitch_hub_parts("J3", g3, t3, BRG[JOINT_BRG["J3"]], act_out_r("J3") + 5)
    can, pocket, ring, motor, x_can0, r_can = can_parts("J4", L)
    neck = hollow_beam_x(12, x_can0 + 2, g3 - 5, g3 - 3, 2.5, 6.0)
    body = union(hub, neck, can).cut(pocket)
    body = body.cut(cyl(5, ((12 + x_can0) / 2, 0, g3), ((12 + x_can0) / 2, 0, g3 - 6)))  # 배선 출입구
    return dict(body=body, motor=motor, extras={
        "j3_adapter": (adapter, "print"), "j3_pin": (pin, "print"), "j4_retainer_ring": (ring, "print")})


def make_link4():
    # 전완: J4 출력에 체결되는 원판 + 몸통 + 손목 비대칭 요크
    L = P["L4"]
    g, t = P["yoke5_gap"], P["yoke5_t"]
    rm = r_motor_cheek("J5")
    disk = cyl(act_r("J4") + 1.5, (0, 0, 0), (5, 0, 0))
    x_br0, x_br1 = L - rm - 18, L - rm - 6
    zb = g + 4
    trunk = cyl(zb, (5, 0, 0), (x_br0 + 2, 0, 0))
    pts = [(x_br0, -zb), (x_br0, zb)]
    # link2와 같은 방식: 한 윤곽을 전체 폭으로 돌출 → 가운데 홈
    yoke = hull_cheek_y(-(g + t), g + t, (L, 0, rm), pts).cut(box(x_br1, L + 80, -g, g, -80, 80))
    body = union(disk, trunk, yoke)
    body = body.cut(cyl(act_out_r("J5") + 1, (L, -(g + t) - 1, 0), (L, -g + 1, 0)))
    seat = bearing_seat_y(cq.Workplane().add(body).translate((-L, 0, 0)).val(), BRG[JOINT_BRG["J5"]], g + t, +1)
    body = cq.Workplane().add(seat).translate((L, 0, 0)).val()
    cap = brg_cap("J5", (L, 0, 0), g + t)
    motor = actuator("J5", (L, -(g + t), 0), (0, -1, 0))
    return dict(body=body, extras={"j5_bearing_cap": (cap, "print")}, motor=motor)


def make_link5():
    # 손목 허브 + 목 + J6 액추에이터 캔
    L = P["L5"]
    g5, t5 = P["yoke5_gap"], P["yoke5_t"]
    hub, adapter, pin = pitch_hub_parts("J5", g5, t5, BRG[JOINT_BRG["J5"]], act_out_r("J5") + 4)
    can, pocket, ring, motor, x_can0, r_can = can_parts("J6", L, r_extra=3.0)
    neck = cyl(g5 - 4, (8, 0, 0), (x_can0 + 2, 0, 0))
    body = union(hub, neck, can).cut(pocket)
    return dict(body=body, motor=motor, extras={
        "j5_adapter": (adapter, "print"), "j5_pin": (pin, "print"), "j6_retainer_ring": (ring, "print")})


def make_link6():
    # 그리퍼 바디: J6 출력 원판 + 그리퍼 액추에이터 캔 + 랙 가이드판 + 피니언
    gl = P["grip_body_len"]
    st = P["finger_stroke"]
    a = ACT[JOINT_ACT["GRIP"]]
    disk = cyl(act_r("J6") + 1.5, (0, 0, 0), (5, 0, 0))
    x_face = 5 + 3 + (a["H"] - a["out_h"])
    r_can = a["D"] / 2 + CLR + 3
    can = cyl(r_can, (5, 0, 0), (x_face, 0, 0))
    ring = cyl(r_can, (x_face, 0, 0), (x_face + 3, 0, 0)).cut(cyl(r_can - 3, (x_face - 1, 0, 0), (x_face + 4, 0, 0)))
    x_plate0, x_plate1 = x_face + 3, gl - 10 - CLR          # 판 앞면과 랙 뒷면 사이 공차
    yw = st + PINION_R + 15
    zw = GRIP_WALL_Z
    plate = box(x_plate0, x_plate1, -yw, yw, -zw, zw, fillet=5.0, axis="X")
    # 바깥 벽 2개: 랙이 피니언 반대쪽(z)으로 빠지지 않게 막음. 앞면에 덮개용 M3 인서트
    x_cov0 = gl - 2 + CLR                                    # 덮개 뒷면 (랙 앞면 + 공차)
    x_cov1 = x_cov0 + GRIP_COVER_T
    for sg in (1, -1):
        plate = union(plate, box(x_plate1 - 1, x_cov0, -yw, yw, *sorted((sg * (RACK_BACK + CLR), sg * zw))))
    wall_zc = (RACK_BACK + CLR + zw) / 2
    bolt_pts = [(yy, zz) for yy in (-(yw - 6), 0.0, yw - 6) for zz in (wall_zc, -wall_zc)]
    for yy, zz in bolt_pts:
        plate = plate.cut(cyl(M3_INSERT_D / 2, (x_cov0 - 6, yy, zz), (x_cov0 + 0.1, yy, zz)))
    # 앞 덮개: 랙 앞을 막고, 스템이 지나는 슬롯 2줄 (위 슬롯은 +y 끝, 아래 슬롯은 −y 끝이 열림
    #   → 덮개를 먼저 달고 핑거를 옆에서 밀어 넣는다)
    cover = box(x_cov0, x_cov1, -yw, yw, -zw, zw, fillet=5.0, axis="X")
    z0, z1 = GRIP_STEM_Z[0] - SLOT_CLR, GRIP_STEM_Z[1] + SLOT_CLR
    cover = cover.cut(box(x_cov0 - 1, x_cov1 + 1, 1.5, yw + 1, z0, z1))
    cover = cover.cut(box(x_cov0 - 1, x_cov1 + 1, -yw - 1, -1.5, -z1, -z0))
    for yy, zz in bolt_pts:
        cover = cover.cut(cyl(M3_CLEAR_D / 2, (x_cov0 - 0.1, yy, zz), (x_cov1 + 0.1, yy, zz)))
    # 카메라 브래킷 장착 패드 (상면 z = 29, M3 인서트 2개)
    pad = box(10, 34, -14, 14, r_can - 4.5, 29, fillet=3.0)
    # 랙 가이드판은 따로 프린팅해 캔 앞에 체결 → 그리퍼 모터를 앞에서 넣은 뒤 판을 덮는다
    body = union(disk, can, ring, pad)
    body = body.cut(cyl(a["D"] / 2 + CLR, (x_face - (a["H"] - a["out_h"]) - 0.5, 0, 0), (x_face + 4, 0, 0)))
    plate = plate.cut(cyl(7, (x_plate0 - 1, 0, 0), (x_plate1 + 1, 0, 0)))               # 피니언 축 구멍
    # 모터 출력부(회전)가 판 뒷면에 닿지 않게 1 mm 깊이로 파 줌
    plate = plate.cut(cyl(act_out_r("GRIP") + 1, (x_plate0 - 0.1, 0, 0), (x_plate0 + 1.0, 0, 0)))
    for yy in (-9, 9):
        body = body.cut(cyl(M3_INSERT_D / 2, (22, yy, 23), (22, yy, 30)))              # 카메라 브래킷용 인서트
    pinion = union(spur_gear(int(round(2 * PINION_R / RACK_M)), RACK_M, gl - 10, gl - 2),
                   cyl(5, (x_face, 0, 0), (gl - 10, 0, 0)))
    motor = actuator("GRIP", (x_face, 0, 0), (-1, 0, 0))
    return dict(body=body, extras={"rack_guide": (plate, "print"), "rack_cover": (cover, "print"),
                                   "pinion_m1_20t": (pinion, "buy")}, motor=motor)


def make_finger(side):
    """
    side=+1: 왼쪽(+y, 피니언 위쪽 랙), -1: 오른쪽(-y, 피니언 아래쪽 랙). 닫힌 상태 기준.
    랙은 핑거와 한 몸으로 프린팅. 안쪽 면 수직 V홈, 바깥쪽 끝 모따기.
    """
    fl = P["finger_len"]
    st = P["finger_stroke"]
    s = side
    xc0 = -2 + CLR + GRIP_COVER_T + CLR          # 덮개 앞면 + 공차 = 연결판 시작 (핑거 좌표)
    pad = box(xc0, fl, *sorted((s * 2, s * 8)), -11, 11, fillet=1.5, axis="Z")
    # 스템: 랙 앞면에서 덮개 슬롯을 지나 앞으로 나옴 (피니언 이끝원 r 11 바깥)
    zs = sorted((s * GRIP_STEM_Z[0], s * GRIP_STEM_Z[1]))
    stem = box(-2.5, xc0 + 0.5, *sorted((s * 2, s * 16)), *zs)
    # 연결판: 덮개 앞에서 스템과 패드를 잇는다
    zc = (-11, GRIP_STEM_Z[1]) if s > 0 else (-GRIP_STEM_Z[1], 11)
    front = box(xc0, xc0 + 4, *sorted((s * 2, s * 16)), zc[0], zc[1], fillet=1.5, axis="X")
    carriage = union(stem, front)
    y_far = -(st + PINION_R + 10)
    rk = rack(s, *sorted((s * y_far, s * 16)), z_back=RACK_BACK)
    vgroove = (
        cq.Workplane("XY", origin=(0, 0, -15))
        .polyline([(fl - 18, s * 1.5), (fl - 11, s * 4.5), (fl - 4, s * 1.5)]).close()
        .extrude(30).val()
    )
    tip = (
        cq.Workplane("XY", origin=(0, 0, -15))
        .polyline([(fl - 8, s * 7.5), (fl + 1, s * 7.5), (fl + 1, s * 3.5)]).close()
        .extrude(30).val()
    )
    body = union(pad, carriage, rk).cut(vgroove).cut(tip)
    pad_tpu = box(4, fl - 20, *sorted((s * 1.0, s * 2.2)), -9, 9)
    return dict(body=body, extras={"pad": (pad_tpu, "tpu")}, motor=None)


def cam_to_link6(shape):
    """카메라 로컬 좌표(본체 중심, x=렌즈 방향) → link6 좌표"""
    return shape.rotate((0, 0, 0), (0, 1, 0), CAM["pitch_deg"]).translate((CAM["x"], 0, CAM["z"]))


def make_camera():
    """
    D435 손목 카메라 마운트.
      bracket : link6 패드에 M3 2개로 체결. 바닥·뒤판·앞 턱으로 카메라 외형을 받친다
      clamp   : 카메라 윗면 뒤쪽을 눌러 고정 (뒤판 윗면에 M3 셀프태핑 2개)
      body    : 카메라 외형 (표시용, 카메라 로컬 좌표)
    D435의 삼각대 나사(1/4-20)·뒷면 M3 위치는 공개 자료에서 확인하지 못해 사용하지 않는다.
    """
    w, d, h = CAM["w"], CAM["d"], CAM["h"]
    t, tb = 4.0, 6.0
    yl = 38.0                               # 받침 폭(반) — 양 끝(USB 포트 쪽)은 열어 둠
    # 카메라 로컬 좌표에서 받침
    bottom = box(-d / 2 - tb, d / 2 + 2, -yl, yl, -h / 2 - t, -h / 2)
    back = box(-d / 2 - tb, -d / 2, -yl, yl, -h / 2 - t, h / 2 - 3)
    lip = box(d / 2, d / 2 + 2, -yl, yl, -h / 2 - t, -h / 2 + 2.5)
    cradle = union(bottom, back, lip)
    for yy in (-20, 20):                    # 클램프 체결 밑구멍
        cradle = cradle.cut(cyl(M3_PILOT_D / 2, (-d / 2 - tb / 2, yy, h / 2 - 11), (-d / 2 - tb / 2, yy, h / 2 - 2)))
    envelope = box(-d / 2 - CLR, d / 2 + CLR, -w / 2 - 1, w / 2 + 1, -h / 2 - CLR, h / 2 + CLR)
    # link6 좌표에서 베이스판 + 기둥
    base = box(10, 34, -14, 14, 29, 33, fillet=3.0)
    strut = box(13, 31, -5, 5, 31, CAM["z"])
    bracket = union(base, strut, cam_to_link6(cradle)).cut(cam_to_link6(envelope))
    for yy in (-9, 9):
        bracket = bracket.cut(cyl(M3_CLEAR_D / 2, (22, yy, 28), (22, yy, 34)))
        bracket = bracket.cut(cyl(3.3, (22, yy, 33), (22, yy, 90)))                 # 렌치 접근 구멍
    # 클램프 (카메라 로컬 → link6)
    ztop = h / 2 + CLR
    clamp = union(box(-d / 2 - tb, -d / 2 + 8, -30, 30, ztop, ztop + 3, fillet=2.0, axis="X"),
                  box(-d / 2 - tb, -d / 2, -30, 30, h / 2 - 3 + 0.3, ztop + 3))
    for yy in (-20, 20):
        clamp = clamp.cut(cyl(M3_CLEAR_D / 2, (-d / 2 - tb / 2, yy, h / 2 - 4), (-d / 2 - tb / 2, yy, ztop + 4)))
    clamp = cam_to_link6(clamp)
    body = box(-d / 2, d / 2, -w / 2, w / 2, -h / 2, h / 2, fillet=3.0, axis="Y")
    return dict(bracket=bracket, clamp=clamp, body=body)


LINKS = {
    "base_link": make_base,
    "link1": make_link1,
    "link2": make_link2,
    "link3": make_link3,
    "link4": make_link4,
    "link5": make_link5,
    "link6": make_link6,
    "finger_left": lambda: make_finger(+1),
    "finger_right": lambda: make_finger(-1),
}
LINK_MOTOR = dict(base_link="J1", link1="J2", link2="J3", link3="J4", link4="J5", link5="J6", link6="GRIP")

BODY_COL = cq.Color(0.90, 0.90, 0.88)     # 프린팅 본체: 웜 화이트
ACCENT_COL = cq.Color(0.95, 0.45, 0.10)   # 캡/패드/분리 프린팅 부품: 주황
MOTOR_COL = cq.Color(0.25, 0.26, 0.28)    # 액추에이터: 짙은 회색
STEEL_COL = cq.Color(0.70, 0.72, 0.75)    # 베어링·피니언: 은색


# ---------------------------------------------------------------------------
# 지지 베어링 (STEP 표시용. 질량·URDF 메쉬에는 넣지 않음)
# ---------------------------------------------------------------------------
def bearing_places():
    """부모 링크: (관절, 관절 중심, 베어링 쪽 요크판 바깥면 y)"""
    return {
        "link1": ("J2", (0, 0, P["j2_z"]), P["yoke2_gap"] + P["yoke2_t"]),
        "link2": ("J3", (0, 0, P["L2"]), P["yoke3_gap"] + P["yoke3_t"]),
        "link4": ("J5", (P["L4"], 0, 0), P["yoke5_gap"] + P["yoke5_t"]),
    }


def bearing_display(link):
    j, (cx, _, cz), y_out = bearing_places()[link]
    d, D, B = BRG[JOINT_BRG[j]]
    return cyl(D / 2, (cx, y_out - B, cz), (cx, y_out, cz)).cut(
        cyl(d / 2, (cx, y_out - B - 1, cz), (cx, y_out + 1, cz)))


# ---------------------------------------------------------------------------
# 분해도 (cad/arm_exploded.step)
#   링크는 롤 관절(J1, J4, J6, 그리퍼)에서만 축 방향으로 벌리고,
#   각 관절의 체결 부품은 조립 방향(끼워 넣는 방향)으로 빼 놓는다.
#   부품 순서 = 조립 순서: 안쪽(링크 쪽)에 있는 부품이 먼저 들어간다.
# ---------------------------------------------------------------------------
EXPLODE_LINK = {   # 링크 전체 이동 [mm] (홈 자세 위치에 더함)
    "base_link": (0, 0, 0),
    "link1": (0, 0, 170), "link2": (0, 0, 170), "link3": (0, 0, 170),
    "link4": (190, 0, 170), "link5": (190, 0, 170),
    "link6": (380, 0, 170),
    "finger_left": (570, 90, 170), "finger_right": (570, -90, 170),   # 덮개를 단 뒤 옆(±y)에서 밀어 넣음
}
EXPLODE_PART = {   # 링크 안에서 부품별 추가 이동 [mm]
    "base_link": {"motor": (0, 0, 75), "base_band": (0, 0, 105)},
    "link1": {"motor": (0, -80, 0), "bearing": (0, 40, 0), "j2_bearing_cap": (0, 90, 0)},
    "link2": {"j2_adapter": (0, -45, 0), "j2_pin": (0, 65, 0),
              "motor": (0, -80, 0), "bearing": (0, 40, 0), "j3_bearing_cap": (0, 90, 0)},
    "link3": {"j3_adapter": (0, -45, 0), "j3_pin": (0, 65, 0), "motor": (80, 0, 0), "j4_retainer_ring": (140, 0, 0)},
    "link4": {"motor": (0, -80, 0), "bearing": (0, 40, 0), "j5_bearing_cap": (0, 90, 0)},
    "link5": {"j5_adapter": (0, -45, 0), "j5_pin": (0, 65, 0), "motor": (80, 0, 0), "j6_retainer_ring": (140, 0, 0)},
    "link6": {"motor": (60, 0, 0), "rack_guide": (90, 0, 0), "pinion_m1_20t": (140, 0, 0), "rack_cover": (190, 0, 0)},
    "camera": {"camera_mount": (0, 0, 60), "camera_d435": (0, 0, 120)},
}


def build_assembly(parts_by_link, cam, home, exploded=False):
    asm = cq.Assembly(name="arm_exploded" if exploded else "arm")

    def add(link, pname, shape, col, base):
        off = V(*EXPLODE_PART.get(link, {}).get(pname, (0, 0, 0))) if exploded else V(0, 0, 0)
        asm.add(shape, name=f"{link}__{pname}", loc=cq.Location(base + off), color=col)

    for name, parts in parts_by_link.items():
        base = V(*home[name]) + (V(*EXPLODE_LINK[name]) if exploded else V(0, 0, 0))
        add(name, "body", parts["body"], BODY_COL, base)
        for ename, (shp, kind) in parts["extras"].items():
            add(name, ename, shp, STEEL_COL if kind == "buy" else ACCENT_COL, base)
        if parts["motor"] is not None:
            add(name, "motor", parts["motor"], MOTOR_COL, base)
        if name in bearing_places():
            add(name, "bearing", bearing_display(name), STEEL_COL, base)
    base6 = V(*home["link6"]) + (V(*EXPLODE_LINK["link6"]) if exploded else V(0, 0, 0))
    add("camera", "camera_mount", union(cam["bracket"], cam["clamp"]), ACCENT_COL, base6)
    add("camera", "camera_d435", cam_to_link6(cam["body"]), MOTOR_COL, base6)
    return asm


# ---------------------------------------------------------------------------
# 질량/관성 (OCCT 정밀 적분)
# ---------------------------------------------------------------------------
def _gprops(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, g)
    c = g.CentreOfMass()
    m = g.MatrixOfInertia()
    return g.Mass(), (c.X(), c.Y(), c.Z()), [[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]


def part_props(shape, density=None, mass=None):
    """밀도 또는 총질량을 주면 (m [kg], com [m], I_com [kg m^2])"""
    vol, c, Iu = _gprops(shape)              # vol mm^3, Iu = 밀도 1 기준 mm^5
    if mass is not None:
        density = mass / (vol * 1e-9)
    m = density * vol * 1e-9
    I = [[Iu[i][j] * density * 1e-15 for j in range(3)] for i in range(3)]
    return m, tuple(v * 1e-3 for v in c), I


def combine(parts):
    M = sum(p[0] for p in parts)
    C = tuple(sum(p[0] * p[1][k] for p in parts) / M for k in range(3))
    I = [[0.0] * 3 for _ in range(3)]
    for m, c, Ic in parts:
        d = [c[k] - C[k] for k in range(3)]
        dd = sum(v * v for v in d)
        for i in range(3):
            for j in range(3):
                I[i][j] += Ic[i][j] + m * ((dd if i == j else 0.0) - d[i] * d[j])
    return dict(mass=M, com=C, ixx=I[0][0], iyy=I[1][1], izz=I[2][2], ixy=I[0][1], ixz=I[0][2], iyz=I[1][2])


# 홈 자세(모든 관절 0)에서 각 링크 원점 위치 [mm] — STEP 조립용
def home_positions():
    zb, z2, L2 = P["base_h"], P["j2_z"], P["L2"]
    j3 = (0, 0, zb + z2 + L2)
    x4 = P["L3"]
    x5 = x4 + P["L4"]
    x6 = x5 + P["L5"]
    xf = x6 + P["grip_body_len"]
    return {
        "base_link": (0, 0, 0),
        "link1": (0, 0, zb),
        "link2": (0, 0, zb + z2),
        "link3": j3,
        "link4": (x4, 0, j3[2]),
        "link5": (x5, 0, j3[2]),
        "link6": (x6, 0, j3[2]),
        "finger_left": (xf, 0, j3[2]),
        "finger_right": (xf, 0, j3[2]),
    }


def write_properties(props):
    out_dir = os.path.join(PKG, "urdf", "generated")
    os.makedirs(out_dir, exist_ok=True)
    mm = lambda v: f"{v * 1e-3:.6f}"
    lines = [
        '<?xml version="1.0"?>',
        "<!-- AUTO-GENERATED by cad/generate_cad.py : 직접 수정하지 말고 스크립트의 P를 수정 -->",
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
        f'  <xacro:property name="arm_base_h"   value="{mm(P["base_h"])}"/>',
        f'  <xacro:property name="arm_j2_z"     value="{mm(P["j2_z"])}"/>',
        f'  <xacro:property name="arm_L2"       value="{mm(P["L2"])}"/>',
        f'  <xacro:property name="arm_L3"       value="{mm(P["L3"])}"/>',
        f'  <xacro:property name="arm_L4"       value="{mm(P["L4"])}"/>',
        f'  <xacro:property name="arm_L5"       value="{mm(P["L5"])}"/>',
        f'  <xacro:property name="arm_grip_len" value="{mm(P["grip_body_len"])}"/>',
        f'  <xacro:property name="arm_finger_len" value="{mm(P["finger_len"])}"/>',
        f'  <xacro:property name="arm_finger_stroke" value="{mm(P["finger_stroke"])}"/>',
        f'  <xacro:property name="arm_cam_x"     value="{mm(CAM["x"])}"/>',
        f'  <xacro:property name="arm_cam_z"     value="{mm(CAM["z"])}"/>',
        f'  <xacro:property name="arm_cam_pitch" value="{math.radians(CAM["pitch_deg"]):.6f}"/>',
        f'  <xacro:property name="arm_cam_d"     value="{mm(CAM["d"])}"/>',
        f'  <xacro:property name="arm_cam_w"     value="{mm(CAM["w"])}"/>',
        f'  <xacro:property name="arm_cam_h"     value="{mm(CAM["h"])}"/>',
        "",
        f"  <!-- 관성: 프린팅 부품 PLA {PLA_DENSITY:.0f} kg/m^3 x 실효 충진율 {PRINT_FILL}"
        " + 액추에이터 사양표 질량(균일 원기둥 가정). 베어링·볼트·배선 미포함 -->",
    ]
    for name, p in props.items():
        lines.append(
            f'  <xacro:macro name="inertial_{name}">\n'
            f"    <inertial>\n"
            f'      <origin xyz="{p["com"][0]:.6f} {p["com"][1]:.6f} {p["com"][2]:.6f}" rpy="0 0 0"/>\n'
            f'      <mass value="{p["mass"]:.6f}"/>\n'
            f'      <inertia ixx="{p["ixx"]:.4e}" ixy="{p["ixy"]:.4e}" ixz="{p["ixz"]:.4e}"\n'
            f'               iyy="{p["iyy"]:.4e}" iyz="{p["iyz"]:.4e}" izz="{p["izz"]:.4e}"/>\n'
            f"    </inertial>\n"
            f"  </xacro:macro>"
        )
    lines.append("</robot>\n")
    with open(os.path.join(out_dir, "arm_properties.xacro"), "w") as f:
        f.write("\n".join(lines))


def export_stl(shape, path):
    cq.exporters.export(cq.Workplane().add(shape), path, tolerance=0.05, angularTolerance=0.1)


def main():
    step_dir = os.path.join(HERE, "step")
    mesh_dir = os.path.join(PKG, "meshes")
    os.makedirs(step_dir, exist_ok=True)
    os.makedirs(mesh_dir, exist_ok=True)
    for f in os.listdir(mesh_dir):
        if f.endswith(".stl"):
            os.remove(os.path.join(mesh_dir, f))

    home = home_positions()
    parts_by_link = {}
    props = {}
    rho_print = PLA_DENSITY * PRINT_FILL

    print_dir = os.path.join(PKG, "print")
    os.makedirs(print_dir, exist_ok=True)
    for f in os.listdir(print_dir):
        if f.endswith(".stl"):
            os.remove(os.path.join(print_dir, f))
    bom_print = []

    for name, fn in LINKS.items():
        parts = fn()
        parts_by_link[name] = parts
        body, motor, extras = parts["body"], parts["motor"], parts["extras"]
        accent = union(*[shp for shp, _ in extras.values()])

        cq.exporters.export(cq.Workplane().add(body), os.path.join(step_dir, f"{name}.step"))
        export_stl(body, os.path.join(mesh_dir, f"{name}_body.stl"))
        export_stl(accent, os.path.join(mesh_dir, f"{name}_accent.stl"))
        # 프린팅용 개별 파일
        export_stl(body, os.path.join(print_dir, f"{name}.stl"))
        bom_print.append((f"{name}.stl", "print", body.Volume()))
        for ename, (shp, kind) in extras.items():
            if kind == "buy":
                continue
            fname = f"{name}__{ename}{'_TPU' if kind == 'tpu' else ''}.stl"
            export_stl(shp, os.path.join(print_dir, fname))
            cq.exporters.export(cq.Workplane().add(shp), os.path.join(step_dir, f"{name}__{ename}.step"))
            bom_print.append((fname, kind, shp.Volume()))
        col = [body, accent]
        mp = [part_props(body, density=rho_print), part_props(accent, density=rho_print)]
        if motor is not None:
            export_stl(motor, os.path.join(mesh_dir, f"{name}_motor.stl"))
            col.append(motor)
            mp.append(part_props(motor, mass=ACT[JOINT_ACT[LINK_MOTOR[name]]]["mass"]))
        export_stl(union(*col), os.path.join(mesh_dir, f"{name}_collision.stl"))
        props[name] = combine(mp)

        p = props[name]
        bb = body.BoundingBox()
        print(f"{name:13s} mass={p['mass'] * 1000:7.1f} g  print={mp[0][0] * 1000:6.1f} g  "
              f"body bbox={bb.xlen:5.1f} x {bb.ylen:5.1f} x {bb.zlen:5.1f} mm  valid={body.isValid()}")

    # 손목 카메라 (브래킷 + 클램프는 link6 좌표, 카메라 외형은 카메라 로컬 좌표)
    cam = make_camera()
    mount = union(cam["bracket"], cam["clamp"])
    export_stl(mount, os.path.join(mesh_dir, "camera_mount_body.stl"))
    export_stl(cam["body"], os.path.join(mesh_dir, "camera_d435_body.stl"))
    for ename in ("bracket", "clamp"):
        fname = f"camera__{ename}.stl"
        export_stl(cam[ename], os.path.join(print_dir, fname))
        cq.exporters.export(cq.Workplane().add(cam[ename]), os.path.join(step_dir, f"camera__{ename}.step"))
        bom_print.append((fname, "print", cam[ename].Volume()))
    props["camera_mount"] = combine([part_props(mount, density=rho_print)])
    props["camera_d435"] = combine([part_props(cam["body"], mass=CAM["mass"])])
    print(f"camera_mount  mass={props['camera_mount']['mass'] * 1000:7.1f} g  valid={cam['bracket'].isValid()} {cam['clamp'].isValid()}")

    build_assembly(parts_by_link, cam, home).export(os.path.join(HERE, "arm_assembly.step"))
    build_assembly(parts_by_link, cam, home, exploded=True).export(os.path.join(HERE, "arm_exploded.step"))
    write_properties(props)
    with open(os.path.join(print_dir, "parts_list.txt"), "w") as f:
        f.write("# 프린팅 부품 목록 (generate_cad.py 자동 생성)\n")
        f.write("# 파일명                                   재질      솔리드 부피[cm^3]\n")
        for fname, kind, vol in bom_print:
            f.write(f"{fname:42s} {'TPU' if kind == 'tpu' else 'PLA/PETG':9s} {vol / 1000:8.1f}\n")
    print("done")


if __name__ == "__main__":
    main()
