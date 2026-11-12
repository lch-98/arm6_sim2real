# 05. 모터 단품 벤치 (로드맵 1단계)

> 상태: **준비물 정리 완료, 부품 도착 대기**. 진행하면서 절차와 결과를 이 문서에 채운다.

목표: 키트 모터 중 2개(4340P 1개, 4310 1개)를 팔에 달기 전에 책상 위에서 돌려 보고, **통신·제어 방법을 확정하고 모터 특성을 측정**한다.
여기서 측정한 값(토크 보정, 마찰, 지연)이 시뮬레이션의 모터 모델이 된다.

## 1. 연결 구성

```
  [PC (Ubuntu)] ──USB── [Damiao USB-CAN 보드 (키트)] ──CAN_H/CAN_L──┐
                         (종단저항 120 Ω)                          │
                                                                   ▼
  [벤치 파워 24 V] ──[퓨즈]──[비상정지]──24V/GND──▶ [XT30(2+2) 전원 분리 보드 (키트)]
                                                                   │
                                         ┌─────────────────────────┘
                                         ▼
                               [DM-J4340P] ═XT30(2+2)═ [DM-J4310] ── [종단저항 120 Ω]
                                 ▲  ↑ 데이지 체인 (전원 + CAN 같이)
                                 │
                       (설정: USB-CAN 보드로 하거나, USB-TTL ──GH1.25 3핀── 한 대씩)
```

- Damiao J 시리즈는 **XT30(2+2)** 커넥터 하나에 전원(24V/GND)과 CAN(CAN_H/CAN_L)이 같이 들어 있다. 모터마다 이 커넥터가 **2개**라서 모터끼리 이어 붙일 수 있다(데이지 체인)
- 전원 분리 보드: 전원은 벤치 파워(나중에 SMPS)에서, CAN은 USB-CAN 보드에서 받아 XT30(2+2) 한 줄로 합쳐 모터 쪽으로 보낸다
- **GH1.25 3핀**(TX/RX/GND)은 설정용 시리얼 포트. CAN ID, 제어 모드 같은 파라미터를 바꿀 때만 쓴다
- 드라이버가 모터 안에 있으므로 **별도 모터 제어기(드라이버, MCU)는 필요 없다**. PC가 CAN으로 직접 명령한다

## 2. 준비물

### 2-1. 모터 키트 (구매 확정)

**Seeed reBot Arm B601-DM Body Motor Kit** (디바이스마트, 1,713,030원 VAT 포함, 2026-11 기준)

| 구성품 | 수량 | 이 프로젝트에서 |
|---|---|---|
| DM-J4340P-2EC | 3 | J1, J2, J3 (벤치에서는 1개) |
| DM-J4310-2EC | 3 | J4, J5, J6 (벤치에서는 1개) |
| Damiao USB-CAN 보드 | 1 | PC ↔ CAN 버스 |
| XT30(2+2) 전원 분리 보드 | 1 | 전원 + CAN 합치기 |
| XT30(2+2) 케이블 | 여러 개 | 모터 ↔ 모터, 보드 ↔ 모터 |

받으면 확인: 정격 전압(24 V), 펌웨어 버전, 정격 토크 표기(4340P 9 N·m 또는 12 N·m), USB-CAN 보드의 종단저항 내장·스위치 여부, 전원 분리 보드의 입력 단자 형태.

### 2-2. 키트와 같이 구매 (이번 단계)

| 품목 | 수량 | 고를 때 확인할 것 | 용도 |
|---|---|---|---|
| 벤치 파워 (0~30 V, 전류 제한) | 1 | **전류 제한(CC) 기능 필수**, 5 A 이상 | 첫 전원 투입 |
| 비상정지 스위치 | 1 | 검색어: "비상정지 스위치 22파이 회전복귀", "푸시락 턴리셋", "E-STOP 버튼 박스". **B접점(NC) 포함**, 버튼 박스(케이스) 일체형이면 배선이 편함. 예: Omron A22E, IDEC XW 계열 | 모터 전원 차단 (2-2-2 참고) |
| 24 V 릴레이 (또는 DC 접촉기) + 소켓 | 1 | 코일 24 V DC, 접점 **DC 24 V 20 A 이상** | 비상정지 버튼 대신 큰 전류를 끊음 |
| 블레이드 퓨즈 홀더 + 5 A 퓨즈 | 1 + 여분 | 18 AWG 선 연결형 | 단락 보호 |
| 120 Ω 저항 | 2~3 | 1/4 W | CAN 버스 끝 종단 (USB-CAN 보드에 내장이면 끝 쪽 1개만 사용) |
| 실리콘 전선 18 AWG (빨강·검정) | 각 2 m | 내열 실리콘 피복 | 벤치 파워 → 퓨즈 → 비상정지 → 분리 보드 |
| 수축튜브 세트 | 1 | | 납땜부 절연 |
| XT30(2+2) 피그테일 (한쪽이 전선) | 1~2 | **선택**. 분리 보드 입력이 단자대가 아니면 필요 | 전원 입력, 종단저항 |
| USB-TTL 변환 모듈 (3.3 V) + GH1.25 3핀 케이블 | 1 | **선택**. USB-CAN 보드로 설정이 되면 생략 | 모터 설정 (시리얼) |

### 2-2-2. 비상정지 배선

비상정지 버튼의 접점은 보통 교류 기준 정격이 크고 **직류 24 V에서는 정격이 훨씬 낮다**. 그래서 버튼으로 모터 전류를 직접 끊지 않고, 버튼이 릴레이 코일을 끊고 릴레이가 모터 전원을 끊게 한다.

```
  [24 V +] ──[퓨즈]──┬──[비상정지 B접점]──[릴레이 코일]── [GND]
                      │
                      └──[릴레이 접점 (a접점)]──▶ 전원 분리 보드 24 V 입력
```

- 버튼을 누르면 코일 전원이 끊겨 릴레이가 떨어지고 모터 전원이 차단된다. 선이 빠져도 차단되는 쪽으로 동작한다
- 차량용 5핀 릴레이(예: JD2914) 단자: 85·86 = 코일, 30 = 공통, 87 = 평상시 열림(a접점), 87a = 평상시 닫힘. **30 → 87**을 쓴다
- 코일 양단에 다이오드(1N4007)를 역방향으로 단다: 띠(캐소드) 쪽을 코일 +24 V 쪽에, 반대쪽을 GND 쪽에. 코일이 끊길 때 생기는 역전압을 흡수한다
- 비상정지 버튼은 **b접점(NC)** 단자를 쓴다 (1a1b 제품이면 b 표시 단자)
- 1단계(모터 1~2개, 벤치 파워 전류 제한 2~3 A)에서는 버튼 데이터시트의 **DC 24 V 정격**이 전류 제한값보다 크면 버튼을 직접 직렬로 넣어도 된다. 3단계부터는 릴레이 방식으로 바꾼다
- 모터에 브레이크가 없으므로 차단하면 팔이 떨어진다. 비상정지 전에 팔을 받칠 수 있는 자세에서 시험한다

### 2-2-1. USB-CAN 보드를 ROS에서 쓰는 방법

키트 보드가 Linux에서 어떻게 잡히는지 먼저 확인한다.

```bash
lsusb                 # 장치가 잡히는지
ip link | grep can    # can0 이 보이면 SocketCAN 방식 → ROS에서 그대로 사용
ls /dev/ttyACM* /dev/ttyUSB*   # 대신 이쪽에 생기면 시리얼 방식 → slcand 또는 Damiao 전용 SDK 사용
```

| 결과 | 할 일 |
|---|---|
| `can0` 생김 | SocketCAN으로 진행 (권장) |
| `/dev/ttyACM0` 등만 생김 | `slcand`로 `can0` 생성 시도, 안 되면 Damiao 제공 SDK로 하드웨어 인터페이스 작성 |
| 둘 다 불편 | candleLight/gs_usb 계열(예: CANable) 어댑터를 따로 사서 ROS용으로 쓰고, 키트 보드는 설정용으로 |

### 2-3. 이미 있거나 다음 단계와 같이 쓰는 것

| 품목 | 용도 |
|---|---|
| 멀티미터 | 전원 극성, CAN 종단 저항 확인 (전원 끈 상태에서 CAN_H–CAN_L 약 60 Ω이면 정상) |
| 납땜 인두, 수축튜브, 와이어 스트리퍼 | XT30 케이블·종단저항 만들기 |
| 디지털 캘리퍼스 | 모터 체결 구멍 실측 (도면이 없을 때) |
| 주방 저울 (0.1 g, 최대 수 kg) | 시험 추 무게 측정 |
| 시험 추 (100~500 g) | 토크 보정. 볼트·너트 묶음을 저울로 재서 써도 됨 |
| 바이스 또는 클램프 | 모터 고정 (첫 구동 시 모터가 튀지 않게) |
| 볼트 세트 M2.5 / M3 | 출력 플랜지에 레버 체결 (규격은 도면 확인 후 구매) |
| Windows PC | Damiao 디버깅 어시스턴트 실행 |

### 2-4. 소프트웨어

```bash
sudo apt install can-utils     # candump, cansend
pip install python-can         # 파이썬에서 CAN 테스트
```

Windows: Damiao 디버깅 어시스턴트(판매처·Damiao 자료실에서 받음). CAN ID, Master ID, 제어 모드, 영점, P_MAX·V_MAX·T_MAX를 설정한다.

## 3. 모터가 오면 먼저 할 일

1. **치수 실측·도면 확보**: 하우징·출력 플랜지 볼트 구멍(개수, 피치원 지름, 나사 규격). 벤치 고정 지그와 단계 2(체결 설계)에 필요
2. 펌웨어 버전, 정격 토크·속도(사양서 기준) 기록 → `docs/01_design.md` 액추에이터 표 갱신
3. 도면 값은 아래 3-1에 정리. 실물에서 볼트 구멍 깊이를 캘리퍼스로 다시 확인
4. 키트 구성품·케이블 개수 확인 → 부족한 XT30(2+2)·GH1.25 케이블만 추가 구매
5. 모터 6개 모두 한 번씩 전원을 넣어 응답 확인 (초기 불량은 빨리 교환)

### 3-1. 판매처 도면 (디바이스마트 키트 상세 페이지)

| 항목 | DM-J4340P-2EC | DM-J4310-2EC |
|---|---|---|
| 하우징 외경 | Ø57 | Ø57 |
| 전체 높이 | 56.5 (본체 54.75 + 출력면 1.5) | 46 (본체 45) |
| 출력면 하우징 체결 | 6 × M3, 깊이 5, 피치원 Ø50 | 6 × M3, 깊이 5, 피치원 Ø50 (60° 간격, 세로 중심선 위·아래에 1개씩) |
| 출력 플랜지 체결 | 6 × M3, 깊이 6, 피치원 Ø27 | 6 × M3, 깊이 5, 피치원 Ø27 (60° 간격) |
| 출력 플랜지 위치 핀 | 도면에 없음 | 3 × Ø4 (+0.02~+0.04), 피치원 Ø24.3, M3 구멍 사이 (120° 간격) |
| 하우징 위치 핀 | 도면에 없음 | 2 × Ø3 (+0.02~+0.04), 깊이 5, 피치원 Ø50, 세로 중심선에서 15°, 서로 마주 봄 |
| 뒷면 체결 | 4 × M2.5, 깊이 4, 피치원 Ø38 | 4 × M3, 깊이 4, 피치원 Ø38 |
| 출력 플랜지 (회전부) | 약 Ø38 (도면 축척으로 잰 값, 실측 확인), 하우징 앞면에서 1.5 | Ø35, 하우징 앞면에서 1.0 |

- 외경·높이·출력 플랜지는 현재 CAD(`ACT` 값)에 반영했다. 질량은 Seeed 자료값(4340P 약 375 g, 4310 약 300 g)이므로 받으면 저울로 재서 바꾼다
- 2단계(체결 설계)에서 이 값으로 볼트 구멍·위치 핀 구멍을 넣는다

## 4. 측정 항목 (진행하면서 채움)

| 항목 | 방법 | 벤치 명령 (5절) | 반영처 |
|---|---|---|---|
| 절대 위치 유지 | 임의 각도에서 전원 껐다 켜기 | `monitor` | 원점 복귀 필요 여부 |
| 토크 보정 | 레버 + 추를 정지 유지할 때 모터 보고 토크 vs m·g·L | `hold --lever` | 토크 보정계수 |
| 마찰 | 여러 속도로 등속 회전 → 평균 토크 | `friction` | URDF `dynamics` (마찰·감쇠) |
| 백래시 | 레버 끝을 양방향으로 밀며 각도 변화 | `monitor` (손으로) | 랜덤화 범위 |
| 통신 지연 | 명령 → 응답 왕복 시간 | `latency`, `candump -td` | 시뮬레이션 지연 모델 |
| 위치 루프 응답 | 기준 스윕(1축)을 실물에 돌려 추종 오차 측정 | `sweep` | Gazebo 대비 모터 모델 |
| 발열 | 정격 근처 토크 유지하며 온도 기록 | `hold` (CSV의 temp 열) | 연속 사용 한계 |

## 5. 벤치 소프트웨어 (`arm_hardware` 패키지)

모터 없이도 미리 연습할 수 있게 **가상 모터**를 같이 넣었다. 실물과 같은 CAN 프로토콜로 동작한다.

| 파일 | 내용 |
|---|---|
| `arm_hardware/dm_protocol.py` | Damiao 프레임 변환 (MIT 명령, 응답, 시스템 명령, 레지스터). ROS 의존 없음 |
| `arm_hardware/transport.py` | CAN 연결: SocketCAN(can0·vcan0), Damiao USB-CAN 시리얼(/dev/ttyACM*) |
| `arm_hardware/motor.py` | 모터 여러 대를 한 버스에서 다루는 클래스 (사용·해제·영점·MIT·레지스터 읽기/쓰기) |
| `arm_hardware/fake_motor.py` | 가상 모터 (단순 동역학 + 레버 추). 마찰·관성 값은 가상값 |
| `arm_hardware/bench.py` | 벤치 명령 `dm_bench` |
| `config/motors.yaml` | 관절별 모터 ID 계획(ESC_ID 0x01~0x07, MST_ID +0x10), 모델별 기본 게인·안전 한계 |
| `test/` | 프로토콜 변환·가상 모터 통합 시험 (`colcon test --packages-select arm_hardware`) |

### 5-1. 준비

```bash
python3 -c "import can; print(can.__version__)"     # 4.x 여야 함
pip install --user "python-can>=4" pyserial          # 3.x 이거나 없으면
cd ~/ros2_ws && colcon build --symlink-install --packages-select arm_hardware && source install/setup.bash
```

### 5-2. 모터 없이 연습

```bash
ros2 run arm_hardware dm_bench --fake scan                       # 가상 모터 7대가 보임
ros2 run arm_hardware dm_bench --fake --joint joint2 move --to 0.5
ros2 run arm_hardware dm_bench --fake --fake-lever 0.5,0.1 --joint joint2 hold --seconds 5 --lever 0.5,0.1
```

`--fake`는 같은 프로그램 안에서 가상 모터를 띄운다. 실제 CAN 경로(SocketCAN)까지 시험하려면 리눅스 가상 CAN을 쓴다.

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
ros2 run arm_hardware dm_fake_motor --channel vcan0 --motor 1:0x11:DM-J4340P-2EC     # 터미널 1
ros2 run arm_hardware dm_bench --channel vcan0 --joint joint1 monitor                # 터미널 2
candump vcan0                                                                         # 터미널 3: 오가는 프레임 보기
```

### 5-3. 모터가 오면 (한 대씩)

연결 옵션: USB-CAN 보드가 `can0`으로 잡히면 `--iface socketcan --channel can0`(기본값), 시리얼로 잡히면 `--iface damiao-serial --channel /dev/ttyACM0` (2-2-1 참고).

```bash
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0   # SocketCAN일 때
```

| 순서 | 명령 | 확인할 것 |
|---|---|---|
| 1 | `dm_bench scan` | 응답하는 모터, 현재 ESC_ID·MST_ID·제어 모드·PMAX/VMAX/TMAX |
| 2 | `dm_bench --id <현재> set-id --new-id 0x02 --new-mst 0x12` | 버스에 **한 대만** 연결. 전원을 껐다 켜고 `scan`으로 확인. 바뀌지 않으면 Damiao 디버깅 도구(Windows)로 변경 |
| 3 | `dm_bench --joint joint2 set-mode --save` | 제어 모드 MIT(1) |
| 4 | `dm_bench --joint joint2 info` | 변환 범위, 해상도, 온도, 보호 설정 |
| 5 | `dm_bench --joint joint2 monitor` | 손으로 돌려 위치 부호·단위 확인. 전원을 껐다 켜도 위치가 유지되는지 (절대 위치) |
| 6 | `dm_bench --joint joint2 zero` | 레버가 수평일 때 영점 저장 |
| 7 | `dm_bench --joint joint2 move --to 0.3 --relative` | 작은 이동부터 |
| 8 | `dm_bench --joint joint2 hold --seconds 10 --lever <kg>,<m> --approach 0.1` | 토크 보정. 위·아래에서 각각 다가와 유지 → 두 방향 평균 = 부하 토크, 차이의 절반 = 정지 마찰 |
| 9 | `dm_bench --joint joint2 friction` | 레버 없이, 출력축이 자유롭게 돌 때 |
| 10 | `dm_bench --joint joint2 latency` | 왕복 시간 분포 |
| 11 | `dm_bench --joint joint2 sweep --ref-joint joint2 --scale 0.3` | 문제없으면 `--scale 1.0` |

- 모든 결과는 `~/arm_bench_logs/`에 CSV로 남는다 (`--out`으로 변경)
- **안전**: 아래 경우 즉시 모터를 사용 안 함으로 바꾸고 멈춘다 — Ctrl+C, 응답 5번 연속 없음, 모터 오류 상태, 토크·속도가 `motors.yaml`의 한계 초과, 시작 위치에서 1.6 rad 초과 이동, 온도 60 °C 초과. 모터에 브레이크가 없으므로 레버가 떨어져도 안전한 방향으로 둔다
- 처음 게인은 작게 잡혀 있다 (4340P kp 8, kd 0.4 / 4310 kp 4, kd 0.15). 바꿀 때는 `--kp`, `--kd`
- MIT 위치 제어는 비례 제어라서 **정지 오차 ≈ 부하 토크 ÷ kp**만큼 남는다. 예: 레버 부하 0.49 N·m, kp 8 → 약 60 mrad 처짐 (정지 마찰이 일부를 받으면 더 작음). 오차를 줄이려면 kp를 올리거나, 팔에서는 중력 토크를 피드포워드(tau)로 같이 보낸다
- `set-id`의 ID 변경·저장 순서는 공개 자료 기준이며 실물에서 처음 확인한다

참고: 프레임 정의는 Damiao 공개 자료와 [DM_Control_Python](https://github.com/cmjang/DM_Control_Python)(MIT License)을 따랐다. 공장 기본 변환 범위(4310: 12.5 / 30 / 10, 4340P: 12.5 / 8 / 28)는 [Seeed 위키](https://wiki.seeedstudio.com/damiao_series/) 기준이며, 도구는 실행할 때 모터 레지스터에서 다시 읽는다.

## 6. ros2_control 하드웨어 인터페이스 (`arm_hardware/DamiaoSystem`)

mock·Gazebo 자리에 실물 모터를 넣는 C++ 플러그인이다. 컨트롤러 설정, 기준 궤적, 기록·분석 스크립트는 0단계와 **그대로** 쓴다.

| 항목 | 내용 |
|---|---|
| 제어 방식 | 200 Hz마다 관절마다 MIT 명령 1프레임 (목표 위치 + kp·kd, 속도·토크 피드포워드는 URDF에 velocity·effort 명령 인터페이스가 있을 때) |
| 상태 | 모터 응답의 위치·속도·토크 → `/joint_states` (effort = 모터 추정 토크) |
| 관절 ↔ 모터 | `모터 각 = direction × scale × 관절 값 + offset` (`config/motors.yaml`) |
| 일부만 연결 | `active_joints`에 없는 관절은 mock처럼 명령을 그대로 상태로 돌려줌 → 모터 1대로도 전체 팔 컨트롤러 실행 |
| 그리퍼 | 모터 1개 = `finger_left_joint` (scale 100 rad/m = 1 / 피니언 반지름 0.01 m), `finger_right_joint`는 그 상태를 따라감 |
| 시작 | 설정(configure) 때 모터 레지스터에서 PMAX/VMAX/TMAX를 읽고, 활성화 때 현재 위치를 명령으로 잡아 튀지 않게 시작 |
| 안전 | 아래 경우 **모든 모터를 사용 안 함**으로 바꾸고 오류: 응답 20주기(0.1 s) 연속 없음, 모터 오류 상태(과전압·저전압·과전류·과열·통신 끊김·과부하), 명령과 현재 위치 차이 0.3 rad 초과 |

실물 전용 컨트롤러 허용 오차는 `arm_bringup/config/controllers_real_overrides.yaml`에서 넓게 잡았다. MIT 위치 제어는 정지 오차가 "부하 토크 ÷ kp"만큼 남아, 시뮬레이션 기준(0.03 rad)으로는 궤적이 중단되기 때문이다. 3단계에서 중력 토크 피드포워드와 게인을 조정한 뒤 다시 좁힌다.

### 6-1. 모터 없이 시험 (가상 모터)

```bash
# 터미널 1: Damiao USB-CAN 보드를 흉내 내는 가상 모터 7대 (sudo 불필요)
ros2 run arm_hardware dm_fake_motor --iface damiao-serial-sim --channel /tmp/ttyDM0 --all
# 터미널 2
ros2 launch arm_bringup real.launch.py transport:=damiao-serial channel:=/tmp/ttyDM0
# 터미널 3: 0단계와 같은 기준 궤적·분석
ros2 run arm_bringup run_reference.py --set sweeps --bag ~/arm_logs/fake_real
ros2 run arm_bringup analyze_baseline.py ~/arm_logs/fake_real
```

- 가상 모터를 끄면(Ctrl+C) 0.1 s 안에 "응답이 21 주기 연속 없음 → 모든 모터 사용 안 함"이 나와야 정상이다
- SocketCAN 경로는 `vcan0`으로 시험한다 (5-2절의 `dm_fake_motor --channel vcan0` + `real.launch.py channel:=vcan0`)
- 가상 모터의 마찰·관성은 가상값이라 추종 오차 숫자 자체는 의미가 없다. 파이프라인이 끝까지 도는지만 본다

### 6-2. 실물 1축 (1단계 마지막)

```bash
ros2 launch arm_bringup real.launch.py active_joints:=joint2                         # can0
ros2 launch arm_bringup real.launch.py active_joints:=joint2 transport:=damiao-serial channel:=/dev/ttyACM0
ros2 run arm_bringup run_reference.py --set sweeps --only joint2 --bag ~/arm_logs/bench_joint2
ros2 run arm_bringup analyze_baseline.py ~/arm_logs/bench_joint2 --compare <Gazebo 기준선 bag>
```

1. 5-3절 순서로 ID·모드·영점을 먼저 맞춘다 (`dm_bench`는 `real.launch.py`와 동시에 쓰지 않는다)
2. 처음에는 레버 없이, 그다음 레버+추로 같은 스윕을 돌린다
3. 관절 + 방향과 모터 + 방향이 반대면 `motors.yaml`의 `direction: -1`
4. 결과 요약을 `docs/results/01_motor_bench/`에 모은다
