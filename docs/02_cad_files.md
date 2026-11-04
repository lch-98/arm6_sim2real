# 02. CAD와 파일

형상은 파이썬 스크립트 하나(`arm_description/cad/generate_cad.py`)가 전부 만든다.
치수를 바꾸면 STEP, STL, URDF의 관절 위치·질량·관성이 **한 번에 같이** 바뀌므로, 형상과 시뮬레이션이 어긋나지 않는다.

## 1. 다시 생성하기

```bash
pip install cadquery            # 처음 한 번 (Ubuntu 22.04 기본 Python 3.10에서 동작)
cd ~/arm_ws/src/arm6_sim2real/arm_description
python3 cad/generate_cad.py     # 1~2분. 링크별 질량과 유효성(valid=True)이 출력됨
cd ~/arm_ws && colcon build && source install/setup.bash
```

`valid=False`가 나오는 부품이 있으면 형상 연산(합치기·빼기)이 깨진 것이므로 직전에 바꾼 값을 되돌린다.

## 2. 바꿀 수 있는 값 (`generate_cad.py` 위쪽)

| 이름 | 내용 |
|---|---|
| `P` | 링크 길이(`L2` 상완, `L3`, `L4` 전완, `L5`), 요크 폭·두께, 베이스 크기, 그리퍼 길이·스트로크 |
| `ACT` | 액추에이터 외경·높이·질량 |
| `JOINT_ACT` | 관절별 액추에이터 배정. 바꾸면 요크·캔·허브 크기가 자동으로 따라 바뀜 |
| `BRG`, `JOINT_BRG` | 반대편 지지 베어링 치수와 배정 |
| `CAM` | 손목 카메라 위치(x, z)와 숙임 각도 |
| `CLR` | 프린팅 공차 (반경 방향, 기본 0.3 mm). 시편 출력 후 조정 |
| `PRINT_FILL` | 프린팅 부품 실효 충진율 (질량 계산용). 시편 무게로 실측값을 넣음 |

관절 한계(각도)는 `urdf/arm_macro.xacro`와 `urdf/arm.ros2_control.xacro` 두 곳에 있다. 바꾸면 두 곳을 같이 바꾼다.

## 3. 파일 종류

### STEP과 STL

| | STEP | STL |
|---|---|---|
| 형식 | 정확한 수학 곡면 (원은 진짜 원) | 작은 삼각형을 이어 붙인 근사 형상 |
| 용도 | FreeCAD 등에서 치수 재기, 구멍 추가 등 **수정** | **슬라이서(프린팅)**, **RViz·Gazebo 표시** |

### 폴더별 역할

| 폴더 | 누가 쓰나 | 나누는 기준 |
|---|---|---|
| `arm_description/print/` | 3D 프린터 (슬라이서) | **출력할 부품 1개 = 파일 1개**. 모터 없음. `parts_list.txt`에 재질·부피 |
| `arm_description/meshes/` | URDF (RViz, Gazebo) | 링크마다 `_body`(흰색), `_accent`(주황: 캡·어댑터·핀·패드), `_motor`(회색: 액추에이터 외형, 표시용), `_collision`(셋을 합친 충돌 형상) |
| `arm_description/cad/step/` | CAD 수정 | 출력 부품마다 STEP 1개 |
| `arm_description/cad/arm_assembly.step` | CAD 확인 | 모터·카메라까지 조립된 홈 자세 전체 |

`print/link2.stl`과 `meshes/link2_body.stl`은 같은 형상이다. **프린팅에는 `print/`만 쓴다.**

### 프린팅 부품 (25개)

| 링크 | 본체 | 추가 부품 |
|---|---|---|
| base_link | `base_link.stl` | `base_band` (상단 띠) |
| link1 | `link1.stl` (터렛 + 어깨 요크) | `j2_bearing_cap` |
| link2 | `link2.stl` (상완 + 팔꿈치 요크) | `j2_adapter`, `j2_pin`, `j3_bearing_cap` |
| link3 | `link3.stl` (팔꿈치 허브 + J4 캔) | `j3_adapter`, `j3_pin`, `j4_retainer_ring` |
| link4 | `link4.stl` (전완 + 손목 요크) | `j5_bearing_cap` |
| link5 | `link5.stl` (손목 허브 + J6 캔) | `j5_adapter`, `j5_pin`, `j6_retainer_ring` |
| link6 | `link6.stl` (그리퍼 바디) | — (피니언은 구매) |
| 핑거 | `finger_left.stl`, `finger_right.stl` (랙 일체) | `pad_TPU` (TPU로 출력) |
| 카메라 | `camera__bracket.stl`, `camera__clamp.stl` | — |

## 4. FreeCAD로 확인하기

- 설치: Ubuntu 22.04 기본 저장소(`apt install freecad`)는 0.19로 오래되었으므로 공식 AppImage(파일 하나, 삭제도 파일만 지우면 됨) 또는 `sudo snap install freecad`를 쓴다
- `cad/arm_assembly.step`을 열면 색이 들어간 조립 모습이 보인다. 부품 하나를 수정하려면 `cad/step/<부품>.step`을 연다
- STL만 빠르게 보려면 `sudo apt install f3d` 후 `f3d arm_description/print/link2.stl`

## 5. 출력할 때 주의할 점

- **현재 STL에는 액추에이터 체결 구멍이 없다.** 공식 도면으로 볼트 패턴을 반영하기 전에는 출력하지 않는다 (로드맵 2단계)
- STL은 로봇에 달린 자세(링크 좌표계) 그대로 저장되어 있다. 슬라이서에서 넓은 면이 바닥에 닿게 직접 눕힌다
- 구조 부품은 PETG 권장 (하중을 오래 걸었을 때 PLA보다 덜 처짐). PLA는 형상 확인·공차 시편용
