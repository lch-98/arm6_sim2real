# arm6_sim2real

3D 프린팅으로 만드는 소형 6축 로봇팔 + 평행 그리퍼. CAN 통신 액추에이터, ROS 2 Humble, ros2_control.
시뮬레이션에서 기준을 먼저 만들고, 실물을 단계별로 조립하면서 측정값을 시뮬레이션에 되돌려 넣는 **Sim2Real** 과정 전체를 기록한다.

| 항목 | 값 |
|---|---|
| 자유도 | 6축 + 평행 그리퍼 (최대 벌림 약 44 mm) |
| 수평 최대 도달 거리 | 0.566 m (J1 축 ~ TCP) |
| 설계 가반하중 | 0.3 kg (최악 자세, 정적 토크 기준) |
| 질량 | 약 2.93 kg (계산값, 손목 카메라 포함) |
| 액추에이터 | Damiao DM-J4340P-2EC ×3, DM-J4310-2EC ×3, DM-H3510 ×1 (그리퍼) |
| 통신 | CAN 2.0, 1 Mbps, 제어 주기 200 Hz |
| 구조 | 전 부품 3D 프린팅 (구매품: 액추에이터, 베어링, 피니언) |
| 손목 카메라 | Intel RealSense D435 (선택) |

## 빠른 시작

```bash
mkdir -p ~/arm_ws/src && cd ~/arm_ws/src
git clone https://github.com/lch-98/arm6_sim2real.git
cd ~/arm_ws && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install && source install/setup.bash

ros2 launch arm_description display.launch.py     # RViz에서 모델 확인 (슬라이더)
ros2 launch arm_bringup mock.launch.py            # ros2_control mock 하드웨어
ros2 launch arm_bringup gazebo.launch.py          # Gazebo 물리 시뮬레이션
ros2 launch arm_bringup real.launch.py            # 실물 Damiao 모터 (CAN), 모터 없이는 docs/05 6-1절의 가상 모터
```

자세한 설치는 [docs/03_dev_setup.md](docs/03_dev_setup.md).

## 패키지

| 패키지 | 내용 |
|---|---|
| `arm_description` | URDF/xacro, 메쉬, CAD 생성 스크립트, 프린팅 STL, ros2_control 하드웨어 정의 |
| `arm_bringup` | 컨트롤러 설정, mock·Gazebo launch, 기준 궤적, 기록·분석 스크립트 |
| `arm_hardware` | ros2_control 하드웨어 인터페이스(Damiao, CAN·MIT 모드), 가상 모터, 1단계 모터 벤치 도구 (`dm_bench`) |

## 문서

순서대로 따라 하면 같은 팔을 만들 수 있도록 정리했다 → **[docs/](docs/README.md)**

| 단계 | 상태 |
|---|---|
| 설계, CAD, 개발 환경 | ✅ |
| 0. 시뮬레이션 기준선 (mock, Gazebo) | ✅ |
| 1. 모터 단품 벤치 | 예정 |
| 2. 체결 설계·프린팅 시편 | 예정 |
| 3. J1~J3 서브 어셈블리 | 예정 |
| 4. 손목·그리퍼 | 예정 |
| 5. 전체 통합 | 예정 |
| 6. Sim2Real 루프 | 예정 |

## 주의

현재 프린팅 STL에는 액추에이터 체결 볼트 구멍이 없다. 공식 도면으로 볼트 패턴을 반영(로드맵 2단계)하기 전에는 출력하지 않는다.

## 라이선스

Apache-2.0 (각 `package.xml`). 저장소에 올릴 때 GitHub의 라이선스 선택으로 `LICENSE` 파일을 추가한다.
