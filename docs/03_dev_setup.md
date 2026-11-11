# 03. 개발 환경

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| 시뮬레이터 | Gazebo Fortress (`ros-humble-ros-gz`가 같이 설치) |
| ros2_control | Humble 배포판 (`ros-humble-ros2-control`, `ros-humble-ros2-controllers`) |

## 1. ROS 2 Humble 설치

공식 설치 문서(<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>)대로 `ros-humble-desktop`까지 설치한다.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. 이 프로젝트에 필요한 패키지

```bash
sudo apt update
sudo apt install \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-ros-gz ros-humble-gz-ros2-control \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-yaml python3-matplotlib
```

`rosdep`을 처음 쓰는 경우:

```bash
sudo rosdep init
rosdep update
```

## 3. 워크스페이스 만들고 빌드

```bash
mkdir -p ~/arm_ws/src && cd ~/arm_ws/src
git clone https://github.com/lch-98/arm6_sim2real.git
cd ~/arm_ws
rosdep install --from-paths src --ignore-src -y    # 빠진 의존성 설치
colcon build --symlink-install
source install/setup.bash
echo "source ~/arm_ws/install/setup.bash" >> ~/.bashrc
```

- `--symlink-install`: 설정 파일(yaml)·launch·스크립트를 고치면 다시 빌드하지 않아도 바로 반영된다. URDF·메쉬는 다시 빌드하는 편이 안전하다
- 예전에 `~/ros2_ws/src`에 `arm_description`만 따로 넣었다면, 패키지 이름이 겹치지 않게 지운다

  ```bash
  rm -rf ~/ros2_ws/src/arm_description ~/ros2_ws/build/arm_description ~/ros2_ws/install/arm_description
  ```

## 4. 설치 확인

```bash
ros2 launch arm_description display.launch.py
```

RViz에 팔이 뜨고, 슬라이더 창(joint_state_publisher_gui)으로 관절을 움직일 수 있으면 끝이다.

| 증상 | 원인과 해결 |
|---|---|
| `Package 'arm_description' not found` | `source ~/arm_ws/install/setup.bash`를 안 했음 |
| RViz에 모델이 하얗게만 보임 | 왼쪽 Displays의 `Fixed Frame`을 `world`로 |
| `joint_sweep.py` 실행 권한 오류 | `chmod +x ~/arm_ws/src/arm6_sim2real/arm_description/scripts/*.py` 후 다시 빌드 |
| `colcon build`에서 CMake 버전 경고 | 무시해도 됨 (빌드 결과에 영향 없음) |
