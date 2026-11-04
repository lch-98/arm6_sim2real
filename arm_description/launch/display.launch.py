"""
RViz 표시용 launch

  ros2 launch arm_description display.launch.py                 # 슬라이더로 관절 조작
  ros2 launch arm_description display.launch.py demo:=true      # 자동으로 관절 스윕
  ros2 launch arm_description display.launch.py stage:=j1to3 test_mass:=0.2   # J1~J3 단계
  ros2 launch arm_description display.launch.py camera:=none    # 손목 카메라 없이
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("arm_description")

    demo = LaunchConfiguration("demo")
    prefix = LaunchConfiguration("prefix")
    stage = LaunchConfiguration("stage")
    test_mass = LaunchConfiguration("test_mass")
    camera = LaunchConfiguration("camera")

    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([pkg, "urdf", "arm.urdf.xacro"]),
            " prefix:=", prefix,
            " stage:=", stage,
            " test_mass:=", test_mass,
            " camera:=", camera,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("demo", default_value="false",
                              description="true면 슬라이더 GUI 대신 관절 스윕 노드 실행"),
        DeclareLaunchArgument("prefix", default_value="",
                              description="링크/관절 이름 접두사 (예: arm_)"),
        DeclareLaunchArgument("stage", default_value="full",
                              description="full: 전체 팔, j1to3: J1~J3 서브 어셈블리"),
        DeclareLaunchArgument("test_mass", default_value="0.0",
                              description="j1to3 단계에서 J4 출력면에 다는 시험 추 [kg]"),
        DeclareLaunchArgument("camera", default_value="d435",
                              description="d435: 손목 RealSense D435, none: 카메라 없음"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=UnlessCondition(demo),
        ),
        Node(
            package="arm_description",
            executable="joint_sweep.py",
            parameters=[{"prefix": prefix}],
            condition=IfCondition(demo),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "display.rviz"])],
            output="screen",
        ),
    ])
