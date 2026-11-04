"""
mock 하드웨어로 ros2_control 파이프라인 실행 (물리 없음)

  ros2 launch arm_bringup mock.launch.py
  ros2 launch arm_bringup mock.launch.py rviz:=false
  ros2 launch arm_bringup mock.launch.py stage:=j1to3 test_mass:=0.2

mock 하드웨어는 명령한 위치를 그대로 현재 위치로 돌려준다.
컨트롤러 설정, 궤적 전송, 기록·분석 스크립트가 제대로 도는지 확인하는 용도이며 토크(effort)는 0으로 나온다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    stage = LaunchConfiguration("stage").perform(context)
    controllers = PathJoinSubstitution([
        FindPackageShare("arm_bringup"), "config",
        "controllers.yaml" if stage == "full" else "controllers_j1to3.yaml",
    ])
    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]),
            " sim_mode:=mock",
            " stage:=", stage,
            " test_mass:=", LaunchConfiguration("test_mass"),
            " camera:=", LaunchConfiguration("camera"),
        ]),
        value_type=str,
    )

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controllers],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ]
    spawn = ["joint_state_broadcaster", "arm_controller"] + (["gripper_controller"] if stage == "full" else [])
    for name in spawn:
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
            output="screen",
        ))
    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("arm_description"), "rviz", "display.rviz"])],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stage", default_value="full", description="full | j1to3"),
        DeclareLaunchArgument("test_mass", default_value="0.0", description="j1to3 단계 시험 추 [kg]"),
        DeclareLaunchArgument("camera", default_value="d435", description="d435 | none"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz 실행 여부"),
        OpaqueFunction(function=_setup),
    ])
