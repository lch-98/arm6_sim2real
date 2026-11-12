"""
실물 Damiao 모터로 ros2_control 실행 (arm_hardware/DamiaoSystem)

  ros2 launch arm_bringup real.launch.py                                   # can0, 모든 관절
  ros2 launch arm_bringup real.launch.py active_joints:=joint2             # 1단계: 모터 1대만, 나머지는 명령을 그대로 돌려줌
  ros2 launch arm_bringup real.launch.py transport:=damiao-serial channel:=/dev/ttyACM0
  ros2 launch arm_bringup real.launch.py stage:=j1to3 active_joints:=joint1,joint2,joint3

모터 없이 시험 (가상 모터):
  ros2 run arm_hardware dm_fake_motor --iface damiao-serial-sim --channel /tmp/ttyDM0 --all     # 터미널 1
  ros2 launch arm_bringup real.launch.py transport:=damiao-serial channel:=/tmp/ttyDM0         # 터미널 2

모터 ID·방향·배율·게인: arm_hardware/config/motors.yaml (hw_config 인자로 다른 파일 지정 가능)
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
            " sim_mode:=real",
            " stage:=", stage,
            " camera:=", LaunchConfiguration("camera"),
            " hw_config:=", LaunchConfiguration("hw_config"),
            " transport:=", LaunchConfiguration("transport"),
            " channel:=", LaunchConfiguration("channel"),
            " active_joints:=", LaunchConfiguration("active_joints"),
        ]),
        value_type=str,
    )

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controllers,
                        PathJoinSubstitution([FindPackageShare("arm_bringup"), "config",
                                              "controllers_real_overrides.yaml"])],
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
        DeclareLaunchArgument("camera", default_value="d435", description="d435 | none"),
        DeclareLaunchArgument("transport", default_value="socketcan", description="socketcan | damiao-serial"),
        DeclareLaunchArgument("channel", default_value="can0", description="can0 | vcan0 | /dev/ttyACM0"),
        DeclareLaunchArgument("active_joints", default_value="all",
                              description="all | none | 쉼표로 나열 (예: joint2). 나머지는 명령을 그대로 돌려줌"),
        DeclareLaunchArgument("hw_config", default_value=PathJoinSubstitution(
            [FindPackageShare("arm_hardware"), "config", "motors.yaml"])),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz 실행 여부"),
        OpaqueFunction(function=_setup),
    ])
