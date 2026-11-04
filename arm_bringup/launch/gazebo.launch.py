"""
Gazebo(Fortress) 물리 시뮬레이션 + ros2_control

  ros2 launch arm_bringup gazebo.launch.py                 # Gazebo 창 + RViz
  ros2 launch arm_bringup gazebo.launch.py gui:=false      # 창 없이 (서버만, 빠름)
  ros2 launch arm_bringup gazebo.launch.py stage:=j1to3 test_mass:=0.2

중력·관성·관절 감쇠가 들어가므로 관절 토크(effort)를 읽을 수 있다.
시뮬레이션 시간(/clock)을 쓰므로, 이 launch와 함께 쓰는 다른 노드도 use_sim_time:=true 로 실행한다.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    stage = LaunchConfiguration("stage").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    controllers = os.path.join(
        get_package_share_directory("arm_bringup"), "config",
        "controllers.yaml" if stage == "full" else "controllers_j1to3.yaml",
    )
    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]),
            " sim_mode:=gazebo",
            " controllers_file:=", controllers,
            " gazebo_overrides_file:=", os.path.join(
                get_package_share_directory("arm_bringup"), "config", "controllers_gazebo_overrides.yaml"),
            " stage:=", stage,
            " test_mass:=", LaunchConfiguration("test_mass"),
            " camera:=", LaunchConfiguration("camera"),
        ]),
        value_type=str,
    )

    # -r: 바로 시뮬레이션 시작, -s: 서버만 (창 없음), -v 3: 경고까지 출력
    gz_args = "-r -v 3 empty.sdf" + ("" if gui else " -s")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "arm", "-allow_renaming", "false"],
        output="screen",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
            output="screen",
        )

    # 로봇이 Gazebo에 생긴 뒤(create 종료) 컨트롤러를 순서대로 켠다
    jsb = spawner("joint_state_broadcaster")
    later = [spawner("arm_controller")] + ([spawner("gripper_controller")] if stage == "full" else [])
    events = [
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=later)),
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("arm_description"), "rviz", "display.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    return [gazebo, clock_bridge, rsp, spawn_entity, *events, rviz]


def generate_launch_description():
    # Gazebo가 package://arm_description/... 메쉬를 찾을 수 있도록 share 폴더를 리소스 경로에 추가
    share_parent = os.path.dirname(get_package_share_directory("arm_description"))
    return LaunchDescription([
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", share_parent),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", share_parent),
        # gz_ros2_control 플러그인(.so) 위치. apt 설치에서는 보통 없어도 찾지만, 환경에 따라 못 찾는 경우가 있어 명시
        AppendEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", os.path.join(get_package_prefix("gz_ros2_control"), "lib")),
        AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", os.path.join(get_package_prefix("gz_ros2_control"), "lib")),
        DeclareLaunchArgument("stage", default_value="full", description="full | j1to3"),
        DeclareLaunchArgument("test_mass", default_value="0.0", description="j1to3 단계 시험 추 [kg]"),
        DeclareLaunchArgument("camera", default_value="d435", description="d435 | none"),
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo 창 표시 여부"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz 실행 여부"),
        OpaqueFunction(function=_setup),
    ])
