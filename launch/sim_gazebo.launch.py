#!/usr/bin/env python
import os
import xacro
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction

# nn = "hxt"
robot_name = "hotdog"


def launch_setup(context, *args, **kwargs):
    nn = LaunchConfiguration("namespace").perform(context)

    # Get world file path
    world_file = os.path.join(
        FindPackageShare("gazebo_bridge").find("gazebo_bridge"),
        "worlds",
        "empty_world.world",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={
            "world": world_file,
            "pause": "false",
            "verbose": "false",
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            f"{nn}/robot_description",
            "-entity",
            f"{robot_name}",
            "-x",
            "0.",
            "-y",
            "0.",
            "-z",
            "0.65",
        ],
        output="screen",
    )

    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot.xacro",
    )

    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "gazebo", "robot_ns": nn}
    ).toxml()

    # 替换package://路径为file://，防止Gazebo不识别
    robot_description = robot_description.replace(
        "package://" + robot_name + "_description",
        "file://" + get_package_share_directory(robot_name + "_description"),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
            {"publish_frequency": 15.0},
            {"frame_prefix": nn + "/"},
        ],
        namespace=nn,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            nn + "/controller_manager",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            nn + "/controller_manager",
        ],
    )
    wbc_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "hotdog_controller",
            "--controller-manager",
            nn + "/controller_manager",
        ],
    )
    nodes = [
        robot_state_pub_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        wbc_controller
    ]

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
    launch.actions.DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="name space",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
