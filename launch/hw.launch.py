# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import sys
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(0, os.path.join(get_package_share_directory("tita_bringup"), "launch"))
from launch_utils import tita_namespace

robot_name = "wl4"

def generate_launch_description():

    prefix = tita_namespace
    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot.xacro",
    )

    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "hw", "ctrl_mode": "wbc"}
    ).toxml()

    robot_controllers = os.path.join(
        get_package_share_directory("hotdog_controller"),
        "config",
        "hotdog_ros2_controllers.yaml",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            robot_controllers,
            {"frame_prefix": prefix + "/"},
        ],
        namespace=prefix,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        # output="both",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"frame_prefix": prefix + "/"},
        ],
        namespace=prefix,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            prefix + "/controller_manager",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            prefix + "/controller_manager",
        ],
    )
    tita_controller_spawner = Node(
        package="controller_manager",
        # output='screen',
        executable="spawner",
        arguments=[
            "hotdog_controller",
            "--controller-manager",
            prefix + "/controller_manager",
        ],  # TODO: change: cheater_tita_control
    )

    # start_controller = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=imu_sensor_broadcaster_spawner,
    #         on_exit=[tita_controller_spawner],
    #     )
    # )
    # canfd_router_node = Node(
    #     package='titati_canfd_router',
    #     executable='titati_canfd_router_node',
    #     name='titati_canfd_router_node',
    #     namespace=tita_namespace,
    #     output='screen',
    # )

    # battery_device_node = Node(
    #     package='battery_device',
    #     executable='battery_device_node',
    #     name='battery_device_node',
    #     namespace=tita_namespace,
    #     output='screen',
    # )

    command_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("joy_controller"),
                "launch",
                "joy_controller.launch.py",
            )
        ),
    )
    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        # battery_device_node,
        # canfd_router_node,
        # start_controller,
        tita_controller_spawner,
        command_launch,
    ]

    return LaunchDescription(nodes)
