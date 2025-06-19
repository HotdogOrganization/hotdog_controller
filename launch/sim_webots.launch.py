#!/usr/bin/env python
import os
import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from launch.actions import OpaqueFunction
import xacro

# nn = "hxt"
robot_name = "hotdog"


def launch_setup(context, *args, **kwargs):
    nn = LaunchConfiguration("namespace").perform(context)

    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + "_description"),
        "xacro",
        "robot.xacro",
    )

    robot_description = xacro.process_file(
        robot_xacro_path, mappings={"hw_env": "webots"}
    ).toxml()
    spawn_robot = URDFSpawner(
        name=robot_name,
        robot_description=robot_description,
        # relative_path_prefix=os.path.join(robot_name + "_description", 'resource'),
        translation="0 0 0.4",
        rotation="0 0 0 0",
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [FindPackageShare("webots_bridge"), "worlds", "empty_world.wbt"]
        ),
        ros2_supervisor=True,
    )

    robot_controllers = os.path.join(
        get_package_share_directory("hotdog_controller"),
        "config",
        "hotdog_ros2_controllers.yaml",
    )

    tita_driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {"robot_description": robot_description},
            {"xacro_mappings": ["name:=" + robot_name]},
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            {"frame_prefix": nn + "/"},
            robot_controllers,
        ],
        respawn=True,
        namespace=nn,
    )


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            # {"robot_description": '<robot name=""><link name=""/></robot>'},
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
    tittai_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "hotdog_controller",
            "--controller-manager",
            nn + "/controller_manager",
        ],
    )

    def get_ros2_nodes(*args):
        return [
            spawn_robot,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessIO(
                    target_action=spawn_robot,
                    on_stdout=lambda event: get_webots_driver_node(
                        event,
                        [
                            robot_state_pub_node,
                            tita_driver,
                            joint_state_broadcaster_spawner,
                            imu_sensor_broadcaster_spawner,
                            tittai_controller_spawner,
                        ],
                    ),
                )
            ),
        ]

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    ros2_reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return [
        webots,
        webots._supervisor,
        webots_event_handler,
        ros2_reset_handler,
    ] + get_ros2_nodes()


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
