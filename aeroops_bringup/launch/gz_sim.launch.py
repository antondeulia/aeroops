from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_path = PathJoinSubstitution(
        [
            FindPackageShare("aeroops_bringup"),
            "worlds",
            "world.sdf",
        ]
    )

    model_path = PathJoinSubstitution(
        [
            FindPackageShare("aeroops_description"),
            "urdf",
            "aeroops.urdf.xacro",
        ]
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                model_path,
            ]
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                world_path,
            ],
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    spawn_uav = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "aeroops",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.5",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_uav,
            clock_bridge,
        ]
    )
