from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = FindPackageShare("aeroops_bringup")

    aeroops_description_path = PathJoinSubstitution(
        [
            FindPackageShare("aeroops_description"),
            "urdf",
            "aeroops.urdf.xacro",
        ]
    )

    rviz2_config = PathJoinSubstitution(
        [bringup_pkg, "rviz2", "config.rviz"]
    )

    robot_description = {
        "robot_description": Command(
            ["xacro ", aeroops_description_path]
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_config],
    )

    return LaunchDescription(
        [robot_state_publisher, joint_state_publisher_gui, rviz2]
    )
