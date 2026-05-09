from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="aeroops_px4_container",
        namespace="aeroops",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="aeroops_px4",
                plugin="aeroops_px4::OffboardTakeoffController",
                name="offboard_takeoff_controller",
                parameters=[
                    {
                        "takeoff_altitude_m": 5.0,
                        "use_sim_time": False,
                    }
                ],
            )
        ],
    )

    return LaunchDescription([container])
