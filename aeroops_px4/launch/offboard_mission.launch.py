from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="aeroops_px4_container",
        package="rclcpp_components",
        executable="component_container_mt",
        namespace="",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="aeroops_px4",
                plugin="aeroops_px4::OffboardMissionController",
                name="offboard_mission_controller",
                parameters=[
                    {
                        "takeoff_altitude_m": 5.0,
                        "square_size_m": 20.0,
                        "waypoint_acceptance_radius_m": 0.6,
                        "use_sim_time": False,
                    }
                ],
            )
        ],
    )

    return LaunchDescription([container])
