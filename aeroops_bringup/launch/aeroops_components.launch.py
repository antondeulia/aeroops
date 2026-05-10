from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

from launch_ros.actions import LifecycleTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    container = ComposableNodeContainer(
        name="aeroops_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="aeroops_px4",
                plugin="aeroops_px4::OffboardControlNode",
                name="offboard_control_node",
            ),
            ComposableNode(
                package="aeroops_perception",
                plugin="aeroops::perception::CameraMonitorNode",
                name="camera_monitor_node",
                parameters=[
                    {
                        "image_topic": "/camera/image",
                    }
                ],
            ),
        ],
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_image_bridge",
        output="screen",
        arguments=[
            "/world/forest/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "--ros-args",
            "-r",
            "/world/forest/model/x500_mono_cam_0/link/camera_link/sensor/camera/image:=/camera/image",
        ],
    )

    configure_offboard = LifecycleTransition(
        lifecycle_node_names=["/offboard_control_node"],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
    )

    activate_offboard = LifecycleTransition(
        lifecycle_node_names=["/offboard_control_node"],
        transition_ids=[Transition.TRANSITION_ACTIVATE],
    )

    configure_camera = LifecycleTransition(
        lifecycle_node_names=["/camera_monitor_node"],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
    )

    activate_camera = LifecycleTransition(
        lifecycle_node_names=["/camera_monitor_node"],
        transition_ids=[Transition.TRANSITION_ACTIVATE],
    )

    return LaunchDescription(
        [
            camera_bridge,
            container,
            configure_camera,
            activate_camera,
            configure_offboard,
            activate_offboard,
        ]
    )
