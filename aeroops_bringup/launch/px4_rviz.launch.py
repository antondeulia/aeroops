from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    camera_gz_topic = LaunchConfiguration("camera_gz_topic")
    camera_info_gz_topic = LaunchConfiguration("camera_info_gz_topic")
    camera_raw_image_topic = LaunchConfiguration("camera_raw_image_topic")
    camera_raw_info_topic = LaunchConfiguration("camera_raw_info_topic")
    rviz_camera_image_topic = LaunchConfiguration("rviz_camera_image_topic")
    rviz_camera_info_topic = LaunchConfiguration("rviz_camera_info_topic")

    description_path = PathJoinSubstitution(
        [
            FindPackageShare("aeroops_description"),
            "urdf",
            "aeroops.urdf.xacro",
        ]
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("aeroops_bringup"),
            "rviz2",
            "px4_x500.rviz",
        ]
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                description_path,
                " robot_name:=x500_mono_cam",
            ]
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    px4_tf_bridge = Node(
        package="aeroops_px4",
        executable="px4_tf_bridge",
        name="px4_tf_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_topic": odom_topic,
                "odom_frame_id": odom_frame,
                "base_frame_id": base_frame,
                "ros_odom_topic": "/px4/odom",
                "publish_tf": True,
                "publish_odometry": True,
            }
        ],
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_image_bridge",
        output="screen",
        arguments=[
            PythonExpression(
                ["'", camera_gz_topic, "' + '@sensor_msgs/msg/Image@gz.msgs.Image'"]
            ),
            PythonExpression(
                [
                    "'",
                    camera_info_gz_topic,
                    "' + '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'",
                ]
            ),
            "--ros-args",
            "-r",
            PythonExpression(
                ["'", camera_gz_topic, "' + ':=' + '", camera_raw_image_topic, "'"]
            ),
            "-r",
            PythonExpression(
                [
                    "'",
                    camera_info_gz_topic,
                    "' + ':=' + '",
                    camera_raw_info_topic,
                    "'",
                ]
            ),
        ],
    )

    camera_frame_relay = Node(
        package="aeroops_px4",
        executable="camera_frame_relay",
        name="camera_frame_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_image_topic": camera_raw_image_topic,
                "input_camera_info_topic": camera_raw_info_topic,
                "output_image_topic": rviz_camera_image_topic,
                "output_camera_info_topic": rviz_camera_info_topic,
                "camera_frame_id": "camera_optical_frame",
                "restamp": True,
            }
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock if PX4/Gazebo is publishing sim time.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/fmu/out/vehicle_odometry",
                description="PX4 VehicleOdometry topic from the DDS bridge.",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="ROS ENU frame published as the TF parent.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="ROS FLU drone body frame published as the TF child.",
            ),
            DeclareLaunchArgument(
                "camera_gz_topic",
                default_value="/world/forest/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
                description="Gazebo camera image topic to bridge into ROS.",
            ),
            DeclareLaunchArgument(
                "camera_raw_image_topic",
                default_value="/rviz/camera/image_raw",
                description="Intermediate ROS image topic from ros_gz_bridge.",
            ),
            DeclareLaunchArgument(
                "camera_info_gz_topic",
                default_value="/world/forest/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info",
                description="Gazebo camera info topic to bridge into ROS.",
            ),
            DeclareLaunchArgument(
                "camera_raw_info_topic",
                default_value="/rviz/camera/camera_info_raw",
                description="Intermediate ROS camera info topic from ros_gz_bridge.",
            ),
            DeclareLaunchArgument(
                "rviz_camera_image_topic",
                default_value="/rviz/camera/image",
                description="Restamped ROS image topic shown in RViz Camera display.",
            ),
            DeclareLaunchArgument(
                "rviz_camera_info_topic",
                default_value="/rviz/camera/camera_info",
                description="Restamped ROS camera info topic for RViz Camera display.",
            ),
            robot_state_publisher,
            px4_tf_bridge,
            camera_bridge,
            camera_frame_relay,
            rviz2,
        ]
    )
