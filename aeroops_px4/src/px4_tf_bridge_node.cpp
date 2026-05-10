#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace aeroops_px4
{
namespace
{
using VehicleOdometry = px4_msgs::msg::VehicleOdometry;

constexpr double kPi = 3.14159265358979323846;

bool finite3(const std::array<float, 3>& values)
{
	return std::isfinite(values[0]) && std::isfinite(values[1]) &&
		   std::isfinite(values[2]);
}

bool finite4(const std::array<float, 4>& values)
{
	return std::isfinite(values[0]) && std::isfinite(values[1]) &&
		   std::isfinite(values[2]) && std::isfinite(values[3]);
}

geometry_msgs::msg::Quaternion to_ros_orientation(
	const std::array<float, 4>& px4_q)
{
	geometry_msgs::msg::Quaternion orientation{};

	if (!finite4(px4_q))
	{
		orientation.w = 1.0;
		return orientation;
	}

	tf2::Quaternion q_px4(px4_q[1], px4_q[2], px4_q[3], px4_q[0]);

	if (q_px4.length2() < 1e-12)
	{
		orientation.w = 1.0;
		return orientation;
	}

	q_px4.normalize();

	tf2::Quaternion q_ned_to_enu;
	q_ned_to_enu.setRPY(kPi, 0.0, kPi / 2.0);

	tf2::Quaternion q_frd_to_flu;
	q_frd_to_flu.setRPY(kPi, 0.0, 0.0);

	auto q_ros = q_ned_to_enu * q_px4 * q_frd_to_flu;
	q_ros.normalize();

	orientation.x = q_ros.x();
	orientation.y = q_ros.y();
	orientation.z = q_ros.z();
	orientation.w = q_ros.w();

	return orientation;
}
} // namespace

class Px4TfBridgeNode final : public rclcpp::Node
{
public:
	explicit Px4TfBridgeNode(const rclcpp::NodeOptions& options)
		: Node("px4_tf_bridge", options)
	{
		odom_topic_ = declare_parameter<std::string>(
			"odom_topic", "/fmu/out/vehicle_odometry");
		odom_frame_id_ =
			declare_parameter<std::string>("odom_frame_id", "odom");
		base_frame_id_ =
			declare_parameter<std::string>("base_frame_id", "base_link");
		ros_odom_topic_ =
			declare_parameter<std::string>("ros_odom_topic", "px4/odom");
		publish_tf_ = declare_parameter<bool>("publish_tf", true);
		publish_odometry_ = declare_parameter<bool>("publish_odometry", true);

		tf_broadcaster_ =
			std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		if (publish_odometry_)
		{
			odom_pub_ =
				create_publisher<nav_msgs::msg::Odometry>(ros_odom_topic_, 10);
		}

		auto sensor_qos =
			rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

		odom_sub_ = create_subscription<VehicleOdometry>(
			odom_topic_, sensor_qos,
			std::bind(&Px4TfBridgeNode::odometry_callback, this,
					  std::placeholders::_1));

		RCLCPP_INFO(get_logger(), "Bridging %s into TF %s -> %s",
					odom_topic_.c_str(), odom_frame_id_.c_str(),
					base_frame_id_.c_str());
	}

private:
	void odometry_callback(const VehicleOdometry::SharedPtr msg)
	{
		if (!finite3(msg->position))
		{
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
								 "Skipping PX4 odometry with invalid position");
			return;
		}

		const auto stamp = get_clock()->now();

		geometry_msgs::msg::TransformStamped transform{};
		transform.header.stamp = stamp;
		transform.header.frame_id = odom_frame_id_;
		transform.child_frame_id = base_frame_id_;
		transform.transform.translation.x = msg->position[1];
		transform.transform.translation.y = msg->position[0];
		transform.transform.translation.z = -msg->position[2];
		transform.transform.rotation = to_ros_orientation(msg->q);

		if (publish_tf_)
		{
			tf_broadcaster_->sendTransform(transform);
		}

		if (!publish_odometry_)
		{
			return;
		}

		nav_msgs::msg::Odometry odom{};
		odom.header = transform.header;
		odom.child_frame_id = transform.child_frame_id;
		odom.pose.pose.position.x = transform.transform.translation.x;
		odom.pose.pose.position.y = transform.transform.translation.y;
		odom.pose.pose.position.z = transform.transform.translation.z;
		odom.pose.pose.orientation = transform.transform.rotation;

		fill_twist(*msg, odom);
		fill_covariance(*msg, odom);

		odom_pub_->publish(odom);
	}

	void fill_twist(const VehicleOdometry& msg, nav_msgs::msg::Odometry& odom)
	{
		if (finite3(msg.velocity))
		{
			if (msg.velocity_frame == VehicleOdometry::VELOCITY_FRAME_NED)
			{
				odom.twist.twist.linear.x = msg.velocity[1];
				odom.twist.twist.linear.y = msg.velocity[0];
				odom.twist.twist.linear.z = -msg.velocity[2];
			}
			else if (msg.velocity_frame == VehicleOdometry::VELOCITY_FRAME_FRD ||
					 msg.velocity_frame ==
						 VehicleOdometry::VELOCITY_FRAME_BODY_FRD)
			{
				odom.twist.twist.linear.x = msg.velocity[0];
				odom.twist.twist.linear.y = -msg.velocity[1];
				odom.twist.twist.linear.z = -msg.velocity[2];
			}
		}

		if (finite3(msg.angular_velocity))
		{
			odom.twist.twist.angular.x = msg.angular_velocity[0];
			odom.twist.twist.angular.y = -msg.angular_velocity[1];
			odom.twist.twist.angular.z = -msg.angular_velocity[2];
		}
	}

	void fill_covariance(const VehicleOdometry& msg,
						 nav_msgs::msg::Odometry& odom)
	{
		if (finite3(msg.position_variance))
		{
			odom.pose.covariance[0] = msg.position_variance[1];
			odom.pose.covariance[7] = msg.position_variance[0];
			odom.pose.covariance[14] = msg.position_variance[2];
		}

		if (finite3(msg.orientation_variance))
		{
			odom.pose.covariance[21] = msg.orientation_variance[0];
			odom.pose.covariance[28] = msg.orientation_variance[1];
			odom.pose.covariance[35] = msg.orientation_variance[2];
		}

		if (finite3(msg.velocity_variance))
		{
			odom.twist.covariance[0] = msg.velocity_variance[1];
			odom.twist.covariance[7] = msg.velocity_variance[0];
			odom.twist.covariance[14] = msg.velocity_variance[2];
		}
	}

	std::string odom_topic_;
	std::string odom_frame_id_;
	std::string base_frame_id_;
	std::string ros_odom_topic_;
	bool publish_tf_{true};
	bool publish_odometry_{true};

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
};

} // namespace aeroops_px4

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aeroops_px4::Px4TfBridgeNode)
