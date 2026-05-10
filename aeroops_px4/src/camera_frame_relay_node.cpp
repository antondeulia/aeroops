#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <string>

namespace aeroops_px4
{

class CameraFrameRelayNode final : public rclcpp::Node
{
public:
	explicit CameraFrameRelayNode(const rclcpp::NodeOptions& options)
		: Node("camera_frame_relay", options)
	{
		input_image_topic_ =
			declare_parameter<std::string>("input_image_topic",
										   "/rviz/camera/image_raw");
		input_camera_info_topic_ =
			declare_parameter<std::string>("input_camera_info_topic",
										   "/rviz/camera/camera_info_raw");
		output_image_topic_ =
			declare_parameter<std::string>("output_image_topic",
										   "/rviz/camera/image");
		output_camera_info_topic_ =
			declare_parameter<std::string>("output_camera_info_topic",
										   "/rviz/camera/camera_info");
		camera_frame_id_ =
			declare_parameter<std::string>("camera_frame_id",
										   "camera_optical_frame");
		restamp_ = declare_parameter<bool>("restamp", true);

		auto sensor_qos =
			rclcpp::QoS(rclcpp::SensorDataQoS()).keep_last(5);

		image_pub_ =
			create_publisher<sensor_msgs::msg::Image>(output_image_topic_,
													  sensor_qos);
		camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
			output_camera_info_topic_, sensor_qos);

		image_sub_ = create_subscription<sensor_msgs::msg::Image>(
			input_image_topic_, sensor_qos,
			std::bind(&CameraFrameRelayNode::image_callback, this,
					  std::placeholders::_1));
		camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
			input_camera_info_topic_, sensor_qos,
			std::bind(&CameraFrameRelayNode::camera_info_callback, this,
					  std::placeholders::_1));

		RCLCPP_INFO(get_logger(), "Relaying camera image into frame %s",
					camera_frame_id_.c_str());
	}

private:
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		auto out = *msg;
		normalize_header(out.header);
		image_pub_->publish(out);
	}

	void camera_info_callback(
		const sensor_msgs::msg::CameraInfo::SharedPtr msg)
	{
		auto out = *msg;
		normalize_header(out.header);
		camera_info_pub_->publish(out);
	}

	void normalize_header(std_msgs::msg::Header& header)
	{
		header.frame_id = camera_frame_id_;

		if (restamp_)
		{
			header.stamp = get_clock()->now();
		}
	}

	std::string input_image_topic_;
	std::string input_camera_info_topic_;
	std::string output_image_topic_;
	std::string output_camera_info_topic_;
	std::string camera_frame_id_;
	bool restamp_{true};

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
		camera_info_pub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
		camera_info_sub_;
};

} // namespace aeroops_px4

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aeroops_px4::CameraFrameRelayNode)
