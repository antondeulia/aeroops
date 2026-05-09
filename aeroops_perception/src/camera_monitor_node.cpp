#include <aeroops_perception/camera_monitor_node.hpp>
#include <filesystem>

namespace aeroops::perception
{

CameraMonitorNode::CameraMonitorNode(const rclcpp::NodeOptions& options)
	: rclcpp_lifecycle::LifecycleNode("camera_monitor_node", options)
{
	RCLCPP_INFO(get_logger(), "CameraMonitorNode constructed");
}

CallbackReturn CameraMonitorNode::on_configure(const rclcpp_lifecycle::State&)
{
	frame_count_ = 0;
	image_received_ = false;

	// Camera image subscription
	image_sub_ =
		create_subscription<Image>("/camera/image", rclcpp::SensorDataQoS(),
								   std::bind(&CameraMonitorNode::image_callback,
											 this, std::placeholders::_1));

	// Snapshot service
	save_snapshot_srv_ = create_service<Trigger>(
		"/aeroops/perception/save_snapshot",
		std::bind(&CameraMonitorNode::save_snapshot_callback, this,
				  std::placeholders::_1, std::placeholders::_2));

	RCLCPP_INFO(get_logger(), "CameraMonitorNode configured");
	return CallbackReturn::SUCCESS;
}

CallbackReturn CameraMonitorNode::on_activate(const rclcpp_lifecycle::State&)
{

	RCLCPP_INFO(get_logger(), "CameraMonitorNode activated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn CameraMonitorNode::on_deactivate(const rclcpp_lifecycle::State&)
{

	RCLCPP_INFO(get_logger(), "CameraMonitorNode deactivate");
	return CallbackReturn::SUCCESS;
}

CallbackReturn CameraMonitorNode::on_cleanup(const rclcpp_lifecycle::State&)
{
	image_sub_.reset();
	frame_count_ = 0;
	image_received_ = false;

	RCLCPP_INFO(get_logger(), "CameraMonitorNode cleaned up");
	return CallbackReturn::SUCCESS;
}

CallbackReturn CameraMonitorNode::on_shutdown(const rclcpp_lifecycle::State&)
{
	image_sub_.reset();

	RCLCPP_INFO(get_logger(), "CameraMonitorNode shutdown");
	return CallbackReturn::SUCCESS;
}

// Image callback
void CameraMonitorNode::image_callback(const Image::SharedPtr msg)
{
	try
	{
		auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");

		{
			std::lock_guard<std::mutex> lock(frame_mutex_);
			latest_frame_ = cv_image->image.clone();
		}
	}
	catch (const cv_bridge::Exception& e)
	{
		RCLCPP_WARN(get_logger(), "Failed to convert camera frame: %s",
					e.what());
		return;
	}
}

// Snapshot callback
void CameraMonitorNode::save_snapshot_callback(
	const std::shared_ptr<Trigger::Request> request,
	std::shared_ptr<Trigger::Response> response)
{
	(void)request;

	cv::Mat frame;
	{
		std::lock_guard<std::mutex> lock(frame_mutex_);

		if (latest_frame_.empty())
		{
			response->success = false;
			response->message = "No camera frame available";
			return;
		}

		frame = latest_frame_.clone();
	}

	std::filesystem::create_directories(snapshot_dir_);

	std::ostringstream path;
	path << snapshot_dir_ << "/inspection_" << snapshot_index_++ << ".png";

	const bool ok = cv::imwrite(path.str(), frame);

	response->success = ok;
	response->message = ok ? path.str() : "Failed to save snapshot";
}

} // namespace aeroops::perception

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aeroops::perception::CameraMonitorNode)