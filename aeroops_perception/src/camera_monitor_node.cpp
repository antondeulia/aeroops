#include <aeroops_perception/camera_monitor_node.hpp>
#include <filesystem>

namespace aeroops::perception
{

CameraMonitorNode::CameraMonitorNode(const rclcpp::NodeOptions& options)
	: rclcpp_lifecycle::LifecycleNode("camera_monitor_node", options)
{
	image_topic_ = declare_parameter<std::string>("image_topic",
												  image_topic_);
	snapshot_dir_ = declare_parameter<std::string>("snapshot_dir",
												   snapshot_dir_);
	window_name_ = declare_parameter<std::string>("window_name",
												  window_name_);
	show_opencv_window_ = declare_parameter<bool>("show_opencv_window",
												  show_opencv_window_);

	RCLCPP_INFO(get_logger(), "CameraMonitorNode constructed");
}

CallbackReturn CameraMonitorNode::on_configure(const rclcpp_lifecycle::State&)
{
	frame_count_ = 0;
	image_received_ = false;
	active_ = false;

	image_topic_ = get_parameter("image_topic").as_string();
	snapshot_dir_ = get_parameter("snapshot_dir").as_string();
	window_name_ = get_parameter("window_name").as_string();
	show_opencv_window_ = get_parameter("show_opencv_window").as_bool();

	// Camera image subscription
	image_sub_ =
		create_subscription<Image>(image_topic_, rclcpp::SensorDataQoS(),
								   std::bind(&CameraMonitorNode::image_callback,
											 this, std::placeholders::_1));

	// Snapshot service
	save_snapshot_srv_ = create_service<SaveSnapshot>(
		"/aeroops/perception/save_snapshot",
		std::bind(&CameraMonitorNode::save_snapshot_callback, this,
				  std::placeholders::_1, std::placeholders::_2));

	RCLCPP_INFO(get_logger(), "CameraMonitorNode configured");
	return CallbackReturn::SUCCESS;
}

CallbackReturn CameraMonitorNode::on_activate(const rclcpp_lifecycle::State&)
{
	active_ = true;

	if (show_opencv_window_)
	{
		cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
		cv::resizeWindow(window_name_, 960, 540);
		cv::startWindowThread();
	}

	RCLCPP_INFO(get_logger(), "CameraMonitorNode activated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn CameraMonitorNode::on_deactivate(const rclcpp_lifecycle::State&)
{
	active_ = false;

	if (show_opencv_window_)
	{
		cv::destroyWindow(window_name_);
	}

	RCLCPP_INFO(get_logger(), "CameraMonitorNode deactivate");
	return CallbackReturn::SUCCESS;
}

CallbackReturn CameraMonitorNode::on_cleanup(const rclcpp_lifecycle::State&)
{
	if (show_opencv_window_)
	{
		cv::destroyWindow(window_name_);
	}

	image_sub_.reset();
	save_snapshot_srv_.reset();
	frame_count_ = 0;
	image_received_ = false;
	active_ = false;

	RCLCPP_INFO(get_logger(), "CameraMonitorNode cleaned up");
	return CallbackReturn::SUCCESS;
}

CallbackReturn CameraMonitorNode::on_shutdown(const rclcpp_lifecycle::State&)
{
	if (show_opencv_window_)
	{
		cv::destroyWindow(window_name_);
	}

	image_sub_.reset();
	save_snapshot_srv_.reset();
	active_ = false;

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

		image_received_ = true;
		++frame_count_;

		if (active_ && show_opencv_window_)
		{
			cv::imshow(window_name_, cv_image->image);
			cv::waitKey(1);
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
	const std::shared_ptr<SaveSnapshot::Request> request,
	std::shared_ptr<SaveSnapshot::Response> response)
{
	cv::Mat frame;
	{
		std::lock_guard<std::mutex> lock(frame_mutex_);

		if (latest_frame_.empty())
		{
			response->success = false;
			response->path = "No camera frame available";
			return;
		}

		frame = latest_frame_.clone();
	}

	std::filesystem::create_directories(snapshot_dir_);

	std::ostringstream path;
	path << snapshot_dir_ << "/inspection_wp_" << request->waypoint_index << "_"
		 << snapshot_index_++ << ".png";

	const bool ok = cv::imwrite(path.str(), frame);

	response->success = ok;
	response->path = ok ? path.str() : "Failed to save snapshot";
}

} // namespace aeroops::perception

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aeroops::perception::CameraMonitorNode)
