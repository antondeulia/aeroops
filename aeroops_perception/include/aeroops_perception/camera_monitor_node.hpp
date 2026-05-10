#pragma once

#include <aeroops_interfaces/srv/save_snapshot.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace aeroops::perception
{

using CallbackReturn =
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using Image = sensor_msgs::msg::Image;

using SaveSnapshot = aeroops_interfaces::srv::SaveSnapshot;

class CameraMonitorNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
	explicit CameraMonitorNode(const rclcpp::NodeOptions& options);

protected:
	CallbackReturn
	on_configure(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn
	on_activate(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn
	on_deactivate(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn
	on_cleanup(const rclcpp_lifecycle::State& prev_state) override;
	CallbackReturn
	on_shutdown(const rclcpp_lifecycle::State& prev_state) override;

private:
	void image_callback(const Image::SharedPtr msg);

	rclcpp::Subscription<Image>::SharedPtr image_sub_;

	std::size_t frame_count_{0};
	bool image_received_{false};
	bool active_{false};

	// Save snapshot:
	void
	save_snapshot_callback(const std::shared_ptr<SaveSnapshot::Request> request,
						   std::shared_ptr<SaveSnapshot::Response> response);

	rclcpp::Service<SaveSnapshot>::SharedPtr save_snapshot_srv_;

	std::mutex frame_mutex_;
	cv::Mat latest_frame_;

	std::string snapshot_dir_{"/tmp/aeroops_snapshots"};
	std::size_t snapshot_index_{0};

	std::string image_topic_{"/camera/image"};
	std::string window_name_{"AeroOps Camera"};
	bool show_opencv_window_{true};
};

} // namespace aeroops::perception
