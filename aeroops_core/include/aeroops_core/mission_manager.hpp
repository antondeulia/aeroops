#pragma once

#include <aeroops_interfaces/msg/mission_status.hpp>
#include <aeroops_interfaces/srv/start_mission.hpp>
#include <aeroops_interfaces/srv/upload_mission.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace aeroops_core
{
using CallbackReturn =
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MissionManager final : public rclcpp_lifecycle::LifecycleNode
{
public:
	explicit MissionManager(const rclcpp::NodeOptions& options);

private:
	CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
	CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
	CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
	CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
	CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

	void handle_upload_mission(
		const std::shared_ptr<aeroops_interfaces::srv::UploadMission::Request>
			request,
		std::shared_ptr<aeroops_interfaces::srv::UploadMission::Response>
			response);

	void handle_start_mission(
		const std::shared_ptr<aeroops_interfaces::srv::StartMission::Request>
			request,
		std::shared_ptr<aeroops_interfaces::srv::StartMission::Response>
			response);

	void publish_status(uint8_t status, const std::string& message);

	rclcpp_lifecycle::LifecyclePublisher<
		aeroops_interfaces::msg::MissionStatus>::SharedPtr mission_status_pub_;

	rclcpp::Service<aeroops_interfaces::srv::UploadMission>::SharedPtr
		upload_mission_srv_;
	rclcpp::Service<aeroops_interfaces::srv::StartMission>::SharedPtr
		start_mission_srv_;

	std::string active_mission_id_;
	uint32_t waypoint_count_{0};
};

} // namespace aeroops_core