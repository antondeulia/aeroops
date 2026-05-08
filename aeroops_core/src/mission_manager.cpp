#include <aeroops_core/mission_manager.hpp>

namespace aeroops_core
{

MissionManager::MissionManager(const rclcpp::NodeOptions& options)
	: rclcpp_lifecycle::LifecycleNode("mission_manager", options)
{
}

CallbackReturn MissionManager::on_configure(const rclcpp_lifecycle::State&)
{
	mission_status_pub_ =
		this->create_publisher<aeroops_interfaces::msg::MissionStatus>(
			"mission/status", rclcpp::QoS(10));

	upload_mission_srv_ =
		this->create_service<aeroops_interfaces::srv::UploadMission>(
			"mission/upload",
			std::bind(&MissionManager::handle_upload_mission, this,
					  std::placeholders::_1, std::placeholders::_2));

	start_mission_srv_ =
		this->create_service<aeroops_interfaces::srv::StartMission>(
			"mission/start",
			std::bind(&MissionManager::handle_start_mission, this,
					  std::placeholders::_1, std::placeholders::_2));

	RCLCPP_INFO(get_logger(), "MissionManager configured");
	return CallbackReturn::SUCCESS;
};

CallbackReturn MissionManager::on_activate(const rclcpp_lifecycle::State&)
{
	mission_status_pub_->on_activate();

	publish_status(aeroops_interfaces::msg::MissionStatus::IDLE, "Mission "
																 "manager "
																 "active");

	RCLCPP_INFO(get_logger(), "MissionManager activated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn MissionManager::on_deactivate(const rclcpp_lifecycle::State&)
{
	mission_status_pub_->on_deactivate();

	RCLCPP_INFO(get_logger(), "MissionManager deactivated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn MissionManager::on_cleanup(const rclcpp_lifecycle::State&)
{
	mission_status_pub_.reset();
	upload_mission_srv_.reset();
	start_mission_srv_.reset();

	active_mission_id_.clear();
	waypoint_count_ = 0;

	RCLCPP_INFO(get_logger(), "MissionManager cleaned up");
	return CallbackReturn::SUCCESS;
}

MissionManager::CallbackReturn
MissionManager::on_shutdown(const rclcpp_lifecycle::State&)
{
	RCLCPP_INFO(get_logger(), "MissionManager shutting down");
	return CallbackReturn::SUCCESS;
}

void MissionManager::handle_upload_mission(
	const std::shared_ptr<aeroops_interfaces::srv::UploadMission::Request>
		request,
	std::shared_ptr<aeroops_interfaces::srv::UploadMission::Response> response)
{
	const auto count = request->latitudes.size();

	if (count == 0 || request->longitudes.size() != count ||
		request->altitudes_m.size() != count)
	{
		response->success = false;
		response->message = "Invalid waypoint arrays";
		response->waypoint_count = 0;
		return;
	}

	active_mission_id_ = request->mission_id;
	waypoint_count_ = static_cast<uint32_t>(count);

	response->success = true;
	response->message = "Mission uploaded";
	response->waypoint_count = waypoint_count_;

	publish_status(aeroops_interfaces::msg::MissionStatus::LOADED, "Mission "
																   "uploaded");
}

void MissionManager::handle_start_mission(
	const std::shared_ptr<aeroops_interfaces::srv::StartMission::Request>
		request,
	std::shared_ptr<aeroops_interfaces::srv::StartMission::Response> response)
{
	if (active_mission_id_.empty())
	{
		response->success = false;
		response->message = "No mission uploaded";
		return;
	}

	if (request->mission_id != active_mission_id_)
	{
		response->success = false;
		response->message = "Mission ID mismatch";
		return;
	}

	response->success = true;
	response->message = "Mission started";

	publish_status(aeroops_interfaces::msg::MissionStatus::RUNNING, "Mission "
																	"started");
}

void MissionManager::publish_status(const uint8_t status,
									const std::string& message)
{
	if (!mission_status_pub_ || !mission_status_pub_->is_activated())
	{
		return;
	}

	aeroops_interfaces::msg::MissionStatus msg;
	msg.header.stamp = this->now();
	msg.header.frame_id = "map";
	msg.status = status;
	msg.mission_id = active_mission_id_;
	msg.current_waypoint = 0;
	msg.total_waypoints = waypoint_count_;
	msg.message = message;

	mission_status_pub_->publish(msg);
}

} // namespace aeroops_core