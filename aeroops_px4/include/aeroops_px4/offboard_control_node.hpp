#pragma once

#include <aeroops_interfaces/srv/save_snapshot.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace aeroops_px4
{

using CallbackReturn =
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
using VehicleCommand = px4_msgs::msg::VehicleCommand;
using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
using VehicleStatus = px4_msgs::msg::VehicleStatus;

using SaveSnapshot = aeroops_interfaces::srv::SaveSnapshot;

class OffboardControlNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
	explicit OffboardControlNode(const rclcpp::NodeOptions& options);

private:
	enum class ControlState
	{
		Preflight,
		Takeoff,
		Hold,
		MoveToWaypoint,
		Inspect,
		Land,

	};
	ControlState control_state_{ControlState::Preflight};

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

	rclcpp_lifecycle::LifecyclePublisher<OffboardControlMode>::SharedPtr
		offboard_mode_pub_;
	rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr
		trajectory_setpoint_pub_;
	rclcpp_lifecycle::LifecyclePublisher<VehicleCommand>::SharedPtr
		vehicle_command_pub_;

	uint64_t now_us() const;

	void publish_offboard_control_mode();
	void publish_position_setpoint(double x, double y, double z, double yaw);

	void publish_vehicle_command(uint16_t command, float param1 = 0.0F,
								 float param2 = 0.0F);

	void arm();
	void disarm();
	void set_offboard_mode();
	void land();

	void control_loop();
	rclcpp::TimerBase::SharedPtr control_timer_;

	int control_tick_{0};

	// Vehicle odometry:
	void vehicle_odometry_callback(const VehicleOdometry::SharedPtr msg);

	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

	double current_x_{0.0};
	double current_y_{0.0};
	double current_z_{0.0};
	double odometry_received_{false};

	int hold_ticks_{0};

	// Waypoints:
	struct Waypoint
	{
		double x;
		double y;
		double z;
		double yaw;
	};

	std::vector<Waypoint> waypoints_{{2.0, 0.0, -2.0, 0.0},
									 {2.0, 2.0, -2.0, 0.0},
									 {0.0, 2.0, -2.0, 0.0},
									 {0.0, 0.0, -2.0, 0.0}};

	std::size_t current_waypoint_index_{0};

	bool reached_waypoint(const Waypoint& waypoint) const;

	// Vehicle status:
	void vehicle_status_callback(const VehicleStatus::SharedPtr msg);

	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;

	bool vehicle_status_received_{false};
	bool armed_{false};
	bool offboard_enabled_{false};

	// Inspect
	int inspect_ticks_{0};

	// Save snapshot client
	rclcpp::Client<SaveSnapshot>::SharedPtr save_snapshot_client_;
};

} // namespace aeroops_px4