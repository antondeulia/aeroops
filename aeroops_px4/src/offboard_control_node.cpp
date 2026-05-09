#include <aeroops_px4/offboard_control_node.hpp>

namespace aeroops_px4
{

OffboardControlNode::OffboardControlNode(const rclcpp::NodeOptions& options)
	: rclcpp_lifecycle::LifecycleNode("offboard_control_node", options)
{
	RCLCPP_INFO(get_logger(), "Constructed");
}

CallbackReturn OffboardControlNode::on_configure(const rclcpp_lifecycle::State&)
{
	offboard_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/"
															   "offboard_"
															   "control_mode",
															   10);
	trajectory_setpoint_pub_ =
		create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
	vehicle_command_pub_ =
		create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

	auto sensor_qos =
		rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

	vehicle_odometry_sub_ = create_subscription<VehicleOdometry>(
		"/fmu/out/vehicle_odometry", sensor_qos,
		std::bind(&OffboardControlNode::vehicle_odometry_callback, this,
				  std::placeholders::_1));

	vehicle_status_sub_ = create_subscription<VehicleStatus>(
		"/fmu/out/vehicle_status_v4", sensor_qos,
		std::bind(&OffboardControlNode::vehicle_status_callback, this,
				  std::placeholders::_1));

	RCLCPP_INFO(get_logger(), "OffboardControlNode Configured");
	return CallbackReturn::SUCCESS;
}

CallbackReturn OffboardControlNode::on_activate(const rclcpp_lifecycle::State&)
{
	offboard_mode_pub_->on_activate();
	trajectory_setpoint_pub_->on_activate();
	vehicle_command_pub_->on_activate();

	control_tick_ = 0;
	control_timer_ =
		create_wall_timer(std::chrono::milliseconds(100),
						  std::bind(&OffboardControlNode::control_loop, this));

	control_state_ = ControlState::Preflight;
	control_tick_ = 0;

	current_waypoint_index_ = 0;
	hold_ticks_ = 0;

	RCLCPP_INFO(get_logger(), "OffboardControlNode Activated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn
OffboardControlNode::on_deactivate(const rclcpp_lifecycle::State&)
{
	offboard_mode_pub_->on_deactivate();
	trajectory_setpoint_pub_->on_deactivate();
	vehicle_command_pub_->on_deactivate();

	control_timer_.reset();

	RCLCPP_INFO(get_logger(), "OffboardControlNode Deactivated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn OffboardControlNode::on_cleanup(const rclcpp_lifecycle::State&)
{
	offboard_mode_pub_.reset();
	trajectory_setpoint_pub_.reset();
	vehicle_command_pub_.reset();

	control_timer_.reset();

	RCLCPP_INFO(get_logger(), "OffboardControlNode Cleaned Up");
	return CallbackReturn::SUCCESS;
}

CallbackReturn OffboardControlNode::on_shutdown(const rclcpp_lifecycle::State&)
{
	return CallbackReturn::SUCCESS;
}

uint64_t OffboardControlNode::now_us() const
{
	return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

void OffboardControlNode::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.timestamp = now_us();

	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_mode_pub_->publish(msg);
}

void OffboardControlNode::publish_position_setpoint(double x, double y,
													double z, double yaw)
{
	TrajectorySetpoint msg{};
	msg.timestamp = now_us();

	msg.position = {
		static_cast<float>(x),
		static_cast<float>(y),
		static_cast<float>(z),
	};

	msg.yaw = static_cast<float>(yaw);

	trajectory_setpoint_pub_->publish(msg);
}

void OffboardControlNode::publish_vehicle_command(uint16_t command,
												  float param1, float param2)
{
	VehicleCommand msg{};
	msg.timestamp = now_us();

	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;

	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_pub_->publish(msg);
}

void OffboardControlNode::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
							1.0F);
}

void OffboardControlNode::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
							0.0F);
}

void OffboardControlNode::set_offboard_mode()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F,
							6.0F);
}

void OffboardControlNode::land()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

// Control loop
void OffboardControlNode::control_loop()
{
	publish_offboard_control_mode();
	publish_position_setpoint(0.0, 0.0, -2.0, 0.0);

	switch (control_state_)
	{
		case ControlState::Preflight:
		{
			publish_position_setpoint(0.0, 0.0, -2.0, 0.0);

			if (control_tick_ >= 20)
			{
				if (!vehicle_status_received_)
				{
					RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
										 "Waiting for vehicle status");
				}

				set_offboard_mode();
				arm();

				if (armed_ && offboard_enabled_)
				{
					control_state_ = ControlState::Takeoff;
					RCLCPP_INFO(get_logger(), "State: TAKEOFF");
				}
			}
			break;
		}

		case ControlState::Takeoff:
		{
			publish_position_setpoint(0.0, 0.0, -2.0, 0.2);

			if (odometry_received_ && std::abs(current_z_ - (-2.0)) < 0.25)
			{
				control_state_ = ControlState::Hold;
				RCLCPP_INFO(get_logger(), "State: HOLD");
			}
			break;
		}

		case ControlState::Hold:
		{
			publish_position_setpoint(0.0, 0.0, -2.0, 0.0);

			++hold_ticks_;

			if (hold_ticks_ >= 30)
			{
				control_state_ = ControlState::MoveToWaypoint;
				RCLCPP_INFO(get_logger(), "State: MOVE_TO_WAYPOINT");
			}
			break;
		}

		case ControlState::MoveToWaypoint:
		{
			const auto& waypoint = waypoints_.at(current_waypoint_index_);

			publish_position_setpoint(waypoint.x, waypoint.y, waypoint.z,
									  waypoint.yaw);

			if (reached_waypoint(waypoint))
			{
				RCLCPP_INFO(get_logger(), "Reached waypoint %zu",
							current_waypoint_index_);

				inspect_ticks_ = 0;
				control_state_ = ControlState::Inspect;
				RCLCPP_INFO(get_logger(), "State: INSPECT");

				if (current_waypoint_index_ + 1 < waypoints_.size())
				{
					++current_waypoint_index_;
				}
				else
				{
					land();
					control_state_ = ControlState::Land;
					RCLCPP_INFO(get_logger(), "State: LAND");
				}
			}
			break;
		}

		case ControlState::Inspect:
		{
			const auto& waypoint = waypoints_.at(current_waypoint_index_);

			publish_position_setpoint(waypoint.x, waypoint.y, waypoint.z,
									  waypoint.yaw);

			if (inspect_ticks_ == 0)
			{
				RCLCPP_INFO(get_logger(), "Inspecting waypoint %zu",
							current_waypoint_index_);
			}

			++inspect_ticks_;

			if (inspect_ticks_ >= 20)
			{
				if (current_waypoint_index_ + 1 < waypoints_.size())
				{
					++current_waypoint_index_;
					control_state_ = ControlState::MoveToWaypoint;
					RCLCPP_INFO(get_logger(), "State: MOVE_TO_WAYPOINT");
				}
				else
				{
					land();
					control_state_ = ControlState::Land;
					RCLCPP_INFO(get_logger(), "State: LAND");
				}
			}

			break;
		}

		case ControlState::Land:
		{
			break;
		}
	}

	++control_tick_;
}

// Vehicle Odometry Callback:
void OffboardControlNode::vehicle_odometry_callback(
	const VehicleOdometry::SharedPtr msg)
{
	current_x_ = msg->position[0];
	current_y_ = msg->position[1];
	current_z_ = msg->position[2];

	odometry_received_ = true;
}

// Waypoints:
bool OffboardControlNode::reached_waypoint(const Waypoint& waypoint) const
{
	return odometry_received_ && std::abs(current_x_ - waypoint.x) < 0.25 &&
		   std::abs(current_y_ - waypoint.y) < 0.25 &&
		   std::abs(current_z_ - waypoint.z) < 0.25;
}

// Vehicle status
void OffboardControlNode::vehicle_status_callback(
	const VehicleStatus::SharedPtr msg)
{
	vehicle_status_received_ = true;

	armed_ = msg->arming_state == VehicleStatus::ARMING_STATE_ARMED;
	offboard_enabled_ =
		msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

} // namespace aeroops_px4

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aeroops_px4::OffboardControlNode);