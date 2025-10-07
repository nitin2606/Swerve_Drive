#include "swerve_drive_hardware/swerve_drive_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace swerve_drive_hardware
{

hardware_interface::CallbackReturn SwerveDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger(
      "controller_manager.resource_manager.hardware_component.system.SwerveDriveBot")
  );

  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // joint_names_.resize(info_.joints.size());

  std::size_t velocity_joints = 0;
  std::size_t position_joints = 0;

  for(const hardware_interface::ComponentInfo & joint : info_.joints) {
    if(joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
      velocity_joints++;
    } else {
      position_joints++;
    }
  }


  command_velocities_.resize(info_.joints.size() / 2, std::numeric_limits<double>::quiet_NaN());
  command_steering_angles_.resize(info_.joints.size() / 2,
      std::numeric_limits<double>::quiet_NaN());
  state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for(const hardware_interface::ComponentInfo & joint : info_.joints) {

    joint_names_.push_back(joint.name);
    if(joint.command_interfaces.size() != 1) {

      RCLCPP_FATAL(
        rclcpp::get_logger("SwerveDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if(joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SwerveDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);

      return CallbackReturn::ERROR;
    }

    if(joint.state_interfaces.size() != 2) {

      RCLCPP_FATAL(
        rclcpp::get_logger("SwerveDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());

      return CallbackReturn::ERROR;

    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("SwerveDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("SwerveDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SwerveDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocities_[i]));

  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SwerveDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  uint counter_steering_angles = 0;
  uint counter_velocity = 0;

  for (auto i = 0u; i < info_.joints.size(); i++) {

    auto joint = info_.joints[i];

    if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &command_velocities_[counter_velocity]));

      names_to_vel_cmd_map_[joint.name] = counter_velocity + 1;   // ADD 1 to differentiate from a key that is not found
      counter_velocity++;
    } else {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_POSITION,
          &command_steering_angles_[counter_steering_angles]));

      names_to_pos_cmd_map_[joint.name] = counter_steering_angles + 1;   // ADD 1 to differentiate from a key that is not found
      counter_steering_angles++;
    }
  }

  return command_interfaces;
}


hardware_interface::CallbackReturn SwerveDriveHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SwerveDriveHardware"), "CONFIGURING HARDWARE ...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SwerveDriveHardware"), "ACTIVATING HARDWARE ...");

  for (auto i = 0u; i < state_positions_.size(); i++) {
    state_positions_[i] = 0.0;
    state_velocities_[i] = 0.0;
  }

  // Set Default Values for Command Interface Arrays
  for (auto i = 0u; i < command_velocities_.size(); i++) {
    command_velocities_[i] = 0.0;
    command_steering_angles_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SwerveDriveHardware"), "Successfully activated!");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveDriveHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveDriveHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveDriveHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::ERROR;
}


hardware_interface::return_type swerve_drive_hardware::SwerveDriveHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{

  double dt = 0.01;
  for (auto i = 0u; i < joint_names_.size(); i++) {

    auto vel_i = names_to_vel_cmd_map_[joint_names_[i]];
    auto pos_i = names_to_pos_cmd_map_[joint_names_[i]];

    if (vel_i > 0) {
      auto vel = command_velocities_[vel_i - 1];
      state_velocities_[i] = vel;
      // state_positions_[i] = state_positions_[i] + dt * vel;
      state_positions_[i] = 0.0;

    } else if (pos_i > 0) {

      auto pos = command_steering_angles_[pos_i - 1];
      state_velocities_[i] = 0.0;
      state_positions_[i] = pos;

    }


  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type swerve_drive_hardware::SwerveDriveHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Simulate sending commands to actuators


    // RCLCPP_INFO(rclcpp::get_logger("SwerveDriveHardware"), "Wheel %u Velocity: %f", i, command_velocities_[i]);

  return hardware_interface::return_type::OK;
}

}  // namespace swerve_drive_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(swerve_drive_hardware::SwerveDriveHardware,
  hardware_interface::SystemInterface)
