#ifndef SWERVE_DRIVE_HARDWARE_HPP_
#define SWERVE_DRIVE_HARDWARE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <map>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"


namespace swerve_drive_hardware
{

class SwerveDriveHardware : public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SwerveDriveHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const{return *logger_;}
  rclcpp::Clock::SharedPtr get_clock() const {return clock_;}

private:
  double hw_start_sec_;
  double hw_stop_sec_;

  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

            // Joint names
  std::vector<std::string> joint_names_;

            // Map for easy lookup name -> joint command value
  std::map<std::string, uint> names_to_vel_cmd_map_;
  std::map<std::string, uint> names_to_pos_cmd_map_;


            // Command vectors
  std::vector<double> command_velocities_;
  std::vector<double> command_steering_angles_;

            // State vectors
  std::vector<double> state_positions_;
  std::vector<double> state_velocities_;

};

} // namespace swerve_drive_hardware

#endif // SWERVE_DRIVE_HARDWARE_HPP_
