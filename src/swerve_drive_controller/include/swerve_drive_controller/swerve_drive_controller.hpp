#ifndef SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "swerve_drive_controller/swerve_drive_kinematics.hpp"
#include <hardware_interface/loaned_command_interface.hpp>

namespace swerve_drive_controller{

using CallbackReturn = controller_interface::CallbackReturn;
// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Wheel{
    public:
        Wheel(std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity, std::string name);
        void set_velocity(double velocity);

    private:
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_;
        std::string name;
};

class Axle{
    public:
        Axle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> position, std::string name);
        void set_position(double position);

    private:
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_;
        std::string name;
        // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;

};

class SwerveController : public controller_interface::ControllerInterface{
    using Twist = geometry_msgs::msg::TwistStamped;


    public:
        SwerveController();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CallbackReturn on_init() override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        

    protected:

        std::shared_ptr<Wheel> get_wheel(const std::string & wheel_name);
        std::shared_ptr<Axle> get_axle(const std::string & axle_name);
    
        // Handles for three wheels and their axles
        std::shared_ptr<Wheel> front_left_wheel_handle_;
        std::shared_ptr<Wheel> front_right_wheel_handle_;
        std::shared_ptr<Wheel> rear_left_wheel_handle_;
        std::shared_ptr<Wheel> rear_right_wheel_handle_;

        std::shared_ptr<Axle> front_left_axle_handle_;
        std::shared_ptr<Axle> front_right_axle_handle_;
        std::shared_ptr<Axle> rear_left_axle_handle_;
        std::shared_ptr<Axle> rear_right_axle_handle_;

        // Joint names for wheels and axles
        std::string front_left_wheel_joint_name_;
        std::string front_right_wheel_joint_name_;
        std::string rear_left_wheel_joint_name_;
        std::string rear_right_wheel_joint_name_;

        std::string front_left_axle_joint_name_;
        std::string front_right_axle_joint_name_;
        std::string rear_left_axle_joint_name_;
        std::string rear_right_axle_joint_name_;

        std::string odometry_topic_;
        std::string base_footprint_;

        double front_left_velocity_threshold_;
        double front_right_velocity_threshold_;
        double rear_left_velocity_threshold_;
        double rear_right_velocity_threshold_;

        SwerveDriveKinematics swerveDriveKinematics_;
        
        std::queue<Twist> previous_commands_;  // last two commands

        double pose_covariance_diagonal_array_[6];
        double twist_covariance_diagonal_array_[6];

        double publish_rate_ = 50.0;
        rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
        rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

        struct WheelParams{
            double x_offset = 0.0; // Chassis Center to Axle Center
            double y_offset = 0.0; // Axle Center to Wheel Center
            double radius = 0.0;   // Assumed to be the same for all wheels
        } wheel_params_;

        // Timeout to consider cmd_vel commands old
        std::chrono::milliseconds cmd_vel_timeout_{500};
        rclcpp::Time previous_update_timestamp_{0};

        // Topic Subscription
        bool subscriber_is_active_ = false;
        rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
            velocity_command_unstamped_subscriber_ = nullptr;

        realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;
        std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_ = nullptr;

        bool is_halted_ = false;
        bool use_stamped_vel_ = true;
        bool reset();
        void halt();
};

} // namespace swerve_controller
#endif // SWERVE_DRIVE_CONTROLLER_HPP_