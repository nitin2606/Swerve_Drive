#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include <tuple>

#include "rclcpp/logging.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "swerve_drive_controller/swerve_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace{

constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";

}  // namespace

namespace swerve_drive_controller{

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;


Wheel::Wheel(std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity, std::string name): velocity_(velocity), name(std::move(name)) {}

void Wheel::set_velocity(double velocity){
  velocity_.get().set_value(velocity);
}

Axle::Axle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> position, std::string name): position_(position), name(std::move(name)) {}

void Axle::set_position(double position){
  position_.get().set_value(position);
}

std::array<std::pair<double, double>, 4> wheel_positions_ = {
  std::make_pair(-0.1, 0.175), // front left
  std::make_pair(0.1, 0.175), // front right
  std::make_pair(-0.1, -0.175), // rear left
  std::make_pair(0.1, -0.175) // rear right
};



SwerveController::SwerveController() : controller_interface::ControllerInterface(), swerveDriveKinematics_(wheel_positions_){
  auto zero_twist = std::make_shared<Twist>();
  zero_twist->header.stamp = rclcpp::Time(0);
  zero_twist->twist.linear.x = 0.0;
  zero_twist->twist.linear.y = 0.0;
  zero_twist->twist.angular.z = 0.0;
  received_velocity_msg_ptr_.set(zero_twist);
}

CallbackReturn SwerveController::on_init(){

    RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE on_init...");

    try{
      // Declare parameters
      auto_declare<std::string>("joint_steering_left_front", front_left_axle_joint_name_);
      auto_declare<std::string>("joint_steering_right_front", front_right_axle_joint_name_);
      auto_declare<std::string>("joint_steering_left_rear", rear_left_axle_joint_name_);
      auto_declare<std::string>("joint_steering_right_rear", rear_right_axle_joint_name_);

      auto_declare<std::string>("joint_wheel_left_front", front_left_wheel_joint_name_);
      auto_declare<std::string>("joint_wheel_right_front", front_right_wheel_joint_name_);
      auto_declare<std::string>("joint_wheel_left_rear", rear_left_wheel_joint_name_);
      auto_declare<std::string>("joint_wheel_right_rear", rear_right_wheel_joint_name_);

      auto_declare<double>("chassis_length", wheel_params_.x_offset);
      auto_declare<double>("chassis_width", wheel_params_.y_offset);
      auto_declare<double>("wheel_radius", wheel_params_.radius);

      auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
      auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
      auto_declare<std::string>("odom", odometry_topic_);
      auto_declare<std::string>("base_footprint", base_footprint_);
      auto_declare<double>("publish_rate", publish_rate_);

      auto_declare<double>("front_left_velocity_threshold", front_left_velocity_threshold_);
      auto_declare<double>("front_right_velocity_threshold", front_right_velocity_threshold_);
      auto_declare<double>("rear_left_velocity_threshold", rear_left_velocity_threshold_);
      auto_declare<double>("rear_right_velocity_threshold", rear_right_velocity_threshold_);

      RCLCPP_INFO(get_node()->get_logger(), "Command Interfaces Length In on_init: %ld", command_interfaces_.size());

    }

    catch (const std::exception &e){

      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;

    }

    return CallbackReturn::SUCCESS;
}

InterfaceConfiguration SwerveController::command_interface_configuration() const{

  RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE command_interface_configuration...");

  std::vector<std::string> conf_names;

  conf_names.push_back(front_left_wheel_joint_name_  +  "/" + HW_IF_VELOCITY);
  conf_names.push_back(front_right_wheel_joint_name_ +  "/" + HW_IF_VELOCITY);
  conf_names.push_back(rear_left_wheel_joint_name_   +  "/" + HW_IF_VELOCITY);
  conf_names.push_back(rear_right_wheel_joint_name_  +  "/" + HW_IF_VELOCITY);

  conf_names.push_back(front_left_axle_joint_name_   +  "/" + HW_IF_POSITION);
  conf_names.push_back(front_right_axle_joint_name_  +  "/" + HW_IF_POSITION);
  conf_names.push_back(rear_left_axle_joint_name_    +  "/" + HW_IF_POSITION);
  conf_names.push_back(rear_right_axle_joint_name_   +  "/" + HW_IF_POSITION);

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}


InterfaceConfiguration SwerveController::state_interface_configuration() const{
  RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE state_interface_configuration...");
  
  return {interface_configuration_type::NONE};
}


CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){

    RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE on_configure...");

    auto logger = get_node()->get_logger();
    try{

        // Fetch joint names from parameters
        front_left_wheel_joint_name_  = get_node()->get_parameter("joint_wheel_left_front").as_string();
        front_right_wheel_joint_name_ = get_node()->get_parameter("joint_wheel_right_front").as_string();
        rear_left_wheel_joint_name_   = get_node()->get_parameter("joint_wheel_left_rear").as_string();
        rear_right_wheel_joint_name_  = get_node()->get_parameter("joint_wheel_right_rear").as_string();

        front_left_axle_joint_name_   = get_node()->get_parameter("joint_steering_left_front").as_string();
        front_right_axle_joint_name_  = get_node()->get_parameter("joint_steering_right_front").as_string();
        rear_left_axle_joint_name_    = get_node()->get_parameter("joint_steering_left_rear").as_string();
        rear_right_axle_joint_name_   = get_node()->get_parameter("joint_steering_right_rear").as_string();

        odometry_topic_ = get_node()->get_parameter("odom").as_string();
        base_footprint_ = get_node()->get_parameter("base_footprint").as_string();
        publish_rate_   = get_node()->get_parameter("publish_rate").as_double();

        wheel_params_.x_offset = get_node()->get_parameter("chassis_length").as_double();
        wheel_params_.y_offset = get_node()->get_parameter("chassis_width").as_double();
        wheel_params_.radius   = get_node()->get_parameter("wheel_radius").as_double();

        front_left_velocity_threshold_  = get_node()->get_parameter("front_left_velocity_threshold").as_double();
        front_right_velocity_threshold_ = get_node()->get_parameter("front_right_velocity_threshold").as_double();
        rear_left_velocity_threshold_   = get_node()->get_parameter("rear_left_velocity_threshold").as_double();
        rear_right_velocity_threshold_  = get_node()->get_parameter("rear_right_velocity_threshold").as_double();


    
        for (std::size_t i = 0; i < 6; ++i) {
          pose_covariance_diagonal_array_[i] = 0.01;
        }   

        for (std::size_t i = 0; i < 6; ++i) {
          twist_covariance_diagonal_array_[i] = 0.01;
        }         


        if(front_left_wheel_joint_name_.empty()){
            RCLCPP_ERROR(logger, "front_wheel_joint_name is not set");
            return CallbackReturn::ERROR;
        }
        if(front_right_wheel_joint_name_.empty()){
            RCLCPP_ERROR(logger, "front_right_wheel_joint_name is not set");
            return CallbackReturn::ERROR;
        }
        if(rear_left_wheel_joint_name_.empty()){
            RCLCPP_ERROR(logger, "rear_left_wheel_joint_name is not set");
            return CallbackReturn::ERROR;
        }
        if(rear_right_wheel_joint_name_.empty()){
            RCLCPP_ERROR(logger, "rear_right_wheel_joint_name is not set");
            return CallbackReturn::ERROR;
        }

        if(front_left_axle_joint_name_.empty()){
            RCLCPP_ERROR(logger, "front_axle_joint_name is not set");
            return CallbackReturn::ERROR;
        }
        if(front_right_axle_joint_name_.empty()){
            RCLCPP_ERROR(logger, "front_right_axle_joint_name is not set");
            return CallbackReturn::ERROR;
        }
        if( rear_left_axle_joint_name_.empty()){
            RCLCPP_ERROR(logger, "rear_left_axle_joint_name_ is not set");
            return CallbackReturn::ERROR;
        }
        if( rear_right_axle_joint_name_.empty()){
            RCLCPP_ERROR(logger, "rear_right_axle_joint_name_ is not set");
            return CallbackReturn::ERROR;
        }

        

        cmd_vel_timeout_ = std::chrono::milliseconds(static_cast<int>(
            get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0));

        use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

        if (!reset()){
          return CallbackReturn::ERROR;
        }

        const Twist empty_twist;
        received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

        // Initialize subscription for velocity command
        if (use_stamped_vel_){
            velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
              DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<Twist> msg) -> void {

                if (!subscriber_is_active_){
                  RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                  return;
                }

                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)){
                    RCLCPP_WARN_ONCE(
                        get_node()->get_logger(),
                        "Received TwistStamped with zero timestamp, setting it to current "
                        "time, this message will only be shown once");
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                received_velocity_msg_ptr_.set(std::move(msg));
            });
            RCLCPP_INFO(logger, "INSIDE STAMPED TWIST SUBSCRIPTION");
        }

        else{
            velocity_command_unstamped_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
                if (!subscriber_is_active_){
                    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                    return;
                }
                // Write fake header in the stored stamped command
                std::shared_ptr<Twist> twist_stamped;
                received_velocity_msg_ptr_.get(twist_stamped);
                twist_stamped->twist = *msg;
                twist_stamped->header.stamp = get_node()->get_clock()->now();
            });
            RCLCPP_INFO(logger, "INSIDE UNSTAMPED TWIST SUBSCRIPTION");
        }


        RCLCPP_INFO(logger, "SWERVEDRIVE CONTROLLER SUCCESSFULLY CONFIGURED ...");

        odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
          DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
        
        realtime_odometry_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
          odometry_publisher_);
        
        std::string tf_prefix = "";
        tf_prefix = std::string(get_node()->get_namespace());

        RCLCPP_INFO(logger, "GOT TF PREFIX: %s", tf_prefix.c_str());

        if (tf_prefix == "/"){
          tf_prefix = "";
        }

        else{
          tf_prefix = tf_prefix + "/";
        }


        const auto odom_frame_id = tf_prefix + odometry_topic_;
        const auto base_frame_id = tf_prefix + base_footprint_;

        RCLCPP_INFO(logger, "GOT ODOM FRAME ID: %s\n GOT BASE FRAME ID: %s", odom_frame_id.c_str(), base_frame_id.c_str());

        auto & odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.frame_id = odom_frame_id;
        odometry_message.child_frame_id = base_frame_id;

        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

        odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

        constexpr std::size_t NUM_DIMENSIONS = 6;
        for (std::size_t index = 0; index < 6; ++index){

          // 0, 7, 14, 21, 28, 35
          const std::size_t diagonal_index = NUM_DIMENSIONS * index + index;
          odometry_message.pose.covariance[diagonal_index] = pose_covariance_diagonal_array_[index];
          odometry_message.twist.covariance[diagonal_index] = twist_covariance_diagonal_array_[index];
          
        } 

        odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
          DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
        
        realtime_odometry_transform_publisher_ = 
          std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);


      auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
      odometry_transform_message.transforms.resize(1);
      odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
      odometry_transform_message.transforms.front().child_frame_id = base_frame_id;
         
      previous_update_timestamp_ = get_node()->get_clock()->now();

    }

    catch (const std::exception &e){
      
      RCLCPP_ERROR(logger, "EXCEPTION DURING on_configure: %s", e.what());
      return CallbackReturn::ERROR;

    }

    std::chrono::seconds sleep_duration(1);
    rclcpp::sleep_for(sleep_duration);


    return CallbackReturn::SUCCESS;
}


controller_interface::return_type SwerveController::update(const rclcpp::Time &time, const rclcpp::Duration & period){

    RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE on_undate...");

    auto logger = get_node()->get_logger();
    if (get_state().id() == State::PRIMARY_STATE_INACTIVE){
        
      if (!is_halted_){

        halt();
        is_halted_ = true;   
      }

      return controller_interface::return_type::OK;
    }

    const auto current_time = time;

    std::shared_ptr<Twist> last_command_msg = std::make_shared<Twist>();
    received_velocity_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr){

      last_command_msg = std::make_shared<Twist>();
      last_command_msg->header.stamp = current_time;
      last_command_msg->twist.linear.x = 0.0;
      last_command_msg->twist.linear.y = 0.0;
      last_command_msg->twist.angular.z = 0.0;

      received_velocity_msg_ptr_.set(last_command_msg);  // Update the shared pointer

      RCLCPP_WARN(logger, "No velocity command received, using zero velocity");
      
    }

    else if (last_command_msg != nullptr){
      RCLCPP_INFO(logger, "X:   %f, Y:   %f, Z_Ang:   %f", last_command_msg->twist.linear.x, last_command_msg->twist.linear.y, last_command_msg->twist.angular.z);
    }
    
    const auto age_of_last_command = current_time - last_command_msg->header.stamp;

    RCLCPP_INFO(logger, "[SWERVE_DRIVE_CONTROLLER] Age of last command: %f", age_of_last_command.seconds());
    
    if (age_of_last_command > cmd_vel_timeout_){

      last_command_msg->twist.linear.x = 0.0;
      last_command_msg->twist.angular.z = 0.0;
    }

    Twist command = *last_command_msg;
    double &linear_x_cmd = command.twist.linear.x;
    double &linear_y_cmd = command.twist.linear.y;
    double &angular_cmd = command.twist.angular.z;

    RCLCPP_INFO(logger, "LINEAR_X: %f  || LINEAR_Y: %f  || ANGULAR_Z: %f\n", linear_x_cmd, linear_y_cmd, angular_cmd);
    RCLCPP_INFO(logger, "-----------------------------------------------------------");

    double x_offset = wheel_params_.x_offset;
    double y_offset = wheel_params_.y_offset;
    double radius = wheel_params_.radius;

    // RCLCPP_INFO(logger, "x_offset: %f  || y_offset: %f  || radius: %f", x_offset, y_offset, radius);

    // ------------------------------------------------------------------------------------------------------------------


    auto wheel_command_ = swerveDriveKinematics_.compute_wheel_commands(linear_x_cmd, linear_y_cmd, angular_cmd);

    std::vector<std::tuple<WheelCommand&, double, std::string>> wheel_data = {
      {wheel_command_[0], front_left_velocity_threshold_, "front_left_wheel"},
      {wheel_command_[1], front_right_velocity_threshold_, "front_right_wheel"},
      {wheel_command_[2], rear_left_velocity_threshold_, "rear_left_wheel"},
      {wheel_command_[3], rear_right_velocity_threshold_, "rear_right_wheel"}
    };

    for (const auto& [command, threshold, label] : wheel_data){
      if(command.drive_velocity > threshold){
        command.drive_velocity = threshold;
        RCLCPP_INFO(logger, "Setting %s velocity to threshold", label.c_str());
      }
    }


    // if(wheel_command_[0].drive_velocity>front_left_velocity_threshold_){
    //   wheel_command_[0].drive_velocity = front_left_velocity_threshold_;
    //   RCLCPP_INFO(logger, "Setting front_left_wheel velocity to threshold");
    // }

    // if(wheel_command_[1].drive_velocity>front_right_velocity_threshold_){
    //   wheel_command_[1].drive_velocity = front_right_velocity_threshold_;
    //   RCLCPP_INFO(logger, "Setting front_right_wheel velocity to threshold");
    // }

    // if(wheel_command_[2].drive_velocity>rear_left_velocity_threshold_){
    //   wheel_command_[2].drive_velocity = rear_left_velocity_threshold_;
    //   RCLCPP_INFO(logger, "Setting rear_left_wheel_velocity to threshold");
    // }

    // if(wheel_command_[3].drive_velocity>rear_right_velocity_threshold_){
    //   wheel_command_[3].drive_velocity = rear_right_velocity_threshold_;
    //   RCLCPP_INFO(logger, "Setting rear_right_wheel_velocity to threshold");
    // }


    const double front_left_velocity = wheel_command_[0].drive_velocity;
    const double front_left_angle = wheel_command_[0].steering_angle;

    const double front_right_velocity = wheel_command_[1].drive_velocity;
    const double front_right_angle = wheel_command_[1].steering_angle;

    const double rear_left_velocity = wheel_command_[2].drive_velocity;
    const double rear_left_angle = wheel_command_[2].steering_angle;

    const double rear_right_velocity = wheel_command_[3].drive_velocity;
    const double rear_right_angle = wheel_command_[3].steering_angle;

  
    // Set velocities and steering angles
    
    try{

      if(front_left_axle_handle_ != nullptr && front_left_wheel_handle_ != nullptr){
        front_left_axle_handle_->set_position(front_left_angle);
        front_left_wheel_handle_->set_velocity(front_left_velocity);
      }
      else{
        RCLCPP_ERROR(logger, "Front Left Axle Handle or Wheel Handle is NULLPTR");
      }

      if(front_right_axle_handle_ != nullptr && front_right_wheel_handle_ != nullptr){
        front_right_axle_handle_->set_position(front_right_angle);
        front_right_wheel_handle_->set_velocity(front_right_velocity);
      }
      else{
        RCLCPP_ERROR(logger, "Front Right Axle Handle or Wheel Handle is NULLPTR");
      }

      
      if(rear_left_axle_handle_ != nullptr && rear_left_wheel_handle_ != nullptr){
        rear_left_axle_handle_->set_position(rear_left_angle); 
        rear_left_wheel_handle_->set_velocity(rear_left_velocity);   
      }
      else{
        RCLCPP_ERROR(logger, "Rear Left Axle or Wheel Handle is NULLPTR");
      }

      if(rear_right_axle_handle_ != nullptr && rear_right_wheel_handle_ != nullptr){
        rear_right_axle_handle_->set_position(rear_right_angle);
        rear_right_wheel_handle_->set_velocity(rear_right_velocity);
      }
      else{
        RCLCPP_ERROR(logger, "Rear Right Axle or Wheel Handle is NULLPTR");
      }
    }

    catch(std::exception &e){
      RCLCPP_INFO(logger, "---------------XXXXXXXXXXXXXXXXX---------------");
      RCLCPP_INFO(logger, "Exception caught: %s", e.what());
      RCLCPP_INFO(logger, "---------------XXXXXXXXXXXXXXXXX---------------");
    }


    RCLCPP_INFO(logger, "-------------------COMPUTED COMMANDS-------------------");
    RCLCPP_INFO(logger, "\nLinear X:  %f, Linear Y:  %f, Angular Z:  %f", linear_x_cmd, linear_y_cmd, angular_cmd);
    RCLCPP_INFO(logger, "Front Left Velocity:  %f, Front Right Velocity: %f, Rear Left Velocity:  %f, Rear Right Velocity:  %f", front_left_velocity, front_right_velocity, rear_left_velocity, rear_right_velocity);
    RCLCPP_INFO(logger, "Front Left Angle:  %f, Front Right Angle: %f, Rear Left Angle:  %f, Rear Right Angle:  %f\n", front_left_angle, front_right_angle, rear_left_angle, rear_right_angle);
    RCLCPP_INFO(logger, "-------------------COMPUTED COMMANDS-------------------");



    const auto update_dt = current_time - previous_update_timestamp_;
    previous_update_timestamp_ = current_time;

    std::array<double,4> velocity_array = {front_left_velocity, front_right_velocity, rear_left_velocity, rear_right_velocity};
    std::array<double,4> steering_angles = {front_left_angle, front_right_angle, rear_left_angle, rear_right_angle};

    auto odometry_ = swerveDriveKinematics_.update_odometry(velocity_array, steering_angles, update_dt.seconds());

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.theta);

    bool should_publish = false;

    try
    {
      if (previous_publish_timestamp_ + publish_period_ < time)
      {
        previous_publish_timestamp_ += publish_period_;
        should_publish = true;
      }
    }
    catch (const std::runtime_error &)
    {
      // Handle exceptions when the time source changes and initialize publish timestamp
      previous_publish_timestamp_ = time;
      should_publish = true;
    }


    if(realtime_odometry_publisher_ && realtime_odometry_publisher_->trylock()){
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.x;
      odometry_message.pose.pose.position.y = odometry_.y;
      odometry_message.pose.pose.orientation.z = odometry_.theta;
      

      RCLCPP_INFO(logger, "-------------------COMPUTED ODOMETRY-------------------");
      RCLCPP_INFO(logger, "Odometry Linear X:  %f, Odometry Linear Y:  %f, Odometry Angular Z:  %f", odometry_.x, odometry_.y, odometry_.theta);
      RCLCPP_INFO(logger, "-------------------COMPUTED ODOMETRY-------------------");

      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (realtime_odometry_transform_publisher_ && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;

      transform.transform.translation.x = odometry_.x;
      transform.transform.translation.y = odometry_.y;
      transform.transform.translation.z = 0.0;     // Add this for completeness

      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}



CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE on_activate...");

  front_left_wheel_handle_ = get_wheel(front_left_wheel_joint_name_);
  front_right_wheel_handle_ = get_wheel(front_right_wheel_joint_name_);
  rear_left_wheel_handle_ = get_wheel(rear_left_wheel_joint_name_);
  rear_right_wheel_handle_= get_wheel(rear_right_wheel_joint_name_);

  front_left_axle_handle_ = get_axle(front_left_axle_joint_name_);
  front_right_axle_handle_ = get_axle(front_right_axle_joint_name_);
  rear_left_axle_handle_ = get_axle(rear_left_axle_joint_name_);
  rear_right_axle_handle_ = get_axle(rear_right_axle_joint_name_);


  if(!front_left_wheel_handle_){
    RCLCPP_ERROR(get_node()->get_logger(), "ERROR IN FETCHING front_left_wheel_handle_");
    return CallbackReturn::ERROR;
  }

  if(!front_right_wheel_handle_){
    RCLCPP_ERROR(get_node()->get_logger(), "ERROR IN FETCHING front_right_wheel_handle_");
    return CallbackReturn::ERROR;
  }

  if(!rear_left_wheel_handle_){
    RCLCPP_ERROR(get_node()->get_logger(), "ERROR IN FETCHING rear_left_wheel_handle_");
    return CallbackReturn::ERROR;
  }

  if(!rear_right_wheel_handle_){
    RCLCPP_ERROR(get_node()->get_logger(), "ERROR IN FETCHING rear_right_wheel_handle_");
    return CallbackReturn::ERROR;
  }

  if(!front_left_axle_handle_){
    RCLCPP_ERROR(get_node()->get_logger(), "ERROR IN FETCHING front_left_axle_handle_");
    return CallbackReturn::ERROR;
  }

  if(!front_right_axle_handle_){
    RCLCPP_ERROR(get_node()->get_logger(), "ERROR IN FETCHING front_right_axle_handle_");
    return CallbackReturn::ERROR;
  }

  if(!rear_left_axle_handle_){
    RCLCPP_ERROR(get_node()->get_logger(), "ERROR IN FETCHING rear_left_axle_handle_");
    // std::chrono::seconds sleep_duration(50);
    // rclcpp::sleep_for(sleep_duration);
    return CallbackReturn::ERROR;
  }

  if(!rear_right_axle_handle_){
    RCLCPP_ERROR(get_node()->get_logger(), "ERROR IN FETCHING rear_right_axle_handle_");
    return CallbackReturn::ERROR;
  }


  is_halted_ = false;
  subscriber_is_active_ = true;

  RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now active.");

    RCLCPP_INFO(get_node()->get_logger(), "Command Interfaces Length In on_activate: %ld", command_interfaces_.size());
    RCLCPP_INFO(get_node()->get_logger(), "AVAILABLE COMMAND INTERFACES:");

    for (const auto & interface : command_interfaces_) {
      RCLCPP_INFO(get_node()->get_logger(), "  Name: %s, Interface: %s", interface.get_name().c_str(), interface.get_interface_name().c_str());
    } 
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE on_deactivate...");

  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE on_cleanup...");

  if (!reset()){
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE on_error...");

  if (!reset()){
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool SwerveController::reset(){
  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  auto zero_twist = std::make_shared<Twist>();
  zero_twist->header.stamp = get_node()->get_clock()->now();
  zero_twist->twist.linear.x = 0.0;
  zero_twist->twist.linear.y = 0.0;
  zero_twist->twist.angular.z = 0.0;
  received_velocity_msg_ptr_.set(zero_twist);

  // received_velocity_msg_ptr_.set(nullptr);
  is_halted_ = false;
  return true;
}

CallbackReturn SwerveController::on_shutdown(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE on_shutdown...");

  return CallbackReturn::SUCCESS;
}

void SwerveController::halt(){

    RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE halt...");


    front_left_wheel_handle_->set_velocity(0.0);
    front_left_axle_handle_->set_position(0.0);
    front_right_wheel_handle_->set_velocity(0.0);
    front_right_axle_handle_->set_position(0.0);
    rear_left_wheel_handle_->set_velocity(0.0);
    rear_left_axle_handle_->set_position(0.0);
    rear_right_wheel_handle_->set_velocity(0.0);
    rear_right_axle_handle_->set_position(0.0);

    RCLCPP_WARN(get_node()->get_logger(), "-----HALT CALLED : STOPPING ALL MOTORS-----");
}

std::shared_ptr<Wheel> SwerveController::get_wheel( const std::string & wheel_name ){

  auto logger = get_node()->get_logger();

  if (wheel_name.empty()){
    RCLCPP_ERROR(logger, "Wheel joint name not given. Make sure all joints are specified.");
    return nullptr;
  }
  // RCLCPP_INFO(logger, "Called get_wheel for %s\n", wheel_name.c_str());

  // Get Command Handle for joint
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&wheel_name](const auto & interface) {
     
      return interface.get_name() == (wheel_name+"/velocity") &&
              interface.get_interface_name() == HW_IF_VELOCITY;
      // return interface.get_interface_name() == HW_IF_VELOCITY;
    });

  // RCLCPP_INFO(get_node()->get_logger(), "WHEEL NAME: %s,    COMMAND_HANDLE_NAME: %s     COMMAND_INTERFACE_NAME: %s", wheel_name.c_str(), command_handle->get_name().c_str(), command_handle->get_interface_name().c_str());

  if (command_handle == command_interfaces_.end()){

    RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", wheel_name.c_str());
    return nullptr;
  }
  return std::make_shared<Wheel>(std::ref(*command_handle), wheel_name);
}

std::shared_ptr<Axle> SwerveController::get_axle( const std::string & axle_name ){
  auto logger = get_node()->get_logger();
  if (axle_name.empty()){

    RCLCPP_ERROR(logger, "Wheel joint name not given. Make sure all joints are specified.");
    return nullptr;
  }

  RCLCPP_INFO(logger, "Called get_axle for %s\n", axle_name.c_str());


  // Log all available command interfaces for debugging
  RCLCPP_INFO(logger, "Available command interfaces:");
  for (const auto& interface : command_interfaces_) {
    RCLCPP_INFO(logger, " - Name: %s, Interface: %s", 
      interface.get_name().c_str(),
      interface.get_interface_name().c_str());
  }

  // Get Command Handle for joint
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&axle_name](const auto & interface) {
      return interface.get_name() == (axle_name + "/position") &&
              interface.get_interface_name() == HW_IF_POSITION;
    });

  if (command_handle == command_interfaces_.end()) {
    RCLCPP_ERROR(logger, "Unable to find command interface for axle: %s", axle_name.c_str());
    RCLCPP_ERROR(logger, "Expected interface name: %s/position", axle_name.c_str());
    return nullptr;
  }

  return std::make_shared<Axle>(std::ref(*command_handle), axle_name);
}

}  // namespace swerve_drive_controller


#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    swerve_drive_controller::SwerveController, controller_interface::ControllerInterface)
