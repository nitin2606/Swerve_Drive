#ifndef SWERVE_DRIVE_CONTROLLER__CUSTOM_VALIDATORS_HPP_
#define SWERVE_DRIVE_CONTROLLER__CUSTOM_VALIDATORS_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rsl/parameter_validators.hpp>
#include <tl_expected/expected.hpp>

namespace swerve_drive_controller
{

/**
 * @brief Ensure the parameter is either greater than or equal to a given value, or NaN.
 */

template <typename T>
tl::expected<void, std::string> gt_eq_or_nan(rclcpp::Parameter const & parameter, T expected_value){

    auto param_value = parameter.as_double();
    if(!std::isnan(param_value)){
        return rsl::gt_eq<T>(parameter, expected_value);
    }
    return {};
}

/**
 * @brief Ensure the parameter is either less than or equal to a given value, or NaN.
 */

template<typename T>
tl::expected<void, std::string> lt_eq_or_nan(rclcpp::Parameter const & parameter, T expected_value){
    
    auto param_value = parameter.as_double();
    if(!std::isnan(param_value)){

        return rsl::lt_eq<T>(parameter, expected_value);
    }
    return {};

}

}  // namespace swerve_drive_controller

#endif // SWERVE_DRIVE_CONTROLLER__CUSTOM_VALIDATORS_HPP_