#ifndef BASE_FORCE_CONTROLLER__BASE_FORCE_CONTROLLER_HPP_
#define BASE_FORCE_CONTROLLER__BASE_FORCE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "base_force_controller_parameters.hpp"

#include "manipulator_controllers/base_controller.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace manipulator_controllers
{
class BaseForceController : public manipulator_controllers::BaseController
{
public:

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // force torque sensor
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // control loop data
  geometry_msgs::msg::Wrench ft_values_;

  // base force parameters
  std::shared_ptr<base_force_controller::ParamListener> base_force_controller_parameter_handler_;
  base_force_controller::Params base_force_controller_parameters_;

  /**
   * @brief Read values from hardware interfaces and set corresponding fields of state_current and
   * ft_values
   */
  void read_state_from_hardware(
    geometry_msgs::msg::Wrench & ft_values
  );
  
};

}  // namespace manipulator_controllers

#endif  // BASE_FORCE_CONTROLLER__BASE_FORCE_CONTROLLER_HPP_
