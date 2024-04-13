#ifndef TWIST_CONTROLLER__TWIST_CONTROLLER_HPP_
#define TWIST_CONTROLLER__TWIST_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "twist_controller_parameters.hpp"

#include "manipulator_controllers/base_controller.hpp"

namespace manipulator_controllers
{
class TwistController : public manipulator_controllers::BaseController
{
public:

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:

  bool set_params();

  // parameters
  std::shared_ptr<twist_controller::ParamListener> twist_controller_parameter_handler_;
  twist_controller::Params twist_controller_parameters_;

  Eigen::Matrix<double, 6, 1> kp_;
  Eigen::VectorXd nullspace_joint_pos_;
  Eigen::VectorXd nullspace_kp_;
  Eigen::VectorXd nullspace_kd_ratio_;
  Eigen::VectorXd nullspace_kd_;
};

}  // namespace manipulator_controllers

#endif  // TWIST_CONTROLLER__TWIST_CONTROLLER_HPP_
