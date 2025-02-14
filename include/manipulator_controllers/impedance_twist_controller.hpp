#ifndef IMPEDANCE_TWIST_CONTROLLER__IMPEDANCE_TWIST_CONTROLLER_HPP_
#define IMPEDANCE_TWIST_CONTROLLER__IMPEDANCE_TWIST_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "impedance_twist_controller_parameters.hpp"

#include "manipulator_controllers/base_force_controller.hpp"
#include "manipulator_controllers/utils.hpp"

namespace manipulator_controllers
{
class ImpedanceTwistController : public manipulator_controllers::BaseForceController
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
  std::shared_ptr<impedance_twist_controller::ParamListener> impedance_twist_controller_parameter_handler_;
  impedance_twist_controller::Params impedance_twist_controller_parameters_;

  Eigen::Matrix<double, 6, 1> damping_;
  Eigen::Matrix<double, 6, 1> max_damping_force_;
  Eigen::Matrix<double, 6, 1> selected_axes_;
  Eigen::Matrix<double, 6, 1> error_scale_;
  Eigen::VectorXd nullspace_joint_pos_;
  Eigen::VectorXd nullspace_stiffness_;
  Eigen::VectorXd nullspace_damping_ratio_;
  Eigen::VectorXd nullspace_damping_;
  
  // Eigen::MatrixXd I_;


};

}  // namespace manipulator_controllers

#endif  // IMPEDANCE_TWIST_CONTROLLER__IMPEDANCE_TWIST_CONTROLLER_HPP_
