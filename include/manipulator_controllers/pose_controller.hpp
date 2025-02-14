#ifndef POSE_CONTROLLER__POSE_CONTROLLER_HPP_
#define POSE_CONTROLLER__POSE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "pose_controller_parameters.hpp"

#include "manipulator_controllers/base_controller.hpp"

namespace manipulator_controllers
{
class PoseController : public manipulator_controllers::BaseController
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
  std::shared_ptr<pose_controller::ParamListener> pose_controller_parameter_handler_;
  pose_controller::Params pose_controller_parameters_;

  Eigen::Matrix<double, 6, 1> kp_;
  Eigen::Matrix<double, 6, 1> kd_ratio_;
  Eigen::Matrix<double, 6, 1> kd_;
  Eigen::Matrix<double, 6, 1> max_twist_;
  Eigen::VectorXd nullspace_joint_pos_;
  Eigen::VectorXd nullspace_kp_;
  Eigen::VectorXd nullspace_kd_ratio_;
  Eigen::VectorXd nullspace_kd_;
};

}  // namespace manipulator_controllers

#endif  // POSE_CONTROLLER__POSE_CONTROLLER_HPP_
