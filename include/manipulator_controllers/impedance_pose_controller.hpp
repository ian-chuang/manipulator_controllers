#ifndef IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_
#define IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "impedance_pose_controller_parameters.hpp"

#include "manipulator_controllers/base_force_controller.hpp"
#include "manipulator_controllers/utils.hpp"

namespace manipulator_controllers
{
class ImpedancePoseController : public manipulator_controllers::BaseForceController
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
  std::shared_ptr<impedance_pose_controller::ParamListener> impedance_pose_controller_parameter_handler_;
  impedance_pose_controller::Params impedance_pose_controller_parameters_;

  Eigen::Matrix<double, 6, 1> stiffness_;
  Eigen::Matrix<double, 6, 1> damping_ratio_;
  Eigen::Matrix<double, 6, 1> damping_;
  Eigen::Matrix<double, 6, 1> selected_axes_;
  // Eigen::Matrix<double, 6, 1> mass_;
  // Eigen::Matrix<double, 6, 1> mass_inv_;
  Eigen::Matrix<double, 6, 1> error_scale_;
  Eigen::Matrix<double, 6, 1> max_spring_force_;
  // Eigen::VectorXd joint_ref_pos_;
  // Eigen::VectorXd joint_cur_pos_;
  // Eigen::VectorXd joint_cur_vel_;
  // Eigen::VectorXd joint_des_pos_;
  // Eigen::VectorXd joint_des_vel_;
  // Eigen::VectorXd joint_des_acc_;
  Eigen::VectorXd nullspace_joint_pos_;
  Eigen::VectorXd nullspace_stiffness_;
  Eigen::VectorXd nullspace_damping_ratio_;
  Eigen::VectorXd nullspace_damping_;
  
  // Eigen::MatrixXd I_;


};

}  // namespace manipulator_controllers

#endif  // IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_
