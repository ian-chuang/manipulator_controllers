#ifndef IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_
#define IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "impedance_pose_controller_parameters.hpp"

#include "manipulator_controllers/base_force_controller.hpp"

namespace manipulator_controllers
{
class ImpedancePoseController : public manipulator_controllers::BaseForceController
{
public:

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:

  // parameters
  std::shared_ptr<impedance_pose_controller::ParamListener> impedance_pose_controller_parameter_handler_;
  impedance_pose_controller::Params impedance_pose_controller_parameters_;

  std::string control_frame_id_;
  double lms_damping_;
  double joint_damping_;
  Eigen::Matrix<double, 6, 1> stiffness_;
  Eigen::Matrix<double, 6, 1> damping_ratio_;
  Eigen::Matrix<double, 6, 1> damping_;
  Eigen::Matrix<double, 6, 1> selected_axes_;
  Eigen::Matrix<double, 6, 1> mass_;
  Eigen::Matrix<double, 6, 1> mass_inv_;
  Eigen::VectorXd joint_ref_pos_;
  Eigen::VectorXd joint_cur_pos_;
  Eigen::VectorXd joint_cur_vel_;
  Eigen::VectorXd joint_des_pos_;
  Eigen::VectorXd joint_des_vel_;
  Eigen::VectorXd joint_des_acc_;
  
  Eigen::MatrixXd I_;

  template <typename T1, typename T2>
  void vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
  {
    for (auto col = 0; col < matrix.cols(); col++)
    {
      for (auto row = 0; row < matrix.rows(); row++)
      {
        matrix(row, col) = data[row + col * matrix.rows()];
      }
    }
  }

};

}  // namespace manipulator_controllers

#endif  // IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_
