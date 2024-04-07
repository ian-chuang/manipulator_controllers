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

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:

  // parameters
  std::shared_ptr<pose_controller::ParamListener> pose_controller_parameter_handler_;
  pose_controller::Params pose_controller_parameters_;

  Eigen::VectorXd k;
  Eigen::VectorXd d;
  // Eigen::Matrix<double, 6, 1> selected_axes;

  template <typename T1, typename T2>
  void vec_to_eigen(const std::vector<T1> & data, T2 & matrix);

};

}  // namespace manipulator_controllers

#endif  // POSE_CONTROLLER__POSE_CONTROLLER_HPP_
