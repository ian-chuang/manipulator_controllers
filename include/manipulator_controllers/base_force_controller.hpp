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
#include "std_srvs/srv/trigger.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "manipulator_controllers/utils.hpp"

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

  // zero wrench service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_wrench_service_;
  std::atomic<bool> zero_wrench_flag_;
  Eigen::Matrix<double, 6, 1> zero_wrench_offset_;

  // wrench publisher
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr w_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> wrench_publisher_;

  // force torque sensor
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // control loop data
  geometry_msgs::msg::Wrench ft_values_;

  // filtered wrench in base frame
  Eigen::Matrix<double, 6, 1> base_wrench_;

  // position of center of gravity in cog_frame
  Eigen::Vector3d cog_pos_;

  // force applied to sensor due to weight of end effector
  Eigen::Vector3d end_effector_weight_;

  // base force parameters
  std::shared_ptr<base_force_controller::ParamListener> base_force_controller_parameter_handler_;
  base_force_controller::Params base_force_controller_parameters_;

  /**
   * @brief Read values from hardware interfaces and set corresponding fields of state_current and
   * ft_values
   */
  void read_state_from_hardware(
    trajectory_msgs::msg::JointTrajectoryPoint & joint_state,
    geometry_msgs::msg::Wrench & ft_values
  );

  void process_wrench_measurements(
    const trajectory_msgs::msg::JointTrajectoryPoint & joint_state,
    const geometry_msgs::msg::Wrench & measured_wrench,
    geometry_msgs::msg::Wrench & processed_wrench
  );
  
};

}  // namespace manipulator_controllers

#endif  // BASE_FORCE_CONTROLLER__BASE_FORCE_CONTROLLER_HPP_
