#ifndef BASE_CONTROLLER__BASE_CONTROLLER_HPP_
#define BASE_CONTROLLER__BASE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "base_controller_parameters.hpp"

#include "controller_interface/helpers.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
// #include "realtime_tools/realtime_publisher.h"
// #include "semantic_components/force_torque_sensor.hpp"
// #include "kinematics_interface/kinematics_interface.hpp"
// #include "pluginlib/class_loader.hpp"
#include "manipulator_controllers/ik_solver.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace manipulator_controllers
{
class BaseController : public controller_interface::ChainableControllerInterface
{
public:
  
  controller_interface::CallbackReturn on_init() override;

  /// Export configuration of required command interfaces.
  /**
   * Allowed types of command interfaces are \ref hardware_interface::POSITION,
   * \ref hardware_interface::VELOCITY, \ref hardware_interface::ACCELERATION.
   */
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /// Export configuration of required state interfaces.
  /**
   * Allowed types of state interfaces are \ref hardware_interface::POSITION,
   * \ref hardware_interface::VELOCITY,
   */
  
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers() override;

  size_t num_joints_ = 0;
  std::vector<std::string> command_joint_names_;

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  bool has_position_state_interface_ = false;
  bool has_velocity_state_interface_ = false;
  bool has_acceleration_state_interface_ = false;
  bool has_position_command_interface_ = false;
  bool has_velocity_command_interface_ = false;
  bool has_acceleration_command_interface_ = false;
  bool has_effort_command_interface_ = false;

  // To reduce number of variables and to make the code shorter the interfaces are ordered in types
  // as the following constants
  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION};

  // internal reference values
  const std::vector<std::string> allowed_reference_interfaces_types_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
  std::vector<std::reference_wrapper<double>> position_reference_;
  std::vector<std::reference_wrapper<double>> velocity_reference_;

  // force torque sensor
  // std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  // ROS subscribers
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
    input_joint_command_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_pose_command_subscriber_;
  // rclcpp::Publisher<control_msgs::msg::AdmittanceControllerState>::SharedPtr s_publisher_;

  // base parameters
  std::shared_ptr<base_controller::ParamListener> base_controller_parameter_handler_;
  base_controller::Params base_controller_parameters_;

  // ROS messages
  std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> joint_command_msg_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> pose_command_msg_;

  // real-time buffer
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>>
    input_joint_command_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> input_pose_command_;
  // std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateMsg>> state_publisher_;

  trajectory_msgs::msg::JointTrajectoryPoint last_joint_command_;
  trajectory_msgs::msg::JointTrajectoryPoint last_joint_reference_;

  // control loop data
  // reference_: reference value read by the controller
  // joint_state_: current joint readings from the hardware
  // reference_admittance_: reference value used by the controller after the admittance values are
  // applied ft_values_: values read from the force torque sensor
  trajectory_msgs::msg::JointTrajectoryPoint joint_reference_, joint_state_, joint_command_;
  // geometry_msgs::msg::Wrench ft_values_;

  // Kinematics interface plugin loader
  std::unique_ptr<manipulator_controllers::IKSolver> ik_solver_;

  /**
   * @brief Read values from hardware interfaces and set corresponding fields of state_current and
   * ft_values
   */
  void read_state_from_hardware(
    trajectory_msgs::msg::JointTrajectoryPoint & state_current
  );

  /**
   * @brief Set fields of state_reference with values from controllers exported position and
   * velocity references
   */
  void read_state_reference_interfaces(trajectory_msgs::msg::JointTrajectoryPoint & state);

  /**
   * @brief Write values from state_command to claimed hardware interfaces
   */
  void write_state_to_hardware(const trajectory_msgs::msg::JointTrajectoryPoint & state_command);

  template <typename T1, typename T2>
  void vec_to_eigen(const std::vector<T1> & data, T2 & matrix);

  void create_gain_matrix(
    const Eigen::Matrix<double, 6, 1> & gains, 
    const Eigen::Isometry3d & base_control_transform,
    Eigen::Matrix<double, 6, 6> & gain_matrix
  ) {
    // Create stiffness matrix in base frame. The user-provided values of admittance_state.stiffness
    // correspond to the six diagonal elements of the stiffness matrix expressed in the control frame
    auto rot_base_control = base_control_transform.rotation();
    Eigen::Matrix<double, 6, 6> G = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> G_pos = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, 3, 3> G_rot = Eigen::Matrix<double, 3, 3>::Zero();
    G_pos.diagonal() = gains.block<3, 1>(0, 0);
    G_rot.diagonal() = gains.block<3, 1>(3, 0);
    // Transform to the control frame
    // A reference is here:  https://users.wpi.edu/~jfu2/rbe502/files/force_control.pdf
    // Force Control by Luigi Villani and Joris De Schutter
    // Page 200
    G_pos = rot_base_control * G_pos * rot_base_control.transpose();
    G_rot = rot_base_control * G_rot * rot_base_control.transpose();
    G.block<3, 3>(0, 0) = G_pos;
    G.block<3, 3>(3, 3) =  G_rot;

    gain_matrix = G;
  }

  
};

}  // namespace manipulator_controllers

#endif  // BASE_CONTROLLER__BASE_CONTROLLER_HPP_
