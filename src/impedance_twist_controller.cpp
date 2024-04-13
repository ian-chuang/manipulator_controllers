#include "manipulator_controllers/impedance_twist_controller.hpp"

namespace manipulator_controllers
{

bool ImpedanceTwistController::set_params() 
{
  vec_to_eigen(impedance_twist_controller_parameters_.impedance.control.damping, damping_);
  vec_to_eigen(impedance_twist_controller_parameters_.impedance.control.selected_axes, selected_axes_);
  vec_to_eigen(impedance_twist_controller_parameters_.impedance.control.error_scale, error_scale_);

  // nullspace joint position check that size is correct
  if (impedance_twist_controller_parameters_.impedance.nullspace.joint_positions.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_joint_position parameter does not match number of joints");
    return false;
  }
  if (impedance_twist_controller_parameters_.impedance.nullspace.stiffness.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_stiffness parameter does not match number of joints");
    return false;
  }
  if (impedance_twist_controller_parameters_.impedance.nullspace.damping_ratio.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_damping_ratio parameter does not match number of joints");
    return false;
  }
  nullspace_joint_pos_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_stiffness_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_damping_ratio_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_damping_ = Eigen::VectorXd::Zero(num_joints_);
  vec_to_eigen(impedance_twist_controller_parameters_.impedance.nullspace.joint_positions, nullspace_joint_pos_);
  vec_to_eigen(impedance_twist_controller_parameters_.impedance.nullspace.stiffness, nullspace_stiffness_);
  vec_to_eigen(impedance_twist_controller_parameters_.impedance.nullspace.damping_ratio, nullspace_damping_ratio_);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    nullspace_damping_[i] = nullspace_damping_ratio_[i] * 2 * sqrt(nullspace_stiffness_[i]);
  }

  return true;
}

controller_interface::CallbackReturn ImpedanceTwistController::on_init()
{
  auto ret = BaseForceController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // initialize controller config
  try
  {
    impedance_twist_controller_parameter_handler_ = std::make_shared<impedance_twist_controller::ParamListener>(get_node());
    impedance_twist_controller_parameters_ = impedance_twist_controller_parameter_handler_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceTwistController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = BaseForceController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  bool success = set_params();
  if (!success)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceTwistController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = BaseForceController::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  bool success = set_params();
  if (!success)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type ImpedanceTwistController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // update input reference from chainable interfaces
  read_state_reference_interfaces(joint_reference_);
  // get all controller inputs
  BaseController::read_state_from_hardware(joint_state_);
  BaseForceController::read_state_from_hardware(joint_state_, ft_values_);

  // get required joint vals (as Eigen)
  Eigen::VectorXd joint_ref_vel = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_cur_pos = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_cur_vel = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_des_pos = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_des_vel = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_des_acc = Eigen::VectorXd::Zero(num_joints_);
  vec_to_eigen(joint_reference_.velocities, joint_ref_vel);
  vec_to_eigen(joint_state_.positions, joint_cur_pos);
  vec_to_eigen(joint_state_.velocities, joint_cur_vel);
  vec_to_eigen(joint_command_.positions, joint_des_pos);
  vec_to_eigen(joint_command_.velocities, joint_des_vel);
  vec_to_eigen(joint_command_.accelerations, joint_des_acc);

  // get wrench
  Eigen::Matrix<double, 6, 1> base_wrench;
  base_wrench(0) = ft_values_.force.x;
  base_wrench(1) = ft_values_.force.y;
  base_wrench(2) = ft_values_.force.z;
  base_wrench(3) = ft_values_.torque.x;
  base_wrench(4) = ft_values_.torque.y;
  base_wrench(5) = ft_values_.torque.z;

  
  // get required transforms and Jacobian
  bool success = true;
  // get current_pose
  Eigen::Isometry3d current_pose;
  success &= fk_solver_->calculate_link_transform(
    joint_cur_pos, 
    base_controller_parameters_.kinematics.robot_base, 
    base_controller_parameters_.kinematics.robot_end_effector, 
    current_pose
  );
  // get control frame
  Eigen::Isometry3d control_frame;
  success &= fk_solver_->calculate_link_transform(
    joint_cur_pos, 
    base_controller_parameters_.kinematics.robot_base, 
    impedance_twist_controller_parameters_.impedance.control.frame, 
    control_frame
  );
  // get jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> J;
  success &= ik_solver_->calculate_jacobian(joint_cur_pos, base_controller_parameters_.kinematics.robot_end_effector, J);

  if (!success)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to calculate required transforms in update_and_write_commands");
    BaseController::write_state_to_hardware(joint_command_);
    return controller_interface::return_type::ERROR;
  }

  // get target pose and nullspace joint positions from reference
  Eigen::Matrix<double, 6, 1> target_twist;
  if (using_twist_reference_) {
    // get twist msg
    target_twist[0] = twist_reference_.twist.linear.x;
    target_twist[1] = twist_reference_.twist.linear.y;
    target_twist[2] = twist_reference_.twist.linear.z;
    target_twist[3] = twist_reference_.twist.angular.x;
    target_twist[4] = twist_reference_.twist.angular.y;
    target_twist[5] = twist_reference_.twist.angular.z;
  } 
  else if (using_joint_reference_) {
    // get reference twist
    target_twist = J * joint_ref_vel; 
  }
  else {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid reference provided");
    BaseController::write_state_to_hardware(joint_command_);
    return controller_interface::return_type::OK;
  }

  // get current twist
  Eigen::Matrix<double, 6, 1> current_twist = J * joint_des_vel; // TODO: use joint_des_vel_ or joint_cur_vel_?

  // get stiffness and damping matrices
  Eigen::Matrix<double, 6, 6> damping_matrix;
  create_gain_matrix(damping_, control_frame, damping_matrix);

  // TODO transform twist to control frame

  // zero out any forces in the control frame
  Eigen::Matrix<double, 6, 1> control_wrench;
  control_wrench.block<3, 1>(0, 0) = control_frame.rotation().transpose() * base_wrench.block<3, 1>(0, 0);
  control_wrench.block<3, 1>(3, 0) = control_frame.rotation().transpose() * base_wrench.block<3, 1>(3, 0);
  control_wrench = control_wrench.cwiseProduct(selected_axes_);
  base_wrench.block<3, 1>(0, 0) = control_frame.rotation() * control_wrench.block<3, 1>(0, 0);
  base_wrench.block<3, 1>(3, 0) = control_frame.rotation() * control_wrench.block<3, 1>(3, 0);

  // calculate impedance control law in base frame
  Eigen::Matrix<double, 6, 1> net_force = base_wrench - damping_matrix * (current_twist - target_twist);

  // apply error scale in the control frame
  net_force.block<3, 1>(0, 0) = control_frame.rotation().transpose() * net_force.block<3, 1>(0, 0);
  net_force.block<3, 1>(3, 0) = control_frame.rotation().transpose() * net_force.block<3, 1>(3, 0);
  net_force = net_force.cwiseProduct(error_scale_);
  net_force.block<3, 1>(0, 0) = control_frame.rotation() * net_force.block<3, 1>(0, 0);
  net_force.block<3, 1>(3, 0) = control_frame.rotation() * net_force.block<3, 1>(3, 0);


  

  // forward dynamics solver
  ik_solver_->forwardDynamics(
    joint_cur_pos, 
    net_force, 
    base_controller_parameters_.kinematics.robot_end_effector, 
    joint_des_acc
  );

  // nullspace stiffness and damping
  joint_des_acc += (Eigen::MatrixXd::Identity(num_joints_,num_joints_) - (J.completeOrthogonalDecomposition().pseudoInverse() * J)) *
                    (
                      nullspace_stiffness_.asDiagonal() * (nullspace_joint_pos_ - joint_cur_pos) -
                      nullspace_damping_.asDiagonal() * joint_des_vel
                    );

  // more joint damping because why not
  joint_des_acc -= impedance_twist_controller_parameters_.impedance.joint_damping * joint_des_vel;

  // integrate
  joint_des_vel += period.seconds() * joint_des_acc;
  joint_des_vel  *= 0.9;  // 10 % global damping against unwanted null space motion.
  joint_des_pos += period.seconds() * joint_des_vel;

  // Numerical time integration with the Euler forward method
  // joint_des_pos += joint_des_vel * period.seconds();
  // joint_des_vel += joint_des_acc * period.seconds();
  // joint_des_vel  *= 0.9;  // 10 % global damping against unwanted null space motion.

  // write commands
  for (size_t i = 0; i < num_joints_; ++i)
  {
    joint_command_.accelerations[i] = joint_des_acc[i];
    joint_command_.velocities[i] = joint_des_vel[i];
    joint_command_.positions[i] = joint_des_pos[i];
  }

  BaseController::write_state_to_hardware(joint_command_);
  return controller_interface::return_type::OK;
}

} // namespace manipulator_controllers


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  manipulator_controllers::ImpedanceTwistController, controller_interface::ChainableControllerInterface)
