#include "manipulator_controllers/impedance_pose_controller.hpp"

namespace manipulator_controllers
{

bool ImpedancePoseController::set_params() 
{
  vec_to_eigen(impedance_pose_controller_parameters_.impedance.control.stiffness, stiffness_);
  vec_to_eigen(impedance_pose_controller_parameters_.impedance.control.damping_ratio, damping_ratio_);
  vec_to_eigen(impedance_pose_controller_parameters_.impedance.control.selected_axes, selected_axes_);
  vec_to_eigen(impedance_pose_controller_parameters_.impedance.control.error_scale, error_scale_);
  vec_to_eigen(impedance_pose_controller_parameters_.impedance.control.max_spring_force, max_spring_force_);
  for (size_t i = 0; i < 6; ++i)
  {
    // mass_inv_[i] = 1.0 / mass_[i];
    // damping_[i] = damping_ratio_[i] * 2 * sqrt(mass_[i] * stiffness_[i]);

    damping_[i] = damping_ratio_[i] * 2 * sqrt(stiffness_[i]);
  }

  // nullspace joint position check that size is correct
  if (impedance_pose_controller_parameters_.impedance.nullspace.joint_positions.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_joint_position parameter does not match number of joints");
    return false;
  }
  if (impedance_pose_controller_parameters_.impedance.nullspace.stiffness.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_stiffness parameter does not match number of joints");
    return false;
  }
  if (impedance_pose_controller_parameters_.impedance.nullspace.damping_ratio.size() != num_joints_)
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
  vec_to_eigen(impedance_pose_controller_parameters_.impedance.nullspace.joint_positions, nullspace_joint_pos_);
  vec_to_eigen(impedance_pose_controller_parameters_.impedance.nullspace.stiffness, nullspace_stiffness_);
  vec_to_eigen(impedance_pose_controller_parameters_.impedance.nullspace.damping_ratio, nullspace_damping_ratio_);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    nullspace_damping_[i] = nullspace_damping_ratio_[i] * 2 * sqrt(nullspace_stiffness_[i]);
  }

  return true;
}

controller_interface::CallbackReturn ImpedancePoseController::on_init()
{
  auto ret = BaseForceController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // initialize controller config
  try
  {
    impedance_pose_controller_parameter_handler_ = std::make_shared<impedance_pose_controller::ParamListener>(get_node());
    impedance_pose_controller_parameters_ = impedance_pose_controller_parameter_handler_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedancePoseController::on_configure(
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

controller_interface::CallbackReturn ImpedancePoseController::on_activate(
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


controller_interface::return_type ImpedancePoseController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // update input reference from chainable interfaces
  read_state_reference_interfaces(joint_reference_);
  // get all controller inputs
  BaseController::read_state_from_hardware(joint_state_);
  BaseForceController::read_state_from_hardware(joint_state_, ft_values_);

  // get required joint vals (as Eigen)
  Eigen::VectorXd joint_ref_pos = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_cur_pos = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_cur_vel = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_des_pos = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_des_vel = Eigen::VectorXd::Zero(num_joints_);
  Eigen::VectorXd joint_des_acc = Eigen::VectorXd::Zero(num_joints_);
  vec_to_eigen(joint_reference_.positions, joint_ref_pos);
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

  // get target pose and nullspace joint positions from reference
  Eigen::Isometry3d target_pose;
  Eigen::VectorXd nullspace_joint_pos;
  if (using_pose_reference_) {
    // get pose msg
    tf2::fromMsg(pose_reference_.pose, target_pose);
    // use nullspace joints from config
    nullspace_joint_pos = nullspace_joint_pos_;
  } 
  else if (using_joint_reference_) {
    // get pose from joint reference
    fk_solver_->calculate_link_transform(
      joint_ref_pos, 
      base_controller_parameters_.kinematics.robot_base, 
      base_controller_parameters_.kinematics.robot_end_effector, 
      target_pose);
    // use nullspace joints from reference
    nullspace_joint_pos = joint_ref_pos;
  }
  else {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid reference provided");
    BaseController::write_state_to_hardware(joint_command_);
    return controller_interface::return_type::OK;
  }

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
    impedance_pose_controller_parameters_.impedance.control.frame, 
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

  // get identity
  auto I = Eigen::MatrixXd::Identity(num_joints_,num_joints_);

  // get jacobian pseudo inverse
  Eigen::MatrixXd J_pinv;
  pseudo_inverse(J.transpose(), J_pinv);

  // get current twist
  Eigen::Matrix<double, 6, 1> current_twist = J * joint_des_vel; // TODO: use joint_des_vel_ or joint_cur_vel_?

  // get stiffness and damping matrices
  Eigen::Matrix<double, 6, 6> stiffness_matrix;
  create_gain_matrix(stiffness_, control_frame, stiffness_matrix);
  Eigen::Matrix<double, 6, 6> damping_matrix;
  create_gain_matrix(damping_, control_frame, damping_matrix);

  // calculate spatial error
  Eigen::Matrix<double, 6, 1> pose_error;
  pose_error.block<3, 1>(0, 0) =
    current_pose.translation() - target_pose.translation();
  auto R = current_pose.rotation() * target_pose.rotation().transpose();
  auto angle_axis = Eigen::AngleAxisd(R);
  pose_error.block<3, 1>(3, 0) = angle_axis.angle() * angle_axis.axis();

  // zero out any forces in the control frame
  Eigen::Matrix<double, 6, 1> control_wrench;
  control_wrench.block<3, 1>(0, 0) = control_frame.rotation().transpose() * base_wrench.block<3, 1>(0, 0);
  control_wrench.block<3, 1>(3, 0) = control_frame.rotation().transpose() * base_wrench.block<3, 1>(3, 0);
  control_wrench = control_wrench.cwiseProduct(selected_axes_);
  base_wrench.block<3, 1>(0, 0) = control_frame.rotation() * control_wrench.block<3, 1>(0, 0);
  base_wrench.block<3, 1>(3, 0) = control_frame.rotation() * control_wrench.block<3, 1>(3, 0);

  // apply max spring force in the control frame
  Eigen::Matrix<double, 6, 1> spring_force = stiffness_matrix * pose_error;
  spring_force.block<3, 1>(0, 0) = control_frame.rotation().transpose() * spring_force.block<3, 1>(0, 0);
  spring_force.block<3, 1>(3, 0) = control_frame.rotation().transpose() * spring_force.block<3, 1>(3, 0);
  spring_force = clip(spring_force, -max_spring_force_, max_spring_force_);
  spring_force.block<3, 1>(0, 0) = control_frame.rotation() * spring_force.block<3, 1>(0, 0);
  spring_force.block<3, 1>(3, 0) = control_frame.rotation() * spring_force.block<3, 1>(3, 0);

  // calculate impedance control law in base frame
  Eigen::Matrix<double, 6, 1> net_force = base_wrench - damping_matrix * current_twist - spring_force;

  // apply error scale in the control frame
  net_force.block<3, 1>(0, 0) = control_frame.rotation().transpose() * net_force.block<3, 1>(0, 0);
  net_force.block<3, 1>(3, 0) = control_frame.rotation().transpose() * net_force.block<3, 1>(3, 0);
  net_force = net_force.cwiseProduct(error_scale_);
  net_force.block<3, 1>(0, 0) = control_frame.rotation() * net_force.block<3, 1>(0, 0);
  net_force.block<3, 1>(3, 0) = control_frame.rotation() * net_force.block<3, 1>(3, 0);

  
  //damped inverse
  //Compute admittance control law in the base frame: F = M*x_ddot + D*x_dot + K*x
  // Eigen::Matrix<double, 6, 1> task_des_acc = 
  //   mass_inv_.cwiseProduct( - damping_matrix * current_twist - clip(stiffness_matrix * pose_error, -max_spring_force_, max_spring_force_));
  // Eigen::Matrix<double, Eigen::Dynamic, 6> J_inverse =
  //    (J.transpose() * J + impedance_pose_controller_parameters_.impedance.pinv_damping * I).inverse() * J.transpose();
  // joint_des_acc = J_inverse * task_des_acc;


  

  // forward dynamics solver
  ik_solver_->task_force_to_joint_acc(
    joint_cur_pos, 
    net_force, 
    base_controller_parameters_.kinematics.robot_end_effector, 
    joint_des_acc
  );

  // nullspace stiffness and damping
  joint_des_acc += (I - J.transpose() * J_pinv) *
                    (
                      nullspace_stiffness_.asDiagonal() * (nullspace_joint_pos - joint_cur_pos) -
                      nullspace_damping_.asDiagonal() * joint_des_vel
                    );

  // more joint damping because why not
  joint_des_acc -= impedance_pose_controller_parameters_.impedance.joint_damping * joint_des_vel;

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
  manipulator_controllers::ImpedancePoseController, controller_interface::ChainableControllerInterface)
