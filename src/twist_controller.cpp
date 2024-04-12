#include "manipulator_controllers/twist_controller.hpp"

namespace manipulator_controllers
{


bool TwistController::set_params() 
{
  vec_to_eigen(twist_controller_parameters_.diff_ik.control.kd, kd_);
 
  // nullspace joint position check that size is correct
  if (twist_controller_parameters_.diff_ik.nullspace.joint_positions.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_joint_position parameter does not match number of joints");
    return false;
  }
  if (twist_controller_parameters_.diff_ik.nullspace.kp.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_kp parameter does not match number of joints");
    return false;
  }
  if (twist_controller_parameters_.diff_ik.nullspace.kd_ratio.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_kd_ratio parameter does not match number of joints");
    return false;
  }
  nullspace_joint_pos_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_kp_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_kd_ratio_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_kd_ = Eigen::VectorXd::Zero(num_joints_);
  vec_to_eigen(twist_controller_parameters_.diff_ik.nullspace.joint_positions, nullspace_joint_pos_);
  vec_to_eigen(twist_controller_parameters_.diff_ik.nullspace.kp, nullspace_kp_);
  vec_to_eigen(twist_controller_parameters_.diff_ik.nullspace.kd_ratio, nullspace_kd_ratio_);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    nullspace_kd_[i] = nullspace_kd_ratio_[i] * 2 * sqrt(nullspace_kp_[i]);
  }

  return true;
}

controller_interface::CallbackReturn TwistController::on_init()
{
  auto ret = BaseController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // initialize controller config
  try
  {
    twist_controller_parameter_handler_ = std::make_shared<twist_controller::ParamListener>(get_node());
    twist_controller_parameters_ = twist_controller_parameter_handler_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = BaseController::on_configure(previous_state);
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

controller_interface::CallbackReturn TwistController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = BaseController::on_activate(previous_state);
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


controller_interface::return_type TwistController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // update input reference from chainable interfaces
  read_state_reference_interfaces(joint_reference_);
  // get all controller inputs
  BaseController::read_state_from_hardware(joint_state_);

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
    twist_controller_parameters_.diff_ik.control.frame, 
    control_frame
  );

  if (!success)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to calculate required transforms in update_and_write_commands");
    BaseController::write_state_to_hardware(joint_command_);
    return controller_interface::return_type::ERROR;
  }

  // get jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> J;
  success &= ik_solver_->calculate_jacobian(joint_cur_pos, base_controller_parameters_.kinematics.robot_end_effector, J);

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

  // get identity
  auto I = Eigen::MatrixXd::Identity(num_joints_,num_joints_);

  // get jacobian pseudo inverse
  Eigen::MatrixXd J_pinv;
  pseudo_inverse(J, J_pinv);

  // get current twist
  Eigen::Matrix<double, 6, 1> current_twist = J * joint_des_vel; // TODO: use joint_des_vel_ or joint_cur_vel_?

  // get kd matrices
  Eigen::Matrix<double, 6, 6> kd_matrix;
  create_gain_matrix(kd_, control_frame, kd_matrix);

  // calculate control law in base frame
  Eigen::Matrix<double, 6, 1> net_error = kd_matrix * (target_twist - current_twist);

  RCLCPP_INFO(get_node()->get_logger(), "Target Twist: %f %f %f %f %f %f", 
    target_twist[0], target_twist[1], target_twist[2], target_twist[3], target_twist[4], target_twist[5]);

  // forward dynamics solver
  ik_solver_->forwardDynamics(
    joint_cur_pos, 
    net_error, 
    base_controller_parameters_.kinematics.robot_end_effector, 
    joint_des_acc
  );

  // nullspace kp and kd
  joint_des_acc += (I - (J.completeOrthogonalDecomposition().pseudoInverse() * J)) *
                    (
                      nullspace_kp_.asDiagonal() * (nullspace_joint_pos_ - joint_cur_pos) -
                      nullspace_kd_.asDiagonal() * joint_des_vel
                    );

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
  manipulator_controllers::TwistController, controller_interface::ChainableControllerInterface)
