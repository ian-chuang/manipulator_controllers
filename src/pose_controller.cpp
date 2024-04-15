#include "manipulator_controllers/pose_controller.hpp"

namespace manipulator_controllers
{


bool PoseController::set_params() 
{
  vec_to_eigen(pose_controller_parameters_.diff_ik.control.kp, kp_);
  vec_to_eigen(pose_controller_parameters_.diff_ik.control.kd_ratio, kd_ratio_);
  vec_to_eigen(pose_controller_parameters_.diff_ik.control.max_twist, max_twist_);
  for (size_t i = 0; i < 6; ++i)
  {
    kd_[i] = kd_ratio_[i] * 2 * sqrt(kp_[i]);
  }

  // nullspace joint position check that size is correct
  if (pose_controller_parameters_.diff_ik.nullspace.joint_positions.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_joint_position parameter does not match number of joints");
    return false;
  }
  if (pose_controller_parameters_.diff_ik.nullspace.kp.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_kp parameter does not match number of joints");
    return false;
  }
  if (pose_controller_parameters_.diff_ik.nullspace.kd_ratio.size() != num_joints_)
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
  vec_to_eigen(pose_controller_parameters_.diff_ik.nullspace.joint_positions, nullspace_joint_pos_);
  vec_to_eigen(pose_controller_parameters_.diff_ik.nullspace.kp, nullspace_kp_);
  vec_to_eigen(pose_controller_parameters_.diff_ik.nullspace.kd_ratio, nullspace_kd_ratio_);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    nullspace_kd_[i] = nullspace_kd_ratio_[i] * 2 * sqrt(nullspace_kp_[i]);
  }

  return true;
}

controller_interface::CallbackReturn PoseController::on_init()
{
  auto ret = BaseController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // initialize controller config
  try
  {
    pose_controller_parameter_handler_ = std::make_shared<pose_controller::ParamListener>(get_node());
    pose_controller_parameters_ = pose_controller_parameter_handler_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseController::on_configure(
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

controller_interface::CallbackReturn PoseController::on_activate(
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


controller_interface::return_type PoseController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // update input reference from chainable interfaces
  read_state_reference_interfaces(joint_reference_);
  // get all controller inputs
  BaseController::read_state_from_hardware(joint_state_);

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
    pose_controller_parameters_.diff_ik.control.frame, 
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

  // get current twist
  Eigen::Matrix<double, 6, 1> current_twist = J * joint_des_vel; // TODO: use joint_des_vel_ or joint_cur_vel_?

  // get kp and kd matrices
  Eigen::Matrix<double, 6, 6> kp_matrix;
  create_gain_matrix(kp_, control_frame, kp_matrix);
  Eigen::Matrix<double, 6, 6> kd_matrix;
  create_gain_matrix(kd_, control_frame, kd_matrix);

  // calculate spatial error
  Eigen::Matrix<double, 6, 1> pose_error;
  pose_error.block<3, 1>(0, 0) =
    current_pose.translation() - target_pose.translation();
  auto R = current_pose.rotation() * target_pose.rotation().transpose();
  auto angle_axis = Eigen::AngleAxisd(R);
  pose_error.block<3, 1>(3, 0) = angle_axis.angle() * angle_axis.axis();

  // create velocity scaling in control frame
  Eigen::Matrix<double, 6, 1> control_twist;
  Eigen::Matrix<double, 6, 1> scaling_factor;
  control_twist.block<3, 1>(0, 0) = control_frame.rotation().transpose() * current_twist.block<3, 1>(0, 0);
  control_twist.block<3, 1>(3, 0) = control_frame.rotation().transpose() * current_twist.block<3, 1>(3, 0);
  for (size_t i = 0; i < 6; ++i)
  {
    if (std::abs(control_twist[i]) > std::abs(max_twist_[i]))
    {
      scaling_factor[i] = std::abs(max_twist_[i]) / std::abs(control_twist[i]);
    }   
    else
    {
      scaling_factor[i] = 1.0;
    }   
  }
  Eigen::Matrix<double, 6, 6> scaling_factor_matrix;
  create_gain_matrix(scaling_factor, control_frame, scaling_factor_matrix);

  // calculate  control law in base frame
  Eigen::Matrix<double, 6, 1> net_force = scaling_factor_matrix * (- kd_matrix * current_twist - kp_matrix * pose_error);

  // forward dynamics solver
  ik_solver_->forwardDynamics(
    joint_cur_pos, 
    net_force, 
    base_controller_parameters_.kinematics.robot_end_effector, 
    joint_des_acc
  );

  // nullspace kp and kd
  joint_des_acc += (Eigen::MatrixXd::Identity(num_joints_,num_joints_) - (J.completeOrthogonalDecomposition().pseudoInverse() * J)) *
                    (
                      nullspace_kp_.asDiagonal() * (nullspace_joint_pos - joint_cur_pos) -
                      nullspace_kd_.asDiagonal() * joint_des_vel
                    );

  // add joint damping if current twist is small (both translational and rotational)
  // must be below both linear threshold and angular threshold
  if (current_twist.block<3, 1>(0, 0).norm() < pose_controller_parameters_.diff_ik.joint_damping.lin_vel_threshold &&
      current_twist.block<3, 1>(3, 0).norm() < pose_controller_parameters_.diff_ik.joint_damping.ang_vel_threshold)
  {
    joint_des_acc -= pose_controller_parameters_.diff_ik.joint_damping.damping * joint_des_vel;
  }

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
  manipulator_controllers::PoseController, controller_interface::ChainableControllerInterface)
