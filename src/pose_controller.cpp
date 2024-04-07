#include "manipulator_controllers/pose_controller.hpp"

namespace manipulator_controllers
{

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
      get_node()->get_logger(), "THIS IS THE POSE CONTROLLER FAILING: %s \n", e.what());
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

  RCLCPP_INFO(get_node()->get_logger(), "BASE CONTROLLER Configured");

  k = Eigen::VectorXd::Zero(num_joints_);
  d = Eigen::VectorXd::Zero(num_joints_);

  vec_to_eigen(pose_controller_parameters_.k, k);
  vec_to_eigen(pose_controller_parameters_.d, d);

  RCLCPP_INFO(get_node()->get_logger(), "POSE CONTROLLER Configured");
  // vec_to_eigen(pose_controller_parameters_.selected_axes, selected_axes);

  return controller_interface::CallbackReturn::SUCCESS;
}

template <typename T1, typename T2>
void PoseController::vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
{
  for (auto col = 0; col < matrix.cols(); col++)
  {
    for (auto row = 0; row < matrix.rows(); row++)
    {
      matrix(row, col) = data[row + col * matrix.rows()];
    }
  }
}

controller_interface::return_type PoseController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // update input reference from chainable interfaces
  read_state_reference_interfaces(joint_reference_);
  // get all controller inputs
  read_state_from_hardware(joint_state_);

  // get target pose
  Eigen::Isometry3d target_pose;
  if (pose_command_msg_.get()) {
    tf2::fromMsg(pose_command_msg_->pose, target_pose);
  } 
  else {
    Eigen::VectorXd joint_reference_positions;
    joint_reference_positions = Eigen::VectorXd::Zero(num_joints_);
    vec_to_eigen(joint_state_.positions, joint_reference_positions);
    ik_solver_->calculate_link_transform(
      joint_reference_positions, base_controller_parameters_.kinematics.tip, target_pose);
  }

  // get current joint position
  Eigen::VectorXd joint_state_positions;
  joint_state_positions = Eigen::VectorXd::Zero(num_joints_);
  vec_to_eigen(joint_state_.positions, joint_state_positions);

  // get current joint velocity
  Eigen::VectorXd joint_state_velocities;
  joint_state_velocities = Eigen::VectorXd::Zero(num_joints_);
  vec_to_eigen(joint_state_.velocities, joint_state_velocities);

  // get current pose
  Eigen::Isometry3d current_pose;
  ik_solver_->calculate_link_transform(
    joint_state_positions, base_controller_parameters_.kinematics.tip, current_pose);

  // get current twist
  Eigen::Matrix<double, 6, 1> X_dot;
  ik_solver_->calculate_link_twist(
    joint_state_positions, joint_state_velocities, base_controller_parameters_.kinematics.tip,
    X_dot);

  // get control frame
  Eigen::Isometry3d control_frame;
  ik_solver_->calculate_link_transform(
      joint_state_positions, pose_controller_parameters_.control.frame.id, control_frame);

  // Create stiffness matrix in base frame. The user-provided values of admittance_state.stiffness
  // correspond to the six diagonal elements of the stiffness matrix expressed in the control frame
  auto rot_base_control = control_frame.rotation();
  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 3, 3> K_pos = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> K_rot = Eigen::Matrix<double, 3, 3>::Zero();
  K_pos.diagonal() = k.block<3, 1>(0, 0);
  K_rot.diagonal() = k.block<3, 1>(3, 0);
  // Transform to the control frame
  // A reference is here:  https://users.wpi.edu/~jfu2/rbe502/files/force_control.pdf
  // Force Control by Luigi Villani and Joris De Schutter
  // Page 200
  K_pos = rot_base_control * K_pos * rot_base_control.transpose();
  K_rot = rot_base_control * K_rot * rot_base_control.transpose();
  K.block<3, 3>(0, 0) = K_pos;
  K.block<3, 3>(3, 3) = K_rot;

  // The same for damping
  Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 3, 3> D_pos = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> D_rot = Eigen::Matrix<double, 3, 3>::Zero();
  D_pos.diagonal() = d.block<3, 1>(0, 0);
  D_rot.diagonal() = d.block<3, 1>(3, 0);
  D_pos = rot_base_control * D_pos * rot_base_control.transpose();
  D_rot = rot_base_control * D_rot * rot_base_control.transpose();
  D.block<3, 3>(0, 0) = D_pos;
  D.block<3, 3>(3, 3) = D_rot;

  
  // calculate spatial error
  Eigen::Matrix<double, 6, 1> X;
  X.block<3, 1>(0, 0) =
    target_pose.translation() - current_pose.translation();
  auto R = target_pose.rotation() * current_pose.rotation().transpose();
  auto angle_axis = Eigen::AngleAxisd(R);
  X.block<3, 1>(3, 0) = angle_axis.angle() * angle_axis.axis();

  // equation of motion
  Eigen::Matrix<double, 6, 1> X_ddot = D * X_dot + K * X;

  // calculate joint velocity
  Eigen::VectorXd joint_deltas;
  joint_deltas = Eigen::VectorXd::Zero(num_joints_);
  bool success = ik_solver_->convert_cartesian_deltas_to_joint_deltas(
    joint_state_positions, X_ddot, base_controller_parameters_.kinematics.tip,
    joint_deltas);

  // apply joint velocity limits

  // write commands
  for (size_t i = 0; i < num_joints_; ++i)
  {
    joint_command_.velocities[i] = joint_deltas[i];
    joint_command_.positions[i] = joint_state_positions[i] + period.seconds() * joint_command_.velocities[i];
  }

  // print joint command positions
  std::string joint_command_positions = "";
  for (size_t i = 0; i < num_joints_; ++i)
  {
    joint_command_positions += std::to_string(joint_deltas[i]) + " ";
  }
  RCLCPP_INFO(get_node()->get_logger(), "Joint Command Positions: %s", joint_command_positions.c_str());

  write_state_to_hardware(joint_command_);
  return controller_interface::return_type::OK;
}

} // namespace manipulator_controllers


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  manipulator_controllers::PoseController, controller_interface::ChainableControllerInterface)
