#include "manipulator_controllers/impedance_pose_controller.hpp"

namespace manipulator_controllers
{


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
      get_node()->get_logger(), "THIS IS THE IMPEDANCE POSE CONTROLLER FAILING: %s \n", e.what());
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


  control_frame_id_ = impedance_pose_controller_parameters_.control.frame.id;
  lms_damping_ = impedance_pose_controller_parameters_.lms_damping;
  joint_damping_ = impedance_pose_controller_parameters_.joint_damping;
  vec_to_eigen(impedance_pose_controller_parameters_.stiffness, stiffness_);
  vec_to_eigen(impedance_pose_controller_parameters_.damping_ratio, damping_ratio_);
  vec_to_eigen(impedance_pose_controller_parameters_.selected_axes, selected_axes_);
  vec_to_eigen(impedance_pose_controller_parameters_.mass, mass_);
  joint_ref_pos_ = Eigen::VectorXd::Zero(num_joints_);
  joint_cur_pos_ = Eigen::VectorXd::Zero(num_joints_);
  joint_cur_vel_ = Eigen::VectorXd::Zero(num_joints_);
  joint_des_pos_ = Eigen::VectorXd::Zero(num_joints_);
  joint_des_vel_ = Eigen::VectorXd::Zero(num_joints_);
  joint_des_acc_ = Eigen::VectorXd::Zero(num_joints_);
  I_ = Eigen::MatrixXd(num_joints_, num_joints_);
  I_.setIdentity();

  for (size_t i = 0; i < 6; ++i)
  {
    mass_inv_[i] = 1.0 / mass_[i];
    damping_[i] = damping_ratio_[i] * 2 * sqrt(mass_[i] * stiffness_[i]);
  }
  

  // nullspace joint position check that size is correct
  if (impedance_pose_controller_parameters_.nullspace_joint_positions.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_joint_position parameter does not match number of joints");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (impedance_pose_controller_parameters_.nullspace_stiffness.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_stiffness parameter does not match number of joints");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (impedance_pose_controller_parameters_.nullspace_damping_ratio.size() != num_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of nullspace_damping_ratio parameter does not match number of joints");
    return controller_interface::CallbackReturn::ERROR;
  }
  nullspace_joint_pos_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_stiffness_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_damping_ratio_ = Eigen::VectorXd::Zero(num_joints_);
  nullspace_damping_ = Eigen::VectorXd::Zero(num_joints_);
  vec_to_eigen(impedance_pose_controller_parameters_.nullspace_joint_positions, nullspace_joint_pos_);
  vec_to_eigen(impedance_pose_controller_parameters_.nullspace_stiffness, nullspace_stiffness_);
  vec_to_eigen(impedance_pose_controller_parameters_.nullspace_damping_ratio, nullspace_damping_ratio_);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    nullspace_damping_[i] = nullspace_damping_ratio_[i] * 2 * sqrt(nullspace_stiffness_[i]);
  }

  // vec_to_eigen(pose_controller_parameters_.selected_axes, selected_axes);

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type ImpedancePoseController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  // update input reference from chainable interfaces
  read_state_reference_interfaces(joint_reference_);
  // get all controller inputs
  BaseController::read_state_from_hardware(joint_state_);
  BaseForceController::read_state_from_hardware(ft_values_);

  // get reference joint state
  vec_to_eigen(joint_reference_.positions, joint_ref_pos_);
  // get current joint state
  vec_to_eigen(joint_state_.positions, joint_cur_pos_);
  vec_to_eigen(joint_state_.velocities, joint_cur_vel_);
  // get command joint state
  vec_to_eigen(joint_command_.positions, joint_des_pos_);
  vec_to_eigen(joint_command_.velocities, joint_des_vel_);
  vec_to_eigen(joint_command_.accelerations, joint_des_acc_);

  // process wrench measurements
  process_wrench_measurements(joint_cur_pos_, ft_values_);

  // get target pose
  Eigen::Isometry3d target_pose;
  if (!using_joint_reference_interface_) {
    tf2::fromMsg(pose_reference_.pose, target_pose);

    vec_to_eigen(impedance_pose_controller_parameters_.nullspace_joint_positions, nullspace_joint_pos_);
  } 
  else {
    ik_solver_->calculate_link_transform(
      joint_ref_pos_, 
      base_controller_parameters_.kinematics.base_link, 
      base_controller_parameters_.kinematics.end_effector_link, 
      target_pose);

    nullspace_joint_pos_ = joint_ref_pos_;
  }


  // get current pose
  Eigen::Isometry3d current_pose;
  ik_solver_->calculate_link_transform(
    joint_cur_pos_, 
    base_controller_parameters_.kinematics.base_link, 
    base_controller_parameters_.kinematics.end_effector_link, 
    current_pose
  );

  // get jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> J;
  ik_solver_->calculate_jacobian(joint_cur_pos_, base_controller_parameters_.kinematics.end_effector_link, J);



  // get jacobian pseudo inverse
  Eigen::MatrixXd J_pinv;
  pseudo_inverse(J.transpose(), J_pinv);




  // get current twist
  Eigen::Matrix<double, 6, 1> current_twist = J * joint_des_vel_; // TODO: use joint_des_vel_ or joint_cur_vel_?

  // get control frame
  Eigen::Isometry3d control_frame;
  ik_solver_->calculate_link_transform(
    joint_cur_pos_, 
    base_controller_parameters_.kinematics.base_link, 
    impedance_pose_controller_parameters_.control.frame.id, 
    control_frame
  );

  // create stiffness and damping matrices
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
  Eigen::Matrix<double, 6, 1> base_wrench = base_wrench_;
  control_wrench.block<3, 1>(0, 0) = control_frame.rotation().transpose() * base_wrench.block<3, 1>(0, 0);
  control_wrench.block<3, 1>(3, 0) = control_frame.rotation().transpose() * base_wrench.block<3, 1>(3, 0);
  control_wrench = control_wrench.cwiseProduct(selected_axes_);
  base_wrench.block<3, 1>(0, 0) = control_frame.rotation() * control_wrench.block<3, 1>(0, 0);
  base_wrench.block<3, 1>(3, 0) = control_frame.rotation() * control_wrench.block<3, 1>(3, 0);

  // Compute admittance control law in the base frame: F = M*x_ddot + D*x_dot + K*x
  Eigen::Matrix<double, 6, 1> task_des_acc = 
    mass_inv_.cwiseProduct(base_wrench_ - damping_matrix * current_twist - stiffness_matrix * pose_error);

  // damped inverse
  Eigen::Matrix<double, Eigen::Dynamic, 6> J_inverse =
     (J.transpose() * J + lms_damping_ * I_).inverse() * J.transpose();
  joint_des_acc_ = J_inverse * task_des_acc;


  joint_des_acc_ += (I_ - J.transpose() * J_pinv) *
                    (
                      nullspace_stiffness_.asDiagonal() * (nullspace_joint_pos_ - joint_cur_pos_) -
                      nullspace_damping_.asDiagonal() * joint_des_vel_
                    );



  // add joint damping 
  joint_des_acc_ -= joint_damping_ * joint_des_vel_;  // TODO: use joint_des_vel_ or joint_cur_vel_?

  // integrate
  joint_des_vel_ += period.seconds() * joint_des_acc_;
  joint_des_pos_ += period.seconds() * joint_des_vel_;

  // write commands
  for (size_t i = 0; i < num_joints_; ++i)
  {
    joint_command_.accelerations[i] = joint_des_acc_[i];
    joint_command_.velocities[i] = joint_des_vel_[i];
    joint_command_.positions[i] = joint_des_pos_[i];
  }

  BaseController::write_state_to_hardware(joint_command_);
  return controller_interface::return_type::OK;
}

} // namespace manipulator_controllers


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  manipulator_controllers::ImpedancePoseController, controller_interface::ChainableControllerInterface)
