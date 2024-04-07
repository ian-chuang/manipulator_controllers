#include "manipulator_controllers/ik_solver.hpp"

namespace manipulator_controllers
{

bool IKSolver::initialize(
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & base_link,
  const std::string & ee_link,
  double alpha
) {
  initialized_ = true;
  base_link_ = base_link;
  ee_link_ = ee_link;
  parameters_interface_ = parameters_interface;
  alpha_ = alpha;

  // get robot description
  auto robot_param = rclcpp::Parameter();
  if (!parameters_interface->get_parameter("robot_description", robot_param))
  {
      RCLCPP_ERROR(rclcpp::get_logger("IKSolver"), "parameter robot_description not set");
      return false;
  }
  auto robot_description = robot_param.as_string();

  // create kinematic chain
  KDL::Tree robot_tree;
  kdl_parser::treeFromString(robot_description, robot_tree);
  if (!robot_tree.getChain(base_link_, ee_link_, chain_))
  {
      RCLCPP_ERROR(
      rclcpp::get_logger("IKSolver"), "failed to find chain from robot base %s to end effector %s", base_link_.c_str(),
      ee_link_.c_str());
      return false;
  }

  // create map from link names to their index
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
  {
    link_name_map_[chain_.getSegment(i).getName()] = i + 1;
  }

  // allocate dynamics memory
  num_joints_ = chain_.getNrOfJoints();
  q_ = KDL::JntArray(num_joints_);
  q_dot_ = KDL::JntArray(num_joints_);
  I_ = Eigen::MatrixXd(num_joints_, num_joints_);
  I_.setIdentity();
  // create KDL solvers
  fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  fk_vel_solver_ = std::make_shared<KDL::ChainFkSolverVel_recursive>(chain_);
  jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
  jacobian_ = std::make_shared<KDL::Jacobian>(num_joints_);

  return true;
}

bool IKSolver::convert_cartesian_deltas_to_joint_deltas(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos,
  const Eigen::Matrix<double, 6, 1> & delta_x, const std::string & link_name,
  Eigen::Matrix<double, Eigen::Dynamic, 1> & delta_theta)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_joint_vector(delta_theta))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  // TODO(anyone): this dynamic allocation needs to be replaced
  Eigen::Matrix<double, 6, Eigen::Dynamic> J = jacobian_->data;
  // damped inverse
  Eigen::Matrix<double, Eigen::Dynamic, 6> J_inverse =
    (J.transpose() * J + alpha_ * I_).inverse() * J.transpose();
  delta_theta = J_inverse * delta_x;

  return true;
}

bool IKSolver::calculate_link_transform(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Isometry3d & pose)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // reset transform_vec
  pose.setIdentity();

  // special case: since the root is not in the robot tree, need to return identity transform
  if (link_name == base_link_)
  {
    return true;
  }

  // create forward kinematics solver
  fk_pos_solver_->JntToCart(q_, frame_, link_name_map_[link_name]);
  tf2::transformKDLToEigen(frame_, pose);
  return true;
}

bool IKSolver::calculate_link_twist(
    const Eigen::VectorXd & joint_pos,
    const Eigen::VectorXd & joint_vel, 
    const std::string & link_name,
    Eigen::Matrix<double, 6, 1> & twist)
{
  // verify inputs
  if (!verify_initialized() || 
    !verify_joint_vector(joint_pos) || 
    !verify_joint_vector(joint_vel) ||
    !verify_link_name(link_name))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;
  q_dot_.data = joint_vel;

  // set zero
  twist.setZero();

  // special case: since the root is not in the robot tree, need to return zero twist
  if (link_name == base_link_)
  {
    return true;
  }

  // Absolute velocity w. r. t. base
  KDL::FrameVel vel;
  fk_vel_solver_->JntToCart(KDL::JntArrayVel(q_, q_dot_), vel);
  twist[0] = vel.deriv().vel.x();
  twist[1] = vel.deriv().vel.y();
  twist[2] = vel.deriv().vel.z();
  twist[3] = vel.deriv().rot.x();
  twist[4] = vel.deriv().rot.y();
  twist[5] = vel.deriv().rot.z();

  return true;
}  

bool IKSolver::calculate_jacobian(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_jacobian(jacobian))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  jacobian = jacobian_->data;

  return true;
}


bool IKSolver::verify_link_name(const std::string & link_name)
{
  if (link_name == base_link_)
  {
    return true;
  }
  if (link_name_map_.find(link_name) == link_name_map_.end())
  {
    std::string links;
    for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
      links += "\n" + chain_.getSegment(i).getName();
    }
    RCLCPP_ERROR(
      rclcpp::get_logger("IKSolver"), "The link %s was not found in the robot chain. Available links are: %s",
      link_name.c_str(), links.c_str());
    return false;
  }
  return true;
}

bool IKSolver::verify_joint_vector(const Eigen::VectorXd & joint_vector)
{
  if (static_cast<size_t>(joint_vector.size()) != num_joints_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("IKSolver"), "Invalid joint vector size (%zu). Expected size is %zu.", joint_vector.size(),
      num_joints_);
    return false;
  }
  return true;
}

bool IKSolver::verify_jacobian(
  const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  if (jacobian.rows() != jacobian_->rows() || jacobian.cols() != jacobian_->columns())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("IKSolver"), "The size of the jacobian (%zu, %zu) does not match the required size of (%u, %u)",
      jacobian.rows(), jacobian.cols(), jacobian_->rows(), jacobian_->columns());
    return false;
  }
  return true;
}


bool IKSolver::verify_initialized()
{
  // check if interface is initialized
  if (!initialized_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("IKSolver"),
      "The KDL kinematics plugin was not initialized. Ensure you called the initialize method.");
    return false;
  }
  return true;
}



}  // namespace manipulator_controllers