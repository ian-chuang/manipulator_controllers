#include "manipulator_controllers/fk_solver.hpp"

namespace manipulator_controllers
{

bool FKSolver::initialize(
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & base_link,
  const std::string & ee_link
) {
  initialized_ = true;
  base_link_ = base_link;
  ee_link_ = ee_link;
  parameters_interface_ = parameters_interface;

  // get robot description
  auto robot_param = rclcpp::Parameter();
  if (!parameters_interface->get_parameter("robot_description", robot_param))
  {
      RCLCPP_ERROR(rclcpp::get_logger("FKSolver"), "parameter robot_description not set");
      return false;
  }
  auto robot_description = robot_param.as_string();

  // create kinematic chain
  KDL::Tree robot_tree;
  kdl_parser::treeFromString(robot_description, robot_tree);
  if (!robot_tree.getChain(base_link_, ee_link_, chain_))
  {
      RCLCPP_ERROR(
      rclcpp::get_logger("FKSolver"), "failed to find chain from robot base %s to end effector %s", base_link_.c_str(),
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
  fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  
  return true;
}

bool FKSolver::calculate_link_transform(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, 
  const std::string & parent_link,
  const std::string & child_link,
  Eigen::Isometry3d & pose
)
{
  // verify inputs
  if (
    !verify_initialized() || 
    !verify_joint_vector(joint_pos) || 
    !verify_link_name(parent_link) ||
    !verify_link_name(child_link)
  )
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  KDL::Frame parent_frame;
  KDL::Frame child_frame;
  Eigen::Isometry3d parent_transform;
  Eigen::Isometry3d child_transform;

  fk_pos_solver_->JntToCart(q_, parent_frame, link_name_map_[parent_link]);
  fk_pos_solver_->JntToCart(q_, child_frame, link_name_map_[child_link]);

  tf2::transformKDLToEigen(parent_frame, parent_transform);
  tf2::transformKDLToEigen(child_frame, child_transform);

  pose = parent_transform.inverse() * child_transform;
  
  return true;
}

bool FKSolver::verify_link_name(const std::string & link_name)
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
      rclcpp::get_logger("FKSolver"), "The link %s was not found in the robot chain. Available links are: %s",
      link_name.c_str(), links.c_str());
    return false;
  }
  return true;
}

bool FKSolver::verify_joint_vector(const Eigen::VectorXd & joint_vector)
{
  if (static_cast<size_t>(joint_vector.size()) != num_joints_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("FKSolver"), "Invalid joint vector size (%zu). Expected size is %zu.", joint_vector.size(),
      num_joints_);
    return false;
  }
  return true;
}

bool FKSolver::verify_initialized()
{
  // check if interface is initialized
  if (!initialized_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("FKSolver"),
      "The KDL kinematics plugin was not initialized. Ensure you called the initialize method.");
    return false;
  }
  return true;
}



}  // namespace manipulator_controllers