#ifndef FK_SOLVER__FK_SOLVER_HPP_
#define FK_SOLVER__FK_SOLVER_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "tf2_eigen_kdl/tf2_eigen_kdl.hpp"

namespace manipulator_controllers
{

class FKSolver
{
public:

  bool initialize(
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string & base_link,
    const std::string & ee_link
    );

  bool calculate_link_transform(
    const Eigen::VectorXd & joint_pos, 
    const std::string & parent_link,
    const std::string & child_link,
    Eigen::Isometry3d & pose);

private:
  bool verify_initialized();
  bool verify_link_name(const std::string & link_name);
  bool verify_joint_vector(const Eigen::VectorXd & joint_vector);

  bool initialized_ = false;

  std::string base_link_;
  std::string ee_link_;
  size_t num_joints_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  KDL::JntArray q_;
  KDL::Frame parent_frame_;
  KDL::Frame child_frame_;
  std::unordered_map<std::string, int> link_name_map_;
  
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

};

}  // namespace manipulator_controllers

#endif // FK_SOLVER__FK_SOLVER_HPP_
