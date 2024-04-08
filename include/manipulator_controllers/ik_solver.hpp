#ifndef IK_SOLVER__IK_SOLVER_HPP_
#define IK_SOLVER__IK_SOLVER_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/treejnttojacsolver.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "tf2_eigen_kdl/tf2_eigen_kdl.hpp"

namespace manipulator_controllers
{

class IKSolver
{
public:

  bool initialize(
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string & base_link,
    const std::string & ee_link
    );

  // bool calculate_link_transform(
  //   const Eigen::VectorXd & joint_pos, const std::string & link_name,
  //   Eigen::Isometry3d & pose);

  bool calculate_link_transform(
    const Eigen::VectorXd & joint_pos, 
    const std::string & parent_link,
    const std::string & child_link,
    Eigen::Isometry3d & pose);
  
  bool calculate_link_twist(
    const Eigen::VectorXd & joint_pos,
    const Eigen::VectorXd & joint_vel, 
    const std::string & link_name,
    Eigen::Matrix<double, 6, 1> & twist);

  bool calculate_jacobian(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian);

  // bool convert_cartesian_deltas_to_joint_deltas(
  //   const Eigen::VectorXd & joint_pos, const Eigen::Matrix<double, 6, 1> & delta_x,
  //   const std::string & link_name, Eigen::VectorXd & delta_theta);

private:
  bool verify_initialized();
  bool verify_link_name(const std::string & link_name);
  bool verify_joint_vector(const Eigen::VectorXd & joint_vector);
  bool verify_jacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian);

  bool initialized_ = false;

  std::string base_link_;
  std::string ee_link_;
  size_t num_joints_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  std::shared_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  KDL::JntArray q_;
  // make vel jnt array
  KDL::JntArray q_dot_;
  KDL::Frame parent_frame_;
  KDL::Frame child_frame_;
  KDL::FrameVel frame_vel_;
  std::shared_ptr<KDL::Jacobian> jacobian_;
  std::unordered_map<std::string, int> link_name_map_;
  // double alpha_;  // damping term for Jacobian inverse
  // Eigen::MatrixXd I_;
  
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

};

}  // namespace manipulator_controllers

#endif // IK_SOLVER__IK_SOLVER_HPP_
