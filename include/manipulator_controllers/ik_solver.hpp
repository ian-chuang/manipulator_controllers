#ifndef IK_SOLVER__IK_SOLVER_HPP_
#define IK_SOLVER__IK_SOLVER_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include <kdl/chaindynparam.hpp>
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
    const std::string & ee_link,
    const double & virtual_link_mass
    );

  bool calculate_jacobian(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian);

  void task_force_to_joint_acc(
    const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, 
    const Eigen::Matrix<double, 6, 1> & net_force,
    const std::string & link_name,
    Eigen::VectorXd & joint_acc
  );

private:
  bool verify_initialized();
  bool verify_link_name(const std::string & link_name);
  bool verify_joint_vector(const Eigen::VectorXd & joint_vector);
  bool verify_jacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian);

  bool buildGenericModel();

  bool initialized_ = false;

  std::string base_link_;
  std::string ee_link_;
  double virtual_link_mass_;
  size_t num_joints_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::shared_ptr<KDL::ChainDynParam> jnt_space_inertia_solver_;  

  KDL::JntArray q_;
  std::shared_ptr<KDL::Jacobian> jacobian_;
  std::shared_ptr<KDL::JntSpaceInertiaMatrix> jnt_space_inertia_;
  std::unordered_map<std::string, int> link_name_map_;
  
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

};

}  // namespace manipulator_controllers

#endif // IK_SOLVER__IK_SOLVER_HPP_
