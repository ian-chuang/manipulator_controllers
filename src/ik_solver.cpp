#include "manipulator_controllers/ik_solver.hpp"

namespace manipulator_controllers
{

bool IKSolver::initialize(
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & base_link,
  const std::string & ee_link,
  const double & virtual_link_mass = 1.0
) {
  initialized_ = true;
  base_link_ = base_link;
  ee_link_ = ee_link;
  virtual_link_mass_ = virtual_link_mass;
  parameters_interface_ = parameters_interface;
  // alpha_ = alpha;

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
  jacobian_ = std::make_shared<KDL::Jacobian>(num_joints_);
  jnt_space_inertia_ = std::make_shared<KDL::JntSpaceInertiaMatrix>(num_joints_);
  // create KDL solvers
  jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
  jnt_space_inertia_solver_ = std::make_shared<KDL::ChainDynParam>(chain_, KDL::Vector::Zero());

  buildGenericModel();

  return true;
}


void IKSolver::forwardDynamics(
    const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, 
    const Eigen::Matrix<double, 6, 1> & net_force,
    const std::string & link_name,
    Eigen::VectorXd & joint_acc
  )
{
  q_.data = joint_pos;

  // Compute joint space inertia matrix with actualized link masses
  buildGenericModel();
  jnt_space_inertia_solver_->JntToMass(q_, *jnt_space_inertia_);

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);

  // Compute joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f) \f$
  joint_acc =
    jnt_space_inertia_->data.inverse() * jacobian_->data.transpose() * net_force;
}

void IKSolver::selectivelyDampedLeastSquares(
    const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, 
    const Eigen::Matrix<double, 6, 1> & net_error,
    const std::string & link_name,
    Eigen::VectorXd & joint_vel
  )
{
  q_.data = joint_pos;

  // Compute joint Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic> > JSVD(
    jacobian_->data, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 6, 6> U = JSVD.matrixU();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V = JSVD.matrixV();
  Eigen::Matrix<double, Eigen::Dynamic, 1> s = JSVD.singularValues();

  // Default recommendation by Buss and Kim.
  const double gamma_max = 3.141592653 / 4;

  Eigen::Matrix<double, Eigen::Dynamic, 1> sum_phi =
    Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(num_joints_);

  // Compute each joint velocity with the SDLS method.  This implements the
  // algorithm as described in the paper (but for only one end-effector).
  // Also see Buss' own implementation:
  // https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/index.html

  for (size_t i = 0; i < num_joints_; ++i)
  {
    double alpha = U.col(i).transpose() * net_error;

    double N = U.col(i).head(3).norm();
    double M = 0;
    for (size_t j = 0; j < num_joints_; ++j)
    {
      double rho = jacobian_->data.col(j).head(3).norm();
      M += std::abs(V.col(i)[j]) * rho;
    }
    M *= 1.0 / s[i];

    double gamma = std::min(1.0, N / M) * gamma_max;

    Eigen::Matrix<double, Eigen::Dynamic, 1> phi =
      clampMaxAbs(1.0 / s[i] * alpha * V.col(i), gamma);
    sum_phi += phi;
  }

  joint_vel = clampMaxAbs(sum_phi, gamma_max);
}

bool IKSolver::buildGenericModel()
{
  // Set all masses and inertias to minimal (yet stable) values.
  double ip_min = 0.000001;
  for (size_t i = 0; i < chain_.segments.size(); ++i)
  {
    // Fixed joint segment
    if (chain_.segments[i].getJoint().getType() == KDL::Joint::None)
    {
      chain_.segments[i].setInertia(KDL::RigidBodyInertia::Zero());
    }
    else  // relatively moving segment
    {
      chain_.segments[i].setInertia(
        KDL::RigidBodyInertia(virtual_link_mass_,                          // mass
                              KDL::Vector::Zero(),            // center of gravity
                              KDL::RotationalInertia(ip_min,  // ixx
                                                     ip_min,  // iyy
                                                     ip_min   // izz
                                                     // ixy, ixy, iyz default to 0.0
                                                     )));
    }
  }

  // Only give the last segment a generic mass and inertia.
  // See https://arxiv.org/pdf/1908.06252.pdf for a motivation for this setting.
  double m = 1;
  double ip = 1;
  chain_.segments[chain_.segments.size() - 1].setInertia(
    KDL::RigidBodyInertia(m, KDL::Vector::Zero(), KDL::RotationalInertia(ip, ip, ip)));

  return true;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> IKSolver::clampMaxAbs(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & w, double d)
{
  if (w.cwiseAbs().maxCoeff() <= d)
  {
    return w;
  }
  else
  {
    return d * w / w.cwiseAbs().maxCoeff();
  }
}

bool IKSolver::calculate_jacobian(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name)
    )
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