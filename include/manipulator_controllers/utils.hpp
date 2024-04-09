#ifndef UTILS__UTILS_HPP_
#define UTILS__UTILS_HPP_


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include <vector>

namespace manipulator_controllers
{
  
  template <typename T1, typename T2>
  inline void vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
  {
    for (auto col = 0; col < matrix.cols(); col++)
    {
      for (auto row = 0; row < matrix.rows(); row++)
      {
        matrix(row, col) = data[row + col * matrix.rows()];
      }
    }
  }

  inline void pseudo_inverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
      S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
  }

  inline void create_gain_matrix(
    const Eigen::Matrix<double, 6, 1> & gains, 
    const Eigen::Isometry3d & base_control_transform,
    Eigen::Matrix<double, 6, 6> & gain_matrix
  ) {
    auto R = base_control_transform.rotation();
    Eigen::Matrix<double, 6, 6> tensor = gains.asDiagonal();
    Eigen::Matrix<double, 6, 6> tmp = Eigen::Matrix<double, 6, 6>::Zero();
    tmp.topLeftCorner<3, 3>() = R * tensor.topLeftCorner<3, 3>() * R.transpose();
    tmp.bottomRightCorner<3, 3>() = R * tensor.bottomRightCorner<3, 3>() * R.transpose();
    gain_matrix = tmp;


    // // Create stiffness matrix in base frame. The user-provided values of admittance_state.stiffness
    // // correspond to the six diagonal elements of the stiffness matrix expressed in the control frame
    // auto rot_base_control = base_control_transform.rotation();
    // Eigen::Matrix<double, 6, 6> G = Eigen::Matrix<double, 6, 6>::Zero();
    // Eigen::Matrix<double, 3, 3> G_pos = Eigen::Matrix<double, 3, 3>::Zero();
    // Eigen::Matrix<double, 3, 3> G_rot = Eigen::Matrix<double, 3, 3>::Zero();
    // G_pos.diagonal() = gains.block<3, 1>(0, 0);
    // G_rot.diagonal() = gains.block<3, 1>(3, 0);
    // // Transform to the control frame
    // // A reference is here:  https://users.wpi.edu/~jfu2/rbe502/files/force_control.pdf
    // // Force Control by Luigi Villani and Joris De Schutter
    // // Page 200
    // G_pos = rot_base_control * G_pos * rot_base_control.transpose();
    // G_rot = rot_base_control * G_rot * rot_base_control.transpose();
    // G.block<3, 3>(0, 0) = G_pos;
    // G.block<3, 3>(3, 3) =  G_rot;

    // gain_matrix = G;


  }


  
} // namespace manipulator_controllers



#endif  // UTILS__UTILS_HPP_