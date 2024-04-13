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

  inline Eigen::MatrixXd clip( const Eigen::MatrixXd& value, const Eigen::MatrixXd& min, const Eigen::MatrixXd& max) {
    // confirm dimensions are the same
    assert(value.rows() == min.rows());
    assert(value.cols() == min.cols());
    assert(value.rows() == max.rows());
    assert(value.cols() == max.cols());

    return value.cwiseMin(max).cwiseMax(min);
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
  }


  
} // namespace manipulator_controllers



#endif  // UTILS__UTILS_HPP_