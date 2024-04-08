#ifndef UTILS__UTILS_HPP_
#define UTILS__UTILS_HPP_


#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include <vector>

namespace manipulator_controllers
{
  
  template <typename T1, typename T2>
  void vec_to_eigen(const std::vector<T1> & data, T2 & matrix)
  {
    for (auto col = 0; col < matrix.cols(); col++)
    {
      for (auto row = 0; row < matrix.rows(); row++)
      {
        matrix(row, col) = data[row + col * matrix.rows()];
      }
    }
  }


  
} // namespace manipulator_controllers



#endif  // UTILS__UTILS_HPP_