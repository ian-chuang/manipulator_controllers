#ifndef IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_
#define IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

// include generated parameter library
#include "impedance_pose_controller_parameters.hpp"

#include "manipulator_controllers/base_force_controller.hpp"

namespace manipulator_controllers
{
class ImpedancePoseController : public manipulator_controllers::BaseForceController
{
public:
  
};

}  // namespace manipulator_controllers

#endif  // IMPEDANCE_POSE_CONTROLLER__IMPEDANCE_POSE_CONTROLLER_HPP_
