#include "manipulator_controllers/impedance_pose_controller.hpp"

namespace manipulator_controllers
{

} // namespace manipulator_controllers


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  manipulator_controllers::ImpedancePoseController, controller_interface::ChainableControllerInterface)
