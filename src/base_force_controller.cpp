#include "manipulator_controllers/base_force_controller.hpp"

namespace manipulator_controllers
{

controller_interface::CallbackReturn BaseForceController::on_init()
{
  auto ret = BaseController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // initialize controller config
  try
  {
    base_force_controller_parameter_handler_ = std::make_shared<base_force_controller::ParamListener>(get_node());
    base_force_controller_parameters_ = base_force_controller_parameter_handler_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BaseForceController::state_interface_configuration()
  const
{
  auto config = BaseController::state_interface_configuration();

  auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
  config.names.insert(
    config.names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return config;
}

controller_interface::CallbackReturn BaseForceController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = BaseController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    semantic_components::ForceTorqueSensor(base_force_controller_parameters_.ft_sensor.name));

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn BaseForceController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = BaseController::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // activate force_torque_sensor
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  read_state_from_hardware(ft_values_);

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn BaseForceController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  // release force torque sensor interface
  force_torque_sensor_->release_interfaces();

  return BaseController::on_deactivate(previous_state);
}

void BaseForceController::read_state_from_hardware(
  geometry_msgs::msg::Wrench & ft_values
  )
{
  // if any ft_values are nan, assume values are zero
  force_torque_sensor_->get_values_as_message(ft_values);
  if (
    std::isnan(ft_values.force.x) || std::isnan(ft_values.force.y) ||
    std::isnan(ft_values.force.z) || std::isnan(ft_values.torque.x) ||
    std::isnan(ft_values.torque.y) || std::isnan(ft_values.torque.z))
  {
    ft_values = geometry_msgs::msg::Wrench();
  }
}



} // namespace manipulator_controllers