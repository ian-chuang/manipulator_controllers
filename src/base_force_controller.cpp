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

  zero_wrench_flag_.store(true);
  zero_wrench_offset_.setZero();
  base_wrench_.setZero();
  end_effector_weight_.setZero();
  end_effector_weight_[2] = -base_force_controller_parameters_.gravity_compensation.CoG.force;
  vec_to_eigen(base_force_controller_parameters_.gravity_compensation.CoG.pos, cog_pos_);


  // setup subscribers and publishers
  auto zero_wrench_callback =
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response>      response
      )
  {  
    zero_wrench_flag_.store(true);
    // wait until flag is set back to false or exceed timeout of 3 seconds
    // or ros shutdown
    auto start = std::chrono::system_clock::now();
    while (zero_wrench_flag_.load())
    {
      if (std::chrono::system_clock::now() - start > std::chrono::seconds(5))
      {
        response->success = false;
        response->message = "Zero wrench service timed out";
        return;
      }
      if (!rclcpp::ok())
      {
        response->success = false;
        response->message = "ROS shutdown";
        return;
      }
      // block for 1ms
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    response->success = true;
    response->message = "Zero wrench service completed";
  };
  zero_wrench_service_ =
    get_node()->create_service<std_srvs::srv::Trigger>(
      "~/zero_ft_sensor", zero_wrench_callback);

  w_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/ft_sensor", rclcpp::SystemDefaultsQoS());
  wrench_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(w_publisher_);
  // Initialize state message
  //wrench_publisher_->lock();
  //wrench_publisher_->msg_ = admittance_->get_controller_state();
  //wrench_publisher_->unlock();

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

void BaseForceController::process_wrench_measurements(
  const Eigen::VectorXd & joint_pos,
  const geometry_msgs::msg::Wrench & measured_wrench
)
{
  bool zero_wrench_flag = zero_wrench_flag_.load();

  Eigen::Isometry3d world_base_transform;
  ik_solver_->calculate_link_transform(
    joint_pos, 
    base_force_controller_parameters_.fixed_world_frame.frame.id,
    base_controller_parameters_.kinematics.base_link,
    world_base_transform
  );

  Eigen::Isometry3d world_sensor_transform;
  ik_solver_->calculate_link_transform(
    joint_pos, 
    base_force_controller_parameters_.fixed_world_frame.frame.id,
    base_force_controller_parameters_.ft_sensor.frame.id,
    world_sensor_transform
  );

  Eigen::Isometry3d world_cog_transform;
  ik_solver_->calculate_link_transform(
    joint_pos, 
    base_force_controller_parameters_.fixed_world_frame.frame.id,
    base_force_controller_parameters_.gravity_compensation.frame.id,
    world_cog_transform
  );

  Eigen::Isometry3d base_compliance_transform;
  ik_solver_->calculate_link_transform(
    joint_pos, 
    base_controller_parameters_.kinematics.base_link,
    base_force_controller_parameters_.compliance.frame.id,
    base_compliance_transform
  );

  Eigen::Isometry3d compliance_sensor_transform;
  ik_solver_->calculate_link_transform(
    joint_pos, 
    base_force_controller_parameters_.compliance.frame.id,
    base_force_controller_parameters_.ft_sensor.frame.id,
    compliance_sensor_transform
  );


  Eigen::Matrix<double, 6, 1> sensor_wrench;
  sensor_wrench(0, 0) = measured_wrench.force.x;
  sensor_wrench(1, 0) = measured_wrench.force.y;
  sensor_wrench(2, 0) = measured_wrench.force.z;
  sensor_wrench(3, 0) = measured_wrench.torque.x;
  sensor_wrench(4, 0) = measured_wrench.torque.y;
  sensor_wrench(5, 0) = measured_wrench.torque.z;

  // transform wrench to world frame
  Eigen::Matrix<double, 6, 1> world_wrench;
  world_wrench.block<3, 1>(0, 0) = world_sensor_transform.rotation() * sensor_wrench.block<3, 1>(0, 0);
  world_wrench.block<3, 1>(3, 0) = world_sensor_transform.rotation() * sensor_wrench.block<3, 1>(3, 0);

  

  // // apply gravity compensation
  // world_wrench(2, 0) -= end_effector_weight_[2];
  world_wrench.block<3, 1>(3, 0) -= (world_cog_transform.rotation() * cog_pos_).cross(end_effector_weight_);

  

  if (zero_wrench_flag) {
    zero_wrench_offset_ = world_wrench;
  }
  world_wrench -= zero_wrench_offset_;


  // transform back to sensor frame
  sensor_wrench.block<3, 1>(0, 0) = world_sensor_transform.rotation().transpose() * world_wrench.block<3, 1>(0, 0);
  sensor_wrench.block<3, 1>(3, 0) = world_sensor_transform.rotation().transpose() * world_wrench.block<3, 1>(3, 0);

  Eigen::Matrix<double, 6, 1> compliance_wrench;
  compliance_wrench.block<3, 1>(0, 0) = compliance_sensor_transform.rotation() * sensor_wrench.block<3, 1>(0, 0);
  compliance_wrench.block<3, 1>(3, 0) = compliance_sensor_transform.rotation() * sensor_wrench.block<3, 1>(3, 0) 
    + compliance_sensor_transform.translation().cross(compliance_wrench.block<3, 1>(0, 0));

  // transform to base frame
  Eigen::Matrix<double, 6, 1> new_base_wrench;
  new_base_wrench.block<3, 1>(0, 0) =
    base_compliance_transform.rotation() * compliance_wrench.block<3, 1>(0, 0);
  new_base_wrench.block<3, 1>(3, 0) =
    base_compliance_transform.rotation() * compliance_wrench.block<3, 1>(3, 0);

  double alpha = base_force_controller_parameters_.ft_sensor.filter_coefficient;
  base_wrench_ = alpha * new_base_wrench + (1 - alpha) * base_wrench_;

  wrench_publisher_->lock();
  wrench_publisher_->msg_.header.frame_id = base_controller_parameters_.kinematics.base_link;
  wrench_publisher_->msg_.header.stamp = get_node()->now();
  wrench_publisher_->msg_.wrench.force.x = base_wrench_(0);
  wrench_publisher_->msg_.wrench.force.y = base_wrench_(1);
  wrench_publisher_->msg_.wrench.force.z = base_wrench_(2);
  wrench_publisher_->msg_.wrench.torque.x = base_wrench_(3);
  wrench_publisher_->msg_.wrench.torque.y = base_wrench_(4);
  wrench_publisher_->msg_.wrench.torque.z = base_wrench_(5);
  wrench_publisher_->unlockAndPublish();


  zero_wrench_flag_.store(false);
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