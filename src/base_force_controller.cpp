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

  // initialize vals
  zero_wrench_flag_.store(true);
  zero_wrench_offset_.setZero();
  base_wrench_.setZero();
  end_effector_weight_.setZero();
  end_effector_weight_[2] = -base_force_controller_parameters_.ft_sensor.gravity_compensation.CoG.force;
  vec_to_eigen(base_force_controller_parameters_.ft_sensor.gravity_compensation.CoG.pos, cog_pos_);

  // zero wrench service
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

  // wrench publisher
  w_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/ft_sensor", rclcpp::SystemDefaultsQoS());
  wrench_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(w_publisher_);

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

  read_state_from_hardware(joint_state_, ft_values_);

  // initialize vals
  zero_wrench_flag_.store(true);
  zero_wrench_offset_.setZero();
  base_wrench_.setZero();
  end_effector_weight_.setZero();
  end_effector_weight_[2] = -base_force_controller_parameters_.ft_sensor.gravity_compensation.CoG.force;
  vec_to_eigen(base_force_controller_parameters_.ft_sensor.gravity_compensation.CoG.pos, cog_pos_);

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
  const trajectory_msgs::msg::JointTrajectoryPoint & joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  geometry_msgs::msg::Wrench & processed_wrench
)
{
  // get joint positions
  Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(num_joints_);
  vec_to_eigen(joint_state.positions, joint_pos);

  // get transforms
  bool success = true;
  Eigen::Isometry3d world_sensor_transform;
  success &= fk_solver_->calculate_link_transform(
    joint_pos, 
    base_force_controller_parameters_.ft_sensor.gravity_compensation.gravity_frame,
    base_force_controller_parameters_.ft_sensor.ft_frame,
    world_sensor_transform
  );
  Eigen::Isometry3d world_cog_transform;
  success &= fk_solver_->calculate_link_transform(
    joint_pos, 
    base_force_controller_parameters_.ft_sensor.gravity_compensation.gravity_frame,
    base_force_controller_parameters_.ft_sensor.gravity_compensation.CoG.frame,
    world_cog_transform
  );
  Eigen::Isometry3d base_compliance_transform;
  success &= fk_solver_->calculate_link_transform(
    joint_pos, 
    base_controller_parameters_.kinematics.robot_base,
    base_force_controller_parameters_.ft_sensor.new_ft_frame,
    base_compliance_transform
  );
  Eigen::Isometry3d compliance_sensor_transform;
  success &= fk_solver_->calculate_link_transform(
    joint_pos, 
    base_force_controller_parameters_.ft_sensor.new_ft_frame,
    base_force_controller_parameters_.ft_sensor.ft_frame,
    compliance_sensor_transform
  );
  if (!success)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to calculate link transforms in process_wrench_measurements, setting wrench to zero.");
    processed_wrench = geometry_msgs::msg::Wrench();
    return;
  }

  // create wrench vector
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


  // apply gravity compensation
  Eigen::Matrix<double, 6, 1>  gravity_compensation = Eigen::Matrix<double, 6, 1>::Zero();
  gravity_compensation(2, 0) = end_effector_weight_[2];
  gravity_compensation.block<3, 1>(3, 0) = (world_cog_transform.rotation() * cog_pos_).cross(end_effector_weight_);
  world_wrench -= gravity_compensation;

  // zero wrench if flag is set
  if (zero_wrench_flag_.load()) {
    zero_wrench_offset_ = world_wrench;
    zero_wrench_flag_.store(false);
  }
  world_wrench -= zero_wrench_offset_;

  // transform back to sensor frame
  sensor_wrench.block<3, 1>(0, 0) = world_sensor_transform.rotation().transpose() * world_wrench.block<3, 1>(0, 0);
  sensor_wrench.block<3, 1>(3, 0) = world_sensor_transform.rotation().transpose() * world_wrench.block<3, 1>(3, 0);

  // transform to compliance frame (factor in compliance frame translation)
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

  // filter wrench
  double alpha = base_force_controller_parameters_.ft_sensor.filter_coefficient;
  base_wrench_ = alpha * new_base_wrench + (1 - alpha) * base_wrench_;

  // set output
  processed_wrench.force.x = base_wrench_(0);
  processed_wrench.force.y = base_wrench_(1);
  processed_wrench.force.z = base_wrench_(2);
  processed_wrench.torque.x = base_wrench_(3);
  processed_wrench.torque.y = base_wrench_(4);
  processed_wrench.torque.z = base_wrench_(5);

  // publish wrench
  wrench_publisher_->lock();
  wrench_publisher_->msg_.header.frame_id = base_controller_parameters_.kinematics.robot_base;
  wrench_publisher_->msg_.header.stamp = get_node()->now();
  wrench_publisher_->msg_.wrench = processed_wrench;
  wrench_publisher_->unlockAndPublish();
}

void BaseForceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & joint_state,
  geometry_msgs::msg::Wrench & output_wrench
  )
{
  geometry_msgs::msg::Wrench wrench;
  // if any ft_values are nan, assume values are zero
  force_torque_sensor_->get_values_as_message(wrench);
  if (
    std::isnan(wrench.force.x) || std::isnan(wrench.force.y) ||
    std::isnan(wrench.force.z) || std::isnan(wrench.torque.x) ||
    std::isnan(wrench.torque.y) || std::isnan(wrench.torque.z))
  {
    wrench = geometry_msgs::msg::Wrench();
  }

  // process wrench measurements
  process_wrench_measurements(joint_state, wrench, output_wrench);
}



} // namespace manipulator_controllers