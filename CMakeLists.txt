cmake_minimum_required(VERSION 3.16)
project(manipulator_controllers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  trajectory_msgs
  kdl_parser
  tf2_eigen_kdl
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

# parameter libraries
generate_parameter_library(base_controller_parameters
  src/base_controller_parameters.yaml
)
generate_parameter_library(base_force_controller_parameters
  src/base_force_controller_parameters.yaml
)
generate_parameter_library(pose_controller_parameters
  src/pose_controller_parameters.yaml
)
generate_parameter_library(impedance_pose_controller_parameters
  src/impedance_pose_controller_parameters.yaml
)

# pose_controller
# add_library(pose_controller SHARED
#   src/pose_controller.cpp
#   src/base_controller.cpp
#   src/ik_solver.cpp
# )
# target_compile_features(pose_controller PUBLIC cxx_std_17)
# target_include_directories(pose_controller PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(pose_controller PUBLIC
#   base_controller_parameters
#   pose_controller_parameters
#   Eigen3::Eigen
# )
# ament_target_dependencies(pose_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# pluginlib_export_plugin_description_file(controller_interface pose_controller.xml)

# impedance_pose_controller
add_library(impedance_pose_controller SHARED
  src/impedance_pose_controller.cpp
  src/base_force_controller.cpp
  src/base_controller.cpp
  src/ik_solver.cpp
)
target_compile_features(impedance_pose_controller PUBLIC cxx_std_17)
target_include_directories(impedance_pose_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(impedance_pose_controller PUBLIC
  base_controller_parameters
  base_force_controller_parameters
  impedance_pose_controller_parameters
  Eigen3::Eigen
)
ament_target_dependencies(impedance_pose_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface impedance_pose_controller.xml)

# install

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
    # pose_controller 
    impedance_pose_controller
    base_controller_parameters
    base_force_controller_parameters
    impedance_pose_controller_parameters

  EXPORT 
    export_manipulator_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_manipulator_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
