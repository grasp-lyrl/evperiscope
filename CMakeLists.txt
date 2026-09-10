cmake_minimum_required(VERSION 3.8)
project(epa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv_ros2_messaging REQUIRED)
find_package(dv-processing REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

# ============================================================
# EPA Controller component (vision pipeline + PID controller)
# ============================================================
add_library(epa_controller_component SHARED
  src/epa_controller.cpp
  src/ev_prop_det.cpp
  src/quad_pose_filter.cpp
  src/quad_controller.cpp
)

target_link_libraries(epa_controller_component
  ${OpenCV_LIBS}
  ${HDF5_CXX_LIBRARIES}
  ${HDF5_LIBRARIES}
  Eigen3::Eigen
  dv::processing
)

ament_target_dependencies(epa_controller_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  cv_bridge
  dv_ros2_msgs
  dv_ros2_messaging
  mavros_msgs
)

rclcpp_components_register_nodes(epa_controller_component "epa::EpaController")

# ============================================================
# EPA Interface component (MAVROS bridge + flight commands)
# ============================================================
add_library(epa_interface_component SHARED
  src/epa_interface.cpp
)

ament_target_dependencies(epa_interface_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  mavros_msgs
  rcl_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rclcpp_components_register_nodes(epa_interface_component "epa::EpaInterface")

# ============================================================
# Install
# ============================================================
install(TARGETS
  epa_controller_component
  epa_interface_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/epa_gui.py
  scripts/pose_to_odom_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Export, so downstream packages can find the headers and libs
# ============================================================
ament_export_include_directories(include)
ament_export_libraries(
  epa_controller_component
  epa_interface_component
)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  mavros_msgs
  cv_bridge
  dv_ros2_msgs
  dv_ros2_messaging
  dv-processing
  OpenCV
  Eigen3
  HDF5
)

ament_package()
