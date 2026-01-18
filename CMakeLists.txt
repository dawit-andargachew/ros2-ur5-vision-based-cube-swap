cmake_minimum_required(VERSION 3.8)
project(group_6_assignment_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# ADDED: Required for doTransform
find_package(tf2_geometry_msgs REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------------------------------------------------------
# Action Servers and Clients (Existing)
# ---------------------------------------------------------

add_executable(pick_action_server src/PickActionServer.cpp)
ament_target_dependencies(pick_action_server
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2
  geometry_msgs
  shape_msgs
)

add_executable(pick_action_client src/PickActionClient.cpp)
ament_target_dependencies(pick_action_client rclcpp rclcpp_action)

add_executable(place_action_server src/PlaceActionServer.cpp)
ament_target_dependencies(place_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2
  geometry_msgs
  shape_msgs
)

add_executable(place_action_client src/PlaceActionClient.cpp)
ament_target_dependencies(place_action_client rclcpp rclcpp_action geometry_msgs tf2)

add_executable(home_action_server src/HomeActionServer.cpp)
ament_target_dependencies(home_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
)

add_executable(home_action_client src/HomeActionClient.cpp)
ament_target_dependencies(home_action_client rclcpp rclcpp_action)

# ---------------------------------------------------------
# Motion Util Library
# ---------------------------------------------------------
add_library(motion_util_lib STATIC src/motion_util.cpp)
ament_target_dependencies(motion_util_lib
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
)
target_include_directories(motion_util_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pick_action_server motion_util_lib)
target_link_libraries(place_action_server motion_util_lib)
target_link_libraries(home_action_server motion_util_lib)

# ---------------------------------------------------------
# Interfaces (Messages/Services/Actions)
# ---------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Pick.action"
  "action/Place.action"
  "action/Home.action"
  "srv/GetCubePoses.srv"
  "srv/GetCubeColors.srv"
  DEPENDENCIES geometry_msgs
)

# ---------------------------------------------------------
# Link generated interfaces to all executables
# ---------------------------------------------------------
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Helper macro to link dependencies and typesupport
macro(configure_standard_node target_name)
    add_dependencies(${target_name} ${PROJECT_NAME})
    target_link_libraries(${target_name} ${cpp_typesupport_target})
endmacro()

configure_standard_node(pick_action_server)
configure_standard_node(pick_action_client)
configure_standard_node(place_action_server)
configure_standard_node(place_action_client)
configure_standard_node(home_action_server)
configure_standard_node(home_action_client)

# ---------------------------------------------------------
# Cube Detector (Existing)
# ---------------------------------------------------------
add_executable(cube_detector src/CubeDetector.cpp)
ament_target_dependencies(cube_detector
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)
configure_standard_node(cube_detector)

# ---------------------------------------------------------
# Orchestrator (Existing)
# ---------------------------------------------------------
add_executable(orchestrator src/Orchestrator.cpp)
ament_target_dependencies(orchestrator
  rclcpp
  rclcpp_action
  behaviortree_cpp
  ament_index_cpp
)
target_include_directories(orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
configure_standard_node(orchestrator)

# ---------------------------------------------------------
# Color Detector (UPDATED)
# ---------------------------------------------------------
add_executable(color_detector src/ColorDetector.cpp)
ament_target_dependencies(color_detector 
    rclcpp 
    cv_bridge 
    sensor_msgs 
    tf2 
    tf2_ros 
    tf2_geometry_msgs  # <--- NEW: Needed for projection math
    geometry_msgs      # <--- NEW: Needed for Poses
    # apriltag_msgs    # REMOVED: No longer used here
)
target_link_libraries(color_detector ${OpenCV_LIBRARIES})
configure_standard_node(color_detector)


# ---------------------------------------------------------
# Installation
# ---------------------------------------------------------
install(TARGETS
  pick_action_server pick_action_client
  place_action_server place_action_client
  home_action_server home_action_client
  cube_detector
  orchestrator
  color_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY behaviour_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()