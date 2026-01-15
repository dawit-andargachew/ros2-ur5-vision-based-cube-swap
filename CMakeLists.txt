cmake_minimum_required(VERSION 3.8)
project(group_6_assignment_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(ament_cmake_python REQUIRED)
# Create executable 'pick_action_server' from 'src/PickActionServer.cpp'
add_executable(pick_action_server src/PickActionServer.cpp)
# Link necessary libraries
ament_target_dependencies(pick_action_server
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
)

# Create executable 'pick_action_client' from 'src/PickActionClient.cpp'
add_executable(pick_action_client src/PickActionClient.cpp)
ament_target_dependencies(pick_action_client
  rclcpp
  rclcpp_action
)

add_executable(place_action_server src/PlaceActionServer.cpp)
# Link necessary libraries
ament_target_dependencies(place_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
)

add_executable(place_action_client src/PlaceActionClient.cpp)
ament_target_dependencies(place_action_client
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
)

# Add the motion_util library
add_library(motion_util_lib STATIC src/motion_util.cpp)
ament_target_dependencies(motion_util_lib
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
)

# Set include directories for the library
target_include_directories(motion_util_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link the library to the pick executable
target_link_libraries(pick_action_server motion_util_lib)
target_link_libraries(place_action_server motion_util_lib)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Pick.action"
  "action/Place.action"
  "srv/GetCubePoses.srv"
  DEPENDENCIES geometry_msgs
)

add_dependencies(pick_action_server ${PROJECT_NAME})
add_dependencies(pick_action_client ${PROJECT_NAME})
add_dependencies(place_action_server ${PROJECT_NAME})
add_dependencies(place_action_client ${PROJECT_NAME})

install(TARGETS
  pick_action_server
  pick_action_client
  place_action_server
  place_action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(cube_detector src/CubeDetector.cpp)
ament_target_dependencies(
  cube_detector
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

add_dependencies(cube_detector ${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(pick_action_server ${cpp_typesupport_target})
target_link_libraries(pick_action_client ${cpp_typesupport_target})
target_link_libraries(place_action_server ${cpp_typesupport_target})
target_link_libraries(place_action_client ${cpp_typesupport_target})
target_link_libraries(cube_detector ${cpp_typesupport_target})

install(TARGETS 
  cube_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()