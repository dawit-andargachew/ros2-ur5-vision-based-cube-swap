/*
 * Orchestrator.cpp
 * ros2 jazzy node
 * State Machine Orchestrator for the swapping of two cubes on two respective tables
 * with UR5 robotic arm using ROS2 and MoveIt2.
 * We let cube 1 be on Table 1
 * We let cube 2 be on Table 2
 * These labels and information is known a priori.
 * 
 * Orchestrator exists at the SYMBOLIC LEVEL, commanding the Pick and Place action servers/clients
 * It does not deal with motion planning, it does not even know the exact poses of the cubes.
 * 
 * The Pick action has the following interface:
 * Goal: cube_id (uint8)
 * Result: success (bool)
 * Feedback: status (string)
 * 
 * The Place action has the following interface:
 * Goal: target_table_id (uint8)
 * Result: success (bool)
 * Feedback: status (string)
 * 
 * both actions can potentially fail if motion planning fails or if the action is cancelled.
 * The Orchestrator must handle these failures and transition to an ERROR state if needed.
 * 
 * Therefore we also define some yaml configuration for the orchestrator parameters such as:
 * - max_retries: number of retries for pick/place before transitioning to ERROR state
 * - wait_durations: durations to wait between retries
 * - logging_level: level of logging detail
 */



#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "group_6_assignment_2/action/pick.hpp"
#include "group_6_assignment_2/action/place.hpp"

using namespace std::chrono_literals;
using Pick = group_6_assignment_2::action::Pick;
using Place = group_6_assignment_2::action::Place;

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include "group_6_assignment_2/orchestrator_action_nodes.hpp" 

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("orchestrator");

  // 1. Setup Factory
  BT::BehaviorTreeFactory factory;
  
  // 2. Register custom nodes
  // The string "PickAction" must match the name used in the XML
  BT::RosNodeParams params;
  params.nh = node;
  factory.registerNodeType<PickActionNode>("PickAction", params);
  factory.registerNodeType<PlaceActionNode>("PlaceAction", params);
  factory.registerNodeType<HomeActionNode>("HomeAction", params);
  factory.registerNodeType<DetectColorAction>("DetectColorAction", params);
  factory.registerNodeType<DisplayColorResult>("DisplayColorResult", params);

  // 3. Load Tree from installed behaviour_trees directory
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("group_6_assignment_2");
  std::string tree_file = package_share_directory + "/behaviour_trees/swap_logic.xml";
  RCLCPP_INFO(node->get_logger(), "Loading BehaviorTree from: %s", tree_file.c_str());
  auto tree = factory.createTreeFromFile(tree_file);
  
  // Assign the max_retries from ROS parameters to the BT Blackboard
  int retries = node->declare_parameter("max_retries", 3);
  tree.rootBlackboard()->set("max_retries", retries);

  // Enable detailed BehaviorTree logging (shows retries, node transitions, etc.)
  bool enable_bt_logging = node->declare_parameter("enable_bt_logging", true);
  std::unique_ptr<BT::StdCoutLogger> logger;
  if (enable_bt_logging) {
    logger = std::make_unique<BT::StdCoutLogger>(tree);
    RCLCPP_INFO(node->get_logger(), "BehaviorTree detailed logging ENABLED (shows all node ticks, retries, and transitions)");
  } else {
    RCLCPP_INFO(node->get_logger(), "BehaviorTree detailed logging DISABLED");
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for action servers to become available...");
  RCLCPP_INFO(node->get_logger(), "Make sure 'pick_action_server', 'place_action_server', 'home_action_server', and 'color_detector' service are running!");
  std::this_thread::sleep_for(2s);

  // 4. Run the tree
  RCLCPP_INFO(node->get_logger(), "=================================================");
  RCLCPP_INFO(node->get_logger(), "Starting BehaviorTree execution with max_retries=%d", retries);
  RCLCPP_INFO(node->get_logger(), "=================================================");
  RCLCPP_INFO(node->get_logger(), "Watch for action feedback and retry messages below...");
  
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  int tick_count = 0;
  auto last_log_time = std::chrono::steady_clock::now();
  
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickExactlyOnce();
    rclcpp::spin_some(node);
    tick_count++;
    
    // Log heartbeat every 5 seconds
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
      RCLCPP_INFO(node->get_logger(), "Orchestrator running... (tick count: %d)", tick_count);
      last_log_time = now;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Report final status
  RCLCPP_INFO(node->get_logger(), "=================================================");
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "SWAP OPERATION COMPLETED SUCCESSFULLY!");
    RCLCPP_INFO(node->get_logger(), "Total ticks: %d", tick_count);
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "SWAP OPERATION FAILED after all retries");
    RCLCPP_ERROR(node->get_logger(), "Total ticks: %d", tick_count);
  } else {
    RCLCPP_WARN(node->get_logger(), "Swap operation ended with unexpected status: %d", static_cast<int>(status));
  }
  RCLCPP_INFO(node->get_logger(), "=================================================");

  rclcpp::shutdown();
  // Return non-zero exit code on failure or unexpected status
  if (status == BT::NodeStatus::SUCCESS) {
    return 0;
  } else if (status == BT::NodeStatus::FAILURE) {
    return 1;
  } else {
    return 2;
  }
}