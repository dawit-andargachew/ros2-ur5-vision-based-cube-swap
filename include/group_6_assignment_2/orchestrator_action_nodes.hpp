#ifndef ORCHESTRATOR_ACTION_NODES_HPP_
#define ORCHESTRATOR_ACTION_NODES_HPP_

#include "group_6_assignment_2/behaviortree_ros2/bt_action_node.hpp"
#include "group_6_assignment_2/action/pick.hpp"
#include "group_6_assignment_2/action/place.hpp"
#include "group_6_assignment_2/action/home.hpp"
#include "group_6_assignment_2/srv/get_cube_colors.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace BT;

// Template to avoid repeating the onFeedback and onResult logic
template <typename T>
class BaseActionNode : public RosActionNode<T>
{
public:
  BaseActionNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<T>(name, conf, params) {}

  // Log feedback status strings
  NodeStatus onFeedback(const std::shared_ptr<const typename T::Feedback> feedback) override {
    auto node = this->node_.lock();
    if (node) {
      RCLCPP_INFO(node->get_logger(), "[%s] Status: %s", this->name().c_str(), feedback->status.c_str());
    }
    return NodeStatus::RUNNING;
  }

  // Standard Success/Failure check
  NodeStatus onResultReceived(const typename RosActionNode<T>::WrappedResult& wr) override {
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result->success) {
      return NodeStatus::SUCCESS;
    }
    auto node = this->node_.lock();
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "[%s] Action failed or was cancelled", this->name().c_str());
    }
    return NodeStatus::FAILURE;
  }
};

class PickActionNode : public BaseActionNode<group_6_assignment_2::action::Pick>
{
public:
  using BaseActionNode<group_6_assignment_2::action::Pick>::BaseActionNode;
  static PortsList providedPorts() { 
    return { 
      InputPort<std::string>("action_name"),
      InputPort<uint8_t>("cube_id") 
    }; 
  }

  bool setGoal(RosActionNode::Goal& goal) override {
    auto res = getInput("cube_id", goal.cube_id);
    return res.has_value();
  }
};

class PlaceActionNode : public BaseActionNode<group_6_assignment_2::action::Place>
{
public:
  using BaseActionNode<group_6_assignment_2::action::Place>::BaseActionNode;
  static PortsList providedPorts() { 
    return { 
      InputPort<std::string>("action_name"),
      InputPort<uint8_t>("grabbed_cube_id"), 
      InputPort<uint8_t>("target_table_id") 
    }; 
  }

  bool setGoal(RosActionNode::Goal& goal) override {
    auto res1 = getInput("grabbed_cube_id", goal.grabbed_cube_id);
    auto res2 = getInput("target_table_id", goal.target_table_id);
    return res1.has_value() && res2.has_value();
  }
};

class HomeActionNode : public BaseActionNode<group_6_assignment_2::action::Home>
{
public:
  using BaseActionNode<group_6_assignment_2::action::Home>::BaseActionNode;
  static PortsList providedPorts() { 
    return { 
      InputPort<std::string>("action_name")
    }; 
  }

  bool setGoal(RosActionNode::Goal&) override {
    // Home action has no goal parameters (empty goal)
    return true;
  }
};

class DetectColorAction : public SyncActionNode
{
public:
  DetectColorAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : SyncActionNode(name, conf), node_(params.nh)
  {
    client_ = node_->create_client<group_6_assignment_2::srv::GetCubeColors>("get_cube_colors");
  }

  static PortsList providedPorts() { 
    return { 
      InputPort<std::string>("action_name"),
      OutputPort<std::string>("color_result")
    }; 
  }

  NodeStatus tick() override {
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "[%s] Service not available", name().c_str());
      return NodeStatus::FAILURE;
    }

    auto request = std::make_shared<group_6_assignment_2::srv::GetCubeColors::Request>();
    auto future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(10)) == 
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->success) {
        // Store result in blackboard instead of logging immediately
        setOutput("color_result", response->message);
        RCLCPP_INFO(node_->get_logger(), "[%s] Color detection completed (stored for later)", name().c_str());
        return NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(node_->get_logger(), "[%s] %s", name().c_str(), response->message.c_str());
        return NodeStatus::FAILURE;
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[%s] Service call timeout", name().c_str());
      return NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<group_6_assignment_2::srv::GetCubeColors>::SharedPtr client_;
};

class DisplayColorResult : public SyncActionNode
{
public:
  DisplayColorResult(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : SyncActionNode(name, conf), node_(params.nh) {}

  static PortsList providedPorts() { 
    return { 
      InputPort<std::string>("color_result")
    }; 
  }

  NodeStatus tick() override {
    auto result = getInput<std::string>("color_result");
    if (result) {
      RCLCPP_INFO(node_->get_logger(), "===== FINAL COLOR DETECTION RESULT =====");
      RCLCPP_INFO(node_->get_logger(), "%s", result.value().c_str());
      RCLCPP_INFO(node_->get_logger(), "========================================");
      return NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(node_->get_logger(), "No color result available to display");
    return NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

#endif