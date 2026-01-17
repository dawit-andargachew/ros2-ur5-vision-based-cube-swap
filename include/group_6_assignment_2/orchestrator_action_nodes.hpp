#ifndef ORCHESTRATOR_ACTION_NODES_HPP_
#define ORCHESTRATOR_ACTION_NODES_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "group_6_assignment_2/action/pick.hpp"
#include "group_6_assignment_2/action/place.hpp"

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

#endif