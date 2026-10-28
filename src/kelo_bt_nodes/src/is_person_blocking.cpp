// IsPersonBlocking: SUCCESS while a person stands (or is about to walk) on the path ahead, as reported
// by the social node on /kelo/person_blocking (std_msgs/Bool). FAILURE when the topic says false or
// has been silent for longer than `timeout` seconds (no social node: behave as before).
//
//   <IsPersonBlocking topic="/kelo/person_blocking" timeout="1.0"/>

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace kelo_bt_nodes
{

class IsPersonBlocking : public BT::ConditionNode
{
public:
  IsPersonBlocking(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf)
  {
    std::string topic = "/kelo/person_blocking";
    getInput("topic", topic);
    getInput("timeout", timeout_);
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      // depth 1: only the latest state counts
      topic, rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        blocking_ = msg->data;
        stamp_ = node_->now();
        received_ = true;
      },
      options);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/kelo/person_blocking", "Bool topic of the social node"),
      BT::InputPort<double>("timeout", 1.0, "Older messages count as 'not blocking' (s)"),
    };
  }

  BT::NodeStatus tick() override
  {
    // spin_all, not spin_some: on the first tick of an instance spin_some only collects the
    // subscription and returns without the waiting message
    executor_.spin_all(std::chrono::milliseconds(10));
    if (!received_ || (node_->now() - stamp_).seconds() > timeout_) {
      return BT::NodeStatus::FAILURE;
    }
    return blocking_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::Time stamp_;
  double timeout_{1.0};
  bool blocking_{false};
  bool received_{false};
};

}  // namespace kelo_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kelo_bt_nodes::IsPersonBlocking>("IsPersonBlocking");
}
