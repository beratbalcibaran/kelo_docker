#include <cmath>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"

namespace kelo_bt_nodes
{

class SkipPathStart : public BT::SyncActionNode
{
public:
  SkipPathStart(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Path to shorten"),
      BT::InputPort<double>("distance", 0.4, "Metres removed from the start of the path"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path without its start"),
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput("input_path", path)) {
      return BT::NodeStatus::FAILURE;
    }
    double distance = 0.4;
    getInput("distance", distance);
    size_t first = 0;
    double travelled = 0.0;
    while (first + 1 < path.poses.size() && travelled < distance) {
      const auto & a = path.poses[first].pose.position;
      const auto & b = path.poses[first + 1].pose.position;
      travelled += std::hypot(b.x - a.x, b.y - a.y);
      ++first;
    }
    nav_msgs::msg::Path out;
    out.header = path.header;

    if (!path.poses.empty()) {
      out.poses.assign(path.poses.begin() + first, path.poses.end());
    }
    setOutput("output_path", out);
    return BT::NodeStatus::SUCCESS;
  }
};

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kelo_bt_nodes::SkipPathStart>("SkipPathStart");
}
