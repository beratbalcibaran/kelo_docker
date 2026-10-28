// SkipPathStart: the path without its first `distance` metres (measured along the path). Used before
// IsPathValid on the path ahead of the robot: IsPathValid also rejects inscribed cells, and a robot that
// stands close to a wall (or got there passing a person) sits in such cells. Then every new path was
// invalid from its first pose and the robot never left. What is closer than `distance` is the
// collision monitor's (stop zone 0.76 m from the centre).
//
//   <SkipPathStart input_path="{path_ahead}" output_path="{path_ahead}" distance="0.4"/>

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
    // a path shorter than `distance` keeps its last pose: IsPathValid needs at least one
    if (!path.poses.empty()) {
      out.poses.assign(path.poses.begin() + first, path.poses.end());
    }
    setOutput("output_path", out);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace kelo_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kelo_bt_nodes::SkipPathStart>("SkipPathStart");
}
