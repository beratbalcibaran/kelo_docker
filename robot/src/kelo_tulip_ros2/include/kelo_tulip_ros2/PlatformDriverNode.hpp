// ROS 2 port of the kelo_tulip node (original: KELO Robotics GmbH, dual license LGPL-2.1/BSD). The core
// classes (EtherCATMaster, PlatformDriver, VelocityPlatformController...) are unchanged; only the ROS
// layer is rewritten as an rclcpp node, following src/PlatformDriverROS.cpp (ROS 1).
//
// Not ported from the ROS 1 original (not used by this robot's configuration, 2 wheels):
//  - the robile_master_battery module (RobileMasterBatteryROS)
//  - joypad support (/joy)
//  - the raw diagnostic topic "wheels_input" (it would need a custom KeloDrivesInput message)

#ifndef KELO_TULIP_ROS2__PLATFORM_DRIVER_NODE_HPP_
#define KELO_TULIP_ROS2__PLATFORM_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kelo_tulip/EtherCATMaster.h"
#include "kelo_tulip/PlatformDriver.h"
#include "kelo_tulip/WheelConfig.h"
#include "kelo_tulip/WheelModel.h"
#include "kelo_tulip_ros2/driver_logic.hpp"

namespace kelo_tulip_ros2
{

class PlatformDriverNode : public rclcpp::Node
{
public:
  explicit PlatformDriverNode(const rclcpp::NodeOptions & options);
  ~PlatformDriverNode() override;

private:
  void readWheelModels();
  void readWheelConfig();
  void initializeEncoderValue();

  void step();
  void checkAndPublishSmartWheelStatus();
  void calculateRobotVelocity2(
    double & vx, double & vy, double & va, double & encDisplacement, double & dt);
  void calculateRobotPose2(double vx, double vy, double va, double dt);
  void publishOdometry(double vx, double vy, double va);
  void broadcastOdomTransform();
  void publishBattery();
  void publishIMU();

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void resetCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void enableCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void checkCmdWatchdog();

  // core driver (no ROS, unchanged from the original kelo_tulip)
  kelo::PlatformDriver * driver_ = nullptr;
  kelo::EtherCATMaster * master_ = nullptr;

  std::vector<kelo::WheelConfig> wheelConfigs_;
  std::vector<kelo::WheelData> wheelData_;
  std::map<std::string, kelo::WheelModel> wheelModels_;

  int nWheels_ = 0;
  double s_w_ = 0.01;
  double d_w_ = 0.0775;
  double s_d_ratio_ = s_w_ / d_w_;
  double r_w_ = 0.0524;

  bool debugMode_ = false;
  bool fixOdomRotation_ = true;
  bool publishTf_ = true;
  std::unique_ptr<CmdWatchdog> cmdWatchdog_;
  rclcpp::Clock watchdogClock_{RCL_STEADY_TIME};

  std::vector<double> prev_left_enc_;
  std::vector<double> prev_right_enc_;
  std::vector<double> prev_pivot_enc_;
  std::vector<uint64_t> prev_ts_;
  double odomx_ = 0.0;
  double odomy_ = 0.0;
  double odoma_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr odomInitializedPublisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batteryPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr errorPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr statusPublisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resetSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr enableSubscriber_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kelo_tulip_ros2

#endif  // KELO_TULIP_ROS2__PLATFORM_DRIVER_NODE_HPP_
