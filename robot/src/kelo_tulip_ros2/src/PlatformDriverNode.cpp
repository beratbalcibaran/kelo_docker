// ROS 2 port of src/PlatformDriverROS.cpp (kelo_tulip, KELO Robotics GmbH, dual license LGPL-2.1/BSD).
// Changes from the ROS 1 original: rclcpp instead of roscpp, and the parameters fix_odom_rotation,
// cmd_vel_timeout_s and publish_tf.

#include "kelo_tulip_ros2/PlatformDriverNode.hpp"

#include <cmath>
#include <chrono>

#include "kelo_tulip_ros2/driver_logic.hpp"

using namespace std::chrono_literals;

namespace kelo_tulip_ros2
{

namespace
{
double norm(double x)
{
  const double TWO_PI = 2.0 * M_PI;
  while (x < -M_PI) x += TWO_PI;
  while (x > M_PI) x -= TWO_PI;
  return x;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

PlatformDriverNode::PlatformDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("platform_driver", options)
{
  nWheels_ = this->declare_parameter<int>("num_wheels", 0);
  if (nWheels_ <= 0) {
    RCLCPP_ERROR(get_logger(), "Missing/invalid num_wheels in config");
    throw std::runtime_error("Invalid num_wheels");
  }

  wheelConfigs_.resize(nWheels_);
  kelo::WheelData data{};
  data.enable = true;
  data.error = false;
  data.errorTimestamp = false;
  wheelData_.resize(nWheels_, data);

  readWheelModels();
  readWheelConfig();

  driver_ = new kelo::PlatformDriver(wheelConfigs_, wheelData_);

  double x;
  x = this->declare_parameter<double>("current_stop", -1.0);
  if (x >= 0) driver_->setCurrentStop(x);
  x = this->declare_parameter<double>("current_drive", -1.0);
  if (x >= 0) driver_->setCurrentDrive(x);
  x = this->declare_parameter<double>("vlin_max", -1.0);
  if (x >= 0) driver_->setMaxvlin(x);
  x = this->declare_parameter<double>("va_max", -1.0);
  if (x >= 0) driver_->setMaxva(x);
  x = this->declare_parameter<double>("vlin_acc_max", -1.0);
  if (x >= 0) driver_->setMaxvlinacc(x);
  x = this->declare_parameter<double>("vlin_dec_max", -1.0);
  if (x >= 0) driver_->setMaxvlindec(x);
  x = this->declare_parameter<double>("angle_acc_max", -1.0);
  if (x >= 0) driver_->setMaxangleacc(x);
  x = this->declare_parameter<double>("va_acc_max", -1.0);
  if (x >= 0) driver_->setMaxvaacc(x);
  x = this->declare_parameter<double>("va_dec_max", -1.0);
  if (x >= 0) driver_->setMaxvadec(x);
  x = this->declare_parameter<double>("pivot_vel_damping_gain", -1.0);
  if (x >= 0) driver_->setPivotVelocityDampingGain(x);

  bool activeByJoypad = this->declare_parameter<bool>("active_by_joypad", false);
  if (!activeByJoypad) {
    driver_->setCanChangeActive();
  }

  std::string device = this->declare_parameter<std::string>("device", "");
  if (device.empty()) {
    RCLCPP_ERROR(get_logger(), "Missing 'device' (EtherCAT network interface) in config");
    throw std::runtime_error("Missing device");
  }
  double delayRetry = this->declare_parameter<double>("start_retry_delay", 0.0);
  fixOdomRotation_ = this->declare_parameter<bool>("fix_odom_rotation", true);
  publishTf_ = this->declare_parameter<bool>("publish_tf", true);
  cmdWatchdog_ = std::make_unique<CmdWatchdog>(
    this->declare_parameter<double>("cmd_vel_timeout_s", 0.5));

  master_ = new kelo::EtherCATMaster(device, std::vector<kelo::EtherCATModule *>{driver_});
  while (!master_->initEthercat()) {
    if (delayRetry <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize EtherCAT on device '%s'", device.c_str());
      throw std::runtime_error("EtherCAT init failed");
    }
    RCLCPP_ERROR(get_logger(), "Failed to initialize EtherCAT, retrying in %.2f s", delayRetry);
    std::this_thread::sleep_for(std::chrono::duration<double>(delayRetry));
  }

  odomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  odomInitializedPublisher_ = this->create_publisher<std_msgs::msg::Empty>("odom_initialized", 10);
  imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  batteryPublisher_ = this->create_publisher<std_msgs::msg::Float32>("battery", 10);
  errorPublisher_ = this->create_publisher<std_msgs::msg::Int32>("error", 10);
  statusPublisher_ = this->create_publisher<std_msgs::msg::Int32>("status", 10);

  cmdVelSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&PlatformDriverNode::cmdVelCallback, this, std::placeholders::_1));
  resetSubscriber_ = this->create_subscription<std_msgs::msg::Empty>(
    "reset", 1, std::bind(&PlatformDriverNode::resetCallback, this, std::placeholders::_1));
  enableSubscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "wheels_enable", 1,
    std::bind(&PlatformDriverNode::enableCallback, this, std::placeholders::_1));

  odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  initializeEncoderValue();

  // same rate as the original ROS 1 loop (20 Hz)
  timer_ = this->create_wall_timer(50ms, std::bind(&PlatformDriverNode::step, this));

  RCLCPP_INFO(get_logger(), "KELO platform_driver (ROS2) initialized: %d wheels on %s",
    nWheels_, device.c_str());
  RCLCPP_INFO(get_logger(), "fix_odom_rotation=%s publish_tf=%s cmd_vel_timeout_s=%.2f",
    fixOdomRotation_ ? "true" : "false", publishTf_ ? "true" : "false",
    this->get_parameter("cmd_vel_timeout_s").as_double());
}

PlatformDriverNode::~PlatformDriverNode()
{
  // same order as the original TulipMain.cpp: the module (driver) first, then the master
  delete driver_;
  delete master_;
}

void PlatformDriverNode::readWheelModels()
{
  // This robot's configuration uses no named wheel_models, only x/y/a/ethercat_number per wheel.
  // If ever needed, read the nested parameter "wheel_models.<name>.*" here (this requires
  // automatically_declare_parameters_from_overrides).
}

void PlatformDriverNode::readWheelConfig()
{
  for (int i = 0; i < nWheels_; i++) {
    std::string prefix = "wheel" + std::to_string(i) + ".";

    kelo::WheelConfig config{};
    config.enable = true;
    config.reverseVelocity = true;

    config.ethercatNumber = this->declare_parameter<int>(prefix + "ethercat_number", 0);
    config.x = this->declare_parameter<double>(prefix + "x", 0.0);
    config.y = this->declare_parameter<double>(prefix + "y", 0.0);
    config.a = this->declare_parameter<double>(prefix + "a", 0.0);

    int reverseVelocity = this->declare_parameter<int>(prefix + "reverse_velocity", 1);
    config.reverseVelocity = (reverseVelocity != 0);

    wheelConfigs_[i] = config;
  }
}

void PlatformDriverNode::initializeEncoderValue()
{
  prev_left_enc_.resize(nWheels_, 0);
  prev_right_enc_.resize(nWheels_, 0);
  prev_pivot_enc_.resize(nWheels_, 0);
  prev_ts_.resize(nWheels_, 0);
  for (int i = 0; i < nWheels_; i++) {
    std::vector<double> enc = driver_->getEncoderValue(i);
    prev_left_enc_[i] = enc[0];
    prev_right_enc_[i] = enc[1];
  }
}

void PlatformDriverNode::step()
{
  checkCmdWatchdog();
  checkAndPublishSmartWheelStatus();

  double vx, vy, va, encDisplacement, dt;
  calculateRobotVelocity2(vx, vy, va, encDisplacement, dt);
  calculateRobotPose2(vx, vy, va, dt);

  publishOdometry(vx, vy, va);
  broadcastOdomTransform();
  publishBattery();
  publishIMU();
}

void PlatformDriverNode::checkAndPublishSmartWheelStatus()
{
  int status = driver_->getDriverStatus();
  int error = (status & 0xffffff00);

  std_msgs::msg::Int32 statusMsg;
  statusMsg.data = status;
  statusPublisher_->publish(statusMsg);

  if (error) {
    debugMode_ = true;
    std_msgs::msg::Int32 errorMsg;
    errorMsg.data = status;
    errorPublisher_->publish(errorMsg);
  } else if (debugMode_) {
    debugMode_ = false;
    std_msgs::msg::Int32 errorMsg;
    errorMsg.data = 0;
    errorPublisher_->publish(errorMsg);
  }
}

void PlatformDriverNode::calculateRobotVelocity2(
  double & vx, double & vy, double & va, double & encDisplacement, double & dt)
{
  dt = 0.05;
  std::vector<double> rx(nWheels_, 0.0);
  std::vector<double> ry(nWheels_, 0.0);

  vx = 0;
  vy = 0;
  va = 0;
  encDisplacement = 0;
  std::vector<WheelPos> wheels(nWheels_);

  for (int i = 0; i < nWheels_; i++) {
    volatile txpdo1_t * swData = driver_->getWheelProcessData(i);
    uint64_t sensor_ts;
    float encoder_1, encoder_2, encoder_pivot, velocity_1, velocity_2, velocity_pivot;
    int repcnt = 3;
    do {
      sensor_ts = swData->sensor_ts;
      encoder_1 = swData->encoder_1;
      encoder_2 = swData->encoder_2;
      encoder_pivot = swData->encoder_pivot;
      velocity_1 = swData->velocity_1;
      velocity_2 = swData->velocity_2;
      velocity_pivot = swData->velocity_pivot;
    } while ((sensor_ts != swData->sensor_ts) && --repcnt);

    double delta_ts = (sensor_ts - prev_ts_[i]) * 0.000000001;
    prev_ts_[i] = sensor_ts;

    double wl, wr, wp;
    if ((delta_ts <= 0.0) || (delta_ts > 0.1)) {
      wl = velocity_1;
      wr = -velocity_2;
      wp = velocity_pivot;
    } else {
      wl = norm(encoder_1 - prev_left_enc_[i]) / delta_ts;
      wr = -norm(encoder_2 - prev_right_enc_[i]) / delta_ts;
      wp = norm(encoder_pivot - prev_pivot_enc_[i]) / delta_ts;
      if (std::fabs(velocity_pivot) > 10 * M_PI) wp = velocity_pivot;
      dt = delta_ts;
    }
    prev_left_enc_[i] = encoder_1;
    prev_right_enc_[i] = encoder_2;
    prev_pivot_enc_[i] = encoder_pivot;

    if (wheelConfigs_[i].reverseVelocity) {
      wl *= -1.0;
      wr *= -1.0;
    }
    double theta = norm(encoder_pivot - wheelConfigs_[i].a);
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);
    encDisplacement = (wl + wr) * delta_ts;

    double cx = 0.5 * r_w_ * (wl + wr);
    double cy = wp * s_w_;
    rx[i] = (cx * cos_theta) - (cy * sin_theta);
    ry[i] = (cx * sin_theta) + (cy * cos_theta);
    wheels[i] = {wheelConfigs_[i].x, wheelConfigs_[i].y};
  }

  const PlatformVelocity v = platformVelocity(wheels, rx, ry, fixOdomRotation_);
  vx = v.vx;
  vy = v.vy;
  va = v.va;
}

void PlatformDriverNode::calculateRobotPose2(double vx, double vy, double va, double dt)
{
  double dx = vx * dt;
  double dy = vy * dt;
  double a_average = norm(odoma_ + (0.5 * va * dt));
  double sin_a = std::sin(a_average);
  double cos_a = std::cos(a_average);
  odomx_ += (dx * cos_a) - (dy * sin_a);
  odomy_ += (dx * sin_a) + (dy * cos_a);
  odoma_ = norm(odoma_ + (va * dt));
}

void PlatformDriverNode::publishOdometry(double vx, double vy, double va)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.covariance[0] = 1e-3;
  odom.pose.covariance[7] = 1e-3;
  odom.pose.covariance[14] = 1e6;
  odom.pose.covariance[21] = 1e6;
  odom.pose.covariance[28] = 1e6;
  odom.pose.covariance[35] = 1e3;
  odom.twist.covariance = odom.pose.covariance;
  odom.pose.pose.position.x = odomx_;
  odom.pose.pose.position.y = odomy_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternionFromYaw(odoma_);
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = va;
  odomPublisher_->publish(odom);
}

void PlatformDriverNode::broadcastOdomTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = odomx_;
  t.transform.translation.y = odomy_;
  t.transform.translation.z = 0.0;
  t.transform.rotation = quaternionFromYaw(odoma_);
  if (publishTf_) {
    odom_broadcaster_->sendTransform(t);
  }
}

void PlatformDriverNode::publishBattery()
{
  double volt = 0;
  for (size_t i = 0; i < wheelConfigs_.size(); i++) {
    double x = driver_->getWheelProcessData(i)->voltage_bus;
    if (x > volt) volt = x;
  }
  std_msgs::msg::Float32 msg;
  msg.data = volt;
  batteryPublisher_->publish(msg);
}

void PlatformDriverNode::publishIMU()
{
  for (size_t i = 0; i < wheelConfigs_.size(); i++) {
    txpdo1_t * swData = driver_->getWheelProcessData(i);
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = this->now();
    imu.angular_velocity.x = swData->gyro_x;
    imu.angular_velocity.y = swData->gyro_y;
    imu.angular_velocity.z = swData->gyro_z;
    imu.linear_acceleration.x = swData->accel_x;
    imu.linear_acceleration.y = swData->accel_y;
    imu.linear_acceleration.z = swData->accel_z;
    imuPublisher_->publish(imu);
  }
}

void PlatformDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const bool moving = msg->linear.x != 0.0 || msg->linear.y != 0.0 || msg->angular.z != 0.0;
  cmdWatchdog_->onCommand(watchdogClock_.now().seconds(), moving);
  driver_->setTargetVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
}

void PlatformDriverNode::checkCmdWatchdog()
{
  if (cmdWatchdog_->expired(watchdogClock_.now().seconds())) {
    driver_->setTargetVelocity(0.0, 0.0, 0.0);
    RCLCPP_WARN(get_logger(), "No /cmd_vel for %.2f s while moving: velocity set to zero",
      this->get_parameter("cmd_vel_timeout_s").as_double());
  }
}

void PlatformDriverNode::resetCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  RCLCPP_INFO(get_logger(), "Reset error flags.");
  driver_->resetErrorFlags();
}

void PlatformDriverNode::enableCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  driver_->setWheelsEnable(msg->data);
}

}  // namespace kelo_tulip_ros2
