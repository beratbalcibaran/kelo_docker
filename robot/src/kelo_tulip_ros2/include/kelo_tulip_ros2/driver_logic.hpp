// Pure calculations of the driver node (no ROS, no EtherCAT), tested by test/test_driver_logic.cpp.

#ifndef KELO_TULIP_ROS2__DRIVER_LOGIC_HPP_
#define KELO_TULIP_ROS2__DRIVER_LOGIC_HPP_

#include <cmath>
#include <vector>

namespace kelo_tulip_ros2
{

struct WheelPos
{
  double x;
  double y;
};

struct PlatformVelocity
{
  double vx;
  double vy;
  double va;
};

// Platform velocity at base_link from the ground velocity (rx, ry) of each wheel, in base_link axes.
//
// fix_rotation = false is the original kelo_tulip calculation. It assumes that the wheels are centred
// on base_link. On a platform with all wheels on one side (2 wheels at x = 0.233, y = +-0.1165) it
// reads the yaw rate as w * y^2 / (x^2 + y^2) = w / 5 and adds a false lateral speed of x * w.
//
// fix_rotation = true is the rigid-body solution, valid for any wheel positions: the yaw rate comes
// from the wheel velocities around the centre of the wheels, and the velocity of that centre is then
// moved to base_link. With wheels centred on base_link both calculations give the same result.
inline PlatformVelocity platformVelocity(
  const std::vector<WheelPos> & wheels, const std::vector<double> & rx, const std::vector<double> & ry,
  bool fix_rotation)
{
  PlatformVelocity out{0.0, 0.0, 0.0};
  const size_t n = wheels.size();
  if (n == 0) {
    return out;
  }
  double xc = 0.0;
  double yc = 0.0;
  for (size_t i = 0; i < n; i++) {
    out.vx += rx[i];
    out.vy += ry[i];
    xc += wheels[i].x;
    yc += wheels[i].y;
  }
  out.vx /= n;
  out.vy /= n;
  xc /= n;
  yc /= n;

  if (fix_rotation) {
    double num = 0.0;
    double den = 0.0;
    for (size_t i = 0; i < n; i++) {
      const double px = wheels[i].x - xc;
      const double py = wheels[i].y - yc;
      num += px * (ry[i] - out.vy) - py * (rx[i] - out.vx);
      den += px * px + py * py;
    }
    if (den > 0.0) {
      out.va = num / den;
    }
    out.vx += out.va * yc;
    out.vy -= out.va * xc;
    return out;
  }

  double v_sum = 0.0;
  double d_sum = 0.0;
  for (size_t i = 0; i < n; i++) {
    const double d = std::sqrt(wheels[i].x * wheels[i].x + wheels[i].y * wheels[i].y);
    if (d > 0.0) {
      const double cos_gamma = wheels[i].x / d;
      const double sin_gamma = wheels[i].y / d;
      v_sum += (ry[i] - out.vy) * cos_gamma - (rx[i] - out.vx) * sin_gamma;
      d_sum += d;
    }
  }
  if (d_sum > 0.0) {
    out.va = v_sum / d_sum;
  }
  return out;
}

// Stops the robot when the velocity commands stop arriving (computer crashed, cable unplugged): the
// platform controller otherwise keeps the last velocity for ever. Armed only by a non-zero command,
// so a robot that was already told to stand still raises no alarm. timeout_s <= 0 turns it off.
class CmdWatchdog
{
public:
  explicit CmdWatchdog(double timeout_s)
  : timeout_s_(timeout_s) {}

  void onCommand(double now_s, bool moving)
  {
    last_s_ = now_s;
    armed_ = moving;
  }

  // true once, at the moment the timeout passes: the caller then commands zero velocity
  bool expired(double now_s)
  {
    if (timeout_s_ <= 0.0 || !armed_ || now_s - last_s_ < timeout_s_) {
      return false;
    }
    armed_ = false;
    return true;
  }

private:
  double timeout_s_;
  double last_s_ = 0.0;
  bool armed_ = false;
};

}  // namespace kelo_tulip_ros2

#endif  // KELO_TULIP_ROS2__DRIVER_LOGIC_HPP_
