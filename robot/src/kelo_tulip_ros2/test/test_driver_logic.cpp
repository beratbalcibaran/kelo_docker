// Tests of driver_logic.hpp. No ROS and no robot needed, from the folder robot/src/kelo_tulip_ros2:
//   g++ -std=c++17 -Iinclude test/test_driver_logic.cpp -o /tmp/test_driver_logic && /tmp/test_driver_logic

#include <cmath>
#include <cstdio>
#include <vector>

#include "kelo_tulip_ros2/driver_logic.hpp"

using kelo_tulip_ros2::CmdWatchdog;
using kelo_tulip_ros2::PlatformVelocity;
using kelo_tulip_ros2::WheelPos;
using kelo_tulip_ros2::platformVelocity;

static int failures = 0;

static void check(const char * name, double got, double want)
{
  const bool ok = std::fabs(got - want) < 1e-9;
  std::printf("%s %-46s got %9.5f want %9.5f\n", ok ? "ok  " : "FAIL", name, got, want);
  failures += ok ? 0 : 1;
}

// wheel ground velocities of a rigid platform moving at (vx, vy, va) at base_link
static void rigid(
  const std::vector<WheelPos> & w, double vx, double vy, double va,
  std::vector<double> & rx, std::vector<double> & ry)
{
  rx.clear();
  ry.clear();
  for (const auto & p : w) {
    rx.push_back(vx - va * p.y);
    ry.push_back(vy + va * p.x);
  }
}

int main()
{
  const std::vector<WheelPos> kelo{{0.233, 0.1165}, {0.233, -0.1165}};
  const std::vector<WheelPos> centred{{0.2, 0.2}, {0.2, -0.2}, {-0.2, 0.2}, {-0.2, -0.2}};
  std::vector<double> rx, ry;
  PlatformVelocity v;

  rigid(kelo, 0.0, 0.0, 0.6, rx, ry);
  v = platformVelocity(kelo, rx, ry, false);
  check("original, turn on the spot: va = w / 5", v.va, 0.6 / 5.0);
  check("original, turn on the spot: false vy = 0.233 w", v.vy, 0.233 * 0.6);
  v = platformVelocity(kelo, rx, ry, true);
  check("fixed, turn on the spot: va", v.va, 0.6);
  check("fixed, turn on the spot: vx", v.vx, 0.0);
  check("fixed, turn on the spot: vy", v.vy, 0.0);

  rigid(kelo, 0.25, -0.1, 0.0, rx, ry);
  v = platformVelocity(kelo, rx, ry, true);
  check("fixed, straight: vx", v.vx, 0.25);
  check("fixed, straight: vy", v.vy, -0.1);
  check("fixed, straight: va", v.va, 0.0);
  v = platformVelocity(kelo, rx, ry, false);
  check("original, straight: vx unchanged", v.vx, 0.25);
  check("original, straight: va unchanged", v.va, 0.0);

  rigid(kelo, 0.2, 0.05, -0.4, rx, ry);
  v = platformVelocity(kelo, rx, ry, true);
  check("fixed, mixed motion: vx", v.vx, 0.2);
  check("fixed, mixed motion: vy", v.vy, 0.05);
  check("fixed, mixed motion: va", v.va, -0.4);

  rigid(centred, 0.2, 0.05, -0.4, rx, ry);
  v = platformVelocity(centred, rx, ry, false);
  check("centred wheels, original: va", v.va, -0.4);
  check("centred wheels, original: vy", v.vy, 0.05);
  v = platformVelocity(centred, rx, ry, true);
  check("centred wheels, fixed: va", v.va, -0.4);
  check("centred wheels, fixed: vy", v.vy, 0.05);

  CmdWatchdog dog(0.5);
  check("watchdog: silent before the first command", dog.expired(100.0), 0.0);
  dog.onCommand(100.0, true);
  check("watchdog: moving, 0.4 s since the command", dog.expired(100.4), 0.0);
  check("watchdog: moving, 0.5 s since the command", dog.expired(100.5), 1.0);
  check("watchdog: fires only once", dog.expired(100.6), 0.0);
  dog.onCommand(101.0, true);
  dog.onCommand(101.4, true);
  check("watchdog: refreshed by a new command", dog.expired(101.8), 0.0);
  dog.onCommand(102.0, false);
  check("watchdog: not armed by a zero command", dog.expired(110.0), 0.0);
  CmdWatchdog off(0.0);
  off.onCommand(100.0, true);
  check("watchdog: timeout 0 turns it off", off.expired(200.0), 0.0);

  std::printf("%s\n", failures ? "FAILED" : "all passed");
  return failures ? 1 : 0;
}
