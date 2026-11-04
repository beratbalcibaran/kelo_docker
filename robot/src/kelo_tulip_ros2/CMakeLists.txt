cmake_minimum_required(VERSION 3.8)
project(kelo_tulip_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(SOEM_INCLUDE_DIRS
  include
  include/kelo_tulip/soem
  include/kelo_tulip/soem/osal
  include/kelo_tulip/soem/osal/linux
  include/kelo_tulip/soem/oshw/linux
)
include_directories(${SOEM_INCLUDE_DIRS})

# --- SOEM (EtherCAT stack, plain C, unchanged from kelo_tulip) ---
add_library(soem STATIC
  src/soem/ethercatbase.c
  src/soem/ethercatcoe.c
  src/soem/ethercatconfig.c
  src/soem/ethercatdc.c
  src/soem/ethercatfoe.c
  src/soem/ethercatmain.c
  src/soem/ethercatprint.c
  src/soem/ethercatsoe.c
  src/soem/osal/linux/osal.c
  src/soem/oshw/linux/nicdrv.c
  src/soem/oshw/linux/oshw.c
)
target_compile_options(soem PRIVATE -Wno-unused-parameter -Wno-unused-variable)

# --- kelo_tulip core library (no ROS, unchanged) ---
add_library(kelo_tulip_core
  src/Utils.cpp
  src/VelocityPlatformController.cpp
  src/EtherCATMaster.cpp
  src/EtherCATModule.cpp
  src/PlatformDriver.cpp
)
target_link_libraries(kelo_tulip_core soem pthread Boost::thread)

# --- ROS 2 node ---
add_executable(platform_driver_node
  src/PlatformDriverNode.cpp
  src/platform_driver_main.cpp
)
target_link_libraries(platform_driver_node kelo_tulip_core)
ament_target_dependencies(platform_driver_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros)

# Raw-socket EtherCAT access without running as root (setcap needs privileges at build time).
option(USE_SETCAP "Apply cap_net_raw+cap_sys_nice to platform_driver_node" ON)
if(USE_SETCAP)
  add_custom_command(TARGET platform_driver_node POST_BUILD
    COMMAND sudo setcap cap_sys_nice,cap_net_raw+ep $<TARGET_FILE:platform_driver_node>
    COMMENT "Setting cap_net_raw+cap_sys_nice on platform_driver_node"
  )
endif()

install(TARGETS platform_driver_node kelo_tulip_core soem
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS platform_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
