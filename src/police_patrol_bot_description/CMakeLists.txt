cmake_minimum_required(VERSION 3.8)
project(police_patrol_bot_description)

find_package(ament_cmake REQUIRED)

# Description-only package: URDF, RViz config, launch files.
# All kinematics / odometry run onboard the RP2040 (see pico/mecanum_base).
install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
