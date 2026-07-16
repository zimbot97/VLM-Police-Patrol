cmake_minimum_required(VERSION 3.8)
project(police_patrol_bot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Bringup package: launch files + EKF config.
# The robot model lives in police_patrol_bot_description; all kinematics /
# odometry run onboard the RP2040 (see pico/mecanum_base).
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

# /wheel_speeds -> /joint_states node (animates wheels in RViz)
install(PROGRAMS
  scripts/wheel_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
