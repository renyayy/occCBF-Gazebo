cmake_minimum_required(VERSION 3.8)
project(occlusion_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# launch/ ディレクトリ全体を 'share/occlusion_sim' にインストール
install(
  DIRECTORY launch worlds models urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_config.py
  scripts/obstacle_controller_base.py
  scripts/cbf_wrapper_node.py
  scripts/multi_obstacle_controller.py
  scripts/single_obstacle_controller.py
  scripts/sensor_visualizer_node.py
  scripts/cmd_vel_converter.py
  scripts/scenario_obstacle_controller.py
  scripts/run_batch_experiment.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/scenarios
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY safe_control
  DESTINATION lib
)

ament_package()
