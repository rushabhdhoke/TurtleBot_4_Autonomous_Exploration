cmake_minimum_required(VERSION 3.8)
project(demo_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/record_bag.sh
  scripts/replay_bag.sh
  scripts/auto_undock.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install exploration algorithms
install(PROGRAMS
  algorithms/frontier_explorer.py
  # Future algorithms will be added here:
  # algorithms/astar_explorer.py
  # algorithms/bfs_explorer.py
  # etc.
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
