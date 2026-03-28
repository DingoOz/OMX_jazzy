cmake_minimum_required(VERSION 3.8)
project(omx_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPosition.srv"
  "srv/MoveJoints.srv"
  "srv/GripperControl.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python executables
install(PROGRAMS
  omx_control/ik_controller.py
  omx_control/fk_controller.py
  omx_control/gripper_controller.py
  omx_control/teach_playback.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package (as data, not as ament_python_install_package to avoid conflict)
install(DIRECTORY omx_control/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
