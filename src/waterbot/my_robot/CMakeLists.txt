cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找必要的套件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(controller_manager REQUIRED)


set(AMENT_CMAKE_SYMLINK_INSTALL FALSE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 安裝 launch / config / description 資料夾
install(
  DIRECTORY
    launch
    config
    description
  DESTINATION share/${PROJECT_NAME}
)

# 安裝 Python Node
install(
  PROGRAMS
    my_robot/move.py
    my_robot/serial_comm.py
    my_robot/keyboard.py
    my_robot/camera.py
    my_robot/yolorun.py
    my_robot/rplidar.py
    my_robot/obstacle1.py
    my_robot/yolorun2.py
    my_robot/partrol3.py
    my_robot/decision4.py
    my_robot/hcsr04.py
    my_robot/encoder.py
    my_robot/ws.py
    my_robot/enocder2.py
    my_robot/water.py
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
