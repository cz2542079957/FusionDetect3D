cmake_minimum_required(VERSION 3.8)
project(main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###################################################################################
# ROS2
###################################################################################
set(ROS2_INCLUDE /opt/ros/humble/include)
include_directories(
    ${ROS2_INCLUDE}
)
# 引入ros2相关包
list(APPEND CMAKE_PREFIX_PATH "/home/chenzhen/Desktop/ros2_ws/install/message")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message REQUIRED)


###################################################################################
# QT6
###################################################################################
#防止qt相关文件找不到
set(QT_PATH "/home/chenzhen/dev/6.6.1/gcc_64")
list(APPEND CMAKE_PREFIX_PATH ${QT_PATH})
set(QT_INCLUDE ${QT_PATH}/include)
include_directories(
    ${QT_INCLUDE}
    ${EIGEN3_INCLUDE_DIR}
)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
#引入Qt6相关包
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)
find_package(Eigen3 REQUIRED)


###################################################################################
#
###################################################################################
# 文件路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/view
    ${CMAKE_CURRENT_SOURCE_DIR}/src/view/pointCloud
    ${CMAKE_CURRENT_SOURCE_DIR}/src/view/infoTree
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ui
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/view
    ${CMAKE_CURRENT_SOURCE_DIR}/include/view/pointCloud
    ${CMAKE_CURRENT_SOURCE_DIR}/include/view/infoTree
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ros2
    ${CMAKE_CURRENT_SOURCE_DIR}/res
)
list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "src/ui")

FILE(GLOB SOURCE
    "include/*.h"
    "include/view/*.h"
    "include/view/infoTree/*.h"
    "include/view/pointCloud/*.h"
    "include/ros2/*.h"
    "src/*.cpp"
    "src/view/*.cpp"
    "src/view/infoTree/*.cpp"
    "src/view/pointCloud/*.cpp"
    "src/ros2/*.cpp"
    main.cpp
    res/res.qrc
)
add_executable(main ${SOURCE})
# ros2 依赖
set(ROS2_DEPENDENCIES rclcpp rcl std_msgs message)
# qt6 依赖
set(QT6_DEPENDENCIES Qt6::Widgets Qt6::OpenGLWidgets)
target_link_libraries(main
    ${QT6_DEPENDENCIES}
)
ament_target_dependencies(main
    ${ROS2_DEPENDENCIES}
)

ament_package()
