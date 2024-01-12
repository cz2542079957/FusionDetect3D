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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)



###################################################################################
# QT6
###################################################################################
#防止qt相关文件找不到
set(QT_PATH "/home/chenzhen/dev/6.6.1/gcc_64")
list(APPEND CMAKE_PREFIX_PATH ${QT_PATH})
set(QT_INCLUDE ${QT_PATH}/include)
include_directories(
    ${QT_INCLUDE}
)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
#引入Qt6相关包
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)



###################################################################################
#
###################################################################################
# 文件路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/view
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2/lidar
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ui
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/view
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ros2
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/ros2/lidar
    ${CMAKE_CURRENT_SOURCE_DIR}/res

)
list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "src/ui")
set(HPP_FILE
    include/mainwindow.h
    include/view/pointcloudwidget.h
    include/view/cameracontroller.h
    include/ros2/devicecontroller.h
    include/ros2/lidar/lidarnode.h
)
set(CPP_FILE
    main.cpp
    src/mainwindow.cpp
    src/view/pointcloudwidget.cpp
    src/view/cameracontroller.cpp
    src/ros2/devicecontroller.cpp
    src/ros2/lidar/lidarnode.cpp
)
set(SOURCE
    ${HPP_FILE}
    ${CPP_FILE}
    src/ui/mainwindow.ui

    res/res.qrc
)
add_executable(main ${SOURCE})
# ros2 依赖
set(ROS2_DEPENDENCIES rclcpp rcl std_msgs sensor_msgs)
# qt6 依赖
set(QT6_DEPENDENCIES Qt6::Widgets Qt6::OpenGLWidgets)
target_link_libraries(main
    ${QT6_DEPENDENCIES}
)
ament_target_dependencies(main
    ${ROS2_DEPENDENCIES}
)

ament_package()
