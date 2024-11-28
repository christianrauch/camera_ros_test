cmake_minimum_required(VERSION 3.18)
project(camera_ros_test)

find_package(camera_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libcamera REQUIRED libcamera>=0.1)

find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_test_param test_param.cpp)
target_include_directories(${PROJECT_NAME}_test_param PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_test_param
    rclcpp
    rclcpp_components
    rcl_interfaces
    camera_ros
)

target_include_directories(${PROJECT_NAME}_test_param PUBLIC ${libcamera_INCLUDE_DIRS})

# link the 'camera_component' via '--no-as-needed' to find all plugins automatically via the ClassLoader
target_link_libraries(${PROJECT_NAME}_test_param "-Wl,--no-as-needed" ${libcamera_LINK_LIBRARIES} camera_ros::camera_component)
