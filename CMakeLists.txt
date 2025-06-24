cmake_minimum_required(VERSION 3.5...3.22)

project(hotdog_controller)

# 防止源码被更改和源码目录内编译
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

# 基本构建设置
set(CMAKE_COLOR_MAKEFILE ON)
set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_CXX_STANDARD 14)

option(BUILD_SIMULATOR "Do not build simulator" ON)
option(BUILD_HARDWARE "Build hot dog hardware driver" ON)
option(BUILD_CONTROL "Build hot dog controller" ON)
option(ONBOARD_BUILD "Use compiler flags for computer on robot" OFF)
option(BUILD_FACTORY "Build production test tools for factory" OFF)
option(BUILD_ROS "Build hot dog for ROS2" ON)

# Find target processor type
if(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*")
  set(X86_64 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|i386.*|x86.*")
  set(X86 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*|arm64.*|ARM64.*)")
  set(AARCH64 1)
endif()

if(ONBOARD_BUILD)
  add_compile_definitions(ONBOARD_BUILD=1)
  set(CMAKE_CXX_FLAGS "-O3 -s -no-pie -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wsign-promo -Wno-format-nonliteral -Wno-strict-aliasing \
    -static-libstdc++")
  set(CMAKE_C_FLAGS "-O3 -ggdb -std=gnu99 -I. -static-libstdc++")
  message("**** Onboard Build enabled ****")
else()
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall -Wno-deprecated-copy \
      -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
      -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
      -Wno-format-nonliteral -Wsign-promo -Wno-strict-aliasing -faligned-new")
    set(CMAKE_C_FLAGS "-O3 -ggdb -march=native -std=gnu99 -I.")
  elseif(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
    set(CMAKE_CXX_FLAGS "-O3 -ggdb -Wall \
      -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
      -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
      -Wno-format-nonliteral -Wsign-promo -Wno-strict-aliasing")
    set(CMAKE_C_FLAGS "-O3 -ggdb -std=gnu99 -I.")
    include_directories("/usr/local/include/")
  endif()
  message("**** Onboard Build disabled ****")
endif()

# package for ROS2 colcon build
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sopu_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
pluginlib_export_plugin_description_file(controller_interface hotdog_controller_plugin.xml)

set(ros_dependencies
  rclcpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  sopu_msgs
  visualization_msgs
  pluginlib
  controller_interface
  generate_parameter_library
)

# =================== 第三方库 ===================
find_package(Eigen3 3.4.0 EXACT REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp 0.7.0 EXACT REQUIRED)

set(third_party_dependencies
  Eigen3
  glog::glog
  yaml-cpp
)

# =================== common模块合并 ===================
# 生成Configuration.h
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/common/config.h.cmake
  ${CMAKE_BINARY_DIR}/Configuration.h)

# common include路径
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  "${CMAKE_SOURCE_DIR}/common/include"
  "${CMAKE_SOURCE_DIR}/common/third_party/ParamHandler"
  "${CMAKE_SOURCE_DIR}/common/third_party/rapidjson"
  "${CMAKE_SOURCE_DIR}/common/third_party/cpptoml/include"
  "${CMAKE_SOURCE_DIR}/../../tita_bringup/include"
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/include # 关键：添加自动生成的参数头文件路径
)

# generate parameter listener hpp file
generate_parameter_library(hotdog_controller_parameters
  config/hotdog_controller_parameters.yaml
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

file(GLOB_RECURSE common_sources "common/src/*.cpp")

# 添加common第三方子库
add_subdirectory(common/third_party/ParamHandler)
add_subdirectory(common/third_party/cpptoml)

# 生成common库
add_library(common SHARED ${common_sources})
target_link_libraries(common
  pthread
  param_handler
  cpptoml
)

install(
  TARGETS common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =================== control库相关 ===================
file(GLOB_RECURSE control_sources "control/src/*.cpp")
file(GLOB_RECURSE third_party_sources "control/third_party/*.cpp")
file(GLOB_RECURSE hardware_sources "hardware/src/*.cpp")
file(GLOB_RECURSE factory_sources "factory/src/*.cpp")
file(GLOB_RECURSE simulator_sources "simulator/src/*.cpp")

include_directories(
  "control/include"
  "control/include/control"
  "control/third_party"
  "ros_bridge/include"
  "common/include"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  SYSTEM "control/third_party/qpOASES/include"
  "${CMAKE_SOURCE_DIR}/control/include"
  "${CMAKE_SOURCE_DIR}/common/third_party"
  "${CMAKE_SOURCE_DIR}/ros_bridge/include"
  ${GLOG_INCLUDE_DIRS}
)

# control库
add_library(control SHARED ${control_sources} ${third_party_sources} ${factory_sources} ${simulator_sources})
target_link_libraries(control
  pthread
  param_handler
  qpOASES
  Goldfarb_Optimizer
  common
  glog::glog
)
target_link_libraries(control hotdog_controller_parameters)
ament_target_dependencies(control controller_interface nav_msgs tf2 tf2_geometry_msgs sopu_msgs)
if(CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(control rt)
endif()

install(
  TARGETS control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_subdirectory(control/third_party)

# =================== hotdog_controller 插件库（ROS2控制器插件） ===================

file(GLOB hotdog_controller_plugin_sources
  "ros_bridge/src/*.cpp"
)

ament_auto_add_library(hotdog_controller SHARED
  ${hotdog_controller_plugin_sources}
)

target_link_libraries(hotdog_controller
  control
  common
  hotdog_controller_parameters
  ${GLOG_LIBRARIES}
  # 如有其他依赖库也可加在这里
)

ament_target_dependencies(hotdog_controller
  controller_interface
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# 导出插件描述文件，确保 hotdog_controller_plugin.xml 在工程根目录下
pluginlib_export_plugin_description_file(controller_interface hotdog_controller_plugin.xml)

install(TARGETS hotdog_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
TARGETS Goldfarb_Optimizer
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

install(
  TARGETS common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS param_handler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =================== 安装 launch 和 config 目录 ===================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
