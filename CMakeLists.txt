cmake_minimum_required(VERSION 3.5...3.22)

# 解决OpenGL策略警告
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

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

option(ONBOARD_BUILD "Use compiler flags for computer on robot" OFF)
option(BUILD_FACTORY "Build production test tools for factory" OFF)
option(BUILD_ROS "Build hot dog for ROS2" ON)



# 识别处理器架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*")
  set(X86_64 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|i386.*|x86.*")
  set(X86 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*|arm64.*|ARM64.*)")
  set(AARCH64 1)
endif()

# 编译选项
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
    include_directories("/usr/local/include/") # lcm includes
  endif()
  message("**** Onboard Build disabled ****")
endif()

# ROS2支持
if(BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_auto REQUIRED)
  find_package(controller_interface REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(generate_parameter_library REQUIRED)
endif()

find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(controller_interface hotdog_controller_plugin.xml)

# =================== common模块合并 ===================
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
configure_file(${CMAKE_SOURCE_DIR}/common/config.h.cmake
  ${CMAKE_BINARY_DIR}/Configuration.h)

# 查找依赖
find_package(Eigen3 3.4.0 EXACT REQUIRED)
find_package(lcm REQUIRED)

# include路径
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
if(BUILD_ROS)
  generate_parameter_library(hotdog_controller_parameters
    config/hotdog_controller_parameters.yaml
  )
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
endif()

if(BUILD_ROS)
  include_directories("/usr/local/include/yaml-cpp")
endif()

# 源码
file(GLOB_RECURSE common_sources "common/src/*.cpp")

# 添加第三方子库
add_subdirectory(common/third_party/ParamHandler)
add_subdirectory(common/third_party/cpptoml)
add_subdirectory(common/lcm_type)

# 生成common库
add_library(common SHARED ${common_sources})
add_dependencies(common lcm_type)
target_link_libraries(common
  pthread
  lcm
  param_handler
  cpptoml
)

# =================== control库相关 ===================
find_package(glog REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB_RECURSE control_sources "control/src/*.cpp")
file(GLOB_RECURSE user_sources "control/user/*.cpp")
file(GLOB_RECURSE third_party_sources "control/third_party/*.cpp")
file(GLOB_RECURSE simbridge_sources "simbridge/src/*.cpp")
file(GLOB_RECURSE hardware_sources "hardware/src/*.cpp")
file(GLOB_RECURSE factory_sources "factory/src/*.cpp")
file(GLOB_RECURSE simulator_sources "simulator/src/*.cpp")

include_directories(
  "./"
  "control/include"
  "control/include/control"
  "control/third_party"
  "control/user"
  "simbridge/include"
  "common/include"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  SYSTEM "control/third_party/qpOASES/include"
  "${CMAKE_SOURCE_DIR}/control/include"
  "${CMAKE_SOURCE_DIR}/common/third_party"
  "${CMAKE_SOURCE_DIR}/simbridge/include"
  ${GLOG_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
)

# control库
if(control_sources)
  add_library(control SHARED ${control_sources} ${user_sources} ${third_party_sources} ${simbridge_sources} ${factory_sources} ${simulator_sources})
  target_link_libraries(control
    pthread
    lcm
    param_handler
    qpOASES
    Goldfarb_Optimizer
    common
    glog::glog
  )
  if(BUILD_ROS)
    target_link_libraries(control hotdog_controller_parameters) # 关键：链接参数库
    ament_target_dependencies(control controller_interface nav_msgs tf2 tf2_geometry_msgs)
  endif()
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    target_link_libraries(control rt)
  endif()
  
  # ====== 这里是新增的 install 规则 ======
  install(
    TARGETS control
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  
endif()

add_subdirectory(control/third_party)


# =================== 其它子目录管理 ===================
if(ONBOARD_BUILD)
  add_subdirectory(hardware)
  if (EXISTS "${CMAKE_SOURCE_DIR}/factory" AND BUILD_FACTORY)
    add_subdirectory(factory)
  endif()
endif()

# 工厂相关自定义target
if(ONBOARD_BUILD AND BUILD_FACTORY)
  add_custom_target(version
    COMMAND ./scripts/get_git_hash.sh
    ${PROJECT_BINARY_DIR}/version.txt
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  add_custom_target(run_pack_script
    COMMAND ./scripts/pack_runnable.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  add_dependencies(run_pack_script
    version
    # hotdog_control
    manager
    ${TEST_TARGETS}
  )

  add_custom_target(final_package ALL)
  add_dependencies(final_package run_pack_script)
endif()

# =================== hotdog_controller 插件库（ROS2控制器插件） ===================
if(BUILD_ROS)
  # 假设你的控制器插件主文件在 control/src/hotdog_controller_node.cpp
  # 如有其他相关 cpp，也请一并添加
  file(GLOB hotdog_controller_plugin_sources
    "control/src/hotdog_controller_node.cpp"
    # "control/src/your_other_controller_src.cpp"
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
endif()

# =================== 安装 launch 和 config 目录 ===================
if(BUILD_ROS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  endif()
  ament_package()
endif()


