
option(PARAM_TEST "Build test programe for ParamHandler")

find_package(yaml-cpp 0.7.0 EXACT REQUIRED)

###
### Library
###
add_library(param_handler SHARED
  ParamHandler.hpp
  ParamHandler.cpp
)

target_include_directories(param_handler PUBLIC
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(param_handler
  ${YAML_CPP_LIBRARIES}
)

if (PARAM_TEST)
  add_executable(test_param
    test/test_param_main.cpp
  )

  target_link_libraries(test_param
    param_handler
  )
endif()
