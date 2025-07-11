cmake_minimum_required(VERSION 3.8)
project(ss_plugins)

#Comment the following instructions to prevent warning messages from being displayed when compiling a plugin
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(QuadController SHARED src/QuadController.cc)
set_property(TARGET QuadController PROPERTY CXX_STANDARD 17)  #important to be able to use 'gzmsg' instructions in the plugin

target_include_directories(QuadController PRIVATE include)

target_link_libraries(QuadController PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
  TARGETS QuadController
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
