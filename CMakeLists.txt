cmake_minimum_required(VERSION 3.5)
project(openspot_sim VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(spotng REQUIRED)

# find ignition
set(IGN_GAZEBO_VER 6)
set(IGN_PLUGIN_VER 1)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo${IGN_GAZEBO_VER} REQUIRED)
find_package(ignition-plugin${IGN_PLUGIN_VER} REQUIRED COMPONENTS register)

add_library(${PROJECT_NAME} SHARED
  src/drive.cpp
  src/itf.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH ON
  CXX_STANDARD 17
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
   spotng::spotng
   ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
   ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)
