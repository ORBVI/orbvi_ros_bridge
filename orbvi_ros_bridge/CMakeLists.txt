cmake_minimum_required(VERSION 3.0.2)
project(orbvi_ros_bridge VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ORBVI_ROS_BRIDGE_LINK_STATIC "Link orbvi_ros_bridge_node with the static Host SDK library" OFF)

set(ORBVI_HOST_SDK_SOURCE_DIR
  ""
  CACHE PATH "Optional path to the ORBVI Host SDK C++ CMake project for development builds"
)
set(ORBVI_HOST_SDK_ROOT
  ""
  CACHE PATH "Optional Host SDK install prefix containing include/ and lib/"
)
set(ORBVI_HOST_SDK_INCLUDE_DIR
  ""
  CACHE PATH "Optional Host SDK include directory containing orbvi_sdk/*.hpp"
)
set(ORBVI_HOST_SDK_LIBRARY
  ""
  CACHE FILEPATH "Optional Host SDK library path, for example liborbvi_sdk.so"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  livox_ros_driver2
)
find_package(OpenCV REQUIRED)

if(NOT "${ORBVI_HOST_SDK_SOURCE_DIR}" STREQUAL "")
  if(NOT EXISTS "${ORBVI_HOST_SDK_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "ORBVI Host SDK CMakeLists.txt not found at ${ORBVI_HOST_SDK_SOURCE_DIR}")
  endif()
  set(ORBVI_SDK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(ORBVI_SDK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(ORBVI_SDK_BUILD_C_ABI OFF CACHE BOOL "" FORCE)
  set(ORBVI_SDK_BUILD_DIAGNOSTIC_TESTS OFF CACHE BOOL "" FORCE)
  add_subdirectory(
    "${ORBVI_HOST_SDK_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/orbvi_host_sdk"
  )
  if(ORBVI_ROS_BRIDGE_LINK_STATIC)
    set(ORBVI_ROS_BRIDGE_HOST_SDK_TARGET orbvi_sdk_static)
    set(ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE "static")
  else()
    set(ORBVI_ROS_BRIDGE_HOST_SDK_TARGET orbvi_sdk_shared)
    set(ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE "shared")
  endif()
else()
  if("${ORBVI_HOST_SDK_INCLUDE_DIR}" STREQUAL "")
    find_path(ORBVI_HOST_SDK_INCLUDE_DIR
      NAMES orbvi_sdk/client.hpp
      HINTS "${ORBVI_HOST_SDK_ROOT}"
      PATH_SUFFIXES include
    )
  endif()
  if("${ORBVI_HOST_SDK_LIBRARY}" STREQUAL "")
    find_library(ORBVI_HOST_SDK_LIBRARY
      NAMES orbvi_sdk
      HINTS "${ORBVI_HOST_SDK_ROOT}"
      PATH_SUFFIXES lib lib64
    )
  endif()
  if(NOT ORBVI_HOST_SDK_INCLUDE_DIR OR NOT ORBVI_HOST_SDK_LIBRARY)
    message(FATAL_ERROR
      "ORBVI Host SDK was not found. Set ORBVI_HOST_SDK_ROOT, "
      "ORBVI_HOST_SDK_INCLUDE_DIR/ORBVI_HOST_SDK_LIBRARY, or "
      "ORBVI_HOST_SDK_SOURCE_DIR for development builds."
    )
  endif()
  add_library(orbvi_sdk_external UNKNOWN IMPORTED)
  set_target_properties(orbvi_sdk_external PROPERTIES
    IMPORTED_LOCATION "${ORBVI_HOST_SDK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ORBVI_HOST_SDK_INCLUDE_DIR}"
  )
  set(ORBVI_ROS_BRIDGE_HOST_SDK_TARGET orbvi_sdk_external)
  get_filename_component(_orbvi_sdk_lib_name "${ORBVI_HOST_SDK_LIBRARY}" NAME)
  set(ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE "${_orbvi_sdk_lib_name}")
endif()

if(NOT TARGET ${ORBVI_ROS_BRIDGE_HOST_SDK_TARGET})
  message(FATAL_ERROR "Host SDK target ${ORBVI_ROS_BRIDGE_HOST_SDK_TARGET} is not available")
endif()

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs livox_ros_driver2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(orbvi_ros_bridge
  src/bridge_node.cpp
  src/frame_conversions.cpp
)
add_dependencies(orbvi_ros_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(orbvi_ros_bridge
  ${ORBVI_ROS_BRIDGE_HOST_SDK_TARGET}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_include_directories(orbvi_ros_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(orbvi_ros_bridge PRIVATE
  ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE="${ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE}"
)

add_executable(orbvi_ros_bridge_node
  src/orbvi_ros_bridge_node.cpp
)
add_dependencies(orbvi_ros_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(orbvi_ros_bridge_node
  orbvi_ros_bridge
  ${catkin_LIBRARIES}
)
target_include_directories(orbvi_ros_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

install(TARGETS orbvi_ros_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS orbvi_ros_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_orbvi_ros_bridge_host_sdk_link test/test_host_sdk_link.cpp)
  if(TARGET test_orbvi_ros_bridge_host_sdk_link)
    target_link_libraries(test_orbvi_ros_bridge_host_sdk_link
      orbvi_ros_bridge
      ${catkin_LIBRARIES}
      gtest
      gtest_main
    )
    target_include_directories(test_orbvi_ros_bridge_host_sdk_link PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
  endif()
endif()
