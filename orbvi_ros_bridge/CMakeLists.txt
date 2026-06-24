cmake_minimum_required(VERSION 3.16)
project(orbvi_ros_bridge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ORBVI_ROS_BRIDGE_LINK_STATIC "Link orbvi_ros_bridge_node with the static Host SDK library" OFF)

get_filename_component(ORBVI_ROS_BRIDGE_PACKAGE_REAL_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}"
  REALPATH
)
get_filename_component(ORBVI_ROS_BRIDGE_REPO_ROOT
  "${ORBVI_ROS_BRIDGE_PACKAGE_REAL_DIR}/.."
  ABSOLUTE
)
set(ORBVI_ROS_BRIDGE_BUNDLED_SDK_ROOT
  "${ORBVI_ROS_BRIDGE_REPO_ROOT}/third_party/orbvi_sdk"
  CACHE PATH "Bundled ORBVI Host SDK root"
)

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

set(_ros_version "$ENV{ROS_VERSION}")
if(_ros_version STREQUAL "2")
  set(ORBVI_ROS_BRIDGE_ROS_VERSION 2)
elseif(_ros_version STREQUAL "1")
  set(ORBVI_ROS_BRIDGE_ROS_VERSION 1)
else()
  find_package(catkin QUIET COMPONENTS roscpp)
  if(catkin_FOUND)
    set(ORBVI_ROS_BRIDGE_ROS_VERSION 1)
  else()
    find_package(ament_cmake QUIET)
    if(ament_cmake_FOUND)
      set(ORBVI_ROS_BRIDGE_ROS_VERSION 2)
    endif()
  endif()
endif()

if(NOT ORBVI_ROS_BRIDGE_ROS_VERSION)
  message(FATAL_ERROR "No supported ROS environment found. Source ROS1 Noetic or ROS2 before building.")
endif()

function(orbvi_detect_bundled_sdk_dir out_var)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
      set(_dir "${ORBVI_ROS_BRIDGE_BUNDLED_SDK_ROOT}/libs/linux-x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
      set(_dir "${ORBVI_ROS_BRIDGE_BUNDLED_SDK_ROOT}/libs/linux-aarch64")
    endif()
  endif()
  set(${out_var} "${_dir}" PARENT_SCOPE)
endfunction()

function(orbvi_configure_host_sdk)
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
      set(_target orbvi_sdk_static)
      set(_linkage "static")
    else()
      set(_target orbvi_sdk_shared)
      set(_linkage "shared")
    endif()
  else()
    if("${ORBVI_HOST_SDK_INCLUDE_DIR}" STREQUAL "")
      unset(ORBVI_HOST_SDK_INCLUDE_DIR CACHE)
      find_path(ORBVI_HOST_SDK_INCLUDE_DIR
        NAMES orbvi_sdk/client.hpp
        HINTS
          "${ORBVI_HOST_SDK_ROOT}"
          "${ORBVI_ROS_BRIDGE_BUNDLED_SDK_ROOT}"
        PATH_SUFFIXES include
        NO_DEFAULT_PATH
      )
      if(NOT ORBVI_HOST_SDK_INCLUDE_DIR)
        find_path(ORBVI_HOST_SDK_INCLUDE_DIR
          NAMES orbvi_sdk/client.hpp
          HINTS "${ORBVI_HOST_SDK_ROOT}"
          PATH_SUFFIXES include
        )
      endif()
    endif()

    if("${ORBVI_HOST_SDK_LIBRARY}" STREQUAL "")
      unset(ORBVI_HOST_SDK_LIBRARY CACHE)
      orbvi_detect_bundled_sdk_dir(_bundled_sdk_lib_dir)
      if(ORBVI_ROS_BRIDGE_LINK_STATIC)
        set(_sdk_names liborbvi_sdk.a orbvi_sdk)
      else()
        set(_sdk_names liborbvi_sdk.so orbvi_sdk)
      endif()
      find_library(ORBVI_HOST_SDK_LIBRARY
        NAMES ${_sdk_names}
        HINTS
          "${ORBVI_HOST_SDK_ROOT}"
          "${_bundled_sdk_lib_dir}"
        PATHS
          "${_bundled_sdk_lib_dir}"
        PATH_SUFFIXES lib lib64
        NO_DEFAULT_PATH
      )
      if(NOT ORBVI_HOST_SDK_LIBRARY)
        find_library(ORBVI_HOST_SDK_LIBRARY
          NAMES ${_sdk_names}
          HINTS "${ORBVI_HOST_SDK_ROOT}"
          PATH_SUFFIXES lib lib64
        )
      endif()
    endif()

    if(NOT ORBVI_HOST_SDK_INCLUDE_DIR OR NOT ORBVI_HOST_SDK_LIBRARY)
      message(FATAL_ERROR
        "ORBVI Host SDK was not found. Use the bundled third_party/orbvi_sdk, "
        "set ORBVI_HOST_SDK_ROOT, ORBVI_HOST_SDK_INCLUDE_DIR/ORBVI_HOST_SDK_LIBRARY, "
        "or set ORBVI_HOST_SDK_SOURCE_DIR for development builds."
      )
    endif()
    add_library(orbvi_sdk_external UNKNOWN IMPORTED)
    set_target_properties(orbvi_sdk_external PROPERTIES
      IMPORTED_LOCATION "${ORBVI_HOST_SDK_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${ORBVI_HOST_SDK_INCLUDE_DIR}"
    )
    set(_target orbvi_sdk_external)
    get_filename_component(_linkage "${ORBVI_HOST_SDK_LIBRARY}" NAME)
  endif()

  if(NOT TARGET ${_target})
    message(FATAL_ERROR "Host SDK target ${_target} is not available")
  endif()

  set(ORBVI_ROS_BRIDGE_HOST_SDK_TARGET "${_target}" PARENT_SCOPE)
  set(ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE "${_linkage}" PARENT_SCOPE)
  set(ORBVI_HOST_SDK_INCLUDE_DIR "${ORBVI_HOST_SDK_INCLUDE_DIR}" PARENT_SCOPE)
  set(ORBVI_HOST_SDK_LIBRARY "${ORBVI_HOST_SDK_LIBRARY}" PARENT_SCOPE)
endfunction()

find_package(OpenCV REQUIRED COMPONENTS core imgproc)
orbvi_configure_host_sdk()

set(_orbvi_runtime_rpath "$ORIGIN:$ORIGIN/..")
get_filename_component(_orbvi_sdk_library_dir "${ORBVI_HOST_SDK_LIBRARY}" DIRECTORY)
if(EXISTS "${_orbvi_sdk_library_dir}")
  set(_orbvi_runtime_rpath "${_orbvi_runtime_rpath}:${_orbvi_sdk_library_dir}")
endif()

if(ORBVI_ROS_BRIDGE_ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    livox_ros_driver2
  )

  catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs livox_ros_driver2
  )

  add_library(orbvi_ros_bridge
    src/bridge_node.cpp
    src/frame_conversions.cpp
  )
  add_dependencies(orbvi_ros_bridge ${catkin_EXPORTED_TARGETS})
  target_include_directories(orbvi_ros_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(orbvi_ros_bridge
    ${ORBVI_ROS_BRIDGE_HOST_SDK_TARGET}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

  add_executable(orbvi_ros_bridge_node
    src/orbvi_ros_bridge_node.cpp
  )
  add_dependencies(orbvi_ros_bridge_node ${catkin_EXPORTED_TARGETS})
  target_include_directories(orbvi_ros_bridge_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(orbvi_ros_bridge_node
    orbvi_ros_bridge
    ${catkin_LIBRARIES}
  )

  set_target_properties(orbvi_ros_bridge orbvi_ros_bridge_node PROPERTIES
    BUILD_RPATH "${_orbvi_runtime_rpath}"
    INSTALL_RPATH "${_orbvi_runtime_rpath}"
  )
  target_compile_definitions(orbvi_ros_bridge PRIVATE
    ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE="${ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE}"
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
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
  )

  if(CATKIN_ENABLE_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(
      NAME test_orbvi_ros_bridge_launch_contracts
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_contracts.py
    )

    catkin_add_gtest(test_orbvi_ros_bridge_host_sdk_link test/test_host_sdk_link.cpp)
    if(TARGET test_orbvi_ros_bridge_host_sdk_link)
      add_dependencies(test_orbvi_ros_bridge_host_sdk_link ${catkin_EXPORTED_TARGETS})
      target_link_libraries(test_orbvi_ros_bridge_host_sdk_link
        orbvi_ros_bridge
        ${catkin_LIBRARIES}
        gtest
        gtest_main
      )
      target_include_directories(test_orbvi_ros_bridge_host_sdk_link PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${catkin_INCLUDE_DIRS}
      )
      set_target_properties(test_orbvi_ros_bridge_host_sdk_link PROPERTIES
        BUILD_RPATH "${_orbvi_runtime_rpath}"
      )
    endif()
  endif()
else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(livox_ros_driver2 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_library(orbvi_ros_bridge
    src/bridge_node_ros2.cpp
    src/frame_conversions_ros2.cpp
  )
  target_include_directories(orbvi_ros_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(orbvi_ros_bridge
    ${ORBVI_ROS_BRIDGE_HOST_SDK_TARGET}
    ${OpenCV_LIBS}
  )
  ament_target_dependencies(orbvi_ros_bridge
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    livox_ros_driver2
  )
  target_compile_definitions(orbvi_ros_bridge PRIVATE
    ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE="${ORBVI_ROS_BRIDGE_HOST_SDK_LINKAGE}"
  )

  add_executable(orbvi_ros_bridge_node
    src/orbvi_ros_bridge_node_ros2.cpp
  )
  target_include_directories(orbvi_ros_bridge_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(orbvi_ros_bridge_node orbvi_ros_bridge)
  ament_target_dependencies(orbvi_ros_bridge_node rclcpp)

  set_target_properties(orbvi_ros_bridge orbvi_ros_bridge_node PROPERTIES
    BUILD_RPATH "${_orbvi_runtime_rpath}"
    INSTALL_RPATH "${_orbvi_runtime_rpath}"
  )

  install(TARGETS orbvi_ros_bridge
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )
  install(TARGETS orbvi_ros_bridge_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY launch/ros2/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.launch.py"
  )
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING
      PATTERN "*.xml"
      PATTERN "*.yaml"
  )

  ament_add_gtest(test_orbvi_ros_bridge_host_sdk_link_ros2 test/test_host_sdk_link_ros2.cpp)
  if(TARGET test_orbvi_ros_bridge_host_sdk_link_ros2)
    target_link_libraries(test_orbvi_ros_bridge_host_sdk_link_ros2
      orbvi_ros_bridge
    )
    target_include_directories(test_orbvi_ros_bridge_host_sdk_link_ros2 PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    ament_target_dependencies(test_orbvi_ros_bridge_host_sdk_link_ros2
      rclcpp
      std_msgs
      sensor_msgs
      nav_msgs
      livox_ros_driver2
    )
    set_target_properties(test_orbvi_ros_bridge_host_sdk_link_ros2 PROPERTIES
      BUILD_RPATH "${_orbvi_runtime_rpath}"
    )
  endif()
  add_test(
    NAME test_orbvi_ros_bridge_launch_contracts
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_contracts.py
  )
endif()

if(EXISTS "${_orbvi_sdk_library_dir}" AND _orbvi_sdk_library_dir MATCHES "/third_party/orbvi_sdk/libs/")
  install(DIRECTORY "${_orbvi_sdk_library_dir}/"
    DESTINATION lib
    FILES_MATCHING PATTERN "liborbvi_sdk.so*"
  )
endif()

if(ORBVI_ROS_BRIDGE_ROS_VERSION EQUAL 2)
  ament_package()
endif()
