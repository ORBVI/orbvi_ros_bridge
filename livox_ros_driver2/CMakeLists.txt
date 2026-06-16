cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CustomPoint.msg"
    "msg/CustomMsg.msg"
    DEPENDENCIES std_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else()
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(
    FILES
      CustomPoint.msg
      CustomMsg.msg
  )

  generate_messages(DEPENDENCIES std_msgs)

  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
endif()
