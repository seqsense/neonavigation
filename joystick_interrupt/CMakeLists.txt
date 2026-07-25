cmake_minimum_required(VERSION 3.1...3.26)
project(joystick_interrupt)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building joystick_interrupt for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp

    geometry_msgs
    sensor_msgs
    std_msgs
    topic_tools

    neonavigation_common

    sq_ros1_rclcpp_compat
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  catkin_package(
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )


  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  include_directories(${catkin_INCLUDE_DIRS})


  add_executable(joystick_interrupt src/joystick_interrupt.cpp)
  target_link_libraries(joystick_interrupt ${catkin_LIBRARIES})

  add_executable(joystick_mux src/joystick_mux.cpp)
  target_link_libraries(joystick_mux ${catkin_LIBRARIES})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)

    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
  endif()


  install(TARGETS
      joystick_interrupt
      joystick_mux
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building joystick_interrupt for ROS 2 (Jazzy)")

  # The joystick_interrupt / joystick_mux nodes are still monolithic ROS 1 nodes
  # (ros::init / NodeHandle / topic_tools). Their logic-relevant surface
  # (message types, rclcpp::Time / Duration / Clock, RCLCPP_* logging) has been
  # migrated to the rclcpp-style API so it builds on ROS 1 via
  # sq_ros1_rclcpp_compat and is ready for reuse. The ROS 2 interface nodes are
  # created by the next stage (migration-hybrid-interface); this branch only
  # configures so `colcon build` succeeds.
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
