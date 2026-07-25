cmake_minimum_required(VERSION 3.1...3.26)
project(trajectory_tracker)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building trajectory_tracker for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp

    dynamic_reconfigure
    geometry_msgs
    interactive_markers
    nav_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros

    neonavigation_common
    trajectory_tracker_msgs

    sq_ros1_rclcpp_compat
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  find_package(Boost REQUIRED COMPONENTS thread)
  find_package(Eigen3 REQUIRED)

  generate_dynamic_reconfigure_options(
    cfg/TrajectoryTracker.cfg
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES trajectory_tracker_controller
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )

  # Generate ROS 2-style compat headers (trajectory_tracker_msgs/msg/*.hpp) for
  # the custom messages the hybrid logic library consumes. The standard message
  # packages already get theirs from sq_ros1_rclcpp_compat.
  generate_ros1_compat_headers(
    PACKAGE trajectory_tracker_msgs
    MSG_DIR "${trajectory_tracker_msgs_SOURCE_PREFIX}/msg"
    OUTPUT_DIR "${CATKIN_DEVEL_PREFIX}/include"
  )

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  include_directories(
    include
    ${CATKIN_DEVEL_PREFIX}/include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )


  add_library(trajectory_tracker_controller src/tracker_controller.cpp)
  target_link_libraries(trajectory_tracker_controller ${catkin_LIBRARIES})
  add_dependencies(trajectory_tracker_controller ${catkin_EXPORTED_TARGETS})

  add_executable(trajectory_tracker src/trajectory_tracker.cpp)
  target_link_libraries(trajectory_tracker trajectory_tracker_controller ${catkin_LIBRARIES})
  add_dependencies(trajectory_tracker ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

  add_executable(trajectory_recorder src/trajectory_recorder.cpp)
  target_link_libraries(trajectory_recorder ${catkin_LIBRARIES})
  add_dependencies(trajectory_recorder ${catkin_EXPORTED_TARGETS})

  add_executable(trajectory_saver src/trajectory_saver.cpp)
  target_link_libraries(trajectory_saver ${catkin_LIBRARIES})
  add_dependencies(trajectory_saver ${catkin_EXPORTED_TARGETS})

  add_executable(trajectory_server src/trajectory_server.cpp)
  target_link_libraries(trajectory_server ${catkin_LIBRARIES})
  add_dependencies(trajectory_server ${catkin_EXPORTED_TARGETS})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)

    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
  endif()


  install(TARGETS
      trajectory_recorder
      trajectory_saver
      trajectory_server
      trajectory_tracker
      trajectory_tracker_controller
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building trajectory_tracker for ROS 2 (Jazzy)")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(Eigen3 REQUIRED)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # Logic library only. The ROS 2 interface node is added later by
  # migration-hybrid-interface.
  ament_auto_add_library(trajectory_tracker_controller SHARED
    src/tracker_controller.cpp)
  target_include_directories(trajectory_tracker_controller PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
