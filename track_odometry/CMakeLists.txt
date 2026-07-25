cmake_minimum_required(VERSION 3.1...3.26)
project(track_odometry)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building track_odometry for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp

    geometry_msgs
    message_filters
    nav_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs

    neonavigation_common

    sq_ros1_rclcpp_compat
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  find_package(Eigen3 REQUIRED)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES track_odometry
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )


  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


  # Hybridized IMU/odometry Kalman fusion logic (builds on ROS 1 via
  # sq_ros1_rclcpp_compat and, from the same source, on ROS 2 via rclcpp).
  add_library(track_odometry src/track_odometry.cpp)
  target_link_libraries(track_odometry ${catkin_LIBRARIES})
  add_dependencies(track_odometry ${catkin_EXPORTED_TARGETS})

  add_executable(track_odometry_node src/track_odometry_node.cpp)
  set_target_properties(track_odometry_node PROPERTIES OUTPUT_NAME track_odometry)
  target_link_libraries(track_odometry_node track_odometry ${catkin_LIBRARIES})
  add_dependencies(track_odometry_node ${catkin_EXPORTED_TARGETS})

  add_executable(tf_projection src/tf_projection_node.cpp src/tf_projection.cpp)
  target_link_libraries(tf_projection ${catkin_LIBRARIES})
  add_dependencies(tf_projection ${catkin_EXPORTED_TARGETS})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)

    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
  endif()


  install(TARGETS
      tf_projection
      track_odometry
      track_odometry_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building track_odometry for ROS 2 (Jazzy)")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(Eigen3 REQUIRED)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # Hybridized ROS-independent IMU/odometry Kalman fusion logic.
  ament_auto_add_library(track_odometry SHARED
    src/track_odometry.cpp)
  target_include_directories(track_odometry PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

  # Hybridized ROS-independent TF projection logic.
  ament_auto_add_library(tf_projection SHARED
    src/tf_projection.cpp)
  target_include_directories(tf_projection PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

  # ROS 2 interface nodes are added later by migration-hybrid-interface.

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
