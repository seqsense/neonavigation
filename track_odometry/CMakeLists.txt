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

  add_executable(track_odometry_node src/ros1_track_odometry_node.cpp)
  set_target_properties(track_odometry_node PROPERTIES OUTPUT_NAME track_odometry)
  target_link_libraries(track_odometry_node track_odometry ${catkin_LIBRARIES})
  add_dependencies(track_odometry_node ${catkin_EXPORTED_TARGETS})

  add_executable(tf_projection src/ros1_tf_projection_node.cpp src/tf_projection.cpp)
  target_link_libraries(tf_projection ${catkin_LIBRARIES})
  add_dependencies(tf_projection ${catkin_EXPORTED_TARGETS})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)
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

  # ROS 2 component interface node for the IMU/odometry Kalman fusion logic.
  ament_auto_add_library(track_odometry_node SHARED
    src/ros2_track_odometry_node.cpp)
  target_link_libraries(track_odometry_node track_odometry)
  rclcpp_components_register_node(track_odometry_node
    PLUGIN "track_odometry::TrackOdometryNode"
    EXECUTABLE track_odometry_node_exec)

  # ROS 2 component interface node for the TF projection logic.
  ament_auto_add_library(tf_projection_node SHARED
    src/ros2_tf_projection_node.cpp)
  target_link_libraries(tf_projection_node tf_projection)
  rclcpp_components_register_node(tf_projection_node
    PLUGIN "track_odometry::TfProjectionNode"
    EXECUTABLE tf_projection_node_exec)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The existing BSD 3-clause headers ("the neonavigation authors") are not
    # matched by ament_copyright's known-license templates (reported as
    # license=<unknown>). Keep the original headers untouched and skip the
    # copyright linter instead of fabricating a recognized license text.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
