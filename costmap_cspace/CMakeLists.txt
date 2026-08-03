cmake_minimum_required(VERSION 3.1...3.26)
project(costmap_cspace)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building costmap_cspace for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp

    geometry_msgs
    laser_geometry
    nav_msgs
    sensor_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2_sensor_msgs

    costmap_cspace_msgs
    neonavigation_common

    sq_ros1_rclcpp_compat
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  find_package(xmlrpcpp REQUIRED)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES costmap_3d_layers
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )

  # costmap_cspace_msgs generates its own ROS 2-style compat headers on the
  # ROS 1 side, so consumers just pick them up from the devel include space.

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)


  include_directories(
    include
    ${CATKIN_DEVEL_PREFIX}/include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/costmap_cspace
  )

  add_library(costmap_3d_layers
    src/costmap_3d_layers.cpp
    src/costmap_3d_handler.cpp
  )
  target_link_libraries(costmap_3d_layers ${catkin_LIBRARIES})
  add_dependencies(costmap_3d_layers ${catkin_EXPORTED_TARGETS})

  add_executable(costmap_3d src/costmap_3d.cpp)
  target_link_libraries(costmap_3d costmap_3d_layers ${catkin_LIBRARIES})
  add_dependencies(costmap_3d ${catkin_EXPORTED_TARGETS})

  add_executable(laserscan_to_map src/laserscan_to_map.cpp)
  target_link_libraries(laserscan_to_map ${catkin_LIBRARIES})
  add_dependencies(laserscan_to_map ${catkin_EXPORTED_TARGETS})

  add_executable(pointcloud2_to_map src/pointcloud2_to_map.cpp)
  target_link_libraries(pointcloud2_to_map ${catkin_LIBRARIES})
  add_dependencies(pointcloud2_to_map ${catkin_EXPORTED_TARGETS})

  add_executable(largemap_to_map src/largemap_to_map.cpp)
  target_link_libraries(largemap_to_map ${catkin_LIBRARIES})
  add_dependencies(largemap_to_map ${catkin_EXPORTED_TARGETS})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)

    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
  endif()


  install(TARGETS
      costmap_3d
      largemap_to_map
      laserscan_to_map
      pointcloud2_to_map
      costmap_3d_layers
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building costmap_cspace for ROS 2 (Jazzy)")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # Hybridized ROS-independent costmap layer chain.
  ament_auto_add_library(costmap_3d_layers SHARED
    src/costmap_3d_layers.cpp
    src/costmap_3d_handler.cpp)

  # ROS 2 interface nodes are added later by migration-hybrid-interface.

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # ROS-independent unit tests shared with the ROS 1 build.
    ament_add_gtest(test_${PROJECT_NAME}_costmap_3d test/src/test_costmap_3d.cpp)
    target_link_libraries(test_${PROJECT_NAME}_costmap_3d costmap_3d_layers)

    ament_add_gtest(test_${PROJECT_NAME}_pointcloud_accumulator
      test/src/test_pointcloud_accumulator.cpp)
    ament_target_dependencies(test_${PROJECT_NAME}_pointcloud_accumulator
      rclcpp sensor_msgs)
    target_include_directories(test_${PROJECT_NAME}_pointcloud_accumulator PRIVATE include)
  endif()

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
