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

  add_executable(costmap_3d src/ros1_costmap_3d.cpp)
  target_link_libraries(costmap_3d costmap_3d_layers ${catkin_LIBRARIES})
  add_dependencies(costmap_3d ${catkin_EXPORTED_TARGETS})

  add_executable(laserscan_to_map src/ros1_laserscan_to_map.cpp)
  target_link_libraries(laserscan_to_map ${catkin_LIBRARIES})
  add_dependencies(laserscan_to_map ${catkin_EXPORTED_TARGETS})

  add_executable(pointcloud2_to_map src/ros1_pointcloud2_to_map.cpp)
  target_link_libraries(pointcloud2_to_map ${catkin_LIBRARIES})
  add_dependencies(pointcloud2_to_map ${catkin_EXPORTED_TARGETS})

  add_executable(largemap_to_map src/ros1_largemap_to_map.cpp)
  target_link_libraries(largemap_to_map ${catkin_LIBRARIES})
  add_dependencies(largemap_to_map ${catkin_EXPORTED_TARGETS})


  # roslint is disabled during hybridization; C++ style is checked by
  # ament_lint_auto on the ROS 2 side (see the ROS 2 block below).
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)
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

  # ROS 2 interface nodes. Each is registered as a component and also gets a
  # standalone executable with the same name as the ROS 1 one.
  ament_auto_add_library(costmap_3d_component SHARED
    src/ros2_costmap_3d_node.cpp)
  target_link_libraries(costmap_3d_component costmap_3d_layers)
  rclcpp_components_register_node(costmap_3d_component
    PLUGIN "costmap_cspace::Costmap3DOFNode"
    EXECUTABLE costmap_3d)

  ament_auto_add_library(laserscan_to_map_component SHARED
    src/ros2_laserscan_to_map_node.cpp)
  rclcpp_components_register_node(laserscan_to_map_component
    PLUGIN "costmap_cspace::LaserscanToMapNode"
    EXECUTABLE laserscan_to_map)

  ament_auto_add_library(pointcloud2_to_map_component SHARED
    src/ros2_pointcloud2_to_map_node.cpp)
  rclcpp_components_register_node(pointcloud2_to_map_component
    PLUGIN "costmap_cspace::Pointcloud2ToMapNode"
    EXECUTABLE pointcloud2_to_map)

  ament_auto_add_library(largemap_to_map_component SHARED
    src/ros2_largemap_to_map_node.cpp)
  rclcpp_components_register_node(largemap_to_map_component
    PLUGIN "costmap_cspace::LargeMapToMapNode"
    EXECUTABLE largemap_to_map)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # ROS 1-only interface sources are not built on ROS 2 and use the ROS 1
    # header/style conventions, so they are kept out of the linters.
    # ament_cmake_clang_format ignores this list, but the files are kept
    # clang-format clean so it passes anyway.
    set(AMENT_LINT_AUTO_FILE_EXCLUDE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ros1_costmap_3d.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ros1_largemap_to_map.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ros1_laserscan_to_map.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/ros1_pointcloud2_to_map.cpp)
    # CPP_FORMATTER=clang_format: exclude uncrustify, keep clang_format + cpplint.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The neonavigation BSD headers are not recognized by ament_copyright's
    # full-text templates (license=<unknown>), so the copyright linter is
    # excluded. Existing license text is kept as-is.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()

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
