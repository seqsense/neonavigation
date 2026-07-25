cmake_minimum_required(VERSION 3.1...3.26)
project(map_organizer)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building map_organizer for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp

    geometry_msgs
    map_server
    nav_msgs
    pcl_conversions
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros

    map_organizer_msgs
    neonavigation_common

    sq_ros1_rclcpp_compat
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
  if(NEW_YAMLCPP_FOUND)
    add_definitions(-DHAVE_NEW_YAMLCPP)
  endif()

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pointcloud_to_maps_core
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )

  # Generate ROS 2-style compat headers (map_organizer_msgs/msg/*.hpp) for the
  # custom message the hybrid logic library returns. The standard message
  # packages already get theirs from sq_ros1_rclcpp_compat.
  generate_ros1_compat_headers(
    PACKAGE map_organizer_msgs
    MSG_DIR "${map_organizer_msgs_SOURCE_PREFIX}/msg"
    OUTPUT_DIR "${CATKIN_DEVEL_PREFIX}/include"
  )

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  include_directories(
    include
    ${CATKIN_DEVEL_PREFIX}/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
  )
  add_definitions(${PCL_DEFINITIONS})


  # Workaround for debian stretch build (https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=894656)
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt"
  )

  # Workaround for the bug in PCL(<1.8.1) https://github.com/PointCloudLibrary/pcl/issues/1406
  remove_definitions(-DDISABLE_LIBUSB-1.0)

  # Binary installed pcl provided by Linux distro is built with -march=native
  # which causes a lot of compatibility problems.
  # Define PCL_NO_PRECOMPILE to disable using the binary version.
  add_definitions(-DPCL_NO_PRECOMPILE)


  add_library(pointcloud_to_maps_core src/pointcloud_to_maps.cpp)
  target_link_libraries(pointcloud_to_maps_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(pointcloud_to_maps_core ${catkin_EXPORTED_TARGETS})

  add_executable(pointcloud_to_maps src/pointcloud_to_maps_node.cpp)
  target_link_libraries(pointcloud_to_maps pointcloud_to_maps_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(pointcloud_to_maps ${catkin_EXPORTED_TARGETS})

  add_executable(tie_maps src/tie_maps.cpp)
  target_link_libraries(tie_maps yaml-cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(tie_maps ${catkin_EXPORTED_TARGETS})

  add_executable(save_maps src/save_maps.cpp)
  target_link_libraries(save_maps ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(save_maps ${catkin_EXPORTED_TARGETS})

  add_executable(select_map src/select_map.cpp)
  target_link_libraries(select_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(select_map ${catkin_EXPORTED_TARGETS})

  add_executable(pose_transform src/pose_transform.cpp)
  target_link_libraries(pose_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(pose_transform ${catkin_EXPORTED_TARGETS})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)

    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
  endif()


  install(TARGETS
      pointcloud_to_maps
      pointcloud_to_maps_core
      pose_transform
      save_maps
      select_map
      tie_maps
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building map_organizer for ROS 2 (Jazzy)")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  add_definitions(-DPCL_NO_PRECOMPILE)

  # Hybridized ROS-independent point-cloud-to-maps logic.
  # PCL is a plain CMake package (not ament), so its include dirs / libraries
  # are added to the target explicitly rather than via ament_auto propagation.
  ament_auto_add_library(pointcloud_to_maps_core SHARED
    src/pointcloud_to_maps.cpp)
  target_include_directories(pointcloud_to_maps_core PUBLIC
    ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(pointcloud_to_maps_core ${PCL_LIBRARIES})

  # ROS 2 interface nodes are added by the migration-hybrid-interface stage.

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
