cmake_minimum_required(VERSION 3.1...3.26)
project(obj_to_pointcloud)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building obj_to_pointcloud for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp

    geometry_msgs
    pcl_conversions
    sensor_msgs

    neonavigation_common

    sq_ros1_rclcpp_compat
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)
  catkin_package(
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )


  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
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


  add_executable(obj_to_pointcloud src/ros1_obj_to_pointcloud.cpp)
  target_link_libraries(obj_to_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(obj_to_pointcloud ${catkin_EXPORTED_TARGETS})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)
  endif()


  install(TARGETS
      obj_to_pointcloud
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building obj_to_pointcloud for ROS 2 (Jazzy)")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # Binary installed pcl provided by Linux distro is built with -march=native
  # which causes a lot of compatibility problems.
  # Define PCL_NO_PRECOMPILE to disable using the binary version.
  add_definitions(-DPCL_NO_PRECOMPILE)

  # obj_to_pointcloud is a thin IO node that was not split into a separate
  # logic library; the OBJ/PCD-to-cloud conversion lives in the interface node.
  # PCL is a plain CMake package (not ament), so its include dirs / libraries
  # are added to the target explicitly rather than via ament_auto propagation.
  ament_auto_add_library(obj_to_pointcloud_component SHARED
    src/ros2_obj_to_pointcloud.cpp)
  target_include_directories(obj_to_pointcloud_component PUBLIC
    ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(obj_to_pointcloud_component ${PCL_LIBRARIES})
  rclcpp_components_register_node(obj_to_pointcloud_component
    PLUGIN "obj_to_pointcloud::ObjToPointcloudNode"
    EXECUTABLE obj_to_pointcloud)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # CPP_FORMATTER=clang_format: exclude uncrustify, keep clang_format + cpplint.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The BSD license headers here are not recognized by ament_copyright's
    # full-text templates (license=<unknown>), so the copyright linter is
    # excluded. The existing license text is kept as-is.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    # ros-humble-ament-clang-format depends on Alpine's clang package, but the
    # clang-format binary lives in clang-extra-tools, so the linter cannot run
    # there at all (jazzy's 0.17.5 packaging fixed this dependency). Formatting
    # is checked on jazzy instead.
    if("$ENV{ROS_DISTRO}" STREQUAL "humble")
      list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_format)
    endif()
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)

    # The ROS 1 rostest sources live under test/ (AMENT_IGNORE'd); the ROS 2
    # port lives under test_ros2/ so it is picked up by ament lint while the
    # untouched ROS 1 test definitions stay excluded.

    # The sample OBJ consumed by the integration test is installed into the
    # package share so the launch description can locate it via
    # get_package_share_directory.
    install(FILES test/data/sample.obj
      DESTINATION share/${PROJECT_NAME}/test/data)

    # Integration test gtest binary driven by launch_testing (mirrors the ROS 1
    # obj_to_pointcloud_rostest.test). It owns its main() / rclcpp::init, so it
    # is built as a plain executable and started as a node by the launch
    # harness, which checks its exit code after shutdown.
    ament_add_gtest_executable(ros2_test_obj_to_pointcloud
      test_ros2/src/ros2_test_obj_to_pointcloud.cpp)
    ament_target_dependencies(ros2_test_obj_to_pointcloud rclcpp sensor_msgs)
    install(TARGETS ros2_test_obj_to_pointcloud DESTINATION lib/${PROJECT_NAME})

    add_launch_test(test_ros2/test/test_obj_to_pointcloud_launch.py
      TIMEOUT 120)
  endif()

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
