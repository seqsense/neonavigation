cmake_minimum_required(VERSION 3.1...3.26)
project(safety_limiter)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building safety_limiter for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
      roscpp

      diagnostic_updater
      dynamic_reconfigure
      geometry_msgs
      pcl_conversions
      pcl_ros
      sensor_msgs
      std_msgs
      tf2_ros
      tf2_sensor_msgs
      safety_limiter_msgs

      neonavigation_common

      sq_ros1_rclcpp_compat
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(xmlrpcpp REQUIRED)

  generate_dynamic_reconfigure_options(
    cfg/SafetyLimiter.cfg
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES safety_limiter_core
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )


  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
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


  add_library(safety_limiter_core src/safety_limiter.cpp)
  target_link_libraries(safety_limiter_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(safety_limiter_core ${catkin_EXPORTED_TARGETS})

  add_executable(safety_limiter src/ros1_safety_limiter_node.cpp)
  target_link_libraries(safety_limiter safety_limiter_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(safety_limiter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    add_subdirectory(test)
  endif()


  install(TARGETS
      safety_limiter
      safety_limiter_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building safety_limiter for ROS 2 (Jazzy)")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  add_definitions(-DPCL_NO_PRECOMPILE)

  # Hybridized ROS-independent collision-prediction logic.
  # PCL is a plain CMake package (not ament), so its include dirs / libraries
  # are added to the target explicitly rather than via ament_auto propagation.
  ament_auto_add_library(safety_limiter_core SHARED
    src/safety_limiter.cpp)
  target_include_directories(safety_limiter_core PUBLIC
    ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(safety_limiter_core ${PCL_LIBRARIES})

  # ROS 2 interface node (component + standalone executable safety_limiter).
  ament_auto_add_library(safety_limiter_component SHARED
    src/ros2_safety_limiter_node.cpp)
  target_link_libraries(safety_limiter_component safety_limiter_core ${PCL_LIBRARIES})
  target_include_directories(safety_limiter_component PUBLIC
    ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  rclcpp_components_register_node(safety_limiter_component
    PLUGIN "safety_limiter::SafetyLimiterNode"
    EXECUTABLE safety_limiter)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # CPP_FORMATTER=clang_format: exclude uncrustify, keep clang_format + cpplint.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The neonavigation BSD headers are not recognized by ament_copyright's
    # full-text templates (license=<unknown>), so the copyright linter is
    # excluded. Existing license text is kept as-is (Copyright=keep).
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    find_package(GTest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    # Pull the test-only message deps (tf2_geometry_msgs) into the ament_auto
    # link set so the ros2_test_* executables below link against them.
    ament_auto_find_test_dependencies()

    include_directories(test/include)

    # Integration test gtest binaries. They own their main() (call rclcpp::init
    # before RUN_ALL_TESTS), so they are built as plain executables linked
    # against GTest and driven by the launch_testing descriptions below, which
    # mirror the ROS 1 safety_limiter_rostest / safety_limiter2_rostest tests.
    ament_auto_add_executable(ros2_test_safety_limiter
      test/src/ros2_test_safety_limiter.cpp)
    target_link_libraries(ros2_test_safety_limiter ${GTEST_LIBRARIES})

    ament_auto_add_executable(ros2_test_safety_limiter2
      test/src/ros2_test_safety_limiter2.cpp)
    target_link_libraries(ros2_test_safety_limiter2 ${GTEST_LIBRARIES})

    install(DIRECTORY test/configs
      DESTINATION share/${PROJECT_NAME}/test/)

    # The gtest matrix itself fits the 120 s ROS 1 budget; the launch harness
    # adds node-startup overhead on top, so give the launch test extra margin.
    add_launch_test(test/test/test_safety_limiter_launch.py
      TARGET test_safety_limiter
      TIMEOUT 180)
    add_launch_test(test/test/test_safety_limiter2_launch.py
      TARGET test_safety_limiter2
      TIMEOUT 60)
  endif()

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
