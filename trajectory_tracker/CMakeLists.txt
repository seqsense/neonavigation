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

  # trajectory_tracker_msgs generates its own ROS 2-style compat headers on the
  # ROS 1 side, so consumers just pick them up from the devel include space.

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

  add_executable(trajectory_tracker src/ros1_trajectory_tracker.cpp)
  target_link_libraries(trajectory_tracker trajectory_tracker_controller ${catkin_LIBRARIES})
  add_dependencies(trajectory_tracker ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

  add_executable(trajectory_recorder src/ros1_trajectory_recorder.cpp)
  target_link_libraries(trajectory_recorder ${catkin_LIBRARIES})
  add_dependencies(trajectory_recorder ${catkin_EXPORTED_TARGETS})

  add_executable(trajectory_saver src/ros1_trajectory_saver.cpp)
  target_link_libraries(trajectory_saver ${catkin_LIBRARIES})
  add_dependencies(trajectory_saver ${catkin_EXPORTED_TARGETS})

  add_executable(trajectory_server src/ros1_trajectory_server.cpp)
  target_link_libraries(trajectory_server ${catkin_LIBRARIES})
  add_dependencies(trajectory_server ${catkin_EXPORTED_TARGETS})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)
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

  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 15)
    # GCC 15 crashes (ICE, "during GIMPLE pass: strlen") while instantiating
    # rclcpp/service.hpp with coverage instrumentation, which is how CI builds.
    # It lands on whichever translation unit instantiates it, and only sometimes,
    # so every package carries this guard. -fno-optimize-strlen is not enough:
    # the pass still runs to emit its printf diagnostics, which is where it dies.
    # The pass names are GCC internals, hence the flag check.
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(-fdisable-tree-strlen1 HAS_DISABLE_TREE_STRLEN1)
    check_cxx_compiler_flag(-fdisable-tree-strlen2 HAS_DISABLE_TREE_STRLEN2)
    if(HAS_DISABLE_TREE_STRLEN1 AND HAS_DISABLE_TREE_STRLEN2)
      add_compile_options(-fdisable-tree-strlen1 -fdisable-tree-strlen2)
    else()
      add_compile_options(-fno-optimize-strlen)
    endif()
  endif()

  find_package(Eigen3 REQUIRED)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # Hybridized ROS-independent control logic.
  ament_auto_add_library(trajectory_tracker_controller SHARED
    src/tracker_controller.cpp)
  target_include_directories(trajectory_tracker_controller PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

  # ROS 2 interface node (component + standalone executable trajectory_tracker).
  ament_auto_add_library(trajectory_tracker_component SHARED
    src/ros2_trajectory_tracker.cpp)
  target_link_libraries(trajectory_tracker_component trajectory_tracker_controller)
  target_include_directories(trajectory_tracker_component PUBLIC
    ${EIGEN3_INCLUDE_DIRS})
  rclcpp_components_register_node(trajectory_tracker_component
    PLUGIN "trajectory_tracker::TrackerNode"
    EXECUTABLE trajectory_tracker)

  # Standalone helper nodes.
  ament_auto_add_executable(trajectory_recorder src/ros2_trajectory_recorder.cpp)
  ament_auto_add_executable(trajectory_saver src/ros2_trajectory_saver.cpp)
  ament_auto_add_executable(trajectory_server src/ros2_trajectory_server.cpp)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # CPP_FORMATTER=clang_format: exclude uncrustify, keep clang_format + cpplint.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The BSD headers here (with the ATR / research-grant preamble) are not
    # recognized by ament_copyright's full-text templates (license=<unknown>),
    # so the copyright linter is excluded. Existing license text is kept as-is.
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
    find_package(GTest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)

    include_directories(include ${EIGEN3_INCLUDE_DIRS})

    # -- ROS-independent unit tests (pure gtest, shared with the ROS 1 build) --
    ament_add_gtest(test_${PROJECT_NAME}_filter test/src/test_filter.cpp)
    ament_target_dependencies(test_${PROJECT_NAME}_filter rclcpp)

    ament_add_gtest(test_${PROJECT_NAME}_path2d test/src/test_path2d.cpp)
    ament_target_dependencies(test_${PROJECT_NAME}_path2d
      rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs trajectory_tracker_msgs)

    # -- Integration tests (launch_testing driving the trajectory_tracker node) --
    # The gtest binaries below own their main() (they call rclcpp::init before
    # RUN_ALL_TESTS), so they are built as plain executables linked against
    # GTest (not gtest_main) and launched from the launch_testing descriptions.
    include_directories(test/include)

    ament_auto_add_executable(ros2_time_source test/src/ros2_time_source.cpp)

    # Deterministic ports of the tracker scenarios: they drive TrackerController
    # directly, so they need no node, no DDS and no simulated clock. Shared with
    # the ROS 1 build (see test/CMakeLists.txt).
    ament_add_gtest(test_${PROJECT_NAME}_scenarios test/src/test_tracker_scenarios.cpp)
    target_include_directories(test_${PROJECT_NAME}_scenarios PRIVATE include test/include)
    target_link_libraries(test_${PROJECT_NAME}_scenarios trajectory_tracker_controller)

    ament_auto_add_executable(ros2_test_trajectory_tracker
      test/src/ros2_test_trajectory_tracker.cpp)
    target_link_libraries(ros2_test_trajectory_tracker ${GTEST_LIBRARIES})

    ament_auto_add_executable(ros2_test_trajectory_tracker_with_odom
      test/src/ros2_test_trajectory_tracker_with_odom.cpp)
    target_link_libraries(ros2_test_trajectory_tracker_with_odom ${GTEST_LIBRARIES})

    ament_auto_add_executable(ros2_test_trajectory_tracker_overshoot
      test/src/ros2_test_trajectory_tracker_overshoot.cpp)
    target_link_libraries(ros2_test_trajectory_tracker_overshoot ${GTEST_LIBRARIES})

    ament_auto_add_executable(ros2_test_trajectory_recorder
      test/src/ros2_test_trajectory_recorder.cpp)
    target_link_libraries(ros2_test_trajectory_recorder ${GTEST_LIBRARIES})

    install(DIRECTORY test/configs
      DESTINATION share/${PROJECT_NAME}/test/)

    # The control law is covered by test_trajectory_tracker_scenarios, which
    # needs no node at all. What runs here has to run as a node: that the
    # interface layer publishes and subscribes what the scenarios assume, once
    # per parameter combination, and that the node's own timers behave - the
    # control rate and the odometry timeout. Each of these drives a single
    # short path rather than the whole scenario set.
    #
    # Each launch test gets a private DDS domain and local-only discovery. ctest
    # runs them one at a time, but a node from the previous test can still be on
    # its way out, and nothing should be able to join from the host either.
    set(_launch_test_domain 91)
    macro(tracker_add_launch_test path)
      add_launch_test(${path} ${ARGN}
        ENV
          "ROS_DOMAIN_ID=${_launch_test_domain}"
          "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
      math(EXPR _launch_test_domain "${_launch_test_domain} + 1")
    endmacro()

    tracker_add_launch_test(test/test/test_trajectory_tracker_launch.py
      TARGET test_trajectory_tracker
      ARGS "use_sim_time:=true"
      TIMEOUT 120)
    tracker_add_launch_test(test/test/test_trajectory_tracker_launch.py
      TARGET test_trajectory_tracker_with_odom_delay
      ARGS "use_sim_time:=true" "odom_delay:=0.04" "use_odom:=true" "use_time_optimal_control:=true"
      TIMEOUT 120)
    tracker_add_launch_test(test/test/test_trajectory_tracker_launch.py
      TARGET test_trajectory_tracker_without_time_optimal_control
      ARGS "use_sim_time:=true" "odom_delay:=0.04" "use_odom:=true" "use_time_optimal_control:=false"
      TIMEOUT 120)
    tracker_add_launch_test(test/test/test_trajectory_tracker_realtime_launch.py
      TARGET test_trajectory_tracker_with_odom
      ARGS "test_executable:=ros2_test_trajectory_tracker_with_odom"
      TIMEOUT 120)
    # Two cases only: what is left to prove here is that a parameter changed at
    # runtime reaches the controller (see the test source).
    tracker_add_launch_test(test/test/test_trajectory_tracker_realtime_launch.py
      TARGET test_trajectory_tracker_overshoot
      ARGS "test_executable:=ros2_test_trajectory_tracker_overshoot"
      TIMEOUT 120)

    tracker_add_launch_test(test/test/test_trajectory_recorder_launch.py
      TARGET test_trajectory_recorder
      TIMEOUT 120)
  endif()

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
