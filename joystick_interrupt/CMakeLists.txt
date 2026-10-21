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


  add_executable(joystick_interrupt src/ros1_joystick_interrupt.cpp)
  target_link_libraries(joystick_interrupt ${catkin_LIBRARIES})

  add_executable(joystick_mux src/ros1_joystick_mux.cpp)
  target_link_libraries(joystick_mux ${catkin_LIBRARIES})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)
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

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # ROS 2 component interface node: interrupts cmd_vel by joystick input.
  ament_auto_add_library(joystick_interrupt SHARED
    src/ros2_joystick_interrupt.cpp)
  rclcpp_components_register_node(joystick_interrupt
    PLUGIN "joystick_interrupt::JoystickInterrupt"
    EXECUTABLE joystick_interrupt_node)

  # ROS 2 component interface node: joystick-selected mux of arbitrary-type
  # topics (topic_tools::ShapeShifter ported to generic pub/sub).
  ament_auto_add_library(joystick_mux SHARED
    src/ros2_joystick_mux.cpp)
  rclcpp_components_register_node(joystick_mux
    PLUGIN "joystick_interrupt::JoystickMux"
    EXECUTABLE joystick_mux_node)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The existing BSD 3-clause headers ("the neonavigation authors") are not
    # matched by ament_copyright's known-license templates (reported as
    # license=<unknown>). Keep the original headers untouched and skip the
    # copyright linter instead of fabricating a recognized license text.
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
    # ports live under test_ros2/ so they are picked up by ament lint while the
    # untouched ROS 1 test definitions stay excluded.

    # Integration test gtest binary driven by launch_testing (mirrors the ROS 1
    # joystick_interrupt_rostest.test). It owns its main() / rclcpp::init, so it
    # is built as a plain executable and started as a node by the launch
    # harness.
    ament_add_gtest_executable(ros2_test_joystick_interrupt
      test_ros2/src/ros2_test_joystick_interrupt.cpp)
    ament_target_dependencies(ros2_test_joystick_interrupt
      rclcpp geometry_msgs sensor_msgs std_msgs)
    install(TARGETS ros2_test_joystick_interrupt DESTINATION lib/${PROJECT_NAME})

    add_launch_test(test_ros2/test/test_joystick_interrupt_launch.py
      TIMEOUT 120)
  endif()

  ament_auto_package(INSTALL_TO_SHARE launch)

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
