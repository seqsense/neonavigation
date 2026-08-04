cmake_minimum_required(VERSION 3.1...3.26)
project(neonavigation_launch)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building neonavigation_launch for ROS 1 (Noetic)")

  find_package(catkin REQUIRED)
  catkin_package(
    CATKIN_DEPENDS
  )


  install(DIRECTORY launch config map
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building neonavigation_launch for ROS 2 (Jazzy)")

  # This package ships no code; it only distributes launch files, parameter
  # files, RViz configurations and the demo maps. The ROS 1 assets are kept
  # untouched next to their ROS 2 counterparts (*.launch / *.launch.py,
  # navigate.yaml / navigate_ros2.yaml, visualization.rviz /
  # visualization_ros2.rviz) so both ROS versions stay usable.
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # The existing BSD 3-clause headers ("the neonavigation authors") are not
    # matched by ament_copyright's known-license templates (reported as
    # license=<unknown>). Keep the original headers untouched and skip the
    # copyright linter instead of fabricating a recognized license text.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_auto_package(INSTALL_TO_SHARE launch config map)

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
