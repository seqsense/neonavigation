cmake_minimum_required(VERSION 3.1...3.26)
project(neonavigation_common)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building neonavigation_common for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )


  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  include_directories(include ${catkin_INCLUDE_DIRS})


  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    add_subdirectory(test)

    # Style checks are unified on the ROS2 side (ament_lint_auto); roslint is
    # intentionally not used any more.
  endif()


  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building neonavigation_common for ROS 2 (Jazzy)")

  # Header-only compatibility package. include/neonavigation_common/compatibility.h
  # is a ROS 1 interface-layer shim (ros::NodeHandle-based subscribe/advertise);
  # it is only included by the ROS 1 interface parts of downstream nodes and is
  # never compiled on ROS 2. The ROS 2 build therefore just installs and exports
  # the headers so downstream packages can resolve the dependency.
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # Both C++ files here (compatibility.h and its gtest) are ROS 1 only and are
    # never compiled by this branch, but they are kept in the linter walk rather
    # than listed in AMENT_LINT_AUTO_FILE_EXCLUDE: they satisfy the ament rules
    # after clang-format, and excluding them would leave cpplint with zero files
    # to check, which it reports as a failure ("No files found").
    # CPP_FORMATTER=clang_format: exclude uncrustify, keep clang_format + cpplint.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The neonavigation BSD headers are not recognized by ament_copyright's
    # full-text templates (license=<unknown>), so the copyright linter is
    # excluded. Existing license text is kept as-is.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()
  endif()

  install(DIRECTORY include/
    DESTINATION include
  )

  ament_export_include_directories(include)

  ament_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
