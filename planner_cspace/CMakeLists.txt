cmake_minimum_required(VERSION 3.1...3.26)
project(planner_cspace)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Building planner_cspace for ROS 1 (Noetic)")

  set(CATKIN_DEPENDS
    roscpp

    actionlib
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    move_base_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs

    costmap_cspace
    costmap_cspace_msgs
    neonavigation_common
    neonavigation_metrics_msgs
    planner_cspace_msgs
    trajectory_tracker
    trajectory_tracker_msgs

    sq_ros1_rclcpp_compat
  )

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
  find_package(OpenMP REQUIRED)

  generate_dynamic_reconfigure_options(
    cfg/Planner3D.cfg
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES planner_3d_core planner_2dof_serial_joints_core
    CATKIN_DEPENDS ${CATKIN_DEPENDS}
  )

  # trajectory_msgs is not part of sq_ros1_rclcpp_compat's built-in list of
  # message packages, so its ROS 2-style compat headers are generated here.
  # The other message packages used by the hybridized logic (costmap_cspace_msgs,
  # neonavigation_metrics_msgs, planner_cspace_msgs, trajectory_tracker_msgs)
  # emit their own compat headers into the devel include space.
  generate_ros1_compat_headers(
    PACKAGE trajectory_msgs
    MSG_DIR "${trajectory_msgs_DIR}/../msg"
    OUTPUT_DIR "${CATKIN_DEVEL_PREFIX}/include")

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  add_compile_options(-funroll-loops -finline-functions ${OpenMP_CXX_FLAGS})
  include_directories(
    include
    ${CATKIN_DEVEL_PREFIX}/include
    ${catkin_INCLUDE_DIRS}
  )


  # ROS-independent path planning logic of planner_3d.
  add_library(planner_3d_core
    src/costmap_bbf.cpp
    src/grid_astar_model_3dof.cpp
    src/motion_cache.cpp
    src/motion_primitive_builder.cpp
    src/planner_3d_core.cpp
    src/distance_map.cpp
    src/rotation_cache.cpp
    src/start_pose_predictor.cpp
  )
  target_link_libraries(planner_3d_core ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
  add_dependencies(planner_3d_core ${catkin_EXPORTED_TARGETS})

  add_executable(planner_3d src/planner_3d.cpp)
  target_link_libraries(planner_3d planner_3d_core ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
  add_dependencies(planner_3d ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
  set_property(TARGET planner_3d PROPERTY PUBLIC_HEADER
    include/planner_cspace/bbf.h
    include/planner_cspace/blockmem_gridmap.h
    include/planner_cspace/cyclic_vec.h
    include/planner_cspace/grid_astar.h
    include/planner_cspace/grid_astar_model.h
    include/planner_cspace/jump_detector.h
    include/planner_cspace/reservable_priority_queue.h
  )

  # ROS-independent path planning logic of planner_2dof_serial_joints.
  add_library(planner_2dof_serial_joints_core
    src/planner_2dof_serial_joints_core.cpp
    src/grid_astar_model_2dof_joints.cpp
  )
  target_link_libraries(planner_2dof_serial_joints_core ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
  add_dependencies(planner_2dof_serial_joints_core ${catkin_EXPORTED_TARGETS})

  add_executable(planner_2dof_serial_joints src/planner_2dof_serial_joints.cpp)
  target_link_libraries(planner_2dof_serial_joints
    planner_2dof_serial_joints_core ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
  add_dependencies(planner_2dof_serial_joints ${catkin_EXPORTED_TARGETS})

  add_executable(dummy_robot src/dummy_robot.cpp)
  target_link_libraries(dummy_robot ${catkin_LIBRARIES})
  add_dependencies(dummy_robot ${catkin_EXPORTED_TARGETS})

  add_executable(patrol src/patrol.cpp)
  target_link_libraries(patrol ${catkin_LIBRARIES})
  add_dependencies(patrol ${catkin_EXPORTED_TARGETS})


  # roslint is disabled during hybridization; C++ style is checked by
  # ament_lint_auto on the ROS 2 side (see the ROS 2 block below).
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)
  endif()


  install(TARGETS
      dummy_robot
      patrol
      planner_2dof_serial_joints
      planner_2dof_serial_joints_core
      planner_3d
      planner_3d_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PUBLIC_HEADER DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

elseif($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Building planner_cspace for ROS 2 (Jazzy)")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
  # ament_auto does not take care of OpenMP, so request it explicitly.
  find_package(OpenMP REQUIRED)

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  add_compile_options(-funroll-loops -finline-functions ${OpenMP_CXX_FLAGS})

  # Hybridized ROS-independent path planning logic of planner_3d.
  ament_auto_add_library(planner_3d_core SHARED
    src/costmap_bbf.cpp
    src/grid_astar_model_3dof.cpp
    src/motion_cache.cpp
    src/motion_primitive_builder.cpp
    src/planner_3d_core.cpp
    src/distance_map.cpp
    src/rotation_cache.cpp
    src/start_pose_predictor.cpp)
  target_link_libraries(planner_3d_core ${OpenMP_CXX_FLAGS})

  # Hybridized ROS-independent trajectory planning logic of
  # planner_2dof_serial_joints.
  ament_auto_add_library(planner_2dof_serial_joints_core SHARED
    src/planner_2dof_serial_joints_core.cpp
    src/grid_astar_model_2dof_joints.cpp)
  target_link_libraries(planner_2dof_serial_joints_core ${OpenMP_CXX_FLAGS})

  # The ROS 2 interface nodes are added by a later migration step; only the
  # logic libraries and their ROS-independent unit tests are built here.

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # ROS 1-only sources (interface nodes, rostest-only helpers and the tests
    # which need a ROS 1 runtime) are not built on ROS 2 and keep the ROS 1
    # header/style conventions, so they are excluded from the linters.
    # ament_cmake_clang_format ignores this list, but the files are kept
    # clang-format clean so it passes anyway.
    set(AMENT_LINT_AUTO_FILE_EXCLUDE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/dummy_robot.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/patrol.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/planner_2dof_serial_joints.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/planner_3d.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/include/planner_cspace/action_test_base.h
      ${CMAKE_CURRENT_SOURCE_DIR}/test/include/planner_cspace/planner_status.h
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_abort.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_costmap_watchdog.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_debug_outputs.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_dynamic_parameter_change.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_navigate.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_navigate_boundary.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_navigate_remember.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_planner_2dof_serial_joints.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_planner_3d_map_size.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_preempt.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/test/src/test_tolerant_action.cpp)
    # CPP_FORMATTER=clang_format: exclude uncrustify, keep clang_format + cpplint.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    # The neonavigation BSD headers are not recognized by ament_copyright's
    # full-text templates (license=<unknown>), so the copyright linter is
    # excluded. Existing license text is kept as-is.
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)

    # ROS-independent unit tests shared with the ROS 1 build.
    ament_add_gtest(test_${PROJECT_NAME}_blockmem_gridmap
      test/src/test_blockmem_gridmap.cpp)
    target_include_directories(test_${PROJECT_NAME}_blockmem_gridmap PRIVATE include)

    ament_add_gtest(test_${PROJECT_NAME}_grid_astar test/src/test_grid_astar.cpp)
    target_include_directories(test_${PROJECT_NAME}_grid_astar PRIVATE include)
    target_link_libraries(test_${PROJECT_NAME}_grid_astar ${OpenMP_CXX_FLAGS})

    ament_add_gtest(test_${PROJECT_NAME}_cyclic_vec test/src/test_cyclic_vec.cpp)
    target_include_directories(test_${PROJECT_NAME}_cyclic_vec PRIVATE include)

    ament_add_gtest(test_${PROJECT_NAME}_distance_map test/src/test_distance_map.cpp)
    target_include_directories(test_${PROJECT_NAME}_distance_map
      PRIVATE include test/include)
    target_link_libraries(test_${PROJECT_NAME}_distance_map
      planner_3d_core ${OpenMP_CXX_FLAGS})

    ament_add_gtest(test_${PROJECT_NAME}_distance_map_fast_update
      test/src/test_distance_map_fast_update.cpp)
    target_include_directories(test_${PROJECT_NAME}_distance_map_fast_update
      PRIVATE include test/include)
    target_link_libraries(test_${PROJECT_NAME}_distance_map_fast_update
      planner_3d_core ${OpenMP_CXX_FLAGS})

    ament_add_gtest(test_${PROJECT_NAME}_grid_metric_converter
      test/src/test_grid_metric_converter.cpp)
    target_include_directories(test_${PROJECT_NAME}_grid_metric_converter PRIVATE include)
    ament_target_dependencies(test_${PROJECT_NAME}_grid_metric_converter
      costmap_cspace_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

    ament_add_gtest(test_${PROJECT_NAME}_planner_3d_cost test/src/test_planner_3d_cost.cpp)
    target_include_directories(test_${PROJECT_NAME}_planner_3d_cost PRIVATE include)
    target_link_libraries(test_${PROJECT_NAME}_planner_3d_cost planner_3d_core)

    ament_add_gtest(test_${PROJECT_NAME}_costmap_bbf test/src/test_costmap_bbf.cpp)
    target_include_directories(test_${PROJECT_NAME}_costmap_bbf PRIVATE include)
    target_link_libraries(test_${PROJECT_NAME}_costmap_bbf planner_3d_core)

    ament_add_gtest(test_${PROJECT_NAME}_motion_cache test/src/test_motion_cache.cpp)
    target_include_directories(test_${PROJECT_NAME}_motion_cache PRIVATE include)
    target_link_libraries(test_${PROJECT_NAME}_motion_cache planner_3d_core)

    ament_add_gtest(test_${PROJECT_NAME}_motion_primitive_builder
      test/src/test_motion_primitive_builder.cpp)
    target_include_directories(test_${PROJECT_NAME}_motion_primitive_builder PRIVATE include)
    target_link_libraries(test_${PROJECT_NAME}_motion_primitive_builder planner_3d_core)

    ament_add_gtest(test_${PROJECT_NAME}_start_pose_predictor
      test/src/test_start_pose_predictor.cpp)
    target_include_directories(test_${PROJECT_NAME}_start_pose_predictor PRIVATE include)
    target_link_libraries(test_${PROJECT_NAME}_start_pose_predictor planner_3d_core)
  endif()

  ament_auto_package()

else()
  message(
    FATAL_ERROR "ROS_VERSION environment variable not set or unsupported.")
endif()
