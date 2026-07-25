include_directories(include)

if(NOT "$ENV{ROS_DISTRO}" STREQUAL "noetic")
  # This flag is needed to compile dynamic_reconfigure::Client on Melodic
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")
endif()

# main() (with ros::Time::init) comes from sq_ros1_rclcpp_compat_gtest_main,
# exported via ${catkin_LIBRARIES}. These pure unit tests reference no ROS
# symbols directly, so --no-as-needed is required to keep librostime on the
# link line (roscpp is a direct dependency ordered ahead of the compat, so its
# librostime would otherwise be dropped before the gtest_main archive that needs
# ros::Time::init() is processed).
catkin_add_gtest(test_${PROJECT_NAME}_filter src/test_filter.cpp)
target_link_libraries(test_${PROJECT_NAME}_filter -Wl,--no-as-needed ${catkin_LIBRARIES})

catkin_add_gtest(test_${PROJECT_NAME}_path2d src/test_path2d.cpp)
target_link_libraries(test_${PROJECT_NAME}_path2d -Wl,--no-as-needed ${catkin_LIBRARIES})

add_rostest_gtest(test_trajectory_recorder test/trajectory_recorder_rostest.test
    src/test_trajectory_recorder.cpp)
target_link_libraries(test_trajectory_recorder ${catkin_LIBRARIES})
add_dependencies(test_trajectory_recorder trajectory_recorder)

add_rostest_gtest(test_trajectory_tracker test/trajectory_tracker_rostest.test
    src/test_trajectory_tracker.cpp)
target_link_libraries(test_trajectory_tracker ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_trajectory_tracker trajectory_tracker)

add_rostest(test/trajectory_tracker_rostest.test
  ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=true
  DEPENDENCIES test_trajectory_tracker
)

add_rostest(test/trajectory_tracker_rostest.test
  ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=false
  DEPENDENCIES test_trajectory_tracker
)

add_rostest_gtest(test_trajectory_tracker_with_odom test/trajectory_tracker_with_odom_rostest.test
    src/test_trajectory_tracker_with_odom.cpp)
target_link_libraries(test_trajectory_tracker_with_odom ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_trajectory_tracker_with_odom trajectory_tracker)

add_rostest_gtest(test_trajectory_tracker_overshoot test/trajectory_tracker_overshoot_rostest.test
    src/test_trajectory_tracker_overshoot.cpp)
target_link_libraries(test_trajectory_tracker_overshoot ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_trajectory_tracker_overshoot trajectory_tracker)
