cmake_minimum_required(VERSION 3.10.0)
project(neonavigation)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED)
  catkin_metapackage()
endif()
