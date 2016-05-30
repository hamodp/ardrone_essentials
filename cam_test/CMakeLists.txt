cmake_minimum_required(VERSION 2.4.6)
project(cam_test)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  rospy 
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs 
  message_filters
  sensor_msgs
)


catkin_package(CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  visualization_msgs
  sensor_msgs
) 
include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(ardrone_autonomy REQUIRED)
include_directories(${ardrone_autonomy_INCLUDE_DIRS})
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_executable(cam_test src/cam_test.cpp)
target_link_libraries(cam_test ${catkin_LIBRARIES})
target_link_libraries(cam_test ${OpenCV_LIBS} ${BULLET_LIBRARIES})

#add_dependencies(movement beginner_tutorials_generate_messages_cpp)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${cam_test} src/cam_test.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(cam_test src/cam_test.cpp)
#target_link_libraries(example ${PROJECT_NAME})
