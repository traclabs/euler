cmake_minimum_required(VERSION 2.8.3)
project(euler_gazebo)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  gazebo_ros
  joint_trajectory_controller
  diff_drive_controller
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  gazebo_msgs
  nav_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package( gazebo REQUIRED )
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )
link_directories( /opt/ros/kinetic/lib )
include_directories( ${OGRE_INCLUDE_DIRS}/Paging )
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   control_msgs#   sensor_msgs#   std_msgs#   trajectory_msgs
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS #include
#  LIBRARIES euler_gazebo
#  CATKIN_DEPENDS control_msgs gazebo_ros joint_trajectory_controller roscpp rospy sensor_msgs std_msgs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  .
  include 
  ${catkin_INCLUDE_DIRS}
)

# add_executable(model_states_converter src/model_states_to_tf_and_odom.cpp)
# target_link_libraries(model_states_converter ${catkin_LIBRARIES})

# add_executable(object_model_to_tf src/object_model_to_tf.cpp)
# target_link_libraries(object_model_to_tf ${catkin_LIBRARIES})

# add_executable(affordance_rviz_mirror src/affordance_rviz_mirror.cpp)
# target_link_libraries(affordance_rviz_mirror ${catkin_LIBRARIES})

# add_library( fixed_ros_openni_kinect SHARED src/gazebo_ros_openni_kinect.cpp )
# target_link_libraries(fixed_ros_openni_kinect gazebo_ros_camera_utils DepthCameraPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
