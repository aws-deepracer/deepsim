cmake_minimum_required(VERSION 3.7.2)
project(deepsim)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    rosgraph_msgs
    gazebo_msgs
    deepsim_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    rosgraph_msgs
    gazebo_msgs
    deepsim_msgs
)

## Tests

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(pytest.launch)
endif()
