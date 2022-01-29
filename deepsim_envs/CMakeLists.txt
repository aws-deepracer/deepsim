cmake_minimum_required(VERSION 3.7.2)
project(deepsim_envs)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_srvs
    ude_ros_env
    deepsim
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_srvs
    ude_ros_env
    deepsim
)

## Tests

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(pytest.launch)
endif()
