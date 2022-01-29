cmake_minimum_required(VERSION 3.7.2)
project(deepsim_btree)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
)

## Tests

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(pytest.launch)
endif()
