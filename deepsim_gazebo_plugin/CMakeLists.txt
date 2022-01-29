cmake_minimum_required(VERSION 3.0.2)
project(deepsim_gazebo_plugin)

find_package(catkin REQUIRED COMPONENTS
             roslint
             roslib
             roscpp
             std_srvs
             std_msgs
             geometry_msgs
             gazebo_msgs
             gazebo_dev
             deepsim_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    LIBRARIES
    deepsim_gazebo_plugin

    CATKIN_DEPENDS
        roslib
        roscpp
        gazebo_ros
        geometry_msgs
        std_srvs
        std_msgs
        gazebo_msgs
        deepsim_msgs)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()

## Plugins
add_library(deepsim_gazebo_plugin 
  src/deepsim_gazebo_system_plugin.cpp
  src/deepsim_gazebo_world_services.cpp
  src/deepsim_gazebo_state_services.cpp
  src/deepsim_gazebo_state.cpp
  src/deepsim_gazebo_visual_services.cpp
  src/deepsim_gazebo_visual_publisher.cpp
  src/converters/geometry_converter.cpp
  src/converters/visual_converter.cpp)
add_dependencies(deepsim_gazebo_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(deepsim_gazebo_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(deepsim_gazebo_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(deepsim_gazebo_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS deepsim_gazebo_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## For examples
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

## Skipping Copyrights check for now.
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-runtime/references,-runtime/init")
roslint_cpp(
  src/deepsim_gazebo_system_plugin.cpp
  src/deepsim_gazebo_world_services.cpp
  src/deepsim_gazebo_state_services.cpp
  src/deepsim_gazebo_state.cpp
  src/deepsim_gazebo_visual_services.cpp
  src/deepsim_gazebo_visual_publisher.cpp
  src/converters/geometry_converter.cpp
  src/converters/visual_converter.cpp)