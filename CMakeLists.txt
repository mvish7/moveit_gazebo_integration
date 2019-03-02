cmake_minimum_required(VERSION 2.8.3)
project(robot_moveit_config)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
             roscpp std_msgs 
             moveit_ros_planning
             moveit_ros_planning_interface
             moveit_core
             tf2_ros
             tf2_geometry_msgs
             moveit_msgs
             geometric_shapes)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
)

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


add_executable(moveit_interface src/moveit_interface.cpp)
target_link_libraries(moveit_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS moveit_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
