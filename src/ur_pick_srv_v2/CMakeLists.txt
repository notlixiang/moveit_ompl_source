cmake_minimum_required(VERSION 2.8.3)
project(ur_pick_srv_v2)

add_compile_options(-std=c++11)

find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    kdl_parser
    trac_ik_lib
)

catkin_package(
  CATKIN_DEPENDS
	roscpp 
	std_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    trac_ik_lib
)

include_directories(${catkin_INCLUDE_DIRS} "../../devel/include" "/usr/include/eigen3")

add_executable(ur_pick_srv_v2_node src/ur_pick_srv.cpp)
target_link_libraries(ur_pick_srv_v2_node ${catkin_LIBRARIES})
