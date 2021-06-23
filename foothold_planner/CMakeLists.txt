cmake_minimum_required(VERSION 2.8.3)
project(foothold_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

####################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    geometry_msgs
    visualization_msgs
	grid_map_ros
	grid_map_core
	grid_map_msgs
    foothold_planner_msgs
    message_generation
)

find_package(NLopt REQUIRED)

set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################## service ##################
add_service_files(FILES
    GlobalFootholdPlan.srv
)
generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
    foothold_planner_msgs
)
####################################
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}_library
  CATKIN_DEPENDS 
    message_runtime
#  DEPENDS system_lib
)

####################################
include_directories(
 include
 lib
 ${catkin_INCLUDE_DIRS}

 SYSTEM
 third_party
 ${NLOPT_INCLUDE_DIRS} 
)

################## libray ##################
add_library(${PROJECT_NAME}_library 
    src/FootholdPlanner.cpp
)
target_link_libraries(${PROJECT_NAME}_library 
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARIES}
)
   
################# executable ##################
add_executable(map_tf 
    src/map_tf.cpp
)
target_link_libraries(map_tf 
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME} 
    src/foothold_planner_node.cpp
)
#add_dependencies(${PROJECT_NAME}
    #${PROJECT_NAME}_generate_messages_cpp
    #${${PROJECT_NAME}_EXPORTED_TARGETS}
    #${catkin_EXPORTED_TARGETS}
    #)
target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_library
)

add_executable(nlopt
    src/nlopt_ros.cpp
    )
target_link_libraries(nlopt
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARIES}
    )
