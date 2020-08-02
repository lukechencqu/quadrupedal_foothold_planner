cmake_minimum_required(VERSION 2.8.3)
project(foothold_planner_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(FILES  
    Foothold.msg
    GlobalFootholds.msg
)

#generate_messages必须在catkin_package前面和在add_message_files的后面
generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)

find_package(roslaunch )

#add_dependencies()

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


