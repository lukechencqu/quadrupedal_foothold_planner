#include <ros/ros.h>
#include "foothold_planner/FootholdPlanner.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "foothold_planner");

    ros::NodeHandle node("~");

    std::cout<<"Node foothold_planner starting..."<<std::endl;
    foothold_planner::FootholdPlanner footholdPlanner(node);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    std::cout<<"Node foothold_planner closed."<<std::endl;
    
    return 0;
}