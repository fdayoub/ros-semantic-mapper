#include <ros/ros.h>
#include "semantic_mapper.h"

int main(int argc, char** argv){
    ROS_INFO_ONCE("semantic mapper started");
    ros::init(argc,argv,"semantic_mapper");
    ros::NodeHandle nh; 
    ros::NodeHandle lnh("~");

    SemanticMapper sm(nh,lnh);
    ros::spin();

}
