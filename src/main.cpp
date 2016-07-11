#include <ros/ros.h>

#include <stdlib.h>
#include <time.h>

#include <multicam_vo/MulticamVOPipeline.h>


int main(int argc, char **argv)
{
    srand(time(NULL));
    
    ros::init(argc, argv, "multicam_vo_node");
    
    odom::MulticamVOPipeline vo_node;    
    vo_node.spin();    
    
    return 0;
}