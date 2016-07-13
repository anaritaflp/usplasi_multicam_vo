#include <ros/ros.h>

#include <stdlib.h>
#include <fstream>
#include <time.h>
#include <signal.h>

#include <multicam_vo/MulticamVOPipeline.h>

std::vector<std::ofstream*> files;

void mySigintHandler(int sig)
{
    for(int i=0; i<3*(NUM_CAMERAS-1); i++)
    {
        (*files[i]) << "\n\t];" << std::endl;
        (*files[i]).close();
    }
    
    ros::shutdown();
}

int main(int argc, char **argv)
{
    srand(time(NULL));
    
    ros::init(argc, argv, "multicam_vo_node");

    // for debugging: write estimated poses in matlab files    
    files.resize(3*(NUM_CAMERAS-1));
    std::string path = "/home/anaritapereira/ROS/catkin_ws/src/multicam_vo/matlab/";
    for(int i=0; i<3*(NUM_CAMERAS-1); i++)
    {
        char filename[20];
        sprintf(filename, "plot_%02d.m", i);
        files[i] = new std::ofstream(path + std::string(filename));
        (*files[i]) << "pose_" << i << " = [" << std::endl;
    }

    odom::MulticamVOPipeline vo_node(files);    

    signal(SIGINT, mySigintHandler);

    vo_node.spin();     
    return 0;
}

