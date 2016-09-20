#include <ros/ros.h>

#include <stdlib.h>
#include <fstream>
#include <time.h>
#include <signal.h>

#include <multicam_vo/MulticamVOSimPipeline.h>

std::vector<std::ofstream*> files;

int main(int argc, char **argv)
{
    srand(time(NULL));
    
    ros::init(argc, argv, "multicam_vo_sim_node");

    // for debugging: write estimated poses in matlab files    
    files.resize(NUM_OMNI_CAMERAS);
    std::string path = "/home/anaritapereira/ROS/catkin_ws/src/multicam_vo/matlab/";
    for(int i=0; i<NUM_OMNI_CAMERAS; i++)
    {
        char filename[20];
        sprintf(filename, "plot_%02d.m", i);
        files[i] = new std::ofstream(path + std::string(filename));
        (*files[i]) << "pose_" << i << " = [" << std::endl;
    }

    odom_sim::MulticamVOSimPipeline vo_sim_pipeline(files);    
    vo_sim_pipeline.loop(files);
 
    return 0;
}

 