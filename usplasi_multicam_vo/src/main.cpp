#include <ros/ros.h>

#include <stdlib.h>
#include <fstream>
#include <time.h>
#include <signal.h>

#include <multicam_vo/MulticamVOPipeline.h>

std::vector<std::ofstream*> files;

void mySigintHandler(int sig)
{
    for(int i=0; i<NUM_OMNI_CAMERAS; i++)
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
    files.resize(NUM_OMNI_CAMERAS);
    std::string path = "/home/anarita/ROS/catkin_ws/src/experiments/bloco_didatico/";
    for(int i=0; i<NUM_OMNI_CAMERAS; i++)
    {
        char filename[20];
        sprintf(filename, "plot_%02d.m", i);
        files[i] = new std::ofstream(path + std::string(filename));
        (*files[i]) << "pose_" << i << " = [" << std::endl;
    }

    odom::MulticamVOPipeline vo_pipeline(files);    

    signal(SIGINT, mySigintHandler);

    vo_pipeline.spin();     
    return 0;
}

