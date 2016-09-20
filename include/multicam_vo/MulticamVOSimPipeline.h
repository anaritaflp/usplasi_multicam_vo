#ifndef MULTICAM_VO_SIM_PIPELINE_H
#define MULTICAM_VO_SIM_PIPELINE_H

// std includes
#include <fstream>
#include <algorithm>
#include <random>

// Eigen includes (GTSAM adaptation)
//#include </home/anaritapereira/gtsam-3.2.1/gtsam/3rdparty/Eigen/Eigen/Core>
#include <Eigen/Eigen>

// Boost includes
#include <boost/algorithm/string.hpp>

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>

// Ladybug2 includes
#include <ladybug2/Ladybug2.h>

// project includes
#include <multicam_vo/MulticamOdometer.h>
#include <multicam_vo/utils.h>

namespace odom_sim
{

class MulticamVOSimPipeline
{
    public:
        
        /** Default MulticamVOSimPipeline constructor.
	     * @param std::vector<std::ofstream*> vector of files to write estimated poses
		 * @return MulticamVOPipeline object */
         MulticamVOSimPipeline(std::vector<std::ofstream*> files);
        
        /** Default MulticamVOSimPipeline destructor. */
        ~MulticamVOSimPipeline();

        /** Loop function, where all files with simulated points are read and motion estimation is computed.
         * @param std::vector<std::ofstream*> vector of output files where individual camera motion estimates shall be written
         * @return void */
        void loop(std::vector<std::ofstream*> files);

    private:

        /** Create a match out of corresponding point coordinates.
         * @param double u-coordinate in previous image
         * @param double v-coordinate in previous image
         * @param double u-coordinate in current image
         * @param double v-coordinate in current image
         * @param int point index in the points file
        * @param int frame number
        * @param int index of the previous camera
        * @param int index of the current camera
        * @return Match match */
        Match createMatch(double uPrev, double vPrev, double uCurr, double vCurr, int pointIndex, int frameCounter, int camNumberPrev, int camNumberCurr);

        /** Reduce a vector of matches to N randomly selected items.
         * @param std::vector<Match> original vector with matches
         * @param int desired number of matches
         * @return std::vector<Match> reduced vector with N of matches */
        std::vector<Match> reduceMatches(std::vector<Match> matches, int N);

        std::vector<Match> addNoise(std::vector<Match> matches);

        bool allDescriptorsDifferent();

        void createDescriptors();

        std::vector<cv::Mat> descriptors_;

        ros::NodeHandle node_;										/*!< ROS node for reading parameters */
        ros::Publisher pubOdom_;									/*!< ROS odometry publisher */ 

        Ladybug2 lb2_;                                              /*!< Ladybug2 object */  
        
        MulticamOdometer odometer_;						            /*!< Multi-camera odometer */ 

        std::string param_pathToSimPoints_;                         /*!< Path to directory with simulated points */
        int param_numFrames_;                                       /*!< Number of frames */
        int param_numPoints_;
        double param_noiseVariance_;                                /*!< Variance of gaussian noise introduced in the 2D points */
};

}

#endif