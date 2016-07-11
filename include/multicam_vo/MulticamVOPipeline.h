#ifndef MULTICAM_VO_PIPELINE_H
#define MULTICAM_VO_PIPELINE_H

// std includes
#include <fstream>
#include <algorithm>

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse_yml.h>

// Ladybug2 includes
#include <ladybug2/ladybugUtilities.h>

// project includes
#include <multicam_vo/FeatureDetector.h>
#include <multicam_vo/FeatureMatcher.h>
#include <multicam_vo/MulticamOdometer.h>

namespace odom
{
    
    class MulticamVOPipeline
    {
        public:
        
            /** Default MulticamVOPipeline constructor.
			 * @param void
			 * @return MulticamVOPipeline object */
            MulticamVOPipeline();
        
            /** Default MulticamVOPipeline destructor. */
            ~MulticamVOPipeline();
        
            /** ROS loop.
			 * @param void
			 * @return void */
            void spin();
            
        
        private:
        
            /** Ladybug2 image callback.
			 * @param sensor_msgs::Image::ConstPtr& ROS image message
			 * @return void */
            void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
        
            /** Rectify individual images.
			 * @param std::vector<cv::Mat> vector with distorted images
			 * @return std::vector<cv::Mat> vector with rectified images */
            std::vector<cv::Mat> rectify(std::vector<cv::Mat> images);
        
            /** Read all camera's calibration parameters from yaml files.
			 * @param void
			 * @return void */
            void getCalibration();
        
            ros::NodeHandle node_;										/*!< ROS node for reading parameters */
            image_transport::Subscriber subImage_;						/*!< ROS subscriber to image messages */
            ros::Publisher pubOdom_;									/*!< ROS odometry publisher */ 
			std::string param_imageTopic_;								/*!< Parameter: image topic */
            
			std::vector<image_geometry::PinholeCameraModel> calib;		/*!< Vector with calibration data of each camera */
            
			bool first_;												/*!< Flag for indicating the first frame */
            int lostFrameCounter;										/*!< Lost frame counter */
            
			int seqNumberPrev;											/*!< Number of the previously processed frame */
			std::vector<cv::Mat> imagesRectPrev;						/*!< Vector with previous rectified images */
            std::vector<std::vector<Feature>> featuresAllCamerasPrev;	/*!< Vector with each cameras' features in previous frame */
            
			FeatureDetector featureDetector;							/*!< Feature detector */
            FeatureMatcher featureMatcher;								/*!< Feature matcher */

    };    
    
    
}


#endif