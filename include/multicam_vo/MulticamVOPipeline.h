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
#include <tf/tf.h>

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
	     * @param std::vector<std::ofstream*> vector of files to write estimated poses
		 * @return MulticamVOPipeline object */
         MulticamVOPipeline(std::vector<std::ofstream*> files);
        
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

        bool computeYaw(cv::Mat imTopPrev, cv::Mat imTopCurr, std::vector<Match> matchesTop, double &angle);
        
        /** Compute line that joins two points, in homogeneous coordinates.
         * @param cv::Point2f first point
         * @param cv::Point2f second point
         * @return cv::Point2f line */
        cv::Point2f computeLine(cv::Point2f point1, cv::Point2f point2);

        /** Compute angle between 2 lines.
         * @param cv::Point2f first line
         * @param cv::Point2f second line
         * @return double angle */
        double computeAngle(cv::Point2f line1, cv::Point2f line2);

        /** Convert transform to odometry message.
         * @param Eigen::Matrix4f transform
         * @param int camera index with the most successful motion estimation
         * @return nav_msgs::Odometry odometry message */
        nav_msgs::Odometry transform2OdometryMsg(Eigen::Matrix4f T, int bestCamera);

        ros::NodeHandle node_;										/*!< ROS node for reading parameters */
        image_transport::Subscriber subImage_;						/*!< ROS subscriber to image messages */
        ros::Publisher pubOdom_;									/*!< ROS odometry publisher */ 
		std::string param_imageTopic_;								/*!< Parameter: image topic */
            
		std::vector<image_geometry::PinholeCameraModel> calib;		/*!< Vector with calibration data of each camera */
            
		bool first_;												/*!< Flag for indicating the first frame */
        int lostFrameCounter_;										/*!< Lost frame counter */
			
        int seqNumberPrev_;											/*!< Number of the previously processed frame */
        int seqNumberOffset_;										/*!< Sequence number of the first frame (it's not always zero) */
		std::vector<cv::Mat> imagesRectPrev_;						/*!< Vector with previous rectified images */
        std::vector<std::vector<Feature>> featuresAllCamerasPrev_;	/*!< Vector with each cameras' features in previous frame */
            
		FeatureDetector featureDetector_;							/*!< Feature detector */
        FeatureMatcher featureMatcher_;							    /*!< Feature matcher */	
        MulticamOdometer odometer_;						            /*!< Multi-camera odometer */
    };    
    
    
}


#endif