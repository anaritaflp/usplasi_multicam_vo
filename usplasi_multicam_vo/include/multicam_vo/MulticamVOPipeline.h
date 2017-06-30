#ifndef MULTICAM_VO_PIPELINE_H
#define MULTICAM_VO_PIPELINE_H

// std includes
#include <fstream>
#include <algorithm>

// Eigen includes
#include <Eigen/Eigen>

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
#include <ladybug2/Ladybug2.h>

// project includes
#include <multicam_vo/MulticamOdometer.h>
#include <multicam_vo/utils.h>
#include <multicam_vo/Match.h>
#include <multicam_vo/vo_features.h>

#define MIN_FEATURES 30

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

        Ladybug2 lb2_;                                               /*!< Ladybug2 object */    

        ros::NodeHandle node_;										/*!< ROS node for reading parameters */
        image_transport::Subscriber subImage_;						/*!< ROS subscriber to image messages */
        ros::Publisher pubOdom_;									/*!< ROS odometry publisher */ 
		
        std::string param_imageTopic_;								/*!< Parameter: image topic */
            
		bool first_;												/*!< Flag for indicating the first frame */
        int lostFrameCounter_;										/*!< Lost frame counter */

        Eigen::Matrix4f absolutePoseGlobal_;
        double scale_;
			
        int seqNumberPrev_;											/*!< Number of the previously processed frame */
        int seqNumberOffset_;										/*!< Sequence number of the first frame (it's not always zero) */
		std::vector<cv::Mat> imagesRectPrev_;						/*!< Vector with previous rectified images */
        std::vector<cv::Mat> imagesRectReducedPrev_;
        std::vector<std::vector<cv::Point2f>> featuresAllCamerasPrev_;	/*!< Vector with each cameras' features in previous frame */
        std::vector<Eigen::Vector3f> stereoPointsPrev_;

        MulticamOdometer odometer_;						            /*!< Multi-camera odometer */

        std::vector<std::vector<double>> param_ROIs_;               /*!< Region of interest of all cameras */
        std::vector<std::vector<double>> cameraOverlaps_;	        /*!< Vector with each camera's left and right overlap limits: Pixels under the left limit overlap with the left camera and pixels above the right limit overlap with the right camera. */		
        std::vector<int> numInliers;
        int counter;
};    
    
    
}


#endif