#ifndef MULTICAM_ODOMETER_H
#define MULTICAM_ODOMETER_H

// std includes
#include <iostream>
#include <vector>
#include <stdlib.h>

// Eigen includes
#include <Eigen/Eigen>
#include <Eigen/SVD>

// ROS includes
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>

// Ladybug2 includes
#include <ladybug2/ladybugUtilities.h>

// project includes
#include <multicam_vo/Match.h>
#include <multicam_vo/FeatureMatcher.h>

//! Class responsible for computing the multi-camera system's motion given a set of feature matches 
class MulticamOdometer
{
    public:
        
		/** MulticamOdometer constructor.
         * @param ros::NodeHandle ROS node
         * @return MulticamOdometer object */
        MulticamOdometer(ros::NodeHandle node);
    
        /** MulticamOdometer destructor. */
        ~MulticamOdometer();
    
        /** Estimate the motion of the multi-camera system.
         * @param std::vector<std::vector<Match>> a vector with each camera's matches
		 * @param int number of cameras that form the omnidirectional system
		 * @param std::vector<image_geometry::PinholeCameraModel> vector with each camera's intrinsic parameters
         * @return Eigen::Matrix4f transformation with the relative motion of the multi-camera system */
        Eigen::Matrix4f estimateMotion(std::vector<std::vector<Match>> matches, int numCameras, std::vector<image_geometry::PinholeCameraModel> intrinsics);
    
    private:

        /** Normalize 2D feature points
		 * @param std::vector<Match> vector with matches
		 * @param Eigen::Matrix3f& output normalization matrix of features in the previous frame
		 * @param Eigen::Matrix3f& output normalization matrix of features in the current frame
		 * @return std::vector<Match> vector with normalized matches */
        std::vector<Match> normalize2DPoints(std::vector<Match> matches, Eigen::Matrix3f &NormTPrev, Eigen::Matrix3f &NormTCurr);
    
        /** Get samplesize random integers between zero and totalNumber-1
		 * @param int maximum possibly sectected number + 1
		 * @param int number of random integers to be selected
		 * @return std::vector<int> vector with the random integers */
        std::vector<int> getRandomSample(int totalNumber, int sampleSize);
		
		/** Check if a given integer x exists in a vector of integers, vec
		 * @param std::vector<int> vector vec with integers
		 * @param int integer x to be searched
		 * @return bool true if x is in vec, false otherwise */
        bool elemInVec(std::vector<int> vec, int x);
    
        /** Compute fundamental matrix out of eight feature correspondences between the previous and current frame.
		 * @param std::vector<Match> vector with feature matches
		 * @param std::vector<int> vector with eight indices of the vector with matches
		 * @return Eigen::Matrix3f fundamental matrix */
        Eigen::Matrix3f getFundamentalMatrix(std::vector<Match> matches, std::vector<int> indices);
    
        /** Get the inliers among all matches that comply with a given fundamental matrix.
		 * @param std::vector<Match> vector with feature matches
		 * @param Eigen::Matrix3f fundamental matrix
		 * @return std::vector<int> vector with indices of the inliers */
        std::vector<int> getInliers(std::vector<Match> matches, Eigen::Matrix3f F);
    
        /** Compute essential matrix out of a given fundamental matrix.
		 * @param Eigen::Matrix3f fundamental matrix
		 * @param Eigen::Matrix3f intrinsic matrix of the camera of the previous frame
		 * @param Eigen::Matrix3f intrinsic matrix of the camera of the current frame
		 * @return Eigen::Matrix3f essential matrix */
        Eigen::Matrix3f F2E(Eigen::Matrix3f F, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr);
    
        /** Extract rotation (R) and translation (t) from a given essential matrix and triangulate feature matches.
		 * @param Eigen::Matrix3f essential matrix
		 * @param Eigen::Matrix3f intrinsic matrix of the camera of the previous frame
		 * @param Eigen::Matrix3f intrinsic matrix of the camera of the current frame
		 * @param std::vector<Match> vector with matches
		 * @param Eigen::Matrix3f output rotation matrix
		 * @param Eigen::Vector3f output translation vector
		 * @param Eigen::Matrix<float, 4, Eigen::Dynamic> output matrix with computed 3D points
		 * @return void */
        void EtoRt(Eigen::Matrix3f E, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, std::vector<Match> matches, Eigen::Matrix3f &R, Eigen::Vector3f &t, Eigen::Matrix<float, 4, Eigen::Dynamic> &points3D);
    
        /** Triangulate 3D points.
		 * @param std::vector<Match> vector with matches
		 * @param Eigen::Matrix3f intrinsic matrix of the camera of the previous frame
		 * @param Eigen::Matrix3f intrinsic matrix of the camera of the current frame
		 * @param Eigen::Matrix3f rotation matrix
		 * @param Eigen::Vector3f translation vector
		 * @param Eigen::Matrix<float, 4, Eigen::Dynamic> output matrix with computed 3D points
		 * @return int number of inliers, i.e., points that satisfy the Chirality constraint */
        int triangulate(std::vector<Match> matches, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, Eigen::Matrix3f R, Eigen::Vector3f t, Eigen::Matrix<float, 4, Eigen::Dynamic> &points3D);
    
        /** Get projection matrix out of an intrinsic matrix, a rotation matrix and a translation vector.
		 * @param Eigen::Matrix3f intrinsic matrix 
		 * @param Eigen::Matrix3f rotation matrix
		 * @param Eigen::Vector3f translation vector
		 * @return Eigen::Matrix<float, 3, 4> projection matrix */
        Eigen::Matrix<float, 3, 4> getProjectionMatrix(Eigen::Matrix3f K, Eigen::Matrix3f R, Eigen::Vector3f t);
    
        /** Get points that are closer to the camera than the median distanced point.
		 * @param std::vector<Eigen::Vector4f> vector with all 3D points 
		 * @param double output median depth of all points
		 * @return std::vector<Eigen::Vector4f> vector with close points only */
        std::vector<Eigen::Vector4f> getClosePoints(std::vector<Eigen::Vector4f> points, double &median);
    
		/** Compute optimal translation scale.
		 * @param std::vector<Eigen::Vector4f> vector with 3D points 
		 * @param double median depth of all points
		 * @param camera pitch angle
		 * @param camera height
		 * @return double optimal scale */
        double getTranslationScale(std::vector<Eigen::Vector4f> points3D, double median, double pitch, double height);

		/** Compute transform out of vector with extrinsic parameters, i.e., Euler angles and translation components.
		 * @param std::vector<double> vector with extrinsic parameters
		 * @return Eigen::Matrix4f transform */
        Eigen::Matrix4f transform2ExtrinsicsVector(std::vector<double> extrinsics);
		
		ros::NodeHandle node_; 						/*!< ROS node for reading odometer parameters */
        
		int param_odometerMinNumberMatches_;		/*!< Odometer parameter: minimum number of points required */
        int param_odometerRansacIters_;				/*!< Odometer parameter: number of RANSAC iterations */
        double param_odometerInlierThreshold_;		/*!< Odometer parameter: inlier threshold */
        double param_odometerMotionThreshold_;		/*!< Odometer parameter: motion threshold */
        
		std::vector<double> param_cameraPitches_;	/*!< Camera parameter: each camera's pitch */
        std::vector<double> param_cameraHeights_;	/*!< Camera parameter: each camera's height */

		std::vector<double> param_extrinsicsCam0_;	/*!< Camera parameter: Camera 0's extrinsic parameters */
		std::vector<double> param_extrinsicsCam1_;	/*!< Camera parameter: Camera 1's extrinsic parameters */
		std::vector<double> param_extrinsicsCam2_;	/*!< Camera parameter: Camera 2's extrinsic parameters */
		std::vector<double> param_extrinsicsCam3_;	/*!< Camera parameter: Camera 3's extrinsic parameters */
		std::vector<double> param_extrinsicsCam4_;	/*!< Camera parameter: Camera 4's extrinsic parameters */
		std::vector<double> param_extrinsicsCam5_;	/*!< Camera parameter: Camera 5's extrinsic parameters */
};

#endif