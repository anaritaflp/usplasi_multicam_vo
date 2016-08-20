#ifndef MONO_ODOMETER_H
#define MONO_ODOMETER_H

// std includes
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>

// Eigen includes
#include <Eigen/Eigen>
#include <Eigen/SVD>

// ROS includes
#include <ros/ros.h>

// project includes
#include <multicam_vo/Match.h>
#include <multicam_vo/FeatureMatcher.h>
#include <multicam_vo/utils.h>

//! Class for estimating monocular visual odometry
class MonoOdometer
{
    public:

        /** Default constructor. */
        MonoOdometer();

        /** Destructor. */
        ~MonoOdometer();

        /** Estimate monocular visual odometry.
	  	 * @param std::vector<Match> vector with matches
		 * @param Eigen::Matrix3f matrix with intrinsic parameters of previous camera
		 * @param Eigen::Matrix3f matrix with intrinsic parameters of current camera
		 * @param Eigen::Matrix3f& (output) estimated rotation matrix
		 * @param Eigen::Vector3f& (output) estimated translation vector
		 * @param bool show optical flow (true), don't show otherwise
		 * @param std::vector<Match> output vector with all inlier matches
		 * @param std::vector<Eigen::Vector3f> output vector with 3D points, triangulated from all inlier matches
		 * @return bool true is motion successfully estimated, false otherwise */
		bool estimateMotion(std::vector<Match> matches, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, Eigen::Matrix3f &R, Eigen::Vector3f &t, bool showOpticalFlow, std::vector<Match> &inlierMatches, std::vector<Eigen::Vector3f> &points3D);

    private:

        /** Normalize 2D feature points.
		 * @param std::vector<Match> vector with matches
		 * @param Eigen::Matrix3f& output normalization matrix of features in the previous frame
		 * @param Eigen::Matrix3f& output normalization matrix of features in the current frame
		 * @return std::vector<Match> vector with normalized matches */
        std::vector<Match> normalize2DPoints(std::vector<Match> matches, Eigen::Matrix3f &NormTPrev, Eigen::Matrix3f &NormTCurr);

        /** Compute fundamental matrix out of eight feature correspondences between the previous and current frame.
		 * @param std::vector<Match> vector with feature matches
		 * @param std::vector<int> vector with eight indices of the vector with matches
		 * @return Eigen::Matrix3f fundamental matrix */
        Eigen::Matrix3f getF(std::vector<Match> matches, std::vector<int> indices);

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
        void E2Rt(Eigen::Matrix3f E, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, std::vector<Match> matches, Eigen::Matrix3f &R, Eigen::Vector3f &t, Eigen::Matrix<float, 4, Eigen::Dynamic> &points3D);
    
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
		 * @param double output median distance of all points
		 * @return std::vector<Eigen::Vector4f> vector with close points only */
        std::vector<Eigen::Vector4f> getClosePoints(std::vector<Eigen::Vector4f> points, double &median);
    
		/** Compute optimal translation scale.
		 * @param std::vector<Eigen::Vector4f> vector with 3D points 
		 * @param double median distance of all points
		 * @param camera pitch angle
		 * @param camera height
		 * @return double optimal scale */
        double getTranslationScale(std::vector<Eigen::Vector4f> points3D, double median, double pitch, double height);

        ros::NodeHandle node_; 								/*!< ROS node for reading odometer parameters */
        
		int param_odometerMinNumberMatches_;				/*!< Odometer parameter: minimum number of points required */
        int param_odometerRansacIters_;						/*!< Odometer parameter: number of RANSAC iterations */
        double param_odometerInlierThreshold_;				/*!< Odometer parameter: inlier threshold */
        double param_odometerMotionThreshold_;				/*!< Odometer parameter: motion threshold */
        std::vector<double> param_cameraPitches_;			/*!< Camera parameter: each camera's pitch */
        std::vector<double> param_cameraHeights_;			/*!< Camera parameter: each camera's height */
};

#endif