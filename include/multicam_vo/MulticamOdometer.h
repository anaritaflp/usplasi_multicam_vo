#ifndef MULTICAM_ODOMETER_H
#define MULTICAM_ODOMETER_H

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
#include <image_geometry/pinhole_camera_model.h>

// Ladybug2 includes
#include <ladybug2/Ladybug2.h>

// project includes
#include <multicam_vo/Match.h>
#include <multicam_vo/FeatureMatcher.h>
#include <multicam_vo/utils.h>

#define PI (3.141592653589793)

//! Class responsible for computing the multi-camera system's motion given a set of feature matches 
class MulticamOdometer
{
    public:
        
		/** MulticamOdometer default constructor. */
		MulticamOdometer();

		/** MulticamOdometer constructor.
		 * @param Ladybug2 ladybug object
		 * @param std::vector<std::ofstream*> vector with files to write estimated poses
         * @return MulticamOdometer object */
        MulticamOdometer(Ladybug2 lb2, std::vector<std::ofstream*> files);
    
        /** MulticamOdometer destructor. */
        ~MulticamOdometer();
    
        /** Estimate the motion of the multi-camera system.
		* @param std::vector<std::vector<Match>> a vector with each camera's matches
		* @param vector of output matlab files to be filled with the estimated poses
		* @param int& output index of the camera with the most successful motion estimation
		* @return Eigen::Matrix4f transformation with the relative motion of the multi-camera system */
		Eigen::Matrix4f estimateMotion(std::vector<std::vector<Match>> matches, int &bestCamera);
    
    private:

        /** Normalize 2D feature points
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

		/** Compute fundamental martix out of rotation and translation.
		* @param Eigen::Matrix3f rotation matrix
		* @param Eigen::Vector3f translation vector
		* @param Eigen::Matrix3f intrinsics matrix of the previous camera
		* @param Eigen::Matrix3f intrinsics matrix of the current camera
		* @return Eigen::Matrix3f fundamental matrix */
		Eigen::Matrix3f Rt2F(Eigen::Matrix3f R, Eigen::Vector3f t, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr);

		ros::NodeHandle node_; 								/*!< ROS node for reading odometer parameters */

		Ladybug2 lb2_;										/*!< Ladybug2 object */
        
		int param_odometerMinNumberMatches_;				/*!< Odometer parameter: minimum number of points required */
        int param_odometerRansacIters_;						/*!< Odometer parameter: number of RANSAC iterations */
        double param_odometerInlierThreshold_;				/*!< Odometer parameter: inlier threshold */
        double param_odometerMotionThreshold_;				/*!< Odometer parameter: motion threshold */
        
		std::vector<double> param_cameraPitches_;			/*!< Camera parameter: each camera's pitch */
        std::vector<double> param_cameraHeights_;			/*!< Camera parameter: each camera's height */

		std::vector<Eigen::Matrix4f> absolutePosesLocal;	/*!< Vector of absolute poses estimated by intra- and consecutive inter-camera matches */

		std::vector<std::ofstream*> files_;					/*!< For debugging: for writing estimated poses in matlab files */
		std::vector<bool> firstRow_;						/*!< For debugging: auxiliary flag for writing MatLab files */

		Eigen::Matrix4f absolutePoseGlobal;					/*!< Absolute pose estimated in the previous frame */
};

#endif
