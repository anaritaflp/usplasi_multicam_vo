#ifndef MULTICAM_ODOMETER_H
#define MULTICAM_ODOMETER_H

// std includes
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>

// Eigen includes
#include <Eigen/Eigen>

// ROS includes
#include <ros/ros.h>

// Ladybug2 includes
#include <ladybug2/Ladybug2.h>

// project includes
#include <multicam_vo/MonoOdometer.h>
#include <multicam_vo/Match.h>
#include <multicam_vo/utils.h>

//! Class for estimating multi-camera visual odometry
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
		* @param std::vector<std::vector<Match>>& output vector with each camera's inlier matches
		* @param std::vector<std::vector<Eigen::Vector3f>>& output vector with each camera's 3D points triangulated from inlier matches
		* @param std::vector<Eigen::Matrix4f>& output vector with each camera's pose estimation, obtained from monocular VO
 		* @return Eigen::Matrix4f transformation with the best relative motion estimate */
		Eigen::Matrix4f estimateMotion(std::vector<std::vector<Match>> matches, int &bestCamera, std::vector<std::vector<Match>> &inlierMatches, std::vector<std::vector<Eigen::Vector3f>> &points3D, std::vector<Eigen::Matrix4f> &monoPoses);

		/** Get the inliers among all matches that comply with a given fundamental matrix.
		 * @param std::vector<Match> vector with feature matches
		 * @param Eigen::Matrix3f fundamental matrix
		 * @param double inlier threshold
		 * @return std::vector<int> vector with indices of the inliers */
        std::vector<int> getInliers(std::vector<Match> matches, Eigen::Matrix3f F, double threshold);

		/** Compute fundamental martix out of rotation and translation.
		* @param Eigen::Matrix3f rotation matrix
		* @param Eigen::Vector3f translation vector
		* @param Eigen::Matrix3f intrinsics matrix
		* @return Eigen::Matrix3f fundamental matrix */
		Eigen::Matrix3f Rt2F(Eigen::Matrix3f R, Eigen::Vector3f t, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr);
    
    private:

		ros::NodeHandle node_; 								/*!< ROS node for reading odometer parameters */

		Ladybug2 lb2_;										/*!< Ladybug2 object */
		std::vector<MonoOdometer> monoOdometers_;
        
        double param_odometerInlierThreshold_;				/*!< Odometer parameter: inlier threshold */
		
		std::vector<std::ofstream*> files_;					/*!< For debugging: for writing estimated poses in matlab files */
		std::vector<bool> firstRow_;						/*!< For debugging: auxiliary flag for writing MatLab files */

		std::vector<Eigen::Matrix4f> absolutePosesLocal_;	/*!< Vector of absolute poses estimated by intra- and consecutive inter-camera matches */
};

#endif
