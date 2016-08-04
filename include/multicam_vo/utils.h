#ifndef UTILS_H
#define UTILS_H

// ROS includes
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// Eigen includes
#include <Eigen/Eigen>

// Ladybug includes
#include <ladybug2/Ladybug2.h>

// project includes
#include <multicam_vo/Match.h>

/** Get previous and current camera associated to a given matches vector position, assuming that the vector is organized this way:
 * position i   -> matches where the previous camera is cam i and the current camera is cam i (intra-camera matches)
 * position i+1 -> matches where the previous camera is cam i and the current camera is the cam right to i (inter-camera matches)
 * position i+2 -> matches where the previous camera is cam i and the current camera is the cam left to i (inter-camera matches)
 * @param Ladybug2 ladybug object
 * @param int vector position
 * @param int& output previous camera index
 * @param int& output current camera index
 * @return bool true if valid vector position, false otherwise */
bool getPrevAndCurrCamIndex(Ladybug2 lb2, int position, int &camPrev, int &camCurr);

/** Get vector position given a previous and current camera index, assuming that the vector is organized this way:
 * position i   -> matches where the previous camera is cam i and the current camera is cam i (intra-camera matches)
 * position i+1 -> matches where the previous camera is cam i and the current camera is the cam right to i (inter-camera matches)
 * position i+2 -> matches where the previous camera is cam i and the current camera is the cam left to i (inter-camera matches)
 * @param Ladybug2 ladybug object
 * @param int previous camera index
 * @param int current camera index
 * @param int& output vector index
 * @return bool true if valid camera indices, false otherwise */
bool getVectorPosition(Ladybug2 lb2, int camPrev, int camCurr, int &position);

/** Check if a given integer x exists in a vector of integers, vec
 * @param std::vector<int> vector vec with integers
 * @param int integer x to be searched
 * @return bool true if x is in vec, false otherwise */
bool elemInVec(std::vector<int> vec, int x);

/** Get samplesize random integers between zero and totalNumber-1
 * @param int maximum possibly sectected number + 1
 * @param int number of random integers to be selected
 * @return std::vector<int> vector with the random integers */
std::vector<int> getRandomSample(int totalNumber, int sampleSize);

/** Convert transform to odometry message.
 * @param Eigen::Matrix4f transform
 * @param int camera index with the most successful motion estimation
 * @return nav_msgs::Odometry odometry message */
nav_msgs::Odometry transform2OdometryMsg(Eigen::Matrix4f T, int bestCamera);

/** Get transform matrix for a given rotation matrix and translation vector.
 * @param Eigen::Matrix3f rotation matrix
 * @param Eigen::Vector3f translation vector
 * @return Eigen::Matrix4f transform */
Eigen::Matrix4f Rt2T(Eigen::Matrix3f R, Eigen::Vector3f t);

/** Extract rotation matrix and translation vector from a given transform matrix.
 * @param Eigen::Matrix4f transform
 * @param Eigen::Matrix3f output rotation matrix
 * @param Eigen::Vector3f output translation vector
 * @return void */
void T2Rt(Eigen::Matrix4f T, Eigen::Matrix3f &R, Eigen::Vector3f &t);

 #endif