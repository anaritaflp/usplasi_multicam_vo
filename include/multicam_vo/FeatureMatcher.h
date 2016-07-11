#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H

// std includes
#include <vector>
#include <iostream>

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/opencv.hpp>

// Ladybug2 includes
#include <ladybug2/ladybugUtilities.h>

// project includes
#include <multicam_vo/Feature.h>
#include <multicam_vo/Match.h>

//! Class with methods that allow finding matches in an omnidirectional multi-camera system.
class FeatureMatcher
{
    
    public:
    
        /** Default FeatureMatcher constructor.
         * @param void
         * @return a FeatureMatcher object */
        FeatureMatcher();

        /** Default FeatureMatcher constructor.
         * @param ros::NodeHandle ROS node
         * @return a FeatureMatcher object */
        FeatureMatcher(ros::NodeHandle node);
    
        /** FeatureMatcher destructor. */
        ~FeatureMatcher(); 
    
        /** Find matches in an omnidirectional multi-camera system. Matches are search in each camera and between onsecutive cameras.
         * @param std::vector<cv::Mat> vector with all cameras' images in the previous frame
		 * @param std::vector<cv::Mat> vector with all cameras' images in the current frame
		 * @param std::vector<std::vector<Feature>> vector with all cameras' features found in the previous frame
		 * @param std::vector<std::vector<Feature>> vector with all cameras' features found in the current frame
		 * @param int number of images
         * @return std::vector<std::vector<Match>> vector with each camera's matches */
        std::vector<std::vector<Match>> findOmniMatches(std::vector<cv::Mat> imagesPrev, std::vector<cv::Mat> imagesCurr, std::vector<std::vector<Feature>> featuresPrev, std::vector<std::vector<Feature>> featuresCurr, int numCameras);
    
        /** Highlight optical flow in an image.
         * @param cv::Mat original image
		 * @param std::vector<Match> intra-camera matches
         * @return cv::Mat image with highlighted optical flow */
        cv::Mat highlightOpticalFlow(cv::Mat image, std::vector<Match> matches);
    
    private:

        /** Track features in a camera.
         * @param cv::Mat image in the previous frame
		 * @param cv::Mat image in the current frame
		 * @param <std::vector<Feature> vector of features found in the previous frame
		 * @param <std::vector<Feature> vector of features found in the current frame
		 * @param std::vector<Feature> output vector of features in the left camera
		 * @param std::vector<Feature> output vector of features in the right camera
         * @return void */
        void trackIntraCamera(cv::Mat imagePrev, cv::Mat imageCurr, std::vector<Feature> featuresPrev, std::vector<Feature> &featuresCurr, std::vector<Feature> &featuresLeft, std::vector<Feature> &featuresRight);
        
        /** Convert a set of features to a cv::Mat of descriptors.
         * @param std::vector<Feature> vector of features
         * @return cv::Mat matrix with descriptors */
        cv::Mat getDescriptorsFromFeatures(std::vector<Feature> features);
    
        /** Convert set of OpenCV Matches to Matches.
         * @param std::vector<Feature> features in the previous frame
		 * @param std::vector<Feature> features in the current frame
		 * @param std::vector<cv::DMatch> vector of DMatches
         * @return std::vector<Match> vector of matches */
        std::vector<Match> DMatches2Matches(std::vector<Feature> features_1, std::vector<Feature> features_2, std::vector<cv::DMatch> dMatches);

        /** Convert set of features to cv::Point2f.
         * @param std::vector<Feature> features 
         * @return std::vector<cv::Point2f> vector of cv::Point2f */
        std::vector<cv::Point2f> features2Points(std::vector<Feature> features);
    
        ros::NodeHandle node_;                  /*!< ROS node for retrieving matching parameters */ 
        int param_matchingDescriptorDistance_;  /*!< Matching parameter: maximum descriptor distance */ 
        int param_trackingWindowSize_;          /*!< Tracking parameter: tracking window size */
        int param_trackingMaxLevel_;            /*!< Tracking parameter: maximum level */
        double param_trackingMinEigThreshold_;  /*!< Tracking parameter: smallest eigenvalue threshold */
        int param_trackingTermcritCount_;       /*!< Tracking parameter: maximum number of iterations */
        double param_trackingTermcritEPS_;      /*!< Tracking parameter: maximum window motion */
        double param_borderPercentage_;         /*!< Tracking parameter: maximum width percentage that a feature can move */
        int matchCounter;                       /*!< Match counter for saving match images in a folder (for debug) */
    
};


#endif