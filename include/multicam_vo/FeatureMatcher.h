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
#include <ladybug2/Ladybug2.h>

// project includes
#include <multicam_vo/Feature.h>
#include <multicam_vo/Match.h>

//! Class with methods that allow finding matches in an omnidirectional multi-camera system.
class FeatureMatcher
{
    
    public:
    
        /** Default FeatureMatcher Constructor. */
        FeatureMatcher();

        /** FeatureMatcher Constructor.
         * @param Ladybug2 ladybug object
         * @return a FeatureMatcher object */
        FeatureMatcher(Ladybug2 lb2);
    
        /** FeatureMatcher destructor. */
        ~FeatureMatcher(); 

        /** Match two sets of features.
         * @param std::vector<Feature> vector with features in the first image
         * @param std::vector<Feature> vector with features in the second image
         * @return std::vector<Match> vector with feature matches */
        std::vector<Match> matchFeatures(std::vector<Feature> features1, std::vector<Feature> features2);

        /** Highlight optical flow in an image.
        * @param cv::Mat original image
        * @param std::vector<Match> intra-camera matches
        * @param cv::Scalar BGR color
        * @return cv::Mat image with highlighted optical flow */
        cv::Mat highlightOpticalFlow(cv::Mat image, std::vector<Match> matches, cv::Scalar color);

        /** Highlight matches.
        * @param cv::Mat first image
        * @param cv::Mat second image
        * @param std::vector<Match> matches
        * @param std::vector<bool> vector indicating whether matches are valid (true) or not (false)
        * @return cv::Mat image with highlighted optical flow */
        cv::Mat highlightMatches(cv::Mat image1, cv::Mat image2, std::vector<Match> matches, std::vector<bool> validMatch);
    
    private:

        /** Track features in a camera.
         * @param cv::Mat image in the previous frame
		 * @param cv::Mat image in the current frame
		 * @param <std::vector<Feature> vector of features found in the previous frame
		 * @param <std::vector<Feature> vector of features found in the current frame
         * @return std::vector<Match> vector with established matches */
         // INCOMPLETE, NOT SURE IF WILL USE
        std::vector<Match> trackIntraCamera(cv::Mat imagePrev, cv::Mat imageCurr, std::vector<Feature> featuresPrev, std::vector<Feature> featuresCurr);
        
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

        Ladybug2 lb2_;                          /*!< Ladybug2 object */

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