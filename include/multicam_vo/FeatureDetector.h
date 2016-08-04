#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

// std includes
#include <iostream>
#include <vector>
#include <math.h>

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"

// project includes
#include <multicam_vo/Feature.h>

const extern std::string FAST = "FAST";
const extern std::string BRISK = "BRISK";
const extern std::string BRIEF = "BRIEF";
const extern std::string ORB = "ORB";
const extern std::string SHI_TOMASI = "SHI_TOMASI";

//! Class containing feature detection and descriptor extraction methods.
class FeatureDetector
{
    public:
    
        /** Default FeatureDetector constructor. By default, Shi-Tomasi corners are detected and an ORB descriptor is extracted.
         * @param void
         * @return FeatureDetector object */
        FeatureDetector();
    
        /** FeatureDetector constructor with arguments. User may choose detection and descriptor extraction methods.
		 * @param std::string detection method
		 * @param std::string descriptor type
         * @return FeatureDetector object */
        FeatureDetector(std::string detectionMethod, std::string descriptorType);
        
        /** FeatureDetector destructor. */
        ~FeatureDetector();
    
        /** Detect features.
         * @param cv::Mat image
		 * @param int frame number
		 * @param int camera index
         * @return std::vector<Feature> vector of features */
        std::vector<Feature> detectFeatures(cv::Mat image, int seqNo, int camNo);
    
        /** Highlight features in an image.
         * @param cv::Mat original image
		 * @param std::vector<Feature> vector of features
         * @return cv::Mat image with highlighted features */
        cv::Mat highlightFeatures(cv::Mat image, std::vector<Feature> features);
    
    private:
    
        /** Get feature detection parameters required for a specified detection method.
         * @param std::string feature detection method
         * @return void */
        void getParams(std::string detectionMethod);

        /** Divide image into sections (buckets).
         * @param cv::Mat image
		 * @param std::vector<cv::Mat> output vector with buckets
		 * @param std::vector<cv::Point2f> output vector with the pixel coordinates of the upper left corner of each bucket
         * @return void */
        void getBuckets(cv::Mat image, std::vector<cv::Mat> &buckets, std::vector<cv::Point2f> &offsets);

        /** Build set of features out of keypoint, descriptor, frame number and camera index.
         * @param std::vector<cv::KeyPoint> vector of keypoints
		 * @param cv::Mat matrix with descriptors
		 * @param int frame number
		 * @param int camera index 
         * @return std::vector<Feature> */
        std::vector<Feature> buildFeatures(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors, int seqNo, int camNo);

        /** Extract set of cv::Point2f from set of cv::KeyPoint.
         * @param std::vector<cv::KeyPoint> keypoints
         * @return std::vector<cv::Point2f> points */
        std::vector<cv::Point2f> keypoints2points(std::vector<cv::KeyPoint> keypoints);

        /** Update cv::Point2f points in set of cv::KeyPoint keypoints.
         * @param std::vector<cv::KeyPoint>& input/output set of keypoints
         * @param std::vector<cv::Point2f> new points */
        void points2keypoints(std::vector<cv::KeyPoint> &keypoints, std::vector<cv::Point2f> points);

        /** Detect FAST corners in an image.
         * @param cv::Mat image
         * @return std::vector<Feature> vector with detected features */
        std::vector<cv::KeyPoint> detectFeatures_FAST(cv::Mat image);

        /** Detect BRISK features in an image.
         * @param cv::Mat image
         * @return std::vector<Feature> vector with detected features */
        std::vector<cv::KeyPoint> detectFeatures_BRISK(cv::Mat image);

        /** Detect ORB features in an image.
         * @param cv::Mat image
         * @return std::vector<Feature> vector with detected features */
        std::vector<cv::KeyPoint> detectFeatures_ORB(cv::Mat image);
    
        /** Detect Shi-Tomasi corners in an image.
         * @param cv::Mat image
         * @return std::vector<Feature> vector with detected features */
        std::vector<cv::KeyPoint> detectFeatures_ShiTomasi(cv::Mat image);

        /** Compute BRISK descriptors for a set of keypoints.
         * @param cv::Mat image
		 * @param std::vector<cv::KeyPoint> vector of keypoints
         * @return cv::Mat matrix with all fetaures' descriptors */
        cv::Mat computeDescriptor_BRISK(cv::Mat image, std::vector<cv::KeyPoint> &keypoints);

        /** Compute BRIEF descriptors for a set of keypoints.
         * @param cv::Mat image
		 * @param std::vector<cv::KeyPoint> vector of keypoints
         * @return cv::Mat matrix with all fetaures' descriptors */
        cv::Mat computeDescriptor_BRIEF(cv::Mat image, std::vector<cv::KeyPoint> &keypoints);

        /** Compute ORB descriptors for a set of keypoints.
         * @param cv::Mat image
		 * @param std::vector<cv::KeyPoint> vector of keypoints
         * @return cv::Mat matrix with all fetaures' descriptors */
        cv::Mat computeDescriptor_ORB(cv::Mat image, std::vector<cv::KeyPoint> &keypoints);

        ros::NodeHandle node_;          /*!< ROS node for retrieving feature detection parameters */
        std::string detectionMethod_;   /*!< String with the feature detection method */
        std::string descriptorType_;    /*!< String with the descriptor extraction method */
        std::vector<cv::KeyPoint> (FeatureDetector::*detectPtr_) (cv::Mat image);    /*!< Function pointer to the specified feature detection method */
        cv::Mat (FeatureDetector::*computeDescriptorPtr_) (cv::Mat image, std::vector<cv::KeyPoint> &keypoints);    /*!< Function pointer to the specified descriptor extraction method */

        int FAST_threshold_;                /*!< FAST detection parameter: threshold */
        bool FAST_nonMaxSup_;               /*!< FAST detection parameter: use non-maximum supression (true) or not (false) */
        int BRISK_threshold_;               /*!< BRISK detection parameter: threshold */
        int BRISK_octaves_;                 /*!< BRISK detection parameter: number of octaves */
        double BRISK_patternScale_;         /*!< BRISK detection parameter: patrern scale */
        int ORB_maxFeatures_;               /*!< ORB detection parameter: maximum number of features */
        double ORB_scale_;                  /*!< ORB detection parameter: scale */
        int ORB_nLevels_;                   /*!< ORB detection parameter: number of levels */
        int ORB_edgeThreshold_;             /*!< ORB detection parameter: edge threshold */
        int ORB_firstLevel_;                /*!< ORB detection parameter: first level */
        int ORB_wtak_;                      /*!< ORB detection parameter: WTA-K */
        int ORB_scoreType_;                 /*!< ORB detection parameter: score type */
        int ORB_patchSize_;                 /*!< ORB detection parameter: patch size */
        int ShiTomasi_maxFeatures_;         /*!< Shi-Tomasi detection parameter: maximum number of features */
        double ShiTomasi_qualityLevel_;     /*!< Shi-Tomasi detection parameter: quality level */
        double ShiTomasi_minDistance_;      /*!< Shi-Tomasi detection parameter: minimum distance between features */
        int ShiTomasi_blockSize_;           /*!< Shi-Tomasi detection parameter: block size */
        int bucket_width_;                  /*!< Bucketing parameter: number of horizontal sections */
        int bucket_height_;                 /*!< Bucketing parameter: number of vertical sections */
};


#endif

