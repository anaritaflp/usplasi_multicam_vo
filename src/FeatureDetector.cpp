#include <multicam_vo/FeatureDetector.h>

/** Default FeatureDetector constructor. 
 * @param void
 * @return FeatureDetector object */
FeatureDetector::FeatureDetector()
{
    
}

/** Default FeatureDetector constructor. By default, Shi-Tomasi corners are detected and an ORB descriptor is extracted.
 * @param ros::NodeHandle ROS node
 * @return FeatureDetector object */
FeatureDetector::FeatureDetector(ros::NodeHandle node): node_(node)
{
    ROS_WARN("No detection method or descriptor type specified. Using ORB detection and ORB descriptor by default.");
    
    detectionMethod_ = SHI_TOMASI;
    descriptorType_ = ORB;
    
    detectPtr_ = &FeatureDetector::detectFeatures_ShiTomasi;
    computeDescriptorPtr_ = &FeatureDetector::computeDescriptor_ORB;
    
    FeatureDetector::getParams(detectionMethod_);
}

/** FeatureDetector constructor with arguments. User may choose detection and descriptor extraction methods.
 * @param ros::NodeHandle ROS node
 * @param std::string detection method
 * @param std::string descriptor type
 * @return FeatureDetector object */
FeatureDetector::FeatureDetector(ros::NodeHandle node, std::string detectionMethod, std::string descriptorType): node_(node)
{
    // if both detection method and descriptor extractor are valid, set function pointers to the corresponding detection and feature extraction methods
    if((detectionMethod == FAST || detectionMethod == BRISK || detectionMethod == ORB || detectionMethod == SHI_TOMASI) && (descriptorType == BRISK || descriptorType == BRIEF || descriptorType == ORB))
    {
        detectionMethod_ = detectionMethod;
        descriptorType_ = descriptorType;
        
        if(detectionMethod == FAST)
            detectPtr_ = &FeatureDetector::detectFeatures_FAST;
        else if(detectionMethod == BRISK)
            detectPtr_ = &FeatureDetector::detectFeatures_BRISK;
        else if(detectionMethod == ORB)
            detectPtr_ = &FeatureDetector::detectFeatures_ORB;
        else if(detectionMethod == SHI_TOMASI)
            detectPtr_ = &FeatureDetector::detectFeatures_ShiTomasi;

        if(descriptorType_ == BRISK)
            computeDescriptorPtr_ = &FeatureDetector::computeDescriptor_BRISK;
        else if(descriptorType_ == BRIEF)
            computeDescriptorPtr_ = &FeatureDetector::computeDescriptor_BRIEF;
        else if(descriptorType_ == ORB)
            computeDescriptorPtr_ = &FeatureDetector::computeDescriptor_ORB;
    }
    // if any of the parameters is invalid, use Shi-Tomasi and ORB by default
    else
    {
        ROS_WARN("Invalid detection method or descriptor type. Using ORB detection and ORB descriptor by default.");
        
        detectionMethod_ = SHI_TOMASI;
        descriptorType_ = ORB;
        detectPtr_ = &FeatureDetector::detectFeatures_ShiTomasi;
        computeDescriptorPtr_ = &FeatureDetector::computeDescriptor_ORB;
    }

    FeatureDetector::getParams(detectionMethod_);
}

/** FeatureDetector destructor. */
FeatureDetector::~FeatureDetector()
{
    
}

/** Detect features.
 * @param cv::Mat image
 * @param int frame number
 * @param int camera index
 * @return std::vector<Feature> vector of features */
std::vector<Feature> FeatureDetector::detectFeatures(cv::Mat image, int seqNo, int camNo)
{
    std::vector<cv::Mat> buckets;
    std::vector<cv::Point2f> offsets;
    
    // divide image into "buckets"
    getBuckets(image, buckets, offsets);
        
    std::vector<Feature> features;
    
    // for each bucket...
    for(int i=0; i<buckets.size(); i++)
    {
        // detect keypoints in the bucket and, for each keypoint, compute a descriptor
        std::vector<cv::KeyPoint> kpts_bucket = ((FeatureDetector*)this->*detectPtr_)(buckets[i]);
        cv::Mat ds = ((FeatureDetector*)this->*computeDescriptorPtr_)(buckets[i], kpts_bucket);
        
        // get the coordinates of the points in the full image
        std::vector<cv::KeyPoint> kpts;
        for(int j=0; j<kpts_bucket.size(); j++)
        {
            cv::KeyPoint kp(kpts_bucket[j]);
            kp.pt = kpts_bucket[j].pt + offsets[i];
            kpts.push_back(kp);
        }        
        
        // get vector with the features in that bucket
        std::vector<Feature> fs = buildFeatures(kpts, ds, seqNo, camNo);

        // append bucket features to the full image feature vector
        features.insert(features.end(), fs.begin(), fs.end());
    }
    
    return features;    
}

/** Highlight features in an image.
 * @param cv::Mat original image
 * @param std::vector<Feature> vector of features
 * @return cv::Mat image with highlighted features */
cv::Mat FeatureDetector::highlightFeatures(cv::Mat image, std::vector<Feature> features)
{
    cv::Mat imageFeatures;
    
    // convert original image (gray scale) to BGR
    cv::cvtColor(image, imageFeatures, CV_GRAY2BGR);
    
    // draw a small green circle around each feature
    for(int i=0; i<features.size(); i++)
    {
        circle(imageFeatures, features[i].getKeypoint().pt, 3, cv::Scalar(0, 255, 0), 2);
    }
    return imageFeatures;
}

/** Get feature detection parameters required for a specified detection method.
 * @param std::string feature detection method
 * @return void */
void FeatureDetector::getParams(std::string detectionMethod)
{
    // feature detection parameters
    if(detectionMethod == FAST)
    {
        node_.param<int>("FAST_threshold", FAST_threshold_, 50);
        node_.param<bool>("FAST_nonMaxSup", FAST_nonMaxSup_, true);
    }
    else if(detectionMethod == BRISK)
    {
        node_.param<int>("BRISK_threshold", BRISK_threshold_, 10);
        node_.param<int>("BRISK_octaves", BRISK_octaves_, 4);
        node_.param<double>("BRISK_patternScale", BRISK_patternScale_, 0.1);
    }
    else if(detectionMethod == ORB)
    {
        node_.param<int>("ORB_maxFeatures", ORB_maxFeatures_, 500);
        node_.param<double>("ORB_scale", ORB_scale_, 1.2);
        node_.param<int>("ORB_nLevels", ORB_nLevels_, 8);
        node_.param<int>("ORB_edgeThreshold", ORB_edgeThreshold_, 31);
        node_.param<int>("ORB_firstLevel", ORB_firstLevel_, 0);
        node_.param<int>("ORB_wtak", ORB_wtak_, 2);
        node_.param<int>("ORB_scoreType", ORB_scoreType_, cv::ORB::HARRIS_SCORE);
        node_.param<int>("ORB_patchSize", ORB_patchSize_, 31);
    }
    else if(detectionMethod == SHI_TOMASI)
    {
        node_.param<int>("ShiTomasi_maxFeatures", ShiTomasi_maxFeatures_, 30);
        node_.param<double>("ShiTomasi_qualityLevel", ShiTomasi_qualityLevel_, 0.01);
        node_.param<double>("ShiTomasi_minDistance", ShiTomasi_minDistance_, 10);
        node_.param<int>("ShiTomasi_blockSize", ShiTomasi_blockSize_, 3);
    }
    
    // bucketing parameters
    node_.param<int>("bucketing_numberHorizontal", bucketing_numberHorizontal_, 1);
    node_.param<int>("bucketing_numberVertical", bucketing_numberVertical_, 2);
}

/** Divide image into sections (buckets).
 * @param cv::Mat image
 * @param std::vector<cv::Mat> output vector with buckets
 * @param std::vector<cv::Point2f> output vector with the pixel coordinates of the upper left corner of each bucket
 * @return void */
void FeatureDetector::getBuckets(cv::Mat image, std::vector<cv::Mat> &buckets, std::vector<cv::Point2f> &offsets)
{
    // compute bucket width and height
    int bucketWidth = round((double)image.cols / bucketing_numberHorizontal_);
    int bucketHeight = round((double)image.rows / bucketing_numberVertical_);
        
    int offset_v = 0;
    double width, height;
    for(int i=0; i<bucketing_numberVertical_; i++)
    {
        int offset_h = 0;
        
        // if it's the last bucket in the column, bucket height is the remainin height
        if(i == bucketing_numberVertical_ - 1)
        {
            height = image.rows - i*bucketHeight;
        }
        else
        {
            height = bucketHeight;
        }
        for(int j=0; j<bucketing_numberHorizontal_; j++)
        {
            // if it's the last bucket in the row, bucket width is the remaining width
            if(j == bucketing_numberHorizontal_ - 1)
            {
                width = image.cols - j*bucketWidth;
            }
            else
            {
                width = bucketWidth;
            }

            // store the bucket (a rectangular section of the original image)
            cv::Mat b = image(cv::Rect(offset_h, offset_v, width, height));
            buckets.push_back(b);
            
            // also store the coordinates of the bucket's upper left corner
            cv::Point2f o(offset_h, offset_v);
            offsets.push_back(o);
            
            offset_h += bucketWidth;
        }
        offset_v += bucketHeight;
    }
}

/** Build set of features out of keypoint, descriptor, frame number and camera index.
 * @param std::vector<cv::KeyPoint> vector of keypoints
 * @param cv::Mat matrix with descriptors
 * @param int frame number
 * @param int camera index 
 * @return std::vector<Feature> */
std::vector<Feature> FeatureDetector::buildFeatures(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors, int seqNo, int camNo)
{
    std::vector<Feature> features;
    for(int i=0; i<keypoints.size(); i++)
    {
        cv::Mat d = descriptors(cv::Rect(0, i, descriptors.cols, 1));
        Feature f(keypoints[i], d, seqNo, camNo);
        features.push_back(f);
    }
    return features;
}

/** Detect FAST corners in an image.
 * @param cv::Mat image
 * @return std::vector<Feature> vector with detected features */
std::vector<cv::KeyPoint> FeatureDetector::detectFeatures_FAST(cv::Mat image)
{
	std::vector<cv::KeyPoint> keypoints;
	cv::FastFeatureDetector detector(FAST_threshold_, FAST_nonMaxSup_);
	detector.detect(image, keypoints);
	return keypoints;
}

/** Detect BRISK features in an image.
 * @param cv::Mat image
 * @return std::vector<Feature> vector with detected features */
std::vector<cv::KeyPoint> FeatureDetector::detectFeatures_BRISK(cv::Mat image)
{
	std::vector<cv::KeyPoint> keypoints;
	cv::BRISK detector(BRISK_threshold_, BRISK_octaves_, BRISK_patternScale_);
	detector.detect(image, keypoints);
	return keypoints;
}

/** Detect ORB features in an image.
 * @param cv::Mat image
 * @return std::vector<Feature> vector with detected features */
std::vector<cv::KeyPoint> FeatureDetector::detectFeatures_ORB(cv::Mat image)
{
	std::vector<cv::KeyPoint> keypoints;
	cv::OrbFeatureDetector detector(ORB_maxFeatures_, ORB_scale_, ORB_nLevels_, ORB_edgeThreshold_, ORB_firstLevel_, ORB_wtak_, ORB_scoreType_, ORB_patchSize_);
	detector.detect(image, keypoints);
	return keypoints;
}

/** Detect Shi-Tomasi corners in an image.
 * @param cv::Mat image
 * @return std::vector<Feature> vector with detected features */
std::vector<cv::KeyPoint> FeatureDetector::detectFeatures_ShiTomasi(cv::Mat image)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::GoodFeaturesToTrackDetector detector(ShiTomasi_maxFeatures_, ShiTomasi_qualityLevel_, ShiTomasi_minDistance_, ShiTomasi_blockSize_);
    detector.detect(image, keypoints);
    return keypoints;
}

/** Compute BRISK descriptors for a set of keypoints.
 * @param cv::Mat image
 * @param std::vector<cv::KeyPoint> vector of keypoints
 * @return cv::Mat matrix with all fetaures' descriptors */
cv::Mat FeatureDetector::computeDescriptor_BRISK(cv::Mat image, std::vector<cv::KeyPoint> &keypoints)
{
	cv::Mat descriptors;
	cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(BRISK);
	extractor->compute(image, keypoints, descriptors); 
	return descriptors;
}

/** Compute BRIEF descriptors for a set of keypoints.
 * @param cv::Mat image
 * @param std::vector<cv::KeyPoint> vector of keypoints
 * @return cv::Mat matrix with all fetaures' descriptors */
cv::Mat FeatureDetector::computeDescriptor_BRIEF(cv::Mat image, std::vector<cv::KeyPoint> &keypoints)
{
	cv::Mat descriptors;
	cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(BRIEF);
	extractor->compute(image, keypoints, descriptors); 
	return descriptors;
}

/** Compute ORB descriptors for a set of keypoints.
 * @param cv::Mat image
 * @param std::vector<cv::KeyPoint> vector of keypoints
 * @return cv::Mat matrix with all fetaures' descriptors */
cv::Mat FeatureDetector::computeDescriptor_ORB(cv::Mat image, std::vector<cv::KeyPoint> &keypoints)
{
	cv::Mat descriptors;
	cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(ORB);
	extractor->compute(image, keypoints, descriptors); 
	return descriptors;
}
