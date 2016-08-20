#include <multicam_vo/FeatureDetector.h>

/** Default FeatureDetector constructor. By default, Shi-Tomasi corners are detected and an ORB descriptor is extracted.
 * @param void
 * @return FeatureDetector object */
FeatureDetector::FeatureDetector(): node_("~")
{

}

/** FeatureDetector constructor with arguments. User may choose detection and descriptor extraction methods.
 * @param std::string detection method
 * @param std::string descriptor type
 * @return FeatureDetector object */
FeatureDetector::FeatureDetector(std::string detectionMethod, std::string descriptorType): node_("~")
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
 * @param bool divide image in buckets (true) or search in the image as a whole (false)
 * @param double resize factor, for resizing small images in order to allow extracting descriptors
 * @return std::vector<Feature> vector of features */
std::vector<Feature> FeatureDetector::detectFeatures(cv::Mat image, int seqNo, int camNo, bool doBucketing, double resizeFactor)
{
    std::vector<cv::Mat> buckets;
    std::vector<cv::Point2f> offsets;
    
    // divide image into "buckets"
    if(bucket_width_ == 0 || bucket_height_ == 0 || !doBucketing)
    {
        buckets.push_back(image);
        offsets.push_back(cv::Point2f(0.0, 0.0));
    }
    else
    {
        getBuckets(image, buckets, offsets);
    }

    std::vector<Feature> features;
    
    // for each bucket...
    for(int i=0; i<buckets.size(); i++)
    {
        // resize the image for finding descriptors
        cv::Mat bucketToGo;
        if(resizeFactor != 1.0)
        {
            cv::resize(buckets[i], bucketToGo, cv::Size(resizeFactor*buckets[i].cols, resizeFactor*buckets[i].rows));
        }
        else
        {
            bucketToGo = buckets[i];
        }

        // detect keypoints in the bucket
        std::vector<cv::KeyPoint> kpts_bucket = ((FeatureDetector*)this->*detectPtr_)(bucketToGo);

        // extract descriptors from the keypoints
        cv::Mat ds = ((FeatureDetector*)this->*computeDescriptorPtr_)(bucketToGo, kpts_bucket);

        // bring keypoints back to the un-resized image
        for(int j=0; j<kpts_bucket.size(); j++)
        {
            kpts_bucket[j].pt.x /= resizeFactor;
            kpts_bucket[j].pt.y /= resizeFactor;
        }

        if(kpts_bucket.size() > 0)
        {
            // subpixel refinement
            std::vector<cv::Point2f> pts = keypoints2points(kpts_bucket);
            cv::Size winSize = cv::Size(5, 5);
            cv::Size zeroZone = cv::Size(-1, -1);
            cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
            cv::cornerSubPix(bucketToGo, pts, winSize, zeroZone, criteria);            
            points2keypoints(kpts_bucket, pts);
        }

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
    node_.param<int>("bucket_width", bucket_width_, 50);
    node_.param<int>("bucket_height", bucket_height_, 50);
}

/** Divide image into sections (buckets).
 * @param cv::Mat image
 * @param std::vector<cv::Mat> output vector with buckets
 * @param std::vector<cv::Point2f> output vector with the pixel coordinates of the upper left corner of each bucket
 * @return void */
void FeatureDetector::getBuckets(cv::Mat image, std::vector<cv::Mat> &buckets, std::vector<cv::Point2f> &offsets)
{
    // compute bucket width and height    
    int nBucketsHorizontal = image.cols / bucket_width_;
    int nBucketsVertical = image.rows / bucket_height_;

    int restHorizontal = image.cols % bucket_width_;
    int restVertical = image.rows % bucket_height_;

    if(restHorizontal >= (double)bucket_width_/2.0)
    {
        nBucketsHorizontal++;
    }
    if(restVertical >= (double)bucket_height_/2.0)
    {
        nBucketsVertical++;
    }
    
    int offset_h, offset_v;
    int width, height;

    offset_v = 0;
    for(int i=0; i<nBucketsVertical; i++)
    {
        offset_h = 0;
        
        if(i == nBucketsVertical - 1)
        {
            height = image.rows - i * bucket_height_;
        }
        else
        {
            height = bucket_height_;
        }

        for(int j=0; j<nBucketsHorizontal; j++)
        {
            if(j == nBucketsHorizontal - 1)
            {
                width = image.cols - j * bucket_width_;
            }
            else
            {
                width = bucket_width_;
            }

            // store the bucket (a rectangular section of the original image)
            cv::Mat b = image(cv::Rect(offset_h, offset_v, width, height));
            buckets.push_back(b);

            // also store the coordinates of the bucket's upper left corner
            cv::Point2f o(offset_h, offset_v);
            offsets.push_back(o);

            offset_h += bucket_width_;
        }
        offset_v += bucket_height_;
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

    if(descriptors.cols > 0)
    {
        for(int i=0; i<keypoints.size(); i++)
        {
            cv::Mat d = descriptors(cv::Rect(0, i, descriptors.cols, 1));
            Feature f(keypoints[i], d, seqNo, camNo);
            features.push_back(f);
        }
    }
    return features;
}

/** Extract set of cv::Point2f from set of cv::KeyPoint.
 * @param std::vector<cv::KeyPoint> keypoints
 * @return std::vector<cv::Point2f> points */
std::vector<cv::Point2f> FeatureDetector::keypoints2points(std::vector<cv::KeyPoint> keypoints)
{
    std::vector<cv::Point2f> points;

    for(int i=0; i<keypoints.size(); i++)
    {
        cv::Point2f pt = keypoints[i].pt;
        points.push_back(pt);
    }

    return points;
}

/** Update cv::Point2f points in set of cv::KeyPoint keypoints.
 * @param std::vector<cv::KeyPoint>& input/output set of keypoints
 * @param std::vector<cv::Point2f> new points */
void FeatureDetector::points2keypoints(std::vector<cv::KeyPoint> &keypoints, std::vector<cv::Point2f> points)
{
    for(int i=0; i<keypoints.size(); i++)
    {
        keypoints[i].pt = points[i];
    }
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
