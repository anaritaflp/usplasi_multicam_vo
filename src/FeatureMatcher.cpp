#include <multicam_vo/FeatureMatcher.h>

/** Default FeatureMatcher Constructor. */
FeatureMatcher::FeatureMatcher()
{

}

/** FeatureMatcher constructor.
 * @param Ladybug2 ladybug object
 * @return a FeatureMatcher object */
FeatureMatcher::FeatureMatcher(Ladybug2 lb2): node_("~")
{
    matchCounter = 0;

    lb2_ = lb2;
    
    // read parameters
    node_.param<int>("param_matchingDescriptorDistance", param_matchingDescriptorDistance_, 50);
    node_.param<int>("param_trackingWindowSize", param_trackingWindowSize_, 21);
    node_.param<int>("param_trackingMaxLevel", param_trackingMaxLevel_, 0);
    node_.param<double>("param_trackingMinEigThreshold", param_trackingMinEigThreshold_, 0.001);
    node_.param<int>("param_trackingTermcritCount", param_trackingTermcritCount_, 20);
    node_.param<double>("param_trackingTermcritEPS", param_trackingTermcritEPS_, 0.03);
    node_.param<double>("param_borderPercentage", param_borderPercentage_, 0.25);
}
    
/** FeatureMatcher destructor. */
FeatureMatcher::~FeatureMatcher()
{
    
}


/** Find matches in an omnidirectional multi-camera system. Matches are search in each camera and between onsecutive cameras.
 * @param std::vector<Feature> vector with features in the first image
 * @param std::vector<Feature> vector with features in the second image
 * @return std::vector<Match> vector with feature matches */
std::vector<Match> FeatureMatcher::matchFeatures(std::vector<Feature> features1, std::vector<Feature> features2)
{
    std::vector<Match> matches;

    if(features1.size() == 0 || features2.size() == 0)
    {
        return matches;
    }
    cv::Mat descriptors1 = getDescriptorsFromFeatures(features1);
    cv::Mat descriptors2 = getDescriptorsFromFeatures(features2);
    
    // brute force matching
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> dmatches;
    matcher.match(descriptors1, descriptors2, dmatches);

    // discard poor descriptor matches
    std::vector<cv::DMatch> goodDMatches;
    for(int i=0; i<dmatches.size(); i++)
    {
        if(dmatches[i].distance < param_matchingDescriptorDistance_)
        {
            goodDMatches.push_back(dmatches[i]);
        }
    }
    matches = DMatches2Matches(features1, features2, goodDMatches);

    return matches;
}


/** Highlight optical flow in an image.
 * @param cv::Mat original image
 * @param std::vector<Match> intra-camera matches
 * @param cv::Scalar BGR color
 * @return cv::Mat image with highlighted optical flow */
cv::Mat FeatureMatcher::highlightOpticalFlow(cv::Mat image, std::vector<Match> matches, cv::Scalar color)
{
    cv::Mat imageOptFlow;

    if(image.channels() == 1)
    {
        cv::cvtColor(image, imageOptFlow, CV_GRAY2BGR);
    }
    else
    {
        imageOptFlow = image;
    }

    // for each intra-camera match draw a line between previous and current feature
    for(int i=0; i<matches.size(); i++)
    {
        cv::Point2f pt_1 = matches[i].getFirstFeature().getKeypoint().pt;
        cv::Point2f pt_2 = matches[i].getSecondFeature().getKeypoint().pt;
        
        cv::line(imageOptFlow, pt_1, pt_2, color);
    }
    
    return imageOptFlow;
}

/** Highlight matches.
 * @param cv::Mat first image
 * @param cv::Mat second image
 * @param std::vector<Match> matches
 * @param std::vector<bool> vector indicating whether matches are valid (true) or not (false)
 * @return cv::Mat image with highlighted optical flow */
cv::Mat FeatureMatcher::highlightMatches(cv::Mat image1, cv::Mat image2, std::vector<Match> matches, std::vector<bool> validMatch)
{
    // leverage images in case of different heights
    if(image1.rows > image2.rows)
    {
        cv::Mat rest(image1.rows - image2.rows, image2.cols, CV_8UC1, cv::Scalar(0));
        cv::vconcat(image2, rest, image2);
    }
    else if(image2.rows > image1.rows)
    {
        cv::Mat rest(image2.rows - image1.rows, image1.cols, CV_8UC1, cv::Scalar(0));
        cv::vconcat(image1, rest, image1);
    }

    // concatenate images horizontally
    cv::Mat cat;
    cv::hconcat(image1, image2, cat);

    // convert to color
    cv::cvtColor(cat, cat, CV_GRAY2BGR);

    // draw lines connecting matching features
    for(int i=0; i<matches.size(); i++)
    {
        cv::Point2f pt1 = matches[i].getFirstFeature().getKeypoint().pt;
        cv::Point2f pt2 = matches[i].getSecondFeature().getKeypoint().pt;
        pt2 += cv::Point2f(image1.cols, 0);

        cv::Scalar color;
        if(validMatch[i] == true)
        {
            color = cv::Scalar(0, 255, 0);
        }
        else
        {
            color = cv::Scalar(0, 0, 255);
        }

        cv::line(cat, pt1, pt2, color);
        cv::circle(cat, pt1, 3, color);
        cv::circle(cat, pt2, 3, color);
    }

    return cat;
}

/** Track features in a camera.
 * @param cv::Mat image in the previous frame
 * @param cv::Mat image in the current frame
 * @param <std::vector<Feature> vector of features found in the previous frame
 * @param <std::vector<Feature> vector of features found in the current frame
 * @return std::vector<Match> vector with established matches */
std::vector<Match> FeatureMatcher::trackIntraCamera(cv::Mat imagePrev, cv::Mat imageCurr, std::vector<Feature> featuresPrev, std::vector<Feature> featuresCurr)
{
    std::vector<Match> matches;

    std::vector<cv::Point2f> pointsPrev = features2Points(featuresPrev);
    std::vector<cv::Point2f> pointsCurr = features2Points(featuresCurr);

    std::vector<uchar> status;
    std::vector<float> error;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, param_trackingTermcritCount_, param_trackingTermcritEPS_);
    cv::Size windowSize(param_trackingWindowSize_, param_trackingWindowSize_);
    
    // track features within the camera
    if(pointsPrev.size() > 0)
    {
        cv::calcOpticalFlowPyrLK(imagePrev, imageCurr, pointsPrev, pointsCurr, status, error, windowSize, param_trackingMaxLevel_, termcrit, 0, param_trackingMaxLevel_);
    }
    
    // if a feature disappears near a lateral border, it shall be searched for in the neighbouring camera 
    for(int i=0; i<featuresPrev.size(); i++)
    {
        // if tracking was not successful
        if(status[i] == (int)1)
        {
            // build vector with matches
            //Match m()
        }
    }

    return matches;
}

/** Convert a set of features to a cv::Mat of descriptors.
 * @param std::vector<Feature> vector of features
 * @return cv::Mat matrix with descriptors */
cv::Mat FeatureMatcher::getDescriptorsFromFeatures(std::vector<Feature> features)
{
    cv::Mat descriptors = features[0].getDescriptor();
    
    for(int i=1; i<features.size(); i++)
    {
        descriptors.push_back(features[i].getDescriptor());
    }
    
    return descriptors;
}

/** Convert set of OpenCV Matches to Matches.
 * @param std::vector<Feature> features in the previous frame
 * @param std::vector<Feature> features in the current frame
 * @param std::vector<cv::DMatch> vector of DMatches
 * @return std::vector<Match> vector of matches */
std::vector<Match> FeatureMatcher::DMatches2Matches(std::vector<Feature> features_1, std::vector<Feature> features_2, std::vector<cv::DMatch> dMatches)
{
    std::vector<Match> matches;
    
    for(int i=0; i<dMatches.size(); i++)
    {
        Match m;
        m.setFirstFeature(features_1[dMatches[i].queryIdx]);
        m.setSecondFeature(features_2[dMatches[i].trainIdx]);
        m.setDistance(dMatches[i].distance);
        matches.push_back(m);
    }
    return matches;
}

/** Convert set of features to cv::Point2f.
 * @param std::vector<Feature> features 
 * @return std::vector<cv::Point2f> vector of cv::Point2f */
std::vector<cv::Point2f> FeatureMatcher::features2Points(std::vector<Feature> features)
{
    std::vector<cv::Point2f> points;
    
    for(int i=0; i<features.size(); i++)
    {
        cv::Point2f p = features[i].getKeypoint().pt;
        points.push_back(p);
    }
    return points;
}

