#include <multicam_vo/MulticamVOPipeline.h>

namespace odom
{
    
/** Default MulticamVOPipeline constructor.
 * @param std::vector<std::ofstream*> vector of files to write estimated poses
 * @return MulticamVOPipeline object */
MulticamVOPipeline::MulticamVOPipeline(std::vector<std::ofstream*> files): node_("~")
{
    // get image topic
    node_.param<std::string>("image_topic", param_imageTopic_, "/camera/image_raw");
    
    // get calibration data
    getCalibration();
        
    // indicate it is the first frame
    first_ = true;
        
    // initialize lost frames counter
    lostFrameCounter_ = 0;
        
    featureDetector_ = FeatureDetector(SHI_TOMASI, ORB);
    featureMatcher_ = FeatureMatcher();
    odometer_ = MulticamOdometer(calib, NUM_CAMERAS-1, files);

    // advertise odometry topic
    pubOdom_ = node_.advertise<nav_msgs::Odometry>("multicam_vo/odometry", 1);
    
    // subscribe to image topic
    image_transport::ImageTransport it(node_);    
    subImage_ = it.subscribe(param_imageTopic_, 1, &MulticamVOPipeline::imageCallback, this);
}
    
/** Default MulticamVOPipeline destructor. */
MulticamVOPipeline::~MulticamVOPipeline()
{
        
}
    
/** ROS loop.
 * @param void
 * @return void */
void MulticamVOPipeline::spin()
{
    ros::spin();
}
    
/** Ladybug2 image callback.
 * @param sensor_msgs::Image::ConstPtr& ROS image message
 * @return void */
void MulticamVOPipeline::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    // convert msg to cv::Mat and split
    cv::Mat_<uint8_t> fullImage = cv::Mat_<uint8_t>(NUM_CAMERAS*IMAGE_WIDTH, IMAGE_HEIGHT, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    std::vector<cv::Mat> splitImages = splitLadybug(fullImage, MONO);
        
    int seqNumber = msg->header.seq;
    std::cout << "FRAME " << seqNumber << std::endl;

    // rectify images
    std::vector<cv::Mat> imagesRect = MulticamVOPipeline::rectify(splitImages);

    // detect features in the current frame
    std::vector<std::vector<Feature>> featuresAllCameras;
    for(int i=0; i<NUM_CAMERAS; i++)
    {   
        std::vector<Feature> features;
        features = featureDetector_.detectFeatures(imagesRect[i], seqNumber, i);
        
        featuresAllCameras.push_back(features);
    }

    if(!first_)
    {
        // check if a frame was lost
        if(seqNumber - seqNumberPrev_ > 1)
        {
            ROS_WARN("Lost %d frames!", seqNumber - seqNumberPrev_ - 1);
            lostFrameCounter_ += seqNumber - seqNumberPrev_ - 1;
        }
        
        // match features
        std::vector<std::vector<Match>> matches = featureMatcher_.findOmniMatches(imagesRectPrev_, imagesRect, featuresAllCamerasPrev_, featuresAllCameras, NUM_CAMERAS-1);
       
        // estimate motion    
        Eigen::Matrix4f T = odometer_.estimateMotion(matches);
            
        // publish odometry
        nav_msgs::Odometry msgOdom = transform2OdometryMsg(T);
        pubOdom_.publish(msgOdom);
    }
    else
    {
        // publish identity on first frame
        nav_msgs::Odometry msgOdom = transform2OdometryMsg(Eigen::Matrix4f::Identity());
        first_ = false;
    }
        
    // update image and features for tracking    
    imagesRectPrev_ = imagesRect;
    featuresAllCamerasPrev_ = featuresAllCameras;

    seqNumberPrev_ = seqNumber;
}    
    
/** Rectify individual images.
 * @param std::vector<cv::Mat> vector with distorted images
 * @return std::vector<cv::Mat> vector with rectified images */
std::vector<cv::Mat> MulticamVOPipeline::rectify(std::vector<cv::Mat> images)
{
    std::vector<cv::Mat> imagesRect;
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        cv::Mat imRect;
        calib[i].rectifyImage(images[i], imRect);
        imagesRect.push_back(imRect);
    }

    return imagesRect;
}
    
/** Read all camera's calibration parameters from yaml files.
 * @param void
 * @return void */
void MulticamVOPipeline::getCalibration()
{
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        // get path of yaml calibration file
        char param_calibPath[50];
        sprintf(param_calibPath, "calib_file_cam_%d", i);
        std::string calibFilename;
        node_.param<std::string>(std::string(param_calibPath), calibFilename, "");
            
        // open calibration yaml file
        std::ifstream fin(calibFilename);  
        
        // parse yaml file to get cameraInfo
        std::string camName;
        sensor_msgs::CameraInfo camInfo;
        bool ret = camera_calibration_parsers::readCalibrationYml(fin, camName, camInfo);
        if(ret == false)
        {
            ROS_ERROR("Invalid calibration file! Exiting...");
            ros::shutdown();
        }

        std::cout << "Found calibration of camera " << camName << std::endl;
        
        // get pinhole camera model out of cameraInfo
        image_geometry::PinholeCameraModel camModel;
        camModel.fromCameraInfo(camInfo);
        calib.push_back(camModel);   
    }
}

/** Compute yaw angle using matches of the top camera.
 * @param std::vector<Match> vector with matches of the top camera
 * @param double& output yaw angle
 * @return bool true is success, false otherwise */
bool MulticamVOPipeline::computeYaw(cv::Mat imTopPrev, cv::Mat imTopCurr, std::vector<Match> matchesTop, double &angle)
{
    if(matchesTop.size() < 2)
    {
        return false;
    }

    // save distances
    std::vector<float> distances;
    for(int i=0; i<matchesTop.size(); i++)
    {
        distances.push_back(matchesTop[i].getDistance());
    }

    cv::Point2f pt1Prev, pt2Prev, pt1Curr, pt2Curr;

    int bestIndex = 0;
    float minDistance = distances[0];

    // get best match
    for(int i=1; i<distances.size(); i++)
    {
        if(distances[i] < minDistance)
        {
            minDistance = distances[i];
            bestIndex = i;
        }
    }
    pt1Prev = matchesTop[bestIndex].getFirstFeature().getKeypoint().pt;
    pt1Curr = matchesTop[bestIndex].getSecondFeature().getKeypoint().pt;

    // get second best match
    distances[bestIndex] = 100000.0;
    bestIndex = 0;
    minDistance = distances[0];
    for(int i=1; i<distances.size(); i++)
    {
        if(distances[i] < minDistance)
        {
            minDistance = distances[i];
            bestIndex = i;
        }
    }
    pt2Prev = matchesTop[bestIndex].getFirstFeature().getKeypoint().pt;
    pt2Curr = matchesTop[bestIndex].getSecondFeature().getKeypoint().pt;

    /*cv::Mat imLinePrev, imLineCurr;
    cv::cvtColor(imTopPrev, imLinePrev, CV_GRAY2BGR);
    cv::cvtColor(imTopCurr, imLineCurr, CV_GRAY2BGR);
    cv::circle(imLinePrev, pt1Prev, 3, cv::Scalar(0, 0, 255), 2);
    cv::circle(imLinePrev, pt2Prev, 3, cv::Scalar(0, 0, 255), 2);
    cv::circle(imLineCurr, pt1Curr, 3, cv::Scalar(0, 0, 255), 2);
    cv::circle(imLineCurr, pt2Curr, 3, cv::Scalar(0, 0, 255), 2);
    cv::line(imLinePrev, pt1Prev, pt2Prev, cv::Scalar(0, 255, 0));
    cv::line(imLineCurr, pt1Curr, pt2Curr, cv::Scalar(0, 255, 0));
    cv::Mat cat;
    cv::hconcat(imLinePrev, imLineCurr, cat);*/

    // build lines from matches
    cv::Point2f linePrev = computeLine(pt1Prev, pt2Prev);
    cv::Point2f lineCurr = computeLine(pt1Curr, pt2Curr);

    // compute angle between lines
    angle = computeAngle(linePrev, lineCurr);
   
    /*char alpha[20];
    sprintf(alpha, "%f", angle);
    cv::putText(cat, alpha, cv::Point2f(20, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));*/

    cv::Mat cat;
    cv::hconcat(imTopPrev, imTopCurr, cat);
    for(int i=0; i<matchesTop.size(); i++)
    {
        cv::Point2f ptPrev = matchesTop[i].getFirstFeature().getKeypoint().pt;
        cv::Point2f ptCurr = matchesTop[i].getSecondFeature().getKeypoint().pt;
        
        ptCurr.x += 768;
        
        cv::line(cat, ptPrev, ptCurr, cv::Scalar(0, 255, 0));
        cv::circle(cat, ptPrev, 3, cv::Scalar(0, 0, 255), 2);
        cv::circle(cat, ptCurr, 3, cv::Scalar(0, 0, 255), 2);
    }


    cv::namedWindow("top", CV_WINDOW_AUTOSIZE);
    cv::imshow("top", cat);
    cv::waitKey(10);

    return true;
}
    
/** Compute line that joins two points, in homogeneous coordinates.
 * @param cv::Point2f first point
 * @param cv::Point2f second point
 * @return cv::Point2f line */
cv::Point2f MulticamVOPipeline::computeLine(cv::Point2f point1, cv::Point2f point2)
{
    cv::Point2f line;
    line.x = point2.x - point1.x;
    line.y = point2.y - point1.y; 
    return line;
}

/** Compute angle between 2 lines.
 * @param cv::Point2f first line
 * @param cv::Point2f second line
 * @return double angle */
double MulticamVOPipeline::computeAngle(cv::Point2f line1, cv::Point2f line2)
{
    double angle;
    if(line1.x == 0 && line2.x != 0) 
    {
        angle = PI/2 - atan(line2.y/line2.x);
    }
    else if(line2.x == 0 && line1.x != 0) 
    {
        angle = PI/2 - atan(line1.y/line1.x);
    }
    else
    {
        angle = fabs(atan(line1.y/line1.x) - atan(line2.y/line2.x));
    }
    
    if(angle > PI/2)
    {
        angle = PI - angle;
    }
    return (angle * 180 / PI);
}

/** Convert transform to odometry message.
 * @param Eigen::Matrix4f transform
 * @return nav_msgs::Odometry odometry message */
nav_msgs::Odometry MulticamVOPipeline::transform2OdometryMsg(Eigen::Matrix4f T)
{
    nav_msgs::Odometry msg;

    // translation
    msg.pose.pose.position.x = T(0, 3);
    msg.pose.pose.position.y = T(1, 3);
    msg.pose.pose.position.z = T(2, 3);

    // rotation
    tf::Matrix3x3 R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
    double roll, pitch, yaw;
    R.getRPY(roll, pitch, yaw);
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    quaternionTFToMsg(q, msg.pose.pose.orientation);

    return msg;
}


}

