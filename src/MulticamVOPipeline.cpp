#include <multicam_vo/MulticamVOPipeline.h>

namespace odom
{
    
/** Default MulticamVOPipeline constructor.
 * @param void
 * @return MulticamVOPipeline object */
MulticamVOPipeline::MulticamVOPipeline(): node_("~")
{
    // get image topic
    node_.param<std::string>("image_topic", param_imageTopic_, "/camera/image_raw");
    
    // get calibration data
    getCalibration();
        
    // indicate it is the first frame
    first_ = true;
        
    // initialize lost frames counter
    lostFrameCounter = 0;
    seqNumberPrev = 0;
        
    featureDetector = FeatureDetector(node_, SHI_TOMASI, ORB);
    featureMatcher = FeatureMatcher(node_);

    // advertise odometry topic
    pubOdom_ = node_.advertise<nav_msgs::Odometry>("multicamOdometer/odometry", 1);
    
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
    cv::Mat_<uint8_t> fullImage = cv::Mat_<uint8_t>(NUM_CAMERAS*LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    std::vector<cv::Mat> splitImages = splitLadybug(fullImage, MONO);
        
    int seqNumber = msg->header.seq;
    //std::cout << "FRAME " << seqNumber << std::endl;
    if(seqNumber - seqNumberPrev > 1)
    {
        ROS_WARN("Lost %d frames!", seqNumber - seqNumberPrev - 1);
        lostFrameCounter += seqNumber - seqNumberPrev - 1;
    }
    seqNumberPrev = seqNumber;

    // rectify images
    std::vector<cv::Mat> imagesRect = MulticamVOPipeline::rectify(splitImages);

    // detect and track features        
    std::vector<std::vector<Feature>> featuresAllCameras;
    for(int i=0; i<NUM_CAMERAS; i++)
    {   
        std::vector<Feature> features;
        features = featureDetector.detectFeatures(imagesRect[i], seqNumber, i);
        
        featuresAllCameras.push_back(features);
    }

    if(!first_)
    {
        // match features
        std::vector<std::vector<Match>> matches = featureMatcher.findOmniMatches(imagesRectPrev, imagesRect, featuresAllCamerasPrev, featuresAllCameras, NUM_CAMERAS-1);
            
        // show optical flow
        /*cv::Mat imFeat;
        cv::cvtColor(imagesRect[3], imFeat, CV_GRAY2BGR);
        for(int i=0; i<matches.size(); i++)
        {
            if(matches[i].getFirstFeature().getCamNumber() == matches[i].getSecondFeature().getCamNumber() && matches[i].getSecondFeature().getCamNumber() == 3)
            {
                cv::Point2f pt1 = matches[i].getFirstFeature().getKeypoint().pt;
                cv::Point2f pt2 = matches[i].getSecondFeature().getKeypoint().pt;
                cv::line(imFeat, pt1, pt2, matches[i].getColor());
            }                
        }
        cv::namedWindow("matches intra camera", CV_WINDOW_AUTOSIZE);
        cv::imshow("matches intra camera", imFeat);*/
            
            
        // estimate motion    
        MulticamOdometer odometer(node_);
        odometer.estimateMotion(matches, NUM_CAMERAS-1, calib);
            
        // publish odometry
        nav_msgs::Odometry msgOdom;
        pubOdom_.publish(msgOdom);
    }
    else
    {
        first_ = false;
    }
        
    // update image and features for tracking
    imagesRectPrev = imagesRect;
    featuresAllCamerasPrev = featuresAllCameras;
        
    // for showing images
    //cv::waitKey(10);       
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
    
}