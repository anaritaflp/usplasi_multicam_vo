#include <multicam_vo/MulticamVOPipeline.h>

namespace odom
{
    
/** Default MulticamVOPipeline constructor.
 * @param std::vector<std::ofstream*> vector of files to write estimated poses
 * @return MulticamVOPipeline object */
MulticamVOPipeline::MulticamVOPipeline(std::vector<std::ofstream*> files): node_("~")
{
    // initialize ladybug
    lb2 = Ladybug2(node_);
    
    // get image topic
    node_.param<std::string>("image_topic", param_imageTopic_, "/camera/image_raw");

    // get regions of interes for each camera
    param_ROIs_.resize(NUM_CAMERAS);
    node_.getParam("cam0_roi", param_ROIs_[0]);
    node_.getParam("cam1_roi", param_ROIs_[1]);
    node_.getParam("cam2_roi", param_ROIs_[2]);
    node_.getParam("cam3_roi", param_ROIs_[3]);
    node_.getParam("cam4_roi", param_ROIs_[4]);
    node_.getParam("cam5_roi", param_ROIs_[5]);
        
    // indicate it is the first frame
    first_ = true;
        
    // initialize lost frames counter
    lostFrameCounter_ = 0;
    
    featureDetector_ = FeatureDetector(SHI_TOMASI, ORB);
    featureMatcher_ = FeatureMatcher(lb2);
    odometer_ = MulticamOdometer(lb2, files);
    
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
    std::vector<cv::Mat> splitImages = lb2.splitLadybug(fullImage, "mono8");

    // rectify images
    std::vector<cv::Mat> imagesRect = lb2.rectify(splitImages);

    // reduce images to their ROI
    std::vector<cv::Mat> imagesRectReduced;
    imagesRectReduced.resize(NUM_CAMERAS);
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        imagesRectReduced[i] = imagesRect[i](cv::Rect(param_ROIs_[i][0], param_ROIs_[i][1], param_ROIs_[i][2], param_ROIs_[i][3]));
    }

    // detect features in all cameras
    int seqNumber = msg->header.seq;
    std::vector<std::vector<Feature>> featuresAllCameras;
    for(int i=0; i<NUM_CAMERAS; i++)
    {   
        std::vector<Feature> features;
        features = featureDetector_.detectFeatures(imagesRectReduced[i], seqNumber, i);   
        for(int j=0; j<features.size(); j++)
        {
            cv::KeyPoint kp = features[j].getKeypoint();
            kp.pt += cv::Point2f(param_ROIs_[i][0], param_ROIs_[i][1]);
            features[j].setKeypoint(kp);
        }       
        featuresAllCameras.push_back(features);
    }


    /*cv::Mat imFeats = featureDetector_.highlightFeatures(imagesRect[0], featuresAllCameras[0]);
    cv::namedWindow("features", CV_WINDOW_AUTOSIZE);
    cv::imshow("features", imFeats);
    cv::waitKey(10);*/

    if(!first_)
    {
        // check if a frame was lost
        if(seqNumber - seqNumberPrev_ > 1)
        {
            ROS_WARN("Lost %d frames!", seqNumber - seqNumberPrev_ - 1);
            lostFrameCounter_ += seqNumber - seqNumberPrev_ - 1;
        }
        
        // match features
        std::vector<std::vector<Match>> matches = featureMatcher_.findOmniMatches(imagesRectPrev_, imagesRect, featuresAllCamerasPrev_, featuresAllCameras, NUM_OMNI_CAMERAS);
        
        std::cout << "#MATCHES: "; std::cout.flush();
        for(int i=0; i<3*NUM_OMNI_CAMERAS; i++)
        {
            std::cout << " " << matches[i].size(); std::cout.flush();
        }
        std::cout << std::endl;
        
        // estimate motion    
        int bestCamera;
        Eigen::Matrix4f T = odometer_.estimateMotion(matches, bestCamera);
        
        // publish odometry
        nav_msgs::Odometry msgOdom = transform2OdometryMsg(T, bestCamera);
        msgOdom.header.stamp = msg->header.stamp;
        pubOdom_.publish(msgOdom);
    }
    else
    {
        seqNumberOffset_ = seqNumber;
        // publish identity on first frame
        nav_msgs::Odometry msgOdom = transform2OdometryMsg(Eigen::Matrix4f::Identity(), -1);
        msgOdom.header.stamp = msg->header.stamp;
        first_ = false;
    }
        
    // update image and features for tracking    
    imagesRectPrev_ = imagesRect;
    featuresAllCamerasPrev_ = featuresAllCameras;

    seqNumberPrev_ = seqNumber;

    std::cout << "PROCESSED FRAME " << (seqNumber - seqNumberOffset_ + 1) << std::endl;
}    
    
}

