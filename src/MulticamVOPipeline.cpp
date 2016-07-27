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
    std::vector<cv::Mat> imagesRect = lb2.rectifyManually(splitImages);

    /*cv::imwrite("/home/anaritapereira/rect0.png", imagesRect[0]);
    cv::imwrite("/home/anaritapereira/rect1.png", imagesRect[1]);
    cv::imwrite("/home/anaritapereira/rect2.png", imagesRect[2]);
    cv::imwrite("/home/anaritapereira/rect3.png", imagesRect[3]);
    cv::imwrite("/home/anaritapereira/rect4.png", imagesRect[4]);
    cv::imwrite("/home/anaritapereira/rect5.png", imagesRect[5]);


    cv::namedWindow("rect 0", CV_WINDOW_AUTOSIZE);
    cv::imshow("rect 0", imagesRect[0]);
    cv::namedWindow("rect 1", CV_WINDOW_AUTOSIZE);
    cv::imshow("rect 1", imagesRect[1]);
    cv::namedWindow("rect 2", CV_WINDOW_AUTOSIZE);
    cv::imshow("rect 2", imagesRect[2]);
    cv::namedWindow("rect 3", CV_WINDOW_AUTOSIZE);
    cv::imshow("rect 3", imagesRect[3]);
    cv::namedWindow("rect 4", CV_WINDOW_AUTOSIZE);
    cv::imshow("rect 4", imagesRect[4]);
    cv::namedWindow("rect 5", CV_WINDOW_AUTOSIZE);
    cv::imshow("rect 5", imagesRect[5]);
    cv::waitKey(0);*/

    // reduce images to their ROI
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        imagesRect[i] = imagesRect[i](cv::Rect(param_ROIs_[i][0], param_ROIs_[i][1], param_ROIs_[i][2], param_ROIs_[i][3]));
    }

    // detect features in all cameras
    int seqNumber = msg->header.seq;
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
        std::vector<std::vector<Match>> matches = featureMatcher_.findOmniMatches(imagesRectPrev_, imagesRect, featuresAllCamerasPrev_, featuresAllCameras, NUM_OMNI_CAMERAS);
       
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

