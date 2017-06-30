#include <multicam_vo/MulticamVOPipeline.h>

namespace odom
{

/** Default MulticamVOPipeline constructor.
 * @param std::vector<std::ofstream*> vector of files to write estimated poses
 * @return MulticamVOPipeline object */
MulticamVOPipeline::MulticamVOPipeline(std::vector<std::ofstream*> files): node_("~")
{
    std::cout << "Creating visual odometry pipeline..." << std::endl;

    // initialize ladybug
    lb2_ = Ladybug2(node_);
    
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
    
    // read left and right overlapping limits
    std::vector<double> overlapCam0, overlapCam1, overlapCam2, overlapCam3, overlapCam4;
    node_.getParam("param_overlapCam0", overlapCam0);
    node_.getParam("param_overlapCam1", overlapCam1);
    node_.getParam("param_overlapCam2", overlapCam2);
    node_.getParam("param_overlapCam3", overlapCam3);
    node_.getParam("param_overlapCam4", overlapCam4);
    cameraOverlaps_.push_back(overlapCam0);
    cameraOverlaps_.push_back(overlapCam1);
    cameraOverlaps_.push_back(overlapCam2);
    cameraOverlaps_.push_back(overlapCam3);
    cameraOverlaps_.push_back(overlapCam4);
      
    // indicate it is the first frame
    first_ = true;
        
    // initialize lost frames counter
    lostFrameCounter_ = 0;
    counter = 0;
    odometer_ = MulticamOdometer(lb2_, files);

    // advertise odometry topic
    pubOdom_ = node_.advertise<nav_msgs::Odometry>("multicam_vo/odometry", 1);
    
    // subscribe to image topic
    image_transport::ImageTransport it(node_);    
    subImage_ = it.subscribe(param_imageTopic_, 1, &MulticamVOPipeline::imageCallback, this);

    absolutePoseGlobal_ = Eigen::Matrix4f::Identity();
    scale_ = 1.0;

    numInliers.resize(NUM_OMNI_CAMERAS);
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
    // measure execution time
    ros::Time startingTime = ros::Time::now();

    // convert msg to cv::Mat and split
    cv::Mat_<uint8_t> fullImage = cv::Mat_<uint8_t>(NUM_CAMERAS*IMAGE_WIDTH, IMAGE_HEIGHT, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    std::vector<cv::Mat> splitImages = lb2_.splitLadybug(fullImage, "mono8");

    // rectify images
    std::vector<cv::Mat> imagesRect = lb2_.rectify(splitImages);

    // vector with vector set of features and intra-camera matches of all cameras
    std::vector<std::vector<cv::Point2f>> featuresAllCameras;
    featuresAllCameras.resize(NUM_OMNI_CAMERAS);

    // get frame number
    int seqNumber = msg->header.seq;

    // check if frames were lost
    if(seqNumber - seqNumberPrev_ > 1)
    {
        ROS_WARN("Lost %d frames!", seqNumber - seqNumberPrev_ - 1);
        lostFrameCounter_ += seqNumber - seqNumberPrev_ - 1;
    }

    std::vector<cv::Mat> imagesRectReduced;
    imagesRectReduced.resize(NUM_OMNI_CAMERAS);
    std::vector<std::vector<Match>> matches; 
    matches.resize(NUM_OMNI_CAMERAS);

    #pragma omp parallel for
    for(int i=0; i<NUM_OMNI_CAMERAS; i++)
    {
        // reduce images to their ROI
        imagesRectReduced[i] = imagesRect[i](cv::Rect(param_ROIs_[i][0], param_ROIs_[i][1], param_ROIs_[i][2], param_ROIs_[i][3]));

        // equalize histogram
        //cv::equalizeHist(imagesRectReduced[i], imagesRectReduced[i]);

        if(first_)
        {
            featureDetection(imagesRectReduced[i], featuresAllCameras[i]);
        }
        else
        {
            if(numInliers[i] < MIN_FEATURES && counter > 1)
            {
                featureDetection(imagesRectReducedPrev_[i], featuresAllCamerasPrev_[i]);
            }

            std::vector<uchar> status;
            featureTracking(imagesRectReducedPrev_[i],imagesRectReduced[i],featuresAllCamerasPrev_[i],featuresAllCameras[i], status);

            cv::Point2f offset(param_ROIs_[i][0], param_ROIs_[i][1]);

            for(int j=0; j<status.size(); j++)
            {
                if((int)status[j] == 1)
                {                    
                    Match m(featuresAllCamerasPrev_[i][j] + offset, featuresAllCameras[i][j] + offset);
                    matches[i].push_back(m);                    
                }
            }

            /*if(i == 0)
            {
                cv::Mat of = highlightOpticalFlow(imagesRect[i], matches[i], cv::Scalar(0, 255, 0));
                cv::namedWindow("Optical flow", CV_WINDOW_AUTOSIZE);
                cv::imshow("Optical flow", of);
                cv::waitKey(10);
            }*/
        }
    }

    // estimate motion (T)
    
    Eigen::Matrix4f TRelative;
    int nInliersBest;
    int bestCamera;
    if(!first_)
    {
        std::vector<std::vector<Match>> inlierMatches;
        std::vector<std::vector<Eigen::Vector3f>> points3D;
        std::vector<Eigen::Matrix4f> monoPoses;

        TRelative = odometer_.estimateMotion(matches, bestCamera, inlierMatches, points3D, monoPoses, msg->header.stamp);
        nInliersBest = inlierMatches[bestCamera].size();
        
        /*std::cout << "INLIERS:\t"; std::cout.flush();
        for(int i=0; i<NUM_OMNI_CAMERAS; i++)
        {
            numInliers[i] = inlierMatches[i].size();
            std::cout << numInliers[i] << "\t"; std::cout.flush();
        }
        std::cout << std::endl;*/
  
        Eigen::Matrix3f RFinal;
        Eigen::Vector3f tFinal;
        T2Rt(TRelative, RFinal, tFinal);
 
        double scale = 1.0;
        Eigen::Matrix4f TFinal = Rt2T(RFinal, scale * tFinal);
        scale_ = scale;

        // concatenate motion estimation of the best camera to absolute pose and return result
        absolutePoseGlobal_ = absolutePoseGlobal_ * TFinal;
    }
    else
    {
        // no motion in first frame -> identity
        absolutePoseGlobal_ = Eigen::Matrix4f::Identity();
        bestCamera = -1;
        nInliersBest = 10000;

        // make frames start at 0
        seqNumberOffset_ = seqNumber;

        first_ = false;
    }

    // publish motion
    //std::cout << absolutePoseGlobal_ << std::endl << std::endl;
    nav_msgs::Odometry msgOdom = transform2OdometryMsg(absolutePoseGlobal_, bestCamera, nInliersBest);
    msgOdom.header.stamp = msg->header.stamp;
    pubOdom_.publish(msgOdom);
        
    // update image and features for tracking    
    imagesRectPrev_ = imagesRect;
    imagesRectReducedPrev_ = imagesRectReduced;
    featuresAllCamerasPrev_ = featuresAllCameras;
    seqNumberPrev_ = seqNumber;

    ros::Time endingTime = ros::Time::now();
    //std::cout << "Processed frame " << (seqNumber - seqNumberOffset_ + 1) << "\tin " << (endingTime - startingTime).toSec() << " seconds" << std::endl;
    std::cout << "Processed frame " << (seqNumber - seqNumberOffset_ + 1) << std::endl;
    counter++;
}    



}

