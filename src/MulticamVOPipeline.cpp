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
    featureDetector_ = FeatureDetector(ORB, ORB);
    featureMatcher_ = FeatureMatcher(lb2_);
    odometer_ = MulticamOdometer(lb2_, files);

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
    // measure execution time
    ros::Time startingTime = ros::Time::now();

    // convert msg to cv::Mat and split
    cv::Mat_<uint8_t> fullImage = cv::Mat_<uint8_t>(NUM_CAMERAS*IMAGE_WIDTH, IMAGE_HEIGHT, const_cast<uint8_t*>(&msg->data[0]), msg->step);
    std::vector<cv::Mat> splitImages = lb2_.splitLadybug(fullImage, "mono8");

    // rectify images
    std::vector<cv::Mat> imagesRect = lb2_.rectify(splitImages);

    // vector with vector set of features and intra-camera matches of all cameras
    std::vector<std::vector<Feature>> featuresAllCameras;
    std::vector<std::vector<Match>> matches;
    featuresAllCameras.resize(NUM_OMNI_CAMERAS);
    matches.resize(NUM_OMNI_CAMERAS);

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
    for(int i=0; i<NUM_OMNI_CAMERAS; i++)
    {
        // reduce images to their ROI
        imagesRectReduced[i] = imagesRect[i](cv::Rect(param_ROIs_[i][0], param_ROIs_[i][1], param_ROIs_[i][2], param_ROIs_[i][3]));
        
        // find features in the ROI
        featuresAllCameras[i] = featureDetector_.detectFeatures(imagesRectReduced[i], seqNumber, i, true, 1.0); 
        
        // correct points coordinates: from the ROI to the full image 
        for(int j=0; j<featuresAllCameras[i].size(); j++)
        {
            cv::KeyPoint kp = featuresAllCameras[i][j].getKeypoint();
            kp.pt += cv::Point2f(param_ROIs_[i][0], param_ROIs_[i][1]);
            featuresAllCameras[i][j].setKeypoint(kp);
        }

        if(!first_)
        {
            // match features
            matches[i] = featureMatcher_.matchFeatures(featuresAllCamerasPrev_[i], featuresAllCameras[i]);
        }
    }

    // estimate motion (T)
    Eigen::Matrix4f T;
    int bestCamera;
    if(!first_)
    {
        std::vector<std::vector<Match>> inlierMatches;
        std::vector<std::vector<Eigen::Vector3f>> points3D;
        T = odometer_.estimateMotion(matches, bestCamera, inlierMatches, points3D);
    }
    else
    {
        // no motion in first frame -> identity
        T = Eigen::Matrix4f::Identity();
        bestCamera = -1;

        // make frames start at 0
        seqNumberOffset_ = seqNumber;

        first_ = false;
    }

    // track previously triangulated stereo points in the current left and right images
    FeatureDetector detectorScale(SHI_TOMASI, ORB);
    std::vector<Feature> featuresScaleLeft = detectorScale.detectFeatures(imagesRect[1], seqNumber, 1, false, 1.0);
    std::vector<Feature> featuresScaleRight = detectorScale.detectFeatures(imagesRect[2], seqNumber, 2, false, 1.0);
    std::vector<Match> matchesLeftScale = featureMatcher_.matchFeatures(featuresStereoLeftPrev_, featuresScaleLeft);
    std::vector<Match> matchesRightScale = featureMatcher_.matchFeatures(featuresStereoRightPrev_, featuresScaleRight);

    cv::Mat ofLeft = featureMatcher_.highlightOpticalFlow(imagesRect[1], matchesLeftScale, cv::Scalar(0, 255, 0));
    cv::Mat ofRight = featureMatcher_.highlightOpticalFlow(imagesRect[2], matchesRightScale, cv::Scalar(0, 255, 0));
    cv::namedWindow("optical flow left", CV_WINDOW_AUTOSIZE);
    cv::imshow("optical flow left", ofLeft);
    cv::namedWindow("optical flow right", CV_WINDOW_AUTOSIZE);
    cv::imshow("optical flow right", ofRight);

    std::cout << "STEREO MATCHES LEFT:  " << matchesLeftScale.size() << std::endl;
    std::cout << "STEREO MATCHES RIGHT: " << matchesRightScale.size() << std::endl;

    // triangulate stereo points for finding the scale in the next frame
    std::vector<Feature> featuresStereoLeft, featuresStereoRight;
    Eigen::Matrix4f tLeftRight = lb2_.getCamX2CamYTransform(1, 2);
    std::vector<Eigen::Vector3f> stereoPoints = triangulateStereo(imagesRectReduced[1], imagesRectReduced[2], cameraOverlaps_[1][1], cameraOverlaps_[2][0], param_ROIs_[1], param_ROIs_[2], lb2_.cameraMatrices_[1], lb2_.cameraMatrices_[2], tLeftRight, featuresStereoLeft, featuresStereoRight);

    // publish motion
    nav_msgs::Odometry msgOdom = transform2OdometryMsg(T, bestCamera);
    msgOdom.header.stamp = msg->header.stamp;
    pubOdom_.publish(msgOdom);
        
    // update image and features for tracking    
    imagesRectPrev_ = imagesRect;
    imagesRectReducedPrev_ = imagesRectReduced;
    featuresAllCamerasPrev_ = featuresAllCameras;
    seqNumberPrev_ = seqNumber;
    stereoPointsPrev_ = stereoPoints;
    featuresStereoLeftPrev_ = featuresStereoLeft;
    featuresStereoRightPrev_ = featuresStereoRight;

    ros::Time endingTime = ros::Time::now();
    std::cout << "Processed frame " << (seqNumber - seqNumberOffset_ + 1) << "\tin " << (endingTime - startingTime).toSec() << " seconds"<< std::endl;
}    

/** Triangulate stereo points, i.e. points that are visible in neighboring cameras at the same time.
 * @param cv::Mat left image, already reduced to their ROI (i.e. without the black borders)
 * @param cv::Mat right image, already reduced to their ROI (i.e. without the black borders)
 * @param int number of overlapping pixels in the left image
 * @param int number of overlapping pixels in the right image
 * @param std::vector<double> vector with horizontal and vertical offset in left image
 * @param std::vector<double> vector with horizontal and vertical offset in right image
 * @param Eigen::Matrix3f left camera matrix
 * @param Eigen::Matrix3f right camera matrix
 * @param Eigen::Matrix4f transformation of right camera relatively to left camera
 * @param std::vector<Feature> output vector with stereo features in the left camera
 * @param std::vector<Feature> output vector with stereo features in the right camera
 * @return std::vector<Eigen::Vector3f> vector with triuangulated 3D stereo points. **/
std::vector<Eigen::Vector3f> MulticamVOPipeline::triangulateStereo(cv::Mat leftImage, cv::Mat rightImage, int overlapLeft, int overlapRight, std::vector<double> offsetLeft, std::vector<double> offsetRight, Eigen::Matrix3f KLeft, Eigen::Matrix3f KRight, Eigen::Matrix4f TLeftRight, std::vector<Feature> &featuresLeft, std::vector<Feature> &featuresRight)
{
    ros::Time begin = ros::Time::now();
    // rotation and translation of right camera relatively to left camera
    Eigen::Matrix3f RLeftRight;
    Eigen::Vector3f tLeftRight;
    T2Rt(TLeftRight, RLeftRight, tLeftRight);

    // rotation and translation of left camera relatively to right camera
    Eigen::Matrix4f TRightLeft = TLeftRight.inverse();
    Eigen::Matrix3f RRightLeft;
    Eigen::Vector3f tRightLeft;
    T2Rt(TRightLeft, RRightLeft, tRightLeft);

    int validPointCounter = 0;

    // cut images to their overlapping parts
    cv::Mat leftOverlap = leftImage(cv::Rect(overlapLeft, 0, leftImage.cols-overlapLeft, leftImage.rows));
    cv::Mat rightOverlap = rightImage(cv::Rect(0, 0, overlapRight, rightImage.rows));

    // detect features in each part
    FeatureDetector stereoFeatureDetector(SHI_TOMASI, ORB);
    std::vector<Feature> featuresLeftAll = stereoFeatureDetector.detectFeatures(leftOverlap, -1, 1, false, 2.0);
    std::vector<Feature> featuresRightAll = stereoFeatureDetector.detectFeatures(rightOverlap, -1, 2, false, 2.0);

    // feature matching
    std::vector<Match> matchesStereoROI = featureMatcher_.matchFeatures(featuresLeftAll, featuresRightAll);

    std::vector<bool> valid;
    std::vector<Eigen::Vector3f> points;
    for(int i=0; i<matchesStereoROI.size(); i++)
    {
        // update features' positions in the full image
        Feature fLeft = matchesStereoROI[i].getFirstFeature();
        Feature fRight = matchesStereoROI[i].getSecondFeature();
        float d = matchesStereoROI[i].getDistance();

        cv::KeyPoint kpLeft = fLeft.getKeypoint();
        cv::KeyPoint kpRight = fRight.getKeypoint();

        kpLeft.pt.x += overlapLeft;
        fLeft.setKeypoint(kpLeft);
        Match m(fLeft, fRight, d);
        matchesStereoROI[i] = m;

        kpLeft.pt.x += offsetLeft[0];
        kpLeft.pt.y += offsetLeft[1];
        fLeft.setKeypoint(kpLeft);

        kpRight.pt.x += offsetRight[0];
        kpRight.pt.y += offsetRight[1];
        fRight.setKeypoint(kpRight);

        // get right and left 3D rays of each point, in right and left camera coordinates respectively
        cv::Point2f ptLeft = fLeft.getKeypoint().pt;
        cv::Point2f ptRight = fRight.getKeypoint().pt;

        double fxLeft = KLeft(0, 0);
        double fyLeft = KLeft(1, 1);
        double cxLeft = KLeft(0, 2);
        double cyLeft = KLeft(1, 2);

        double fxRight = KRight(0, 0);
        double fyRight = KRight(1, 1);
        double cxRight = KRight(0, 2);
        double cyRight = KRight(1, 2);

        Eigen::Vector3f rayLeft, rayRight;            
        rayLeft << ((ptLeft.x - cxLeft) / fxLeft), ((ptLeft.y - cyLeft) / fyLeft), 1.0;
        rayRight << ((ptRight.x - cxRight) / fxRight), ((ptRight.y - cyRight) / fyRight), 1.0;

        // get approximate intersection of both rays in right camera coordinates
        Eigen::Matrix3f M;
        Eigen::Vector3f mc1, mc2, mc3;
        mc1 = rayLeft;
        mc2 = - (RRightLeft.transpose() * rayRight);
        mc3 = rayLeft.cross(RRightLeft.transpose() * rayRight);
        M << mc1, mc2, mc3;

        Eigen::Vector3f abc = M.colPivHouseholderQr().solve(tLeftRight);
        Eigen::Vector3f point3D = abc(0)*rayLeft + (abc(2)/2) * rayLeft.cross(RRightLeft.transpose() * rayRight);
        
        if(point3D(2) > 0)
        {
            points.push_back(point3D);
            validPointCounter++;
            valid.push_back(true);

            featuresLeft.push_back(fLeft);
            featuresRight.push_back(fRight);
        }            
        else
        {
            valid.push_back(false);
        }
    }

    cv::Mat imMatches = featureMatcher_.highlightMatches(leftImage, rightImage, matchesStereoROI, valid);
    cv::namedWindow("Stereo matches", CV_WINDOW_AUTOSIZE);
    cv::imshow("Stereo matches", imMatches);
    cv::waitKey(10);     

    std::cout << "----------------- # VALID POINTS: " << validPointCounter << std::endl;

    ros::Time end = ros::Time::now();

    return points;
}

}

