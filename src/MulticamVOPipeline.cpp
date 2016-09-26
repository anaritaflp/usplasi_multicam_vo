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
    featureDetector_ = FeatureDetector(FAST, ORB);
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

    std::vector<cv::Mat> imageRectReduced;
    imageRectReduced.resize(NUM_OMNI_CAMERAS);
    for(int i=0; i<NUM_OMNI_CAMERAS; i++)
    {
        // reduce images to their ROI
        imageRectReduced[i] = imagesRect[i](cv::Rect(param_ROIs_[i][0], param_ROIs_[i][1], param_ROIs_[i][2], param_ROIs_[i][3]));
        
        // find features in the ROI
        featuresAllCameras[i] = featureDetector_.detectFeatures(imageRectReduced[i], seqNumber, i, true, 1.0); 
        
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

        for(int i=0; i<NUM_OMNI_CAMERAS; i++)
        {
            //FeatureDetector d(FAST, ORB);
            //std::vector<Feature> fs = d.detectFeatures(imageRectReduced[i], seqNumber, i, false, 1.0);
            std::vector<Match> matches = featureMatcher_.matchFeatures(stereoFeaturesPrev_[i], featuresAllCameras[i]); 
            std::cout << "#MATCHES: " << matches.size() << std::endl;  
        }
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

    // find out the best scale
    std::vector<std::vector<Feature>> stereoFeatures;
    std::vector<std::vector<Eigen::Vector3f>> stereoPoints = triangulateStereo(imageRectReduced, seqNumber, stereoFeatures);

    // publish motion
    nav_msgs::Odometry msgOdom = transform2OdometryMsg(T, bestCamera);
    msgOdom.header.stamp = msg->header.stamp;
    pubOdom_.publish(msgOdom);
        
    // update image and features for tracking    
    imagesRectPrev_ = imagesRect;
    featuresAllCamerasPrev_ = featuresAllCameras;
    seqNumberPrev_ = seqNumber;
    stereoPointsPrev_ = stereoPoints;
    stereoFeaturesPrev_ = stereoFeatures;

    ros::Time endingTime = ros::Time::now();
    std::cout << "Processed frame " << (seqNumber - seqNumberOffset_ + 1) << "\tin " << (endingTime - startingTime).toSec() << " seconds"<< std::endl;
}    

/** Triangulate stereo points, i.e. points that are visible in neighboring cameras at the same time.
 * @param std::vector<cv::Mat vector with images, already reduced to their ROI (i.e. without the black borders)
 * @param int sequence number of the current frame
 * @param std::vector<std::vector<Feature>> output vector with stereo features in each camera, for matching with the next image
 * @return std::vector<std::vector<Eigen::Vector3f>> vector with 3D stereo points in each neighboring camera pair. I.e, position 0 has points triangulated using camera 0 and 1. Position 1 has points triangulated using camera 1 and 2. etc. **/
std::vector<std::vector<Eigen::Vector3f>> MulticamVOPipeline::triangulateStereo(std::vector<cv::Mat> imagesROI, int seqNumber, std::vector<std::vector<Feature>> &features)
{
    int validPointCounter = 0;

    // store 3D stereo points
    std::vector<std::vector<Eigen::Vector3f>> points;
    points.resize(NUM_OMNI_CAMERAS);

    // store features of stereo points
    features.resize(NUM_OMNI_CAMERAS);
    
    std::vector<cv::Mat> overlappingParts;
    for(int i=0; i<NUM_OMNI_CAMERAS; i++)
    {
        // indices of the left and right overlapping cameras
        int iRight = i, iLeft;
        if(i == 0)
        {
            iLeft = NUM_OMNI_CAMERAS - 1;
        }
        else
        {
            iLeft = i - 1;
        }

        // cut images to their overlapping parts
        cv::Mat leftOverlap = imagesROI[iLeft](cv::Rect(cameraOverlaps_[iLeft][1], 0, imagesROI[iLeft].cols-cameraOverlaps_[iLeft][1], imagesROI[iLeft].rows));
        cv::Mat rightOverlap = imagesROI[iRight](cv::Rect(0, 0, cameraOverlaps_[iRight][0], imagesROI[iRight].rows));
        
        // detect features in each part
        FeatureDetector stereoFeatureDetector(FAST, ORB);
        std::vector<Feature> featuresLeft = stereoFeatureDetector.detectFeatures(leftOverlap, seqNumber, iLeft, false, 2.0);
        std::vector<Feature> featuresRight = stereoFeatureDetector.detectFeatures(rightOverlap, seqNumber, iRight, false, 2.0);

        // feature matching
        std::vector<Match> matchesStereoROI = featureMatcher_.matchFeatures(featuresLeft, featuresRight);

        // update features' positions...
        std::vector<Match> matchesStereo;
        matchesStereo.resize(matchesStereoROI.size());
        for(int j=0; j<matchesStereoROI.size(); j++)
        {
            // ... in the ROI
            Feature fLeft = matchesStereoROI[j].getFirstFeature();
            Feature fRight = matchesStereoROI[j].getSecondFeature();
            float d = matchesStereoROI[j].getDistance();

            cv::KeyPoint kpLeft = fLeft.getKeypoint();
            cv::KeyPoint kpRight = fRight.getKeypoint();

            kpLeft.pt.x += cameraOverlaps_[iLeft][1];
            fLeft.setKeypoint(kpLeft);

            matchesStereoROI[j] = Match(fLeft, fRight, d);

            // ... and in the full images
            kpLeft.pt.x += param_ROIs_[iLeft][0];
            kpLeft.pt.y += param_ROIs_[iLeft][1];
            fLeft.setKeypoint(kpLeft);

            kpRight.pt.x += param_ROIs_[iRight][0];
            kpRight.pt.y += param_ROIs_[iRight][1];
            fRight.setKeypoint(kpRight);

            matchesStereo[j] = Match(fLeft, fRight, d);
        }

        // left - right transformation
        Eigen::Matrix4f TLeftRight = lb2_.getCamX2CamYTransform(iLeft, iRight);
        Eigen::Matrix3f RLeftRight;
        Eigen::Vector3f tLeftRight;
        T2Rt(TLeftRight, RLeftRight, tLeftRight);
        
        // right - left transformation
        Eigen::Matrix4f TRightLeft = TLeftRight.inverse();
        Eigen::Matrix3f RRightLeft;
        Eigen::Vector3f tRightLeft;
        T2Rt(TRightLeft, RRightLeft, tRightLeft);

        // remove wrong matches using the known static transformations between cameras
        /*Eigen::Matrix3f FStereo = odometer_.Rt2F(RRightLeft, tRightLeft, lb2_.cameraMatrices_[iLeft], lb2_.cameraMatrices_[iRight]);
        std::vector<int> inlierIndices = odometer_.getInliers(matchesStereo, FStereo, 1.0);
        
        std::vector<Match> inlierMatches;
        for(int j=0; j<matchesStereo.size(); j++)
        {
            if(elemInVec(inlierIndices, i))
            {
                inlierMatches.push_back(matchesStereo[i]);
            }
        }*/

        std::vector<Match> inlierMatches = matchesStereo;

        // triangulate 3D points
        std::vector<bool> valid;
        for(int j=0; j<inlierMatches.size(); j++)
        {
            // get right and left 3D rays of each point, in right and left camera coordinates respectively
            cv::Point2f ptLeft = inlierMatches[j].getFirstFeature().getKeypoint().pt;
            cv::Point2f ptRight = inlierMatches[j].getSecondFeature().getKeypoint().pt;

            double fxLeft = lb2_.intrinsics_[iLeft].fx();
            double fyLeft = lb2_.intrinsics_[iLeft].fy();
            double cxLeft = lb2_.intrinsics_[iLeft].cx();
            double cyLeft = lb2_.intrinsics_[iLeft].cy();

            double fxRight = lb2_.intrinsics_[iRight].fx();
            double fyRight = lb2_.intrinsics_[iRight].fy();
            double cxRight = lb2_.intrinsics_[iRight].cx();
            double cyRight = lb2_.intrinsics_[iRight].cy();

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
                points[i].push_back(point3D);
                validPointCounter++;
                valid.push_back(true);
                features[iLeft].push_back(inlierMatches[j].getFirstFeature());
                features[iRight].push_back(inlierMatches[j].getSecondFeature());
            }            
            else
            {
                valid.push_back(false);
            }
        }

        cv::Mat imMatches = featureMatcher_.highlightMatches(imagesROI[iLeft], imagesROI[iRight], matchesStereoROI, valid);
        char name[20];
        sprintf(name, "m_%d", i);
        cv::namedWindow(std::string(name), CV_WINDOW_AUTOSIZE);
        cv::imshow(std::string(name), imMatches);
        cv::waitKey(10);        
    }

    std::cout << "----------------- # VALID POINTS: " << validPointCounter << std::endl;

    return points;
}

}

