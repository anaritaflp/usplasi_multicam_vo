#include <multicam_vo/MulticamVOPipeline.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


struct ReprojectionError
{
    ReprojectionError(Eigen::Matrix3f RMotion, Eigen::Vector3f tMotion, Eigen::Matrix4f TCam, Eigen::Matrix3f K, Eigen::Vector3f p3, cv::Point2f p2)
    : RMotion_(RMotion), tMotion_(tMotion), TCam_(TCam), K_(K), p3_(p3), p2_(p2)
    {

    }
    template<typename T> bool operator()(const T* const scale, T* residual) const
    {
        T scaledtx = T(scale[0]) * T(tMotion_(0));
        T scaledty = T(scale[0]) * T(tMotion_(1));
        T scaledtz = T(scale[0]) * T(tMotion_(2));
        Eigen::Vector3f scaledt;
        scaledt << double(scaledtx), double(scaledty), double(scaledtz);
        Eigen::Matrix4f TMotion = Rt2T(RMotion_, scaledt);
        //Eigen::Matrix4f TExt4x4 = TCam_ * TMotion;
        //Eigen::Matrix<float, 3, 4> TExt3x4 = TExt4x4.block(0, 0, 3, 4);
        //Eigen::Matrix<float, 3, 4> P = K_ * TExt3x4;
        Eigen::Matrix<float, 3, 4> P;
        residual[0] = T(p2_.x) - T(P(0, 0))*T(p3_(0)) - T(P(0, 1))*T(p3_(1)) - T(P(0, 2))*T(p3_(2)) - T(P(0, 3));
        residual[1] = T(p2_.y) - T(P(1, 0))*T(p3_(0)) - T(P(1, 1))*T(p3_(1)) - T(P(1, 2))*T(p3_(2)) - T(P(1, 3));
        residual[2] = 1.0 -      T(P(2, 0))*T(p3_(0)) - T(P(2, 1))*T(p3_(1)) - T(P(2, 2))*T(p3_(2)) - T(P(2, 3));
        return true;
    }
    private:
        const Eigen::Matrix3f RMotion_;
        const Eigen::Vector3f tMotion_;
        const Eigen::Matrix4f TCam_;
        const Eigen::Matrix3f K_;
        const Eigen::Vector3f p3_;
        const cv::Point2f p2_;
};

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
    featureDetector_ = FeatureDetector(SHI_TOMASI, ORB);
    featureMatcher_ = FeatureMatcher(lb2_);
    odometer_ = MulticamOdometer(lb2_, files);

    // advertise odometry topic
    pubOdom_ = node_.advertise<nav_msgs::Odometry>("multicam_vo/odometry", 1);
    
    // subscribe to image topic
    image_transport::ImageTransport it(node_);    
    subImage_ = it.subscribe(param_imageTopic_, 1, &MulticamVOPipeline::imageCallback, this);

    absolutePoseGlobal_ = Eigen::Matrix4f::Identity();
    scale_ = 1.0;
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
        featuresAllCameras[i] = featureDetector_.detectFeatures(imagesRectReduced[i], seqNumber, i, false, 1.0); 
        
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
            matches[i] = featureMatcher_.matchFeatures(featuresAllCamerasPrev_[i], featuresAllCameras[i], 30.0);
        }
    }

    Eigen::Matrix4f tLeftRight = lb2_.getCamX2CamYTransform(1, 2);

    // estimate motion (T)
    Eigen::Matrix4f TRelative;
    int bestCamera;
    if(!first_)
    {
        std::vector<std::vector<Match>> inlierMatches;
        std::vector<std::vector<Eigen::Vector3f>> points3D;
        std::vector<Eigen::Matrix4f> monoPoses;
        TRelative = odometer_.estimateMotion(matches, bestCamera, inlierMatches, points3D, monoPoses);

        // track previously triangulated stereo points in the current left and right images
        double offset = 100;
        cv::Mat fullLeft = imagesRectReduced[1];
        cv::Mat fullRight = imagesRectReduced[2];
        cv::Mat partLeft = fullLeft(cv::Rect(fullLeft.cols-offset, 0, offset, fullLeft.rows));
        cv::Mat partRight = fullRight(cv::Rect(0, 0, offset, fullRight.rows));

        FeatureDetector detectorScale(SHI_TOMASI, ORB);
        std::vector<Feature> featuresScaleLeft = detectorScale.detectFeatures(partLeft, seqNumber, 1, false, 8.0);
        std::vector<Feature> featuresScaleRight = detectorScale.detectFeatures(partRight, seqNumber, 2, false, 8.0);

        featuresScaleLeft = displaceFeatures(featuresScaleLeft, fullLeft.cols-offset+param_ROIs_[1][0], param_ROIs_[1][1]);
        featuresScaleRight = displaceFeatures(featuresScaleRight, param_ROIs_[2][0], param_ROIs_[2][1]);

        std::vector<Match> matchesLeftScale = featureMatcher_.matchFeatures(featuresStereoLeftPrev_, featuresScaleLeft, 70.0);
        std::vector<Match> matchesRightScale = featureMatcher_.matchFeatures(featuresStereoRightPrev_, featuresScaleRight, 70.0);

        // test for inliers      

        Eigen::Matrix4f TLeft = monoPoses[1];
        Eigen::Matrix4f TRight = monoPoses[2];

        Eigen::Matrix3f RLeft, RRight;
        Eigen::Vector3f tLeft, tRight;        

        T2Rt(TLeft, RLeft, tLeft);
        T2Rt(TRight, RRight, tRight);

        Eigen::Matrix3f FLeft = odometer_.Rt2F(RLeft, tLeft, lb2_.cameraMatrices_[1], lb2_.cameraMatrices_[1]);
        Eigen::Matrix3f FRight = odometer_.Rt2F(RRight, tRight, lb2_.cameraMatrices_[2], lb2_.cameraMatrices_[2]);

        std::vector<int> inlierIndicesLeft = odometer_.getInliers(matchesLeftScale, FLeft, 40.0);
        std::vector<int> inlierIndicesRight = odometer_.getInliers(matchesRightScale, FRight, 40.0);

        std::vector<Match> inlierMatchesLeft, outlierMatchesLeft, inlierMatchesRight, outlierMatchesRight;
        for(int i=0; i<matchesLeftScale.size(); i++)
        {
            if(elemInVec(inlierIndicesLeft, i))
            {
                inlierMatchesLeft.push_back(matchesLeftScale[i]);
            }
            else
            {
                outlierMatchesLeft.push_back(matchesLeftScale[i]);
            }
        }
        for(int i=0; i<matchesRightScale.size(); i++)
        {
            if(elemInVec(inlierIndicesRight, i))
            {
                inlierMatchesRight.push_back(matchesRightScale[i]);
            }
            else
            {
                outlierMatchesRight.push_back(matchesRightScale[i]);
            }
        }

        // count features going forwards and features going backwards
        int forwardsCounter = 0, backwardsCounter = 0;
        for(int i=0; i<inlierMatchesLeft.size(); i++)
        {
            if(inlierMatchesLeft[i].getFirstFeature().getKeypoint().pt.x < inlierMatchesLeft[i].getSecondFeature().getKeypoint().pt.x)
                forwardsCounter++;
            else if(inlierMatchesLeft[i].getFirstFeature().getKeypoint().pt.x > inlierMatchesLeft[i].getSecondFeature().getKeypoint().pt.x)
                backwardsCounter++;
        }
        for(int i=0; i<inlierMatchesRight.size(); i++)
        {
            if(inlierMatchesRight[i].getFirstFeature().getKeypoint().pt.x < inlierMatchesRight[i].getSecondFeature().getKeypoint().pt.x)
                forwardsCounter++;
            else if(inlierMatchesRight[i].getFirstFeature().getKeypoint().pt.x > inlierMatchesRight[i].getSecondFeature().getKeypoint().pt.x)
                backwardsCounter++;
        }

        // discard features going the opposite way than the majority
        for(int i=0; i<inlierMatchesLeft.size(); i++)
        {
            if(forwardsCounter > backwardsCounter)
            {
                if(inlierMatchesLeft[i].getFirstFeature().getKeypoint().pt.x > inlierMatchesLeft[i].getSecondFeature().getKeypoint().pt.x)
                {
                    inlierMatchesLeft.erase(inlierMatchesLeft.begin() + i);
                    i--;
                }
            }
            else if(forwardsCounter < backwardsCounter)
            {
                if(inlierMatchesLeft[i].getFirstFeature().getKeypoint().pt.x < inlierMatchesLeft[i].getSecondFeature().getKeypoint().pt.x)
                {
                    inlierMatchesLeft.erase(inlierMatchesLeft.begin() + i);
                    i--;
                }
            }
        }
        for(int i=0; i<inlierMatchesRight.size(); i++)
        {
            if(forwardsCounter > backwardsCounter)
            {
                if(inlierMatchesRight[i].getFirstFeature().getKeypoint().pt.x > inlierMatchesRight[i].getSecondFeature().getKeypoint().pt.x)
                {
                    inlierMatchesRight.erase(inlierMatchesRight.begin() + i);
                    i--;
                }
            }
            else if(forwardsCounter < backwardsCounter)
            {
                if(inlierMatchesRight[i].getFirstFeature().getKeypoint().pt.x < inlierMatchesRight[i].getSecondFeature().getKeypoint().pt.x)
                {
                    inlierMatchesRight.erase(inlierMatchesRight.begin() + i);
                    i--;
                }
            }
        }


        Problem problem;
        Eigen::Vector3f p3;
        cv::Point2f p2;
        double scale = scale_;
        if(inlierMatchesLeft.size() + inlierMatchesRight.size() > 0)
        {
            for(int i=0; i<inlierMatchesLeft.size(); i++)
            {
                int index = findFeatureIndex(featuresStereoLeftPrev_, inlierMatchesLeft[i], 1);
                if(index != -1)
                {
                    p3 = stereoPointsPrev_[index];
                    p2 = inlierMatchesLeft[i].getSecondFeature().getKeypoint().pt;
                }
                // ReprojectionError(Eigen::Matrix3f RMotion, Eigen::Vector3f tMotion, Eigen::Matrix4f TCam, Eigen::Matrix3f K, Eigen::Vector3f p3, cv::Point2f p2)
                problem.AddResidualBlock(new AutoDiffCostFunction<ReprojectionError, 3, 1>(new ReprojectionError(RLeft, tLeft, Eigen::Matrix4f::Identity(), lb2_.cameraMatrices_[1], p3, p2)), NULL, &scale);
            }
            for(int i=0; i<inlierMatchesRight.size(); i++)
            {
                int index = findFeatureIndex(featuresStereoRightPrev_, inlierMatchesRight[i], 1);
                if(index != -1)
                {
                    p3 = stereoPointsPrev_[index];
                    p2 = inlierMatchesRight[i].getSecondFeature().getKeypoint().pt;
                }
                problem.AddResidualBlock(new AutoDiffCostFunction<ReprojectionError, 3, 1>(new ReprojectionError(RRight, tRight, tLeftRight, lb2_.cameraMatrices_[2], p3, p2)), NULL, &scale);
            }

            Solver::Options options;
            options.max_num_iterations = 25;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = false;
            Solver::Summary summary;
            Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";
        }

        if(scale > 1.0 || scale < -1.0)
        {
            scale = scale_;
        }


        Eigen::Matrix3f RFinal;
        Eigen::Vector3f tFinal;
        T2Rt(TRelative, RFinal, tFinal);

        std::cout << "=========================================================================" << std::endl;
        std::cout << "===" << inlierMatchesLeft.size() << "\t" << inlierMatchesRight.size() << "\t" << scale << "\t(" << tFinal(0) << ", " << tFinal(1) << ")" << std::endl;
        std::cout << "=========================================================================" << std::endl;

        //scale = -1;
        Eigen::Matrix4f TFinal = Rt2T(RFinal, -scale * tFinal);
        scale_ = scale;

        // concatenate motion estimation of the best camera to absolute pose and return result
        absolutePoseGlobal_ = absolutePoseGlobal_ * TFinal;

        //std::cout << "******************************* # MOTION INLIERS LEFT:  " << inlierIndicesLeft.size() << std::endl;
        //std::cout << "******************************* # MOTION INLIERS RIGHT: " << inlierIndicesRight.size() << std::endl;

        inlierMatchesLeft = displaceMatches(inlierMatchesLeft, -param_ROIs_[1][0], -param_ROIs_[1][1], -param_ROIs_[1][0], -param_ROIs_[1][1]);
        outlierMatchesLeft = displaceMatches(outlierMatchesLeft, -param_ROIs_[1][0], -param_ROIs_[1][1], -param_ROIs_[1][0], -param_ROIs_[1][1]);
        inlierMatchesRight = displaceMatches(inlierMatchesRight, -param_ROIs_[2][0], -param_ROIs_[2][1], -param_ROIs_[2][0], -param_ROIs_[2][1]);
        outlierMatchesRight = displaceMatches(outlierMatchesRight, -param_ROIs_[2][0], -param_ROIs_[2][1], -param_ROIs_[2][0], -param_ROIs_[2][1]);

        fullLeft = featureMatcher_.highlightOpticalFlow(fullLeft, inlierMatchesLeft, cv::Scalar(0, 255, 0));
        fullLeft = featureMatcher_.highlightOpticalFlow(fullLeft, outlierMatchesLeft, cv::Scalar(0, 0, 255));
        fullRight = featureMatcher_.highlightOpticalFlow(fullRight, inlierMatchesRight, cv::Scalar(0, 255, 0));
        fullRight = featureMatcher_.highlightOpticalFlow(fullRight, outlierMatchesRight, cv::Scalar(0, 0, 255));
        cv::namedWindow("optical flow left", CV_WINDOW_AUTOSIZE);
        cv::imshow("optical flow left", fullLeft);
        cv::namedWindow("optical flow right", CV_WINDOW_AUTOSIZE);
        cv::imshow("optical flow right", fullRight);

        //std::cout << "STEREO MATCHES LEFT:  " << matchesLeftScale.size() << std::endl;
        //std::cout << "STEREO MATCHES RIGHT: " << matchesRightScale.size() << std::endl;
    }
    else
    {
        // no motion in first frame -> identity
        absolutePoseGlobal_ = Eigen::Matrix4f::Identity();
        bestCamera = -1;

        // make frames start at 0
        seqNumberOffset_ = seqNumber;

        first_ = false;
    }

    // triangulate stereo points for finding the scale in the next frame
    std::vector<Feature> featuresStereoLeft, featuresStereoRight;
    std::vector<Eigen::Vector3f> stereoPoints = triangulateStereo(imagesRectReduced[1], imagesRectReduced[2], cameraOverlaps_[1][1], cameraOverlaps_[2][0], param_ROIs_[1], param_ROIs_[2], lb2_.cameraMatrices_[1], lb2_.cameraMatrices_[2], tLeftRight, featuresStereoLeft, featuresStereoRight);


    // publish motion
    //std::cout << absolutePoseGlobal_ << std::endl << std::endl;
    nav_msgs::Odometry msgOdom = transform2OdometryMsg(absolutePoseGlobal_, bestCamera);
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
    std::cout << "Processed frame " << (seqNumber - seqNumberOffset_ + 1) << "\tin " << (endingTime - startingTime).toSec() << " seconds" << std::endl;
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
    std::vector<Feature> featuresLeftAll = stereoFeatureDetector.detectFeatures(leftOverlap, 1, 1, false, 8.0);
    std::vector<Feature> featuresRightAll = stereoFeatureDetector.detectFeatures(rightOverlap, 2, 2, false, 8.0);

    // update features' positions in the full image
    featuresLeftAll = displaceFeatures(featuresLeftAll, offsetLeft[0] + overlapLeft, offsetLeft[1]);
    featuresRightAll = displaceFeatures(featuresRightAll, offsetRight[0], offsetRight[1]);

    // feature matching
    std::vector<Match> matchesStereo = featureMatcher_.matchFeatures(featuresLeftAll, featuresRightAll, 70.0);

    Eigen::Matrix3f FStereo = odometer_.Rt2F(RRightLeft, tRightLeft, KLeft, KRight);
    std::vector<int> inlierIndices = odometer_.getInliers(matchesStereo, FStereo, 30.0);
    //std::cout << "***************** # STEREO INLIERS: " << inlierIndices.size() << std::endl;

    std::vector<bool> valid;
    std::vector<Eigen::Vector3f> points;
    for(int i=0; i<matchesStereo.size(); i++)
    {
        if(elemInVec(inlierIndices, i))
        {
            Feature fLeft = matchesStereo[i].getFirstFeature();
            Feature fRight = matchesStereo[i].getSecondFeature();

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
        else
        {
            matchesStereo.erase(matchesStereo.begin() + i);
            i--;
        }
    }

    matchesStereo = displaceMatches(matchesStereo, -offsetLeft[0], -offsetLeft[1], -offsetRight[0], -offsetRight[1]);

    cv::Mat imMatches = featureMatcher_.highlightMatches(leftImage, rightImage, matchesStereo, valid);
    cv::namedWindow("Stereo matches", CV_WINDOW_AUTOSIZE);
    cv::imshow("Stereo matches", imMatches);
    cv::waitKey(10);

    //std::cout << "----------------- # VALID POINTS: " << validPointCounter << std::endl;

    ros::Time end = ros::Time::now();

    return points;
}

std::vector<Feature> MulticamVOPipeline::displaceFeatures(std::vector<Feature> features, double offsetX, double offsetY)
{
    std::vector<Feature> displacedFeatures;
    for(int i=0; i<features.size(); i++)
    {
        cv::KeyPoint kp = features[i].getKeypoint();
        kp.pt.x += offsetX;
        kp.pt.y += offsetY;
        Feature f = features[i];
        f.setKeypoint(kp);
        displacedFeatures.push_back(f);
    }
    return displacedFeatures;
}

std::vector<Match> MulticamVOPipeline::displaceMatches(std::vector<Match> matches, double offsetPrevX, double offsetPrevY, double offsetCurrX, double offsetCurrY)
{
    std::vector<Match> displacedMatches;
    for(int i=0; i<matches.size(); i++)
    {
        Feature fPrev = matches[i].getFirstFeature();
        Feature fCurr = matches[i].getSecondFeature();
        double d = matches[i].getDistance();

        cv::KeyPoint kpPrev = fPrev.getKeypoint();
        cv::KeyPoint kpCurr = fCurr.getKeypoint();

        kpPrev.pt.x += offsetPrevX;
        kpPrev.pt.y += offsetPrevY;

        kpCurr.pt.x += offsetCurrX;
        kpCurr.pt.y += offsetCurrY;

        fPrev.setKeypoint(kpPrev);
        fCurr.setKeypoint(kpCurr);

        Match m(fPrev, fCurr, d);
        displacedMatches.push_back(m);
    }

    return displacedMatches;
}

int MulticamVOPipeline::findFeatureIndex(std::vector<Feature> features, Match match, int firstOrSecond)
{
    Feature f;
    if(firstOrSecond == 1)
    {
        f = match.getFirstFeature();
    }
    else
    {
        f = match.getSecondFeature();
    }

    for(int i=0; i<features.size(); i++)
    {
        if(!cv::countNonZero(f.getDescriptor() != features[i].getDescriptor()))
        {
            return i;
        }
    }
    return -1;
}

}

