
#include <multicam_vo/ISAMOptimizer.h>

ISAMOptimizer::ISAMOptimizer()
{

}

ISAMOptimizer::ISAMOptimizer(Eigen::Matrix3f cameraMatrix) : node_("~")
{
    // read noise parameters
    std::vector<double> vecPoseNoise;
    double measurementNoise, pointNoise;
    vecPoseNoise.resize(6);

    node_.param<double>("measurementNoise", measurementNoise, 1.0);
    node_.param<double>("pointNoise", pointNoise, 0.1);
    node_.getParam("poseNoise", vecPoseNoise);
    
    measurementNoise_ = gtsam::noiseModel::Isotropic::Sigma(2, measurementNoise);
    pointNoise_ = gtsam::noiseModel::Isotropic::Sigma(3, pointNoise);
    poseNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << vecPoseNoise[0], vecPoseNoise[1], vecPoseNoise[2], vecPoseNoise[3], vecPoseNoise[4], vecPoseNoise[5]));

    // read other parameters
    node_.param<int>("minCorrespondedPoints", minCorrespondedPoints_, 20);
    node_.param<int>("ISAMIters", ISAMIters_, 2);
    node_.param<int>("ISAMRelinearizeInterval", ISAMRelinearizeInterval_, 5);
    node_.param<double>("ISAMRelinearizeThreshold", ISAMRelinearizeThreshold_, 0.1);
    node_.param<int>("ISAMRelinearizeSkip", ISAMRelinearizeSkip_, 10);

    // add calibration
    double fx = cameraMatrix(0, 0);
    double fy = cameraMatrix(1, 1);
    double cx = cameraMatrix(0, 2);
    double cy = cameraMatrix(1, 2);
    K_ =  gtsam::Cal3_S2(fx, fy, 0, cx, cy);

    // create nonlinear ISAM
    isam_ = gtsam::NonlinearISAM(ISAMRelinearizeInterval_);

    // initialize pose number
    poseNumber_ = 0;
}

ISAMOptimizer::~ISAMOptimizer()
{

}

void ISAMOptimizer::reset()
{

}

bool ISAMOptimizer::addData(std::vector<Eigen::Vector3f> points, std::vector<Match> matches, Eigen::Matrix4f pose)
{
    int visiblePoints = 0;

    // add 2D measurements
    for(int i=0; i<matches.size(); i++)
    {
        // check if the corresponding point already exists
        int correspondingIndex = findCorrespondingPoint(matches[i]);

        if(correspondingIndex > 0)
        {
            points_[correspondingIndex].addMeasurement(matches[i].getSecondFeature());
            visiblePoints++;

            cv::KeyPoint kpt = matches[i].getFirstFeature().getKeypoint();
            gtsam::Point2 point2D(kpt.pt.x, kpt.pt.y);
            //graph_.add(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(point2D, measurementNoise_, gtsam::Symbol('x', poseNumber_), gtsam::Symbol('l', correspondingIndex), K_));
        }
    }

    std::cout << "\tVisible points: " << visiblePoints << std::endl;
    if(visiblePoints < minCorrespondedPoints_)
    {
        return false;
    }

    // add initial pose estimate
    gtsam::Rot3 R(pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2), pose(2, 0), pose(2, 1), pose(2, 2));
    gtsam::Point3 t(pose(0, 3), pose(1, 3), pose(2, 3));
    gtsam::Pose3 pose3(R, t);    
    initialEstimate_.insert(gtsam::Symbol('x', poseNumber_), pose3);

    // if it is the first pose
    if(poseNumber_ == 0)
    {
        // add prior on first pose
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', 0), pose3, poseNoise_));

        // add prior on first landmark
        gtsam::Point3 point3(points[0](0), points[0](1), points[0](2));
        graph_.add(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 0), point3, pointNoise_));
    
        // add initial estimates of all 3D points
        for(int j=0; j<points.size(); j++)
        {
            point3 = gtsam::Point3(points[0](0), points[0](1), points[0](2));
            initialEstimate_.insert(gtsam::Symbol('l', j), point3);
        }
    }

    poseNumber_++;
    return true;
}

int ISAMOptimizer::findCorrespondingPoint(Match match)
{
    for(int i=0; i<points_.size(); i++)
    {
        if(points_[i].isCorresponding(match.getFirstFeature()))
        {
            return i;
        }
    }
    return -1;
}

void ISAMOptimizer::optimize()
{

}