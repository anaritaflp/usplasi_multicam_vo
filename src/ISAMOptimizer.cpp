
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
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fx, fy, 0.0, cy, cy));
    K_ = K;

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
    points_.clear();
    poses_.clear();
    poseNumber_ = 0;
}

bool ISAMOptimizer::addData(std::vector<Eigen::Vector3f> points, std::vector<Match> matches, Eigen::Matrix4f pose)
{
    bool ret;

    // add initial pose estimate
    gtsam::Rot3 R(pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2), pose(2, 0), pose(2, 1), pose(2, 2));
    gtsam::Point3 t(pose(0, 3), pose(1, 3), pose(2, 3));
    gtsam::Pose3 pose3(R, t);    
    poses_.push_back(pose3);

    // optimizer was reset - time to consider new 3D points
    if(poseNumber_ == 0)
    {
        std::cout << "\tADDING NEW 3D: " << points.size() << std::endl;
        if(points.size() > 0)
        {
            for(int i=0; i<points.size(); i++)
            {
                gtsam::Point3 point3 = gtsam::Point3(points[i](0), points[i](1), points[i](2));

                // add points to points_ vector
                Point3D p(point3, matches[i]);
                points_.push_back(p);
            }
        }

        ret = true;
    }
    else
    {
        // mark all points as outdated
        markPointsAsOutdated();

        // add 2D measurements
        for(int i=0; i<matches.size(); i++)
        {
            // check if the corresponding point already exists
            int correspondingIndex = findCorrespondingPoint(matches[i]);

            if(correspondingIndex > 0)
            {
                points_[correspondingIndex].addMeasurement(matches[i].getSecondFeature());
                points_[correspondingIndex].markAsUpdated();

            }
        }

        // remove outdated points
        removeOutdatedPoints();
    }

    // check number of visible points
    std::cout << "# VISIBLE POINTS: " << points_.size() << "   MIN: " << minCorrespondedPoints_ << std::endl;
    if(points_.size() < minCorrespondedPoints_)
    {
        // add poses to graph
        for(int i=0; i<poses_.size(); i++)
        {
            // add 2D measurements for that pose
            for(int j=0; j<points_.size(); j++)
            {
                std::cout << "adding 2D measurement on x-" << i << " l-" << j << std::endl;
                gtsam::Point2 p2 = points_[j].getMeasurements()[i].getMeasurement();
                graph_.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(p2, measurementNoise_, gtsam::Symbol('x', i), gtsam::Symbol('l', j), K_));
            }

            // add initial guess on poses
            std::cout << "adding pose on x-" << i << std::endl;
            initialEstimate_.insert(gtsam::Symbol('x', i), poses_[i]);
            std::cout << "aqui 1" << std::endl;
            // if first pose, add prior
            if(i == 0)
            {
                std::cout << "adding prior on pose x-" << i << std::endl;
                graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', i), poses_[i], poseNoise_));
            
                std::cout << "adding prior on 3D point l-" << i << std::endl;
                graph_.push_back(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 0), points_[0].getPoint(), pointNoise_));

                // add initial guess of 3D points to graph
                for(int j=0; j<points_.size(); j++)
                {
                    std::cout << "adding 3D point on l-" << j << std::endl;
                    initialEstimate_.insert(gtsam::Symbol('l', j), points_[j].getPoint());
                }
            
            }

            else
            {
                // update ISAM estimates
                graph_.print();
                std::cout << "aqui 2" << std::endl;
                isam_.update(graph_, initialEstimate_);
                std::cout << "aqui 3" << std::endl;
                gtsam::Values currentEstimate = isam_.estimate();
                std::cout << "aqui 4" << std::endl;
                // clear graph and initial estimates
                std::cout << "cleaning everything" << std::endl;
                graph_.resize(0);
                initialEstimate_.clear();
            }
        }
    }

    // update pose number
    poseNumber_++;
    return ret;
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

void ISAMOptimizer::markPointsAsOutdated()
{
    for(int i=0; i<points_.size(); i++)
    {
        points_[i].markAsOutdated();
    }
}

void ISAMOptimizer::removeOutdatedPoints()
{
    for(int i=0; i<points_.size(); i++)
    {
        if(!points_[i].isUpdated())
        {
            points_.erase(points_.begin() + i);
            i--;
        }
    }
}

void ISAMOptimizer::optimize()
{

}