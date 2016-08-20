
#include <multicam_vo/ISAMOptimizer.h>

ISAMOptimizer::ISAMOptimizer() : node_("~")
{
    // read params, fill variables
    std::vector<double> vecPoseNoise;
    double measurementNoise, pointNoise;
    vecPoseNoise.resize(6);

    node_.param<double>("measurementNoise", measurementNoise, 1.0);
    node_.param<double>("pointNoise", pointNoise, 0.1);
    node_.getParam("poseNoise", vecPoseNoise);
    
    measurementNoise_ = gtsam::noiseModel::Isotropic::Sigma(2, measurementNoise);
    pointNoise_ = gtsam::noiseModel::Isotropic::Sigma(3, pointNoise);
    poseNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << vecPoseNoise[0], vecPoseNoise[1], vecPoseNoise[2], vecPoseNoise[3], vecPoseNoise[4], vecPoseNoise[5]));

    node_.param<int>("minCorrespondedPoints", minCorrespondedPoints_, 20);
    node_.param<int>("ISAMIters", ISAMIters_, 2);
    node_.param<double>("ISAMRelinearizeThreshold", ISAMRelinearizeThreshold_, 0.1);
    node_.param<int>("ISAMRelinearizeSkip", ISAMRelinearizeSkip_, 10);
}

ISAMOptimizer::~ISAMOptimizer()
{

}

void ISAMOptimizer::reset()
{
    // leave params, clear points and poses vectors
}

void ISAMOptimizer::addPriorPose(Eigen::Matrix4f priorPose)
{

}

void ISAMOptimizer::addPoints(std::vector<Eigen::Vector3f> points, std::vector<Match> matches)
{
    // fill points vector
    for(int i=0; i<points.size(); i++)
    {
        // build Point3D structure
        gtsam::Point3 p(points[i](0), points[i](1), points[i](2));
        Point3D p3D(p, matches[i]);
        points_.push_back(p3D);

        // add point estimates of points to graph
        initialEstimate_.insert(gtsam::Symbol('l', i), points_[i].getPoint());

        // add prior of first landmark to graph
        if(i == 0)
        {
            graph_.push_back(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 0), points_[0].getPoint(), pointNoise_));
        }
    }
}

void ISAMOptimizer::addPoseEstimates(std::vector<Eigen::Matrix4f> poseEstimates)
{

}

bool ISAMOptimizer::addMeasurements(std::vector<Match> matches)
{
    std::vector<int> correspondedPointsIndices;
    for(int i=0; i<matches.size(); i++)
    {
        for(int j=0; j<points_.size(); j++)
        {
            if(elemInVec(correspondedPointsIndices, j))
            {
                continue;
            }

            if(points_[j].isCorresponding(matches[i].getFirstFeature()))
            {
                points_[j].addMeasurement(matches[i].getFirstFeature());
                correspondedPointsIndices.push_back(j);
                break;
            }
        }
    }

    if(correspondedPointsIndices.size() < minCorrespondedPoints_)
    {
        return false;
    }

    return true;
}

void ISAMOptimizer::optimize()
{

}