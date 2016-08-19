
#include <multicam_vo/ISAMOptimizer.h>

ISAMOptimizer::ISAMOptimizer()
{
    // read params, fill variables

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

void ISAMOptimizer::addPoints(std::vector<Eigen::Vector3f> points)
{

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