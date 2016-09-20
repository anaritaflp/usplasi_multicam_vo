#ifndef ISAM_OPTIMIZER_H
#define ISAM_OPTIMIZER_H

// std includes
#include <iostream>
#include <vector>

// ROS includes
#include <ros/ros.h>

// GTSAM includes
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>

// project includes
#include <multicam_vo/Points.h>
#include <multicam_vo/utils.h>

class ISAMOptimizer
{
    public:
        ISAMOptimizer();
        ISAMOptimizer(Eigen::Matrix3f cameraMatrix);
        ~ISAMOptimizer();
        void reset();
        bool addData(std::vector<Eigen::Vector3f> points, std::vector<Match> matches, Eigen::Matrix4f pose);

    private:

        int findCorrespondingPoint(Match match);
        void markPointsAsOutdated();
        void removeOutdatedPoints();        
        void orderPoints();

        std::vector<Point3D> points_;
        std::vector<gtsam::Pose3> poses_;
        gtsam::Cal3_S2::shared_ptr K_;
        gtsam::Values initialEstimate_;
        int poseNumber_;

        gtsam::noiseModel::Isotropic::shared_ptr measurementNoise_;
        gtsam::noiseModel::Isotropic::shared_ptr pointNoise_;
        gtsam::noiseModel::Diagonal::shared_ptr poseNoise_;
        int minCorrespondedPoints_;

        gtsam::ISAM2Params isamParameters_;
        gtsam::NonlinearISAM isam_;
        gtsam::NonlinearFactorGraph graph_;
        int ISAMRelinearizeInterval_;
        double ISAMRelinearizeThreshold_;
        int ISAMRelinearizeSkip_;
        int ISAMIters_;

        ros::NodeHandle node_;     
};

#endif