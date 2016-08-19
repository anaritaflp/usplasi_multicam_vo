#ifndef ISAM_OPTIMIZER_H
#define ISAM_OPTIMIZER_H

// std includes
#include <iostream>
#include <vector>

// GTSAM includes
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// project includes
#include <multicam_vo/Points.h>
#include <multicam_vo/utils.h>

class ISAMOptimizer
{
    public:
        ISAMOptimizer();
        ~ISAMOptimizer();
        void reset();
        void addPriorPose(Eigen::Matrix4f priorPose);
        void addPoints(std::vector<Eigen::Vector3f> points);
        void addPoseEstimates(std::vector<Eigen::Matrix4f> poseEstimates);
        bool addMeasurements(std::vector<Match> matches);
        void optimize();

    private:
        int nCameras_;        
        int nPoses_;

        std::vector<Point3D> points_;
        std::vector<std::vector<gtsam::Pose3>> cameraPoses_;
        std::vector<gtsam::Cal3_S2> Ks_;

        gtsam::noiseModel::Isotropic::shared_ptr measurementNoise_;
        gtsam::noiseModel::Isotropic::shared_ptr pointNoise_;
        gtsam::noiseModel::Diagonal::shared_ptr poseNoise_;
        int minCorrespondedPoints_;

        gtsam::ISAM2Params isamParameters_;
        gtsam::ISAM2 isam_;
        gtsam::NonlinearFactorGraph graph_;
        int ISAMIters_;

        
        
};

#endif