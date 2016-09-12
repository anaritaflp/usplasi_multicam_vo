
#ifndef POINTS_H
#define POINTS_H

// std includes
#include <iostream>
#include <vector>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>

// GTSAM includes
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

// project includes
#include <multicam_vo/Feature.h>
#include <multicam_vo/Match.h>

class Point2D
{
    public:
        Point2D();
        Point2D(Feature feature);
        Point2D(const Point2D &other);
        Point2D & operator=(const Point2D &other);
        ~Point2D();

        int getCamNumber();
        int getSeqNumber();
        gtsam::Point2 getMeasurement();
        bool isUpdated();

        void setCamNumber(int camNumber);
        void setSeqNumber(int seqNumber);
        void setMeasurement(gtsam::Point2 measurement);
        void markAsOutdated();
        void markAsUpdated();


    private:
        int camNumber_;
        int seqNumber_;
        gtsam::Point2 measurement_;
};

class Point3D
{
    public:
        Point3D();
        Point3D(gtsam::Point3 point, Match match);
        Point3D(const Point3D &other);
        Point3D & operator=(const Point3D &other);
        ~Point3D();

        gtsam::Point3 getPoint();
        std::vector<Point2D> getMeasurements();
        cv::Mat getLastDescriptor();

        void addMeasurement(Feature feature);
        bool isCorresponding(Feature feature);
        bool isUpdated();
        void markAsOutdated();
        void markAsUpdated();

        
    private:
        gtsam::Point3 point_;
        std::vector<Point2D> measurements_;
        cv::Mat lastDescriptor_;
        bool flagUpdated_;
};

#endif