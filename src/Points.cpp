#include <multicam_vo/Points.h>

Point2D::Point2D()
{
    camNumber_ = 0;
    seqNumber_ = 0;
    measurement_ = gtsam::Point2(0.0, 0.0);
}

Point2D::Point2D(Feature feature)
{
    camNumber_ = feature.getCamNumber();
    seqNumber_ = feature.getSeqNumber();
    measurement_ = gtsam::Point2(feature.getKeypoint().pt.x, feature.getKeypoint().pt.y); 
}

Point2D::Point2D(const Point2D &other)
{
    camNumber_ = other.camNumber_;
    seqNumber_ = other.seqNumber_;
    measurement_ = other.measurement_; 
}

Point2D & Point2D::operator=(const Point2D &other)
{
    camNumber_ = other.camNumber_;
    seqNumber_ = other.seqNumber_;
    measurement_ = other.measurement_; 
}

Point2D::~Point2D()
{

}

int Point2D::getCamNumber()
{
    return camNumber_;
}

int Point2D::getSeqNumber()
{
    return seqNumber_;
}

gtsam::Point2 Point2D::getMeasurement()
{
    return measurement_;
}

void Point2D::setCamNumber(int camNumber)
{
    camNumber_ = camNumber;
}

void Point2D::setSeqNumber(int seqNumber)
{
    seqNumber_ = seqNumber;
}

void Point2D::setMeasurement(gtsam::Point2 measurement)
{
    measurement_ = measurement;
}

Point3D::Point3D()
{
    point_ = gtsam::Point3(0.0, 0.0, 0.0);
}

Point3D::Point3D(gtsam::Point3 point, Match match)
{
    point_ = point;
    Point2D p2D(match.getSecondFeature());
    measurements_.push_back(p2D);
    lastDescriptor_ = match.getSecondFeature().getDescriptor();
}

Point3D::Point3D(const Point3D &other)
{
    point_ = other.point_;
    measurements_ = other.measurements_;
    lastDescriptor_ = other.lastDescriptor_;
}

Point3D & Point3D::operator=(const Point3D &other)
{
    point_ = other.point_;
    measurements_ = other.measurements_;
    lastDescriptor_ = other.lastDescriptor_;
}

Point3D::~Point3D()
{

}

gtsam::Point3 Point3D::getPoint()
{
    return point_;
}

std::vector<Point2D> Point3D::getMeasurements()
{
    return measurements_;
}

cv::Mat Point3D::getLastDescriptor()
{
    return lastDescriptor_;
}

void Point3D::addMeasurement(Feature feature)
{
    Point2D p2D(feature);
    measurements_.push_back(p2D);
    lastDescriptor_ = feature.getDescriptor();
}

bool Point3D::isCorresponding(Feature feature)
{
    if(!cv::countNonZero(feature.getDescriptor() != lastDescriptor_))
    {
        return true;
    }
    return false;
}

