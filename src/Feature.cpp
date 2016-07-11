#include <multicam_vo/Feature.h>

/** Default Feature constructor.
 * @param void
 * @return Feature object */
Feature::Feature()
{
    // empty variables
    keypoint_ = cv::KeyPoint();
    descriptor_ = cv::Mat();
    seqNumber_ = -1;
    camNumber_ = -1;
}

/** Feature constructor with arguments.
 * @param cv::KeyPoint keypoint
 * @param cv::Mat descriptor
 * @param int frame number
 * @param int camera number
 * @return Feature object */
Feature::Feature(cv::KeyPoint keypoint, cv::Mat descriptor, int seqNumber, int camNumber)
{
    keypoint_ = keypoint;
    descriptor_ = descriptor;
    seqNumber_ = seqNumber;
    camNumber_ = camNumber;
}

/** Feature copy constructor.
 * @param Feature object
 * @return Feature object */
Feature::Feature(const Feature &other)
{
	keypoint_ = other.keypoint_;
	descriptor_ = other.descriptor_;
	seqNumber_ = other.seqNumber_;
	camNumber_ = other.camNumber_;
}

/** Equal operator.
 * @param Feature object
 * @return Feature object */
Feature &Feature::operator=(const Feature &other)
{
	keypoint_ = other.keypoint_;
	descriptor_ = other.descriptor_;
	seqNumber_ = other.seqNumber_;
	camNumber_ = other.camNumber_;
}
        
/** Feature destructor. */
Feature::~Feature()
{
    
}
    
/** Get the feature's keypoint.
 * @param void
 * @return cv::KeyPoint keypoint */
cv::KeyPoint Feature::getKeypoint()
{
    return keypoint_;
}

/** Get the feature's descriptor.
 * @param void
 * @return cv::Mat descriptor */
cv::Mat Feature::getDescriptor()
{
    return descriptor_;
}

/** Get the frame number of the image where the feature was found.
 * @param void
 * @return int frame number */
int Feature::getSeqNumber()
{
    return seqNumber_;
}

/** Get the index of the camera where the feature was found.
 * @param void
 * @return int camera number */
int Feature::getCamNumber()
{
    return camNumber_;
}
    
/** Set the feature's keypoint.
 * @param cv::KeyPoint keypoint
 * @return void */
void Feature::setKeypoint(cv::KeyPoint keypoint)
{
    keypoint_ = keypoint;
}

/** Set the feature's descriptor.
 * @param cv::Mat descriptor
 * @return void */
void Feature::setDescriptor(cv::Mat descriptor)
{
    descriptor_ = descriptor;
}

/** Set the frame number of the image where the feature was found.
 * @param int frame number
 * @return void */
void Feature::setSeqNumber(int seqNumber)
{
    seqNumber_ = seqNumber;
}

/** Set the index of the camera where the feature was found.
 * @param int camera number
 * @return void */
void Feature::setCamNumber(int camNumber)
{
    camNumber_ = camNumber;
}

