#include <multicam_vo/Match.h>

/** Default Match constructor.
 * @return Match object */
Match::Match()
{
    // empty variables
    feature_1_ = Feature();
    feature_2_ = Feature();
    distance_ = 0;
    
    // create a random color for the match
    int randB = rand() % 255;
    int randG = rand() % 255;
    int randR = rand() % 255;
    
    color_ = cv::Scalar(randB, randG, randR);
}
    
/** Match constructor with arguments.
 * @param Feature previous feature
 * @param Feature current feature
 * @param float matching distance
 * @return Match object */
Match::Match(Feature feature_1, Feature feature_2, float distance)
{
    feature_1_ = feature_1;
    feature_2_ = feature_2;
    distance_ = distance;
    
    int randB = rand() % 255;
    int randG = rand() % 255;
    int randR = rand() % 255;
    
    color_ = cv::Scalar(randB, randG, randR);
}

/** Match copy constructor.
 * @param Match object
 * @return Match object */
Match::Match(const Match &other)
{
	feature_1_ = other.feature_1_;
	feature_2_ = other.feature_2_;
	distance_ = other.distance_;
	color_ = other.color_;
}

/** Equal operator.
 * @param Match object
 * @return Match object */
Match &Match::operator=(const Match &other)
{
	feature_1_ = other.feature_1_;
	feature_2_ = other.feature_2_;
	distance_ = other.distance_;
	color_ = other.color_;
}
        
/** Match destructor. */
Match::~Match()
{
    
}

/** Get the match's previous feature.
 * @param void
 * @return Feature previous feature */
Feature Match::getFirstFeature()
{
    return feature_1_;
}

/** Get the match's current feature.
 * @param void
 * @return Feature current feature */
Feature Match::getSecondFeature()
{
    return feature_2_;
}

/** Get the match's matching distance.
 * @param void
 * @return int distance */
float Match::getDistance()
{
    return distance_;
}

/** Get the match's color.
 * @param void
 * @return cv::Scalar color */
cv::Scalar Match::getColor()
{
    return color_;
}
    
/** Set the match's previous feature.
 * @param Feature previous feature
 * @return void */
void Match::setFirstFeature(Feature feature_1)
{
    feature_1_ = feature_1;
}

/** Set the match's current feature.
 * @param Feature current feature
 * @return void */
void Match::setSecondFeature(Feature feature_2)
{
    feature_2_ = feature_2;
}

/** Set the match's matching distance.
 * @param int distance
 * @return void */
void Match::setDistance(float distance)
{
    distance_ = distance;
}

/** Set the match's color.
 * @param cv::Scalar color
 * @return void */
void Match::setColor(cv::Scalar color)
{
    color_ = color;
}
