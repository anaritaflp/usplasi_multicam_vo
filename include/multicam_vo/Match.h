#ifndef MATCH_H
#define MATCH_H

// std includes
#include <stdlib.h>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>

// project includes
#include <multicam_vo/Feature.h>

//! Class representing a feature match. It contains both matched features and the matching distance.
class Match
{
    public:
    
        /** Default Match constructor.
         * @param void
         * @return Match object */
        Match();
    
        /** Match constructor with arguments.
         * @param Feature previous feature
		 * @param Feature current feature
		 * @param float matching distance
         * @return Match object */
        Match(Feature feature_1, Feature feature_2, float distance);
        
		/** Match copy constructor.
         * @param Match object
         * @return Match object */
        Match(const Match &other); 
		
		/** Equal operator.
         * @param Match object
         * @return Match object */
        Match & operator=(const Match &other); 
		
        /** Match destructor. */
        ~Match();
    
        /** Get the match's previous feature.
         * @param void
         * @return Feature previous feature */
        Feature getFirstFeature();
    
        /** Get the match's current feature.
         * @param void
         * @return Feature current feature */
        Feature getSecondFeature();
    
        /** Get the match's matching distance.
         * @param void
         * @return int distance */
        float getDistance();
    
        /** Get the match's color.
         * @param void
         * @return cv::Scalar color */
        cv::Scalar getColor();
        
        /** Set the match's previous feature.
         * @param Feature previous feature
         * @return void */
        void setFirstFeature(Feature feature_1);
    
        /** Set the match's current feature.
         * @param Feature current feature
         * @return void */
        void setSecondFeature(Feature feature_2);
    
        /** Set the match's matching distance.
         * @param int distance
         * @return void */
        void setDistance(float distance);
    
        /** Set the match's color.
         * @param cv::Scalar color
         * @return void */
        void setColor(cv::Scalar color);
    
    private:
    
        Feature feature_1_;     /*!< Feature in the previous frame */
        Feature feature_2_;     /*!< Correspondng feature in the current frame */
        float distance_;        /*!< Matching distance. The smaller the distance, the stronger the match. */
        cv::Scalar color_;      /*!< A color for the match. This is useful if you want do indicate the features in the images. Features with the same color in consecutive frames have been matched. */
};

#endif