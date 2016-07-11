#ifndef FEATURE_H
#define FEATURE_H

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"

//! Class containing all information concerned to a feature, including keypoint, descriptor, frame number and camera index.
class Feature
{
    public:
    
        /** Default Feature constructor.
         * @param void
         * @return Feature object */
        Feature();    
    
        /** Feature constructor with arguments.
         * @param cv::KeyPoint keypoint
		 * @param cv::Mat descriptor
		 * @param int frame number
		 * @param int camera number
         * @return Feature object */
        Feature(cv::KeyPoint keypoint, cv::Mat descriptor, int seqNumber, int camNumber);
		
		/** Feature copy constructor.
         * @param Feature object
         * @return Feature object */
        Feature(const Feature &other); 
		
		/** Equal operator.
         * @param Feature object
         * @return Feature object */
        Feature & operator=(const Feature &other); 
        
        /** Feature destructor. */
        ~Feature();
    
        /** Get the feature's keypoint.
         * @param void
         * @return cv::KeyPoint keypoint */
        cv::KeyPoint getKeypoint();
    
        /** Get the feature's descriptor.
         * @param void
         * @return cv::Mat descriptor */
        cv::Mat getDescriptor();
    
        /** Get the frame number of the image where the feature was found.
         * @param void
         * @return int frame number */
        int getSeqNumber();
    
        /** Get the index of the camera where the feature was found.
         * @param void
         * @return int camera number */
        int getCamNumber();
    
        /** Set the feature's keypoint.
         * @param cv::KeyPoint keypoint
         * @return void */
        void setKeypoint(cv::KeyPoint keypoint);
    
        /** Set the feature's descriptor.
         * @param cv::Mat descriptor
         * @return void */
        void setDescriptor(cv::Mat descriptor);
    
        /** Set the frame number of the image where the feature was found.
         * @param int frame number
         * @return void */
        void setSeqNumber(int seqNumber);
    
        /** Set the index of the camera where the feature was found.
         * @param int camera number
         * @return void */
        void setCamNumber(int camNumber);
    
    private:
        cv::KeyPoint keypoint_;     /*!< cv::Keypoint of the feature, which includes the pixel location */
        cv::Mat descriptor_;        /*!< Descriptor computer by a descriptor extraction method */
        int seqNumber_;             /*!< Frame number */
        int camNumber_;             /*!< Camera index in the multi-camera system */
};

#endif
