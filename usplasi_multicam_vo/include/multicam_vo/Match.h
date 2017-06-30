#ifndef MATCH_H
#define MATCH_H

#include <vector>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Match
{
    public:
        
        Match();
        Match(cv::Point2f pPrev, cv::Point2f pCurr);
        ~Match();
        Match(const Match &other); 
        Match& operator=(const Match &other); 
        cv::Point2f pPrev_, pCurr_;
};

cv::Mat highlightOpticalFlow(cv::Mat image, std::vector<Match> matches, cv::Scalar color);
std::vector<cv::Point2f> getPrevPoints(std::vector<Match> matches);
std::vector<cv::Point2f> getCurrPoints(std::vector<Match> matches);

#endif