#include <multicam_vo/Match.h>

Match::Match()
{
    pPrev_ = cv::Point2f(0.0, 0.0);
    pCurr_ = cv::Point2f(0.0, 0.0);
}

Match::Match(cv::Point2f pPrev, cv::Point2f pCurr)
{
    pPrev_ = cv::Point(pPrev.x, pPrev.y);
    pCurr_ = cv::Point(pCurr.x, pCurr.y);
}

Match::Match(const Match &other)
{
    pPrev_ = other.pPrev_;
    pCurr_ = other.pCurr_;
} 

Match& Match::operator=(const Match &other)
{
    pPrev_ = other.pPrev_;
    pCurr_ = other.pCurr_;
} 

Match::~Match()
{

}

cv::Mat highlightOpticalFlow(cv::Mat image, std::vector<Match> matches, cv::Scalar color)
{
    cv::Mat imageOptFlow;

    if(image.channels() == 1)
    {
        cv::cvtColor(image, imageOptFlow, CV_GRAY2BGR);
    }
    else
    {
        imageOptFlow = image;
    }

    // for each intra-camera match draw a line between previous and current feature
    for(int i=0; i<matches.size(); i++)
    {
        cv::Point2f pt_1 = matches[i].pPrev_;
        cv::Point2f pt_2 = matches[i].pCurr_;
        
        cv::line(imageOptFlow, pt_1, pt_2, color);
        //cv::circle(imageOptFlow, pt_2, 2, color);
    }
    return imageOptFlow;
}

std::vector<cv::Point2f> getPrevPoints(std::vector<Match> matches)
{
    std::vector<cv::Point2f> prevPoints;
    for(int i=0; i<matches.size(); i++)
    {
        prevPoints.push_back(matches[i].pPrev_);
    }
    return prevPoints;
}
std::vector<cv::Point2f> getCurrPoints(std::vector<Match> matches)
{
    std::vector<cv::Point2f> currPoints;
    for(int i=0; i<matches.size(); i++)
    {
        currPoints.push_back(matches[i].pCurr_);
    }
    return currPoints;
}
