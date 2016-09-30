#include <multicam_vo/MulticamVOSimPipeline.h>

namespace odom_sim
{

/** Default MulticamVOSimPipeline constructor.
 * @param std::vector<std::ofstream*> vector of files to write estimated poses
 * @return MulticamVOPipeline object */
MulticamVOSimPipeline::MulticamVOSimPipeline(std::vector<std::ofstream*> files)
{
    // private node
    node_ = ros::NodeHandle("~");

    // create publisher and publish first pose, which is identity
    pubOdom_ = node_.advertise<nav_msgs::Odometry>("sim_multicam_vo/odometry", 1);
    nav_msgs::Odometry msgOdom = transform2OdometryMsg(Eigen::Matrix4f::Identity(), -1);
    pubOdom_.publish(msgOdom);

    // initialize frame counter
    int frameCounter = 1;

    // initialize ladybug
    lb2_ = Ladybug2(node_);

    // create odometer
    odometer_ = MulticamOdometer(lb2_, files);

    // read parameters
    node_.param<std::string>("path_to_sim_points", param_pathToSimPoints_, "");
    node_.param<int>("num_frames", param_numFrames_, 0);
    node_.param<int>("num_points", param_numPoints_, 0);
    node_.param<double>("noise_variance", param_noiseVariance_, 0.0);

    // create random descriptor for each point. All descriptors must be different.
    /*bool flag = true;
    while(flag)
    {
        createDescriptors();
        flag = !allDescriptorsDifferent();
    }*/
    createDescriptors();

    absolutePoseGlobal_ = Eigen::Matrix4f::Identity();
}
        
/** Default MulticamVOSimPipeline destructor. */
MulticamVOSimPipeline::~MulticamVOSimPipeline()
{

}

/** Loop function, where all files with simulated points are read and motion estimation is computed.
 * @param std::vector<std::ofstream*> vector of output files where individual camera motion estimates shall be written
 * @return void */
void MulticamVOSimPipeline::loop(std::vector<std::ofstream*> files)
{
    int frameCounter = 1;

    while(1)
    {
        if(frameCounter >= param_numFrames_)
        {
            for(int i=0; i<3*(NUM_CAMERAS-1); i++)
            {
                (*files[i]) << "\n\t];" << std::endl;
                (*files[i]).close();
            }
            
            ros::shutdown();
            return;
        }

        // open files with simulated points of previous and current frames
        char fnamePrev[100], fnameCurr[100];
        sprintf(fnamePrev, "points2D_%06d.txt", frameCounter);
        sprintf(fnameCurr, "points2D_%06d.txt", frameCounter+1);
        std::ifstream finPrev, finCurr;
        finPrev.open((param_pathToSimPoints_ + fnamePrev).c_str());
        finCurr.open((param_pathToSimPoints_ + fnameCurr).c_str());
        
        // create matches vector
        std::vector<std::vector<Match>> matches;
        matches.resize(3 * NUM_OMNI_CAMERAS);

        // read the two files and fill matches vector
        int iPrev = 1, iCurr = 1;
        int pointIndex = 0;
        while(1)
        {
            // variables to be read
            int ok0p, ok1p, ok2p, ok3p, ok4p, ok5p;
            int ok0c, ok1c, ok2c, ok3c, ok4c, ok5c;
            double u0p, v0p, u1p, v1p, u2p, v2p, u3p, v3p, u4p, v4p, u5p, v5p;
            double u0c, v0c, u1c, v1c, u2c, v2c, u3c, v3c, u4c, v4c, u5c, v5c;
            
            // read line
            std::string linePrev, lineCurr;
		    std::vector<std::string> wordsPrev, wordsCurr;
            
            getline(finPrev, linePrev);
		    getline(finCurr, lineCurr);

            if(linePrev == "" || lineCurr == "")
            {
                break;
            }
            
            // split line into numbers
            boost::split(wordsPrev, linePrev, boost::is_any_of(" "));
		    boost::split(wordsCurr, lineCurr, boost::is_any_of(" "));
            
            // if this is not the first frame
            if(frameCounter > 0 && wordsPrev.size() > 1 && wordsCurr.size() > 1)
            {
                // turn strings into numbers
                ok0p = atoi(wordsPrev[0].c_str());
                ok1p = atoi(wordsPrev[1].c_str());
                ok2p = atoi(wordsPrev[2].c_str());
                ok3p = atoi(wordsPrev[3].c_str());
                ok4p = atoi(wordsPrev[4].c_str());
                ok5p = atoi(wordsPrev[5].c_str());

                ok0c = atoi(wordsCurr[0].c_str());
                ok1c = atoi(wordsCurr[1].c_str());
                ok2c = atoi(wordsCurr[2].c_str());
                ok3c = atoi(wordsCurr[3].c_str());
                ok4c = atoi(wordsCurr[4].c_str());
                ok5c = atoi(wordsCurr[5].c_str());

                u0p = atof(wordsPrev[6].c_str());  v0p = atof(wordsPrev[7].c_str());
                u1p = atof(wordsPrev[8].c_str());  v1p = atof(wordsPrev[9].c_str());
                u2p = atof(wordsPrev[10].c_str()); v2p = atof(wordsPrev[11].c_str());
                u3p = atof(wordsPrev[12].c_str()); v3p = atof(wordsPrev[13].c_str());
                u4p = atof(wordsPrev[14].c_str()); v4p = atof(wordsPrev[15].c_str());
                u5p = atof(wordsPrev[16].c_str()); v5p = atof(wordsPrev[17].c_str());

                u0c = atof(wordsCurr[6].c_str());  v0c = atof(wordsCurr[7].c_str());
                u1c = atof(wordsCurr[8].c_str());  v1c = atof(wordsCurr[9].c_str());
                u2c = atof(wordsCurr[10].c_str()); v2c = atof(wordsCurr[11].c_str());
                u3c = atof(wordsCurr[12].c_str()); v3c = atof(wordsCurr[13].c_str());
                u4c = atof(wordsCurr[14].c_str()); v4c = atof(wordsCurr[15].c_str());
                u5c = atof(wordsCurr[16].c_str()); v5c = atof(wordsCurr[17].c_str());

                // matches where the previous feature is in camera 0
                // intra-camera match
                if(ok0p == 1 && ok0c == 1)
                {
                    Match m = createMatch(u0p, v0p, u0c, v0c, pointIndex, frameCounter, 0, 0);
                    matches[0].push_back(m);
                }
                // match with right camera
                if(ok0p == 1 && ok1c == 1)
                {
                    Match m = createMatch(u0p, v0p, u1c, v1c, pointIndex, frameCounter, 0, 1);
                    matches[1].push_back(m);
                }
                // match with left camera
                if(ok0p == 1 && ok4c == 1)
                {
                    Match m = createMatch(u0p, v0p, u4c, v4c, pointIndex, frameCounter, 0, 4);
                    matches[2].push_back(m);
                }

                // matches where the previous feature is in camera 1
                // intra-camera match
                if(ok1p == 1 && ok1c == 1)
                {
                    Match m = createMatch(u1p, v1p, u1c, v1c, pointIndex, frameCounter, 1, 1);
                    matches[3].push_back(m);
                }
                // match with right camera
                if(ok1p == 1 && ok2c == 1)
                {
                    Match m = createMatch(u1p, v1p, u2c, v2c, pointIndex, frameCounter, 1, 2);
                    matches[4].push_back(m);
                }
                // match with left camera
                if(ok1p == 1 && ok0c == 1)
                {
                    Match m = createMatch(u1p, v1p, u0c, v0c, pointIndex, frameCounter, 1, 0);
                    matches[5].push_back(m);
                }

                // matches where the previous feature is in camera 2
                // intra-camera match
                if(ok2p == 1 && ok2c == 1)
                {
                    Match m = createMatch(u2p, v2p, u2c, v2c, pointIndex, frameCounter, 2, 2);
                    matches[6].push_back(m);
                }
                // match with right camera
                if(ok2p == 1 && ok3c == 1)
                {
                    Match m = createMatch(u2p, v2p, u3c, v3c, pointIndex, frameCounter, 2, 3);
                    matches[7].push_back(m);
                }
                // match with left camera
                if(ok2p == 1 && ok1c == 1)
                {
                    Match m = createMatch(u2p, v2p, u1c, v1c, pointIndex, frameCounter, 2, 1);
                    matches[8].push_back(m);
                }

                // matches where the previous feature is in camera 3
                // intra-camera match
                if(ok3p == 1 && ok3c == 1)
                {
                    Match m = createMatch(u3p, v3p, u3c, v3c, pointIndex, frameCounter, 3, 3);
                    matches[9].push_back(m);
                }
                // match with right camera
                if(ok3p == 1 && ok4c == 1)
                {
                    Match m = createMatch(u3p, v3p, u4c, v4c, pointIndex, frameCounter, 3, 4);
                    matches[10].push_back(m);
                }
                // match with left camera
                if(ok3p == 1 && ok2c == 1)
                {
                    Match m = createMatch(u3p, v3p, u2c, v2c, pointIndex, frameCounter, 3, 2);
                    matches[11].push_back(m);
                }

                // matches where the previous feature is in camera 4
                // intra-camera match
                if(ok4p == 1 && ok4c == 1)
                {
                    Match m = createMatch(u4p, v4p, u4c, v4c, pointIndex, frameCounter, 4, 4);
                    matches[12].push_back(m);
                }
                // match with right camera
                if(ok4p == 1 && ok0c == 1)
                {
                    Match m = createMatch(u4p, v4p, u0c, v0c, pointIndex, frameCounter, 4, 0);
                    matches[13].push_back(m);
                }
                // match with left camera
                if(ok4p == 1 && ok3c == 1)
                {
                    Match m = createMatch(u4p, v4p, u3c, v3c, pointIndex, frameCounter, 4, 3);
                    matches[14].push_back(m);
                }
            }
            pointIndex++;
        }

        std::cout << "FRAME: " << frameCounter << std::endl;
        for(int i=0; i<3*NUM_OMNI_CAMERAS; i++)
        {
            std::cout << "\t" << matches[i].size();
            std::cout.flush();
        }
        std::cout << std::endl;

        // reduce number of matches
        std::vector<std::vector<Match>> reducedMatches;
        reducedMatches.resize(3 * NUM_OMNI_CAMERAS);
        for(int i=0; i<3*NUM_OMNI_CAMERAS; i++)
        {
            reducedMatches[i] = reduceMatches(matches[i], 100);  
            reducedMatches[i] = addNoise(reducedMatches[i]); 
        }

        // estimate motion
        int bestCamera;
        std::vector<std::vector<Match>> inlierMatches;
        std::vector<std::vector<Eigen::Vector3f>> points3D;
        std::vector<Eigen::Matrix4f> monoPoses;
        Eigen::Matrix4f TRelative = odometer_.estimateMotion(matches, bestCamera, inlierMatches, points3D, monoPoses);

        // concatenate motion estimation of the best camera to absolute pose and return result
        absolutePoseGlobal_ = absolutePoseGlobal_ * TRelative;

        // publish optimal motion estimation
        nav_msgs::Odometry msgOdom = transform2OdometryMsg(absolutePoseGlobal_, bestCamera);
        pubOdom_.publish(msgOdom);

        frameCounter++;
    }
}

/** Create a match out of corresponding point coordinates.
 * @param double u-coordinate in previous image
 * @param double v-coordinate in previous image
 * @param double u-coordinate in current image
 * @param double v-coordinate in current image
 * @param int point index in the points file
 * @param int frame number
 * @param int index of the previous camera
 * @param int index of the current camera
 * @return Match match */
Match MulticamVOSimPipeline::createMatch(double uPrev, double vPrev, double uCurr, double vCurr, int pointIndex, int frameCounter, int camNumberPrev, int camNumberCurr)
{
    cv::KeyPoint kpPrev;
    kpPrev.pt = cv::Point2f(uPrev, vPrev);
    Feature fPrev(kpPrev, descriptors_[pointIndex], frameCounter, camNumberPrev);

    cv::KeyPoint kpCurr;
    kpCurr.pt = cv::Point2f(uCurr, vCurr);
    Feature fCurr(kpCurr, descriptors_[pointIndex], frameCounter+1, camNumberCurr);


    Match m(fPrev, fCurr, 0.0);
    return m;
}

/** Reduce a vector of matches to N randomly selected items.
 * @param std::vector<Match> original vector with matches
 * @param int desired number of matches
 * @return std::vector<Match> reduced vector with N of matches */
std::vector<Match> MulticamVOSimPipeline::reduceMatches(std::vector<Match> matches, int N)
{
    int total = matches.size();

    if(total <= N)
    {
        return matches;
    }
    
    std::vector<int> indices = getRandomSample(total, N);    
    std::vector<Match> reducedMatches;
    for(int i=0; i<N; i++)
    {
        reducedMatches.push_back(matches[indices[i]]);
    }

    return reducedMatches;
}

std::vector<Match> MulticamVOSimPipeline::addNoise(std::vector<Match> matches)
{
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, param_noiseVariance_);

    std::vector<Match> matchesNoise;

    for(int i=0; i<matches.size(); i++)
    {
        double xOffsetPrev = distribution(generator);
        double yOffsetPrev = distribution(generator);

        double xOffsetCurr = distribution(generator);
        double yOffsetCurr = distribution(generator);

        cv::KeyPoint kpPrev = matches[i].getFirstFeature().getKeypoint();
        cv::KeyPoint kpCurr = matches[i].getSecondFeature().getKeypoint();

        kpPrev.pt += cv::Point2f(xOffsetPrev, yOffsetPrev);
        kpCurr.pt += cv::Point2f(xOffsetCurr, yOffsetCurr);

        Feature fPrev(matches[i].getFirstFeature());
        Feature fCurr(matches[i].getSecondFeature());

        fPrev.setKeypoint(kpPrev);
        fCurr.setKeypoint(kpCurr);

        Match mNoise(fPrev, fCurr, 0.0);
        matchesNoise.push_back(mNoise);
    }

    return matchesNoise;
}

bool MulticamVOSimPipeline::allDescriptorsDifferent()
{
    for(int i=0; i<descriptors_.size(); i++)
    {
        for(int j=i+1; j<descriptors_.size(); j++)
        {
            if(!cv::countNonZero(descriptors_[i] != descriptors_[j]))
            {
                return false;
            }
        }
    }
    return true;
}

void MulticamVOSimPipeline::createDescriptors()
{
    for(int i=0; i<param_numPoints_; i++)
    {
        cv::Mat d(1, 1, CV_32S, i);
        descriptors_.push_back(d);
    }
}

}