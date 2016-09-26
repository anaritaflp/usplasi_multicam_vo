
#include <multicam_vo/ISAMOptimizer.h>

ISAMOptimizer::ISAMOptimizer()
{

}

ISAMOptimizer::ISAMOptimizer(Eigen::Matrix3f cameraMatrix) : node_("~")
{
    // read noise parameters
    std::vector<double> vecPoseNoise;
    double measurementNoise, pointNoise;
    vecPoseNoise.resize(6);

    node_.param<double>("measurementNoise", measurementNoise, 1.0);
    node_.param<double>("pointNoise", pointNoise, 0.1);
    node_.getParam("poseNoise", vecPoseNoise);
    
    measurementNoise_ = gtsam::noiseModel::Isotropic::Sigma(2, measurementNoise);
    pointNoise_ = gtsam::noiseModel::Isotropic::Sigma(3, pointNoise);
    poseNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << vecPoseNoise[0], vecPoseNoise[1], vecPoseNoise[2], vecPoseNoise[3], vecPoseNoise[4], vecPoseNoise[5]).finished());

    // read other parameters
    node_.param<int>("minCorrespondedPoints", minCorrespondedPoints_, 20);
    node_.param<int>("ISAMIters", ISAMIters_, 2);
    node_.param<int>("ISAMRelinearizeInterval", ISAMRelinearizeInterval_, 5);
    node_.param<double>("ISAMRelinearizeThreshold", ISAMRelinearizeThreshold_, 0.1);
    node_.param<int>("ISAMRelinearizeSkip", ISAMRelinearizeSkip_, 10);

    // add calibration
    double fx = cameraMatrix(0, 0);
    double fy = cameraMatrix(1, 1);
    double cx = cameraMatrix(0, 2);
    double cy = cameraMatrix(1, 2);
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fx, fy, 0.0, cy, cy));
    K_ = K;

    // create nonlinear ISAM
    isam_ = gtsam::NonlinearISAM(ISAMRelinearizeInterval_);

    // initialize pose number
    poseNumber_ = 0;
}

ISAMOptimizer::~ISAMOptimizer()
{

}

void ISAMOptimizer::reset()
{
    std::cout << "RESETTING..." << std::endl;

    // clear points and poses vectors
    points_.clear();
    poses_.clear();

    // reset pose number
    poseNumber_ = 0;

    // clear graph and initial estimates
    //graph_.resize(0);
    //initialEstimate_.clear();
}

bool ISAMOptimizer::addData(std::vector<Eigen::Vector3f> points, std::vector<Match> matches, Eigen::Matrix4f pose)
{
    bool ret;

    //std::cout << "GRAPH IN THE BEGINNING" << std::endl;
    //graph_.print();

    // optimizer was reset - time to consider new 3D points
    if(poseNumber_ == 0)
    {
        poses_.push_back(gtsam::Pose3::identity());

        //std::cout << "ADDING NEW 3D POINTS: " << points.size() << std::endl;
        if(points.size() > 0)
        {
            for(int i=0; i<points.size(); i++)
            {
                gtsam::Point3 point3 = gtsam::Point3(points[i](0), points[i](1), points[i](2));

                // add points to points_ vector
                Point3D p(point3, matches[i]);
                points_.push_back(p);
            }
        }

        ret = true;
    }
    else
    {
        // add pose estimate
        gtsam::Rot3 R(pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2), pose(2, 0), pose(2, 1), pose(2, 2));
        gtsam::Point3 t(pose(0, 3), pose(1, 3), pose(2, 3));
        gtsam::Pose3 pRelative(R, t);
        gtsam::Pose3 p = poses_[poseNumber_ - 1] * pRelative;
        poses_.push_back(p);

        // mark all points as outdated
        markPointsAsOutdated();

        // add 2D measurements
        for(int i=0; i<matches.size(); i++)
        {
            // check if the corresponding point already exists
            int correspondingIndex = findCorrespondingPoint(matches[i]);
            if(correspondingIndex > 0)
            {
                points_[correspondingIndex].addMeasurement(matches[i].getSecondFeature());
                points_[correspondingIndex].markAsUpdated();
            }
        }

        // remove outdated points
        removeOutdatedPoints();

        // order points
        orderPoints();

        // check number of visible points
        //std::cout << "# VISIBLE POINTS: " << points_.size() << "   MIN: " << minCorrespondedPoints_ << std::endl;
        if(points_.size() < minCorrespondedPoints_)
        {
            //std::cout << "NOT ENOUGH POINTS. ADDING EVERYTHING TO GRAPH AND OPTIMIZING..." << std::endl;

            // add poses to graph
            for(int i=0; i<poses_.size(); i++)
            {
                //std::cout << "ADDING DATA FOR POSE " << i << std::endl;
                // add 2D measurements for that pose
                for(int j=0; j<points_.size(); j++)
                {
                    //std::cout << "\tADDING 2D POINT x-" << i << " l-" << j << " KEYS: pose - " << gtsam::Symbol('x', i) << " landmark: " << gtsam::Symbol('l', j) << std::endl;
                    gtsam::Point2 p2 = points_[j].getMeasurements()[i].getMeasurement();
                    graph_.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(p2, measurementNoise_, gtsam::Symbol('x', i), gtsam::Symbol('l', j), K_));
                }

                // add initial guess on poses
                //std::cout << "\tADDING POSE x-" << i << " KEY: " << gtsam::Symbol('x', i) << std::endl;
                initialEstimate_.insert(gtsam::Symbol('x', i), poses_[i]);
                
                // if first pose, add prior
                if(i == 0)
                {
                    //std::cout << "FIRST POSE: ADDING PRIORS..." << std::endl;
                    //std::cout << "\tADDING PRIOR POSE x-" << i << " KEY: " << gtsam::Symbol('x', i) << std::endl;
                    graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', i), poses_[i], poseNoise_));
                
                    if(points_.size() > 0)
                    {
                        //std::cout << "\tADDING PRIOR 3D POINT l-" << 0 << "  *  POINT: (" << points_[0].getPoint().x() << ", " << points_[0].getPoint().y() << ", " << points_[0].getPoint().z() << ")" << " * KEY: " << gtsam::Symbol('l', 0) <<std::endl;
                        graph_.push_back(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 0), points_[0].getPoint(), pointNoise_));
                    }
                    
                    // add initial guess of 3D points to graph
                    for(int j=0; j<points_.size(); j++)
                    {
                        //std::cout << "\tADDING 3D POINT l-" << j << " KEY: " << gtsam::Symbol('l', j) << std::endl;
                        initialEstimate_.insert(gtsam::Symbol('l', j), points_[j].getPoint());
                    }
                
                }

                else
                {
                    //std::cout << "NOT FIRST POSE. OPTIMIZING..." << std::endl;
                    
                    gtsam::FastSet<gtsam::Key> keys = graph_.keys();
                    //std::cout << "# KEYS: " << keys.size() << std::endl;
                    //keys.print();
                    
                    // update ISAM estimates
                    std::string gn = "MY_GRAPH";
                    std::string kf = "KEY"; 
                    //graph_.print();
                    try
                    {
                        isam_.update(graph_, initialEstimate_);
                        gtsam::Values currentEstimate = isam_.estimate();
                        std::cout << "+++++++++++++++++++++++++ OPTIMIZED!!! ++++++++++++++++++++++++++++++++" << std::endl;
                    }
                    catch (const gtsam::ValuesKeyAlreadyExists& eVKAE)
                    {   
                        std::cout << "*** VALUES KEY ALREADY EXISTS" << std::endl;
                        std::cout << "\tEXISTING KEY: " << eVKAE.key() << std::endl;
                    }
                    catch (const gtsam::IndeterminantLinearSystemException& eILS)
                    {   
                        std::cout << "*** INDETERMINANT LINEAR SYSTEM" << std::endl;
                        std::cout << "\tNEAR VALUE WITH KEY: " << eILS.nearbyVariable() << std::endl;
                    }
                    catch(const tbb::captured_exception& eCE)
                    {
                        std::cout << "*** CAPTURED EXCEPTION" << std::endl;
                        std::cout << "\tTHIS HAPPENED: " << eCE.what() << std::endl;
                    }

                    // clear graph and initial estimates
                    //std::cout << "CLEANING GRAPH..." << std::endl;
                    graph_.resize(0);
                    initialEstimate_.clear();
                }
            }

            ret = false;
        }
        else
        {
            ret = true;
        }
    }

    // update pose number
    poseNumber_++;
    return ret;
}

int ISAMOptimizer::findCorrespondingPoint(Match match)
{
    for(int i=0; i<points_.size(); i++)
    {
        if(points_[i].isCorresponding(match.getFirstFeature()))
        {
            return i;
        }
    }
    return -1;
}

void ISAMOptimizer::markPointsAsOutdated()
{
    for(int i=0; i<points_.size(); i++)
    {
        points_[i].markAsOutdated();
    }
}

void ISAMOptimizer::removeOutdatedPoints()
{
    for(int i=0; i<points_.size(); i++)
    {
        if(!points_[i].isUpdated())
        {
            points_.erase(points_.begin() + i);
            i--;
        }
    }
}

void ISAMOptimizer::orderPoints()
{
    for(int i=1; i<points_.size(); i++)
    {
        gtsam::Point2 p2_prev_i = points_[i].getMeasurements()[0].getMeasurement();
        gtsam::Point2 p2_curr_i = points_[i].getMeasurements()[1].getMeasurement();
        double distance_i = sqrt((p2_prev_i.x() - p2_curr_i.x()) * (p2_prev_i.x() - p2_curr_i.x()) + (p2_prev_i.y() - p2_curr_i.y()) * (p2_prev_i.y() - p2_curr_i.y()));

        for(int j=i+1; j<points_.size(); j++)
        {
            gtsam::Point2 p2_prev_j = points_[j].getMeasurements()[0].getMeasurement();
            gtsam::Point2 p2_curr_j = points_[j].getMeasurements()[1].getMeasurement();
            double distance_j = sqrt((p2_prev_j.x() - p2_curr_j.x()) * (p2_prev_j.x() - p2_curr_j.x()) + (p2_prev_j.y() - p2_curr_j.y()) * (p2_prev_j.y() - p2_curr_j.y()));

            if(distance_j > distance_i)
            {
                Point3D aux = points_[i];
                points_[i] = points_[j];
                points_[j] = aux;
            }
        }
    }
}