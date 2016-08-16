#include <multicam_vo/MulticamOdometer.h>

/** MulticamOdometer default constructor. */
MulticamOdometer::MulticamOdometer()
{
    
}

/** MulticamOdometer constructor.
 * @param Ladybug2 ladybug object
 * @param std::vector<std::ofstream*> vector with files to write estimated poses
 * @return MulticamOdometer object */
MulticamOdometer::MulticamOdometer(Ladybug2 lb2, std::vector<std::ofstream*> files) : node_("~")
{
    files_ = files;
    lb2_ = lb2;
    
	// read odometer parameters
    node_.param<double>("param_odometerInlierThreshold", param_odometerInlierThreshold_, 0.00001);

    // initialize absolute poses with identities
    for(int i=0; i<3*NUM_OMNI_CAMERAS; i++)
    {
        absolutePosesLocal_.push_back(Eigen::Matrix4f::Identity());
        firstRow_.push_back(true);
    }
    absolutePoseGlobal_ = Eigen::Matrix4f::Identity();
}

/** MulticamOdometer destructor. */
MulticamOdometer::~MulticamOdometer()
{

}

/** Estimate the motion of the multi-camera system.
 * @param std::vector<std::vector<Match>> a vector with each camera's matches
 * @param vector of output matlab files to be filled with the estimated poses
 * @param int& output index of the camera with the most successful motion estimation
 * @return Eigen::Matrix4f transformation with the relative motion of the multi-camera system */
Eigen::Matrix4f MulticamOdometer::estimateMotion(std::vector<std::vector<Match>> matches, int &bestCamera)
{
    // rotation and translation estimates obtained with the correspondences between each combination of cameras
    std::vector<Eigen::Matrix3f> R;
    std::vector<Eigen::Vector3f> t;
	
	// flags indicating whether R,t was successfully obtained or not
	std::vector<bool> success(3*NUM_OMNI_CAMERAS, false);
    
    // vectors with the rotation matrices / translation vectors generated by all intra- and consecutive inter-camera matches
	R.resize(3 * NUM_OMNI_CAMERAS);
    t.resize(3 * NUM_OMNI_CAMERAS);

	// compute rotation and translation for each set of intra- and consecutive inter-camera matches
    MonoOdometer monoOdometer;
    for(int i=0; i<3*NUM_OMNI_CAMERAS; i++)
    {
		// get camera indexes of the cameras of the current and previous frames
        int camNoPrev = i/3;
        int camNoCurr;
        if(i%3 == 1)
        {
            int aux;
            lb2_.getLeftRightCameras(camNoPrev, aux, camNoCurr);
        }
        else if(i%3 == 2)
        {
            int aux;
            lb2_.getLeftRightCameras(camNoPrev, camNoCurr, aux);
        }
        else
        {
            camNoCurr = camNoPrev;
        }

        // get intrinsic matrices K
        Eigen::Matrix3f KPrev(lb2_.cameraMatrices_[camNoPrev]);
        Eigen::Matrix3f KCurr(lb2_.cameraMatrices_[camNoCurr]);

        // estimate monocular visual odometry
        success[i] = monoOdometer.estimateMotion(matches[i], KPrev, KCurr, R[i], t[i], false);
    }

	// let ALL matches vote for the best R,t estimated in the previous mono visual odometry step
    int maxInliers = 0;
    bestCamera = 0;
    Eigen::Matrix4f bestPose = Eigen::Matrix4f::Identity();
    for(int i=0; i<matches.size(); i++)
    {
        // get camNoPrev and camNoCurr for camera i            
        int camNoPrev_i, camNoCurr_i;
        getPrevAndCurrCamIndex(lb2_, i, camNoPrev_i, camNoCurr_i);  
        
        // transform motion estimation to global (ladybug) coordinates
        Eigen::Matrix4f T = Rt2T(R[i], t[i]);
        Eigen::Matrix4f TGlobal = lb2_.cam2LadybugRef(T, camNoPrev_i, camNoCurr_i);

        // integrate to absolute mono VO pose estimated by camera i
        absolutePosesLocal_[i] = absolutePosesLocal_[i] * TGlobal;

        // write individual camera estimates in matlab file for plotting (for debugging)
        if(firstRow_[i])
        {
            firstRow_[i] = false;
        }
        else
        {
            (*files_[i]) << ";" << std::endl;
        }
        Eigen::Matrix4f p(absolutePosesLocal_[i]);                
        (*files_[i]) << "\t" << p(0, 0) << ",\t" << p(0, 1) << ",\t" << p(0, 2) << ",\t" << p(0, 3) << ",\t"
                                << p(1, 0) << ",\t" << p(1, 1) << ",\t" << p(1, 2) << ",\t" << p(1, 3) << ",\t"
                                << p(2, 0) << ",\t" << p(2, 1) << ",\t" << p(2, 2) << ",\t" << p(2, 3) << ",\t"
                                << p(3, 0) << ",\t" << p(3, 1) << ",\t" << p(3, 2) << ",\t" << p(3, 3);
        (*files_[i]).flush();

        // if R,t were successfully obtained in camera i...
        if(success[i])
        {
            // vote for motion estimation by camera i
            int sumInliers = 0;

            // test motion estimation by camera i in each camera j
            //std::cout << "#Inliers cam " << i; std::cout.flush();
			for(int j=0; j<matches.size(); j++)
            {
                // get camNoPrev and camNoCurr for camera j
                int camNoPrev_j, camNoCurr_j;
                getPrevAndCurrCamIndex(lb2_, j, camNoPrev_j, camNoCurr_j); 

                // transform motion i to camera j coordinates
                Eigen::Matrix4f TLocal = lb2_.Ladybug2CamRef(TGlobal, camNoPrev_j, camNoCurr_j);
                
                // compute F for camera j with the (R,t) obtained with camera i
                Eigen::Matrix3f RLocal;
                Eigen::Vector3f tLocal;
                T2Rt(TLocal, RLocal, tLocal);
                Eigen::Matrix3f Fj = Rt2F(RLocal, tLocal, lb2_.cameraMatrices_[camNoPrev_j], lb2_.cameraMatrices_[camNoCurr_j]);

                // count inliers of camera j for motion estimation of camera i
                std::vector<int> inlierIndices = getInliers(matches[j], Fj, param_odometerInlierThreshold_);
                sumInliers += inlierIndices.size();  

                //std::cout << "\t" << inlierIndices.size(); std::cout.flush();
            }
            //std::cout << std::endl;
            // the camera i with the most inliers is the best camera
            if(sumInliers > maxInliers)
            {
                maxInliers = sumInliers;
                bestCamera = i;
                bestPose = TGlobal;
            }
        }
    }

    // concatenate motion estimation of the best camera to absolute pose and return result
    absolutePoseGlobal_ = absolutePoseGlobal_ * bestPose;
    return absolutePoseGlobal_;
}

/** Get the inliers among all matches that comply with a given fundamental matrix.
 * @param std::vector<Match> vector with feature matches
 * @param Eigen::Matrix3f fundamental matrix
 * @param double inlier threshold
 * @return std::vector<int> vector with indices of the inliers */
std::vector<int> MulticamOdometer::getInliers(std::vector<Match> matches, Eigen::Matrix3f F, double threshold)
{
    std::vector<int> inlierIndices;
    
    for(int i=0; i<matches.size(); i++)
    {
        cv::Point2f pPrev = matches[i].getFirstFeature().getKeypoint().pt;
        cv::Point2f pCurr = matches[i].getSecondFeature().getKeypoint().pt;
        Eigen::Vector3f pPrevHomog, pCurrHomog;
        pPrevHomog << pPrev.x, pPrev.y, 1.0;
        pCurrHomog << pCurr.x, pCurr.y, 1.0;

		// xCurr^T * F * xPrev 
        double x2tFx1 = pCurrHomog.transpose() * F * pPrevHomog;

		// F * xPrev
        Eigen::Vector3f Fx1 = F * pPrevHomog;
		
		// F^T * xCurr
        Eigen::Vector3f Ftx2 = F.transpose() * pCurrHomog;
        
		// compute Sampson distance (distance to epipolar line)
        double dSampson = (x2tFx1 * x2tFx1) / ((Fx1(0)*Fx1(0)) + (Fx1(1)*Fx1(1)) + (Ftx2(0)*Ftx2(0)) + (Ftx2(1)*Ftx2(1)));

        if(dSampson < threshold)
        {
            inlierIndices.push_back(i);
        }
    }
    return inlierIndices;
}

/** Compute fundamental martix out of rotation and translation.
 * @param Eigen::Matrix3f rotation matrix
 * @param Eigen::Vector3f translation vector
 * @param Eigen::Matrix3f intrinsics matrix of the previous camera
 * @param Eigen::Matrix3f intrinsics matrix of the current camera
 * @return Eigen::Matrix3f fundamental matrix */
Eigen::Matrix3f MulticamOdometer::Rt2F(Eigen::Matrix3f R, Eigen::Vector3f t, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr)
{
    // get skew symmetric matrix of translation vector
    Eigen::Matrix3f S;
    S << 0.0, -t(2), t(1), t(2), 0.0, -t(0), -t(1), t(0), 0.0;

    // testar R*S
    Eigen::Matrix3f E = S * R;

    Eigen::Matrix3f F = (KCurr.transpose()).inverse() * E * KPrev.inverse();

    // re-enforce rank 2 constraint on fundamental matrix
    Eigen::JacobiSVD<Eigen::Matrix3f> svdF(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f D(Eigen::Matrix3f::Zero());
    D(0, 0) = svdF.singularValues()(0);
    D(1, 1) = svdF.singularValues()(1);
    
    F = (svdF.matrixU()) * D * (svdF.matrixV()).transpose();
    
    return F;
}

