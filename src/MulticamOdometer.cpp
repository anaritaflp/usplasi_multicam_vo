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
    node_.param<int>("param_odometerMinNumberMatches", param_odometerMinNumberMatches_, 20);
    node_.param<int>("param_odometerRansacIters", param_odometerRansacIters_, 400);
    node_.param<double>("param_odometerInlierThreshold", param_odometerInlierThreshold_, 0.00001);
    node_.param<double>("param_odometerMotionThreshold", param_odometerMotionThreshold_, 5.0);
    
    // read camera parameters
    node_.getParam("param_cameraPitches", param_cameraPitches_);
    node_.getParam("param_cameraHeights", param_cameraHeights_);

    // initialize vector of absolute poses with identities
    for(int i=0; i<3*NUM_OMNI_CAMERAS; i++)
    {
        absolutePosesLocal.push_back(Eigen::Matrix4f::Identity());
        firstRow_.push_back(true);
    }
    absolutePoseGlobal = Eigen::Matrix4f::Identity();
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

        // check number of correspondences
        int N = matches[i].size();
        if(N < param_odometerMinNumberMatches_)
        {
            // too few matches to compute F
            continue;
        }
        
        // get intrinsic matrices K
        Eigen::Matrix3f KPrev(lb2_.cameraMatrices_[camNoPrev]);
        Eigen::Matrix3f KCurr(lb2_.cameraMatrices_[camNoCurr]);
        
        // normalize 2D features
        Eigen::Matrix3f NormTPrev, NormTCurr;
        std::vector<Match> matchesNorm = normalize2DPoints(matches[i], NormTPrev, NormTCurr);
        //std::vector<Match> matchesNorm = matches[i];
        //NormTPrev = Eigen::Matrix3f::Identity();
        //NormTCurr = Eigen::Matrix3f::Identity();

        Eigen::Matrix3f F, E;
        std::vector<int> inlierIndices;
		// RANSAC
        for(int j=0; j<param_odometerRansacIters_; j++)
        {
            // get random sample
            std::vector<int> chosenIndices = getRandomSample(matchesNorm.size(), 8);
            
            // compute fundamental matrix
            F = getF(matchesNorm, chosenIndices);
            
            // get inliers
            std::vector<int> inlierIndicesCurr = getInliers(matchesNorm, F);
            
            if(inlierIndicesCurr.size() > inlierIndices.size())
            {
                inlierIndices = inlierIndicesCurr;
            }
        }
        
		// check number of inliers
        if(inlierIndices.size() < param_odometerMinNumberMatches_)
        {
            continue;
        }
        
		// compute fundamental matrix out of all inliers
        F = getF(matchesNorm, inlierIndices);

        /*if(i == 0)
        {
            std::vector<Match> inlierMatches;
            std::vector<Match> outlierMatches;
            for(int j=0; j<matches[i].size(); j++)
            {
                if(elemInVec(inlierIndices, j))
                {
                    inlierMatches.push_back(matches[i][j]);
                }
                else
                {
                    outlierMatches.push_back(matches[i][j]);
                }
            }

            std::cout << "\t#INLIERS: " << inlierIndices.size() << " / " << matches[i].size() << std::endl;
            cv::Mat image(1024, 768, CV_8UC1, cv::Scalar(0));
            FeatureMatcher fm;
            cv::Mat of1 = fm.highlightOpticalFlow(image, inlierMatches, cv::Scalar(0, 255, 0));
            cv::Mat of2 = fm.highlightOpticalFlow(of1, outlierMatches, cv::Scalar(0, 0, 255));
            cv::namedWindow("optical flow", CV_WINDOW_AUTOSIZE);
            cv::imshow("optical flow", of2);
            cv::waitKey(10);
        }*/
       
		// denormalize F
        F = NormTCurr.transpose() * F * NormTPrev;
		
		// compute essential matrix E
        E = F2E(F, KPrev, KCurr);

		// get rotation and translation and triangulate points
        Eigen::Matrix<float, 4, Eigen::Dynamic> points3D;
        E2Rt(E, KPrev, KCurr, matches[i], R[i], t[i], points3D);

        // normalize 3D points (force last coordinate to 0)
        std::vector<Eigen::Vector4f> valid3DPoints;
        for(int j=0; j<points3D.cols(); j++)
        {
            Eigen::Vector4f pt = points3D.block<4, 1>(0, j);
            pt = pt / pt(3);
            if(pt(2) > 0)
            {
                valid3DPoints.push_back(pt);
            }
        }

        // check number of valid points
        if(valid3DPoints.size() < param_odometerMinNumberMatches_)
        {
            continue;
        }

        // discard far-away points (i.e., that are more distant than the median)
        double median;
        std::vector<Eigen::Vector4f> close3DPoints = getClosePoints(valid3DPoints, median);
        
        // if big median (small motion), return error
        if(median > param_odometerMotionThreshold_)
        {
            continue;
        }
        
        success[i] = true;
        
		// adjust translation scale
        //double scale = getTranslationScale(close3DPoints, median, 0.0, 1.65);
        //t[i] = t[i] * scale;
    }

	// find out which R,t gets the most inliers from ALL matches
    int maxInliers = 0;
    bestCamera = 0;
    Eigen::Matrix4f bestPose = Eigen::Matrix4f::Identity();
    for(int i=0; i<matches.size(); i++)
    {
		// if R,t were successfully obtained...
        if(success[i])
        {
            // get camNoPrev and camNoCurr for that i            
            int camNoPrev_i, camNoCurr_i;
            getPrevAndCurrCamIndex(lb2_, i, camNoPrev_i, camNoCurr_i);  
            
            // transform to global (ladybug) coordinates
            Eigen::Matrix4f T;
            T << R[i](0, 0), R[i](0, 1), R[i](0, 2), t[i](0), R[i](1, 0), R[i](1, 1), R[i](1, 2), t[i](1), R[i](2, 0), R[i](2, 1), R[i](2, 2), t[i](2), 0.0, 0.0, 0.0, 1.0;
            Eigen::Matrix4f TGlobal = lb2_.extrinsics_[camNoPrev_i] * T.inverse() * lb2_.extrinsics_[camNoCurr_i].inverse();
            Eigen::Matrix3f RGlobal;
            Eigen::Vector3f tGlobal;
            RGlobal << TGlobal(0, 0), TGlobal(0, 1), TGlobal(0, 2), TGlobal(1, 0), TGlobal(1, 1), TGlobal(1, 2), TGlobal(2, 0), TGlobal(2, 1), TGlobal(2, 2);
            tGlobal << TGlobal(0, 3), TGlobal(1, 3), TGlobal(2, 3);

            // integrate to absolute pose
            absolutePosesLocal[i] = absolutePosesLocal[i] * TGlobal;

            // plot inidvidual camera estimates (for debugging)
            if(firstRow_[i])
            {
                firstRow_[i] = false;
            }
            else
            {
                (*files_[i]) << ";" << std::endl;
            }

            Eigen::Matrix4f p(absolutePosesLocal[i]);                
            (*files_[i]) << "\t" << p(0, 0) << ",\t" << p(0, 1) << ",\t" << p(0, 2) << ",\t" << p(0, 3) << ",\t"
                                 << p(1, 0) << ",\t" << p(1, 1) << ",\t" << p(1, 2) << ",\t" << p(1, 3) << ",\t"
                                 << p(2, 0) << ",\t" << p(2, 1) << ",\t" << p(2, 2) << ",\t" << p(2, 3) << ",\t"
                                 << p(3, 0) << ",\t" << p(3, 1) << ",\t" << p(3, 2) << ",\t" << p(3, 3);
            (*files_[i]).flush();

            // count number of inliers from all sets of matches (i.e., all intra- and consecutive inter-camera matches)
            int sumInliers = 0;
			for(int j=0; j<matches.size(); j++)
            {
                // get camNoPrev and camNoCurr for that j
                int camNoPrev_j, camNoCurr_j;
                getPrevAndCurrCamIndex(lb2_, j, camNoPrev_j, camNoCurr_j); 

                // transform to camera j coordinates
                Eigen::Matrix4f TLocalInv = lb2_.extrinsics_[camNoPrev_j].inverse() * TGlobal * lb2_.extrinsics_[camNoCurr_j];

                Eigen::Matrix4f TLocal = TLocalInv.inverse();
                Eigen::Matrix3f RLocal;
                Eigen::Vector3f tLocal;
                RLocal << TLocal(0, 0), TLocal(0, 1), TLocal(0, 2), TLocal(1, 0), TLocal(1, 1), TLocal(1, 2), TLocal(2, 0), TLocal(2, 1), TLocal(2, 2);
                tLocal << TLocal(0, 3), TLocal(1, 3), TLocal(2, 3);

                // compute F for the camera combination j with the (R,t) obtained with the camera combination i
                Eigen::Matrix3f Fj = Rt2F(RLocal, tLocal, lb2_.cameraMatrices_[camNoPrev_j], lb2_.cameraMatrices_[camNoCurr_j]);

                // count inliers
                std::vector<int> inlierIndices = getInliers(matches[j], Fj);
                sumInliers += inlierIndices.size();  
            }
            if(sumInliers > maxInliers)
            {
                maxInliers = sumInliers;
                bestCamera = i;
                bestPose = TGlobal;
            }
        }
    }
    absolutePoseGlobal = absolutePoseGlobal * bestPose;
    return absolutePoseGlobal;
}

/** Normalize 2D feature points
 * @param std::vector<Match> vector with matches
 * @param Eigen::Matrix3f& output normalization matrix of features in the previous frame
 * @param Eigen::Matrix3f& output normalization matrix of features in the current frame
 * @return std::vector<Match> vector with normalized matches */
std::vector<Match> MulticamOdometer::normalize2DPoints(std::vector<Match> matches, Eigen::Matrix3f &NormTPrev, Eigen::Matrix3f &NormTCurr)
{
    std::vector<cv::KeyPoint> keypointsPrev, keypointsCurr;
    for(int i=0; i<matches.size(); i++)
    {
        cv::KeyPoint kpPrev = matches[i].getFirstFeature().getKeypoint();
        cv::KeyPoint kpCurr = matches[i].getSecondFeature().getKeypoint();
        keypointsPrev.push_back(kpPrev);
        keypointsCurr.push_back(kpCurr);
    }
    
	// compute centroids of all features from the previous and current frames separately
    cv::Point2f sumPrev = cv::Point2f(0.0, 0.0);
    cv::Point2f sumCurr = cv::Point2f(0.0, 0.0);
    for(int i=0; i<matches.size(); i++)
    {
        sumPrev += keypointsPrev[i].pt;        
        sumCurr += keypointsCurr[i].pt;
    }
    sumPrev.x /= matches.size();
    sumPrev.y /= matches.size();
    sumCurr.x /= matches.size();
    sumCurr.y /= matches.size();

	// subtract feature points by the corresponding centroid
    for(int i=0; i<matches.size(); i++)
    {
        keypointsPrev[i].pt = keypointsPrev[i].pt - sumPrev;
        keypointsCurr[i].pt = keypointsCurr[i].pt - sumCurr;            
    }

	// multiplicate points by a scale factor, so that the average distance to the origin is sqrt(2)
    double scalePrev = 0.0;
    double scaleCurr = 0.0;    
    for(int i=0; i<matches.size(); i++)
    {
        cv::Point2f pt1 = keypointsPrev[i].pt;
        cv::Point2f pt2 = keypointsCurr[i].pt;
        scalePrev += sqrt(pt1.x * pt1.x + pt1.y * pt1.y);
        scaleCurr += sqrt(pt2.x * pt2.x + pt2.y * pt2.y);
    }
    
    scalePrev = sqrt(2) * matches.size() / scalePrev;
    scaleCurr = sqrt(2) * matches.size() / scaleCurr;

    for(int i=0; i<matches.size(); i++)
    {
        cv::Point2f pt1 = keypointsPrev[i].pt;
        cv::Point2f pt2 = keypointsCurr[i].pt;
        keypointsPrev[i].pt = scalePrev * pt1;
        keypointsCurr[i].pt = scaleCurr * pt2;
        
        Feature f1(matches[i].getFirstFeature());
        f1.setKeypoint(keypointsPrev[i]);
        
        Feature f2(matches[i].getSecondFeature());
        f2.setKeypoint(keypointsCurr[i]);
        
        Match m(f1, f2, matches[i].getDistance());
        m.setColor(matches[i].getColor());
        matches[i] = m;
    }

	// build normalization matrices of the previous and current frames 
    NormTPrev << scalePrev, 0.0, -scalePrev*sumPrev.x, 0.0, scalePrev, -scalePrev*sumPrev.y, 0.0, 0.0, 1.0;
    NormTCurr << scaleCurr, 0.0, -scaleCurr*sumCurr.x, 0.0, scaleCurr, -scaleCurr*sumCurr.y, 0.0, 0.0, 1.0;
    
    return matches;
}

/** Compute fundamental matrix out of eight feature correspondences between the previous and current frame.
 * @param std::vector<Match> vector with feature matches
 * @param std::vector<int> vector with eight indices of the vector with matches
 * @return Eigen::Matrix3f fundamental matrix */
Eigen::Matrix3f MulticamOdometer::getF(std::vector<Match> matches, std::vector<int> indices)
{
    int N = indices.size();

	// see Trucco & Verri, Introductory Techniques for 3D Computer Vision, chapter 6
    Eigen::Matrix<float, Eigen::Dynamic, 9> A;
    A.resize(N, 9);

    for(int i=0; i<N; i++)
    {
        cv::Point2f pPrev = matches[indices[i]].getFirstFeature().getKeypoint().pt;
        cv::Point2f pCurr = matches[indices[i]].getSecondFeature().getKeypoint().pt;
        
        A(i, 0) = pCurr.x * pPrev.x;
        A(i, 1) = pCurr.x * pPrev.y;
        A(i, 2) = pCurr.x;
        A(i, 3) = pCurr.y * pPrev.x;
        A(i, 4) = pCurr.y * pPrev.y;
        A(i, 5) = pCurr.y;
        A(i, 6) = pPrev.x;
        A(i, 7) = pPrev.y;
        A(i, 8) = 1;
    }
    
    Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, 9>> svdA(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3f F;
    F << svdA.matrixV()(0, 8), svdA.matrixV()(1, 8), svdA.matrixV()(2, 8), svdA.matrixV()(3, 8), svdA.matrixV()(4, 8), svdA.matrixV()(5, 8), svdA.matrixV()(6, 8), svdA.matrixV()(7, 8), svdA.matrixV()(8, 8);
    
	// re-enforce rank 2 constraint on fundamental matrix
    Eigen::JacobiSVD<Eigen::Matrix3f> svdF(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f D(Eigen::Matrix3f::Zero());
    D(0, 0) = svdF.singularValues()(0);
    D(1, 1) = svdF.singularValues()(1);
    
    F = (svdF.matrixU()) * D * (svdF.matrixV()).transpose();
    
    return F;
}

/** Get the inliers among all matches that comply with a given fundamental matrix.
 * @param std::vector<Match> vector with feature matches
 * @param Eigen::Matrix3f fundamental matrix
 * @return std::vector<int> vector with indices of the inliers */
std::vector<int> MulticamOdometer::getInliers(std::vector<Match> matches, Eigen::Matrix3f F)
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

        if(dSampson < param_odometerInlierThreshold_)
        {
            inlierIndices.push_back(i);
        }
    }
    return inlierIndices;
}

/** Compute essential matrix out of a given fundamental matrix.
 * @param Eigen::Matrix3f fundamental matrix
 * @param Eigen::Matrix3f intrinsic matrix of the camera of the previous frame
 * @param Eigen::Matrix3f intrinsic matrix of the camera of the current frame
 * @return Eigen::Matrix3f essential matrix */
Eigen::Matrix3f MulticamOdometer::F2E(Eigen::Matrix3f F, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr)
{
	// multiply fundamental matrix by intrinsic matrices to obtain essential matrix
    Eigen::Matrix3f E = KCurr.transpose() * F * KPrev;

	// re-enforce rank 2 constraint on essential matrix
    Eigen::JacobiSVD<Eigen::Matrix3f> svdE(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f D(Eigen::Matrix3f::Zero());
    D(0, 0) = 1.0;
    D(1, 1) = 1.0;
    
    E = (svdE.matrixU()) * D * (svdE.matrixV()).transpose();
    return E;
}

/** Extract rotation (R) and translation (t) from a given essential matrix and triangulate feature matches.
 * @param Eigen::Matrix3f essential matrix
 * @param Eigen::Matrix3f intrinsic matrix of the camera of the previous frame
 * @param Eigen::Matrix3f intrinsic matrix of the camera of the current frame
 * @param std::vector<Match> vector with matches
 * @param Eigen::Matrix3f output rotation matrix
 * @param Eigen::Vector3f output translation vector
 * @param Eigen::Matrix<float, 4, Eigen::Dynamic> output matrix with computed 3D points
 * @return void */
void MulticamOdometer::E2Rt(Eigen::Matrix3f E, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, std::vector<Match> matches, Eigen::Matrix3f &R, Eigen::Vector3f &t, Eigen::Matrix<float, 4, Eigen::Dynamic> &points3D)
{
    Eigen::Matrix3f W, Z;
    W << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Z << 0, 1, 0, -1, 0, 0, 0, 0, 0;
    
	// singular values decomposition of E
    Eigen::JacobiSVD<Eigen::Matrix3f> svdE(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
	// get skew symmetric matrix of translation vector
    Eigen::Matrix3f S = svdE.matrixU() * Z * (svdE.matrixU()).transpose();
	
	// get possible rotation matrices
    Eigen::Matrix3f Ra = svdE.matrixU() * W * (svdE.matrixV()).transpose();
    Eigen::Matrix3f Rb = svdE.matrixU() * W.transpose() * (svdE.matrixV()).transpose();

	// get translation vetor from skew symmetric matrix
    t << S(2, 1), S(0, 2), S(1, 0);
    
	// correct signal of both rotation matrices
    if(Ra.determinant() < 0)
    {
        Ra = -Ra;
    }
    if(Rb.determinant() < 0)
    {
        Rb = -Rb;
    }
    
	// 4 possible combinations of R,t
    std::vector<Eigen::Matrix3f> solutionsR;
    std::vector<Eigen::Vector3f> solutionst;    
    solutionsR.push_back(Ra); solutionst.push_back(t);
    solutionsR.push_back(Ra); solutionst.push_back(-t);
    solutionsR.push_back(Rb); solutionst.push_back(t);
    solutionsR.push_back(Rb); solutionst.push_back(-t);
    
	// test Chirality constraint for all 4 solutions
    Eigen::Matrix<float, 4, Eigen::Dynamic> points3DCurr;
    int maxInliers = 0;
    for(int i=0; i<4; i++)
    {
        int nInliers = triangulate(matches, KPrev, KCurr, solutionsR[i], solutionst[i], points3DCurr);

		// solution with the most inliers wins
		if(nInliers > maxInliers)
        {
            maxInliers = nInliers;
            points3D = points3DCurr;
            R = solutionsR[i];
            t = solutionst[i];
        }
    }
}

/** Triangulate 3D points.
 * @param std::vector<Match> vector with matches
 * @param Eigen::Matrix3f intrinsic matrix of the camera of the previous frame
 * @param Eigen::Matrix3f intrinsic matrix of the camera of the current frame
 * @param Eigen::Matrix3f rotation matrix
 * @param Eigen::Vector3f translation vector
 * @param Eigen::Matrix<float, 4, Eigen::Dynamic> output matrix with computed 3D points
 * @return int number of inliers, i.e., points that satisfy the Chirality constraint */
int MulticamOdometer::triangulate(std::vector<Match> matches, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, Eigen::Matrix3f R, Eigen::Vector3f t, Eigen::Matrix<float, 4, Eigen::Dynamic> &points3D)
{
    points3D.resize(4, matches.size());
    
	// get projection matrices 
	//    previous camera has identity rotation and translation
	//    current camera has rotation R and translation t
    Eigen::Matrix<float, 3, 4> projMatrixPrev = getProjectionMatrix(KPrev, Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());        
    Eigen::Matrix<float, 3, 4> projMatrixCurr = getProjectionMatrix(KCurr, R, t);
    
	// triangulation by orthogonal regression
    Eigen::Matrix4f J;
    for(int i=0; i<matches.size(); i++)
    {
        cv::Point2f pPrev = matches[i].getFirstFeature().getKeypoint().pt;
        cv::Point2f pCurr = matches[i].getSecondFeature().getKeypoint().pt;
        
        for(int j=0; j<4; j++)
        {
            J(0, j) = projMatrixPrev(2, j) * pPrev.x - projMatrixPrev(0, j);
            J(1, j) = projMatrixPrev(2, j) * pPrev.y - projMatrixPrev(1, j);
            J(2, j) = projMatrixCurr(2, j) * pCurr.x - projMatrixCurr(0, j);
            J(3, j) = projMatrixCurr(2, j) * pCurr.y - projMatrixCurr(1, j);
        }

        Eigen::JacobiSVD<Eigen::Matrix4f> svdJ(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        points3D(0, i) = svdJ.matrixV()(0, 3);
        points3D(1, i) = svdJ.matrixV()(1, 3);
        points3D(2, i) = svdJ.matrixV()(2, 3);
        points3D(3, i) = svdJ.matrixV()(3, 3);
    }
    
	// get reprojection points
    Eigen::Matrix<float, 3, Eigen::Dynamic> p2DPrev, p2DCurr;
    p2DPrev.resize(3, matches.size());
    p2DCurr.resize(3, matches.size());
    p2DPrev = projMatrixPrev * points3D;
    p2DCurr = projMatrixCurr * points3D;

	// test Chirality constraint and count number of inliers
    int inlierCounter = 0;
    for(int i=0; i<matches.size(); i++)
    {
        if(p2DPrev(2, i) * points3D(3, i) > 0 && p2DCurr(2, i) * points3D(3, i) > 0)
        {
            inlierCounter++;
        }
    }
    
    return inlierCounter;
}

/** Get projection matrix out of an intrinsic matrix, a rotation matrix and a translation vector.
 * @param Eigen::Matrix3f intrinsic matrix 
 * @param Eigen::Matrix3f rotation matrix
 * @param Eigen::Vector3f translation vector
 * @return Eigen::Matrix<float, 3, 4> projection matrix */
Eigen::Matrix<float, 3, 4> MulticamOdometer::getProjectionMatrix(Eigen::Matrix3f K, Eigen::Matrix3f R, Eigen::Vector3f t)
{
    Eigen::Matrix<float, 3, 4> P;

    P << R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2), t(1), R(2, 0), R(2, 1), R(2, 2), t(2);
    
    P = K * P;
    
    return P;
}

/** Get points that are closer to the camera than the median distanced point.
 * @param std::vector<Eigen::Vector4f> vector with all 3D points 
 * @param double output median distance of all points
 * @return std::vector<Eigen::Vector4f> vector with close points only */
std::vector<Eigen::Vector4f> MulticamOdometer::getClosePoints(std::vector<Eigen::Vector4f> points, double &median)
{
	// compute distance of all points from the camera's optical center
    std::vector<double> distances;
    std::vector<int> indexes;
    for(int i=0; i<points.size(); i++)
    {
        double d = fabs(points[i](0)) + fabs(points[i](1)) + fabs(points[i](2));
        distances.push_back(d);
        indexes.push_back(i);
    }
    
	// sort distances and indexes vectors by the points' distances
    for(int i=0; i<distances.size(); i++)
    {
        for(int j=i+1; j<distances.size(); j++)
        {
            if(distances[j] < distances[i])
            {
                double dAux = distances[i];
                distances[i] = distances[j];
                distances[j] = dAux;
                
                int iAux = indexes[i];
                indexes[i] = indexes[j];
                indexes[j] = iAux;
            }
        }
    }
    
	// get median distance
    median = distances[distances.size()/2];
    
	// return points with a distance below median
    std::vector<Eigen::Vector4f> close3DPoints;
    for(int i=0; i<distances.size()/2 + 1; i++)
    {
        close3DPoints.push_back(points[indexes[i]]);
    }
    
    return close3DPoints;    
}

/** Compute optimal translation scale.
 * @param std::vector<Eigen::Vector4f> vector with 3D points 
 * @param double median distance of all points
 * @param camera pitch angle
 * @param camera height
 * @return double optimal scale */
double MulticamOdometer::getTranslationScale(std::vector<Eigen::Vector4f> points3D, double median, double pitch, double height)
{
    double sigma = median/50.0;
    double weight = 1.0/(2.0*sigma*sigma);

	// ds stores the height of all points from the ground (?)
    std::vector<Eigen::Vector2f> pointsPlane;
    std::vector<double> ds;
    Eigen::Vector2f inclination;
    inclination(0) = cos(-pitch);
    inclination(1) = sin(-pitch);
    for(int i=0; i<points3D.size(); i++)
    {
        double d = inclination.transpose() * points3D[i].segment<2>(1);
        ds.push_back(d);
    }
    
    int bestIndex = 0;
    double maxSum = 0.0;
    
    for(int i=0; i<points3D.size(); i++)
    {
        if(ds[i] > median / param_odometerMotionThreshold_)
        {
            double sum = 0.0;
            for(int j=0; j<points3D.size(); j++)
            {
                double dist = ds[j] - ds[i];
                sum += exp(-dist*dist*weight);
            }
            if(sum > maxSum)
            {
                maxSum = sum;
                bestIndex = i;
            }
        }        
    }
    
    double scale = height / ds[bestIndex];
    return scale;
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
