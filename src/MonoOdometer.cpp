#include <multicam_vo/MonoOdometer.h>

/** Default constructor. */
MonoOdometer::MonoOdometer() : node_("~")
{
    // read odometer parameters
    node_.param<int>("param_odometerMinNumberMatches", param_odometerMinNumberMatches_, 20);
    node_.param<int>("param_odometerRansacIters", param_odometerRansacIters_, 400);
    node_.param<double>("param_odometerInlierThreshold", param_odometerInlierThreshold_, 0.00001);
    node_.param<double>("param_odometerMotionThreshold", param_odometerMotionThreshold_, 5.0);
    
    // read camera parameters
    node_.getParam("param_cameraPitches", param_cameraPitches_);
    node_.getParam("param_cameraHeights", param_cameraHeights_);
}

/** Destructor. */
MonoOdometer::~MonoOdometer()
{

}

/** Estimate monocular visual odometry.
 * @param std::vector<Match> vector with matches
 * @param Eigen::Matrix3f matrix with intrinsic parameters of previous camera
 * @param Eigen::Matrix3f matrix with intrinsic parameters of current camera
 * @param Eigen::Matrix3f& (output) estimated rotation matrix
 * @param Eigen::Vector3f& (output) estimated translation vector
 * @param bool show optical flow (true), don't show otherwise
 * @param std::vector<Match> output vector with all inlier matches
 * @param std::vector<Eigen::Vector3f> output vector with 3D points, triangulated from all inlier matches
 * @return bool true is motion successfully estimated, false otherwise */
bool MonoOdometer::estimateMotion(std::vector<Match> matches, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, Eigen::Matrix3f &R, Eigen::Vector3f &t, bool showOpticalFlow, std::vector<Match> &inlierMatches, std::vector<Eigen::Vector3f> &points3D)
{
    // check number of correspondences
    int N = matches.size();
    if(N < param_odometerMinNumberMatches_)
    {
        // too few matches to compute F
        R = Eigen::Matrix3f::Identity();
        t << 0.0, 0.0, 0.0;
        return false;
    }

    // normalize 2D features
    Eigen::Matrix3f NormTPrev, NormTCurr;
    std::vector<Match> matchesNorm = normalize2DPoints(matches, NormTPrev, NormTCurr);

    Eigen::Matrix3f F, E;
    std::vector<int> inlierIndices;
		
    // RANSAC loop
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
        R = Eigen::Matrix3f::Identity();
        t << 0.0, 0.0, 0.0;
        return false;
    }

    // compute fundamental matrix out of all inliers
    F = getF(matchesNorm, inlierIndices);

    // save inlier and outlier matches
    std::vector<Match> outlierMatches;
    for(int i=0; i<matches.size(); i++)
    {
        if(elemInVec(inlierIndices, i))
        {
            inlierMatches.push_back(matches[i]);
        }
        else
        {
            outlierMatches.push_back(matches[i]);
        }
    }

    // plot optical flow and print #inliers (for debugging)
    if(showOpticalFlow)
    {
        cv::Mat image(1024, 768, CV_8UC1, cv::Scalar(0));
        FeatureMatcher fm;
        cv::Mat of1 = fm.highlightOpticalFlow(image, inlierMatches, cv::Scalar(0, 255, 0));
        cv::Mat of2 = fm.highlightOpticalFlow(of1, outlierMatches, cv::Scalar(0, 0, 255));
        cv::namedWindow("Optical flow", CV_WINDOW_AUTOSIZE);
        cv::imshow("Optical flow", of2);
        cv::waitKey(10);
    }
    
    // denormalize F
    F = NormTCurr.transpose() * F * NormTPrev;
    
    // compute essential matrix E
    E = F2E(F, KPrev, KCurr);

    // get rotation and translation and triangulate points
    Eigen::Matrix<float, 4, Eigen::Dynamic> points3DMat;
    E2Rt(E, KPrev, KCurr, inlierMatches, R, t, points3DMat);

    // normalize 3D points (force last coordinate to 0)
    for(int j=0; j<points3DMat.cols(); j++)
    {
        Eigen::Vector3f pt = points3DMat.block<3, 1>(0, j);
        double lastCoord = points3DMat(3, j);
        pt = pt / lastCoord;
        if(pt(2) > 0)
        {
            points3D.push_back(pt);
        }
        else
        {
            // remove match if not a valid point
            inlierMatches.erase(inlierMatches.begin() + j);
        }
    }

    // check number of valid points
    if(points3D.size() < param_odometerMinNumberMatches_)
    {
        R = Eigen::Matrix3f::Identity();
        t << 0.0, 0.0, 0.0;
        return false;
    }
    
    return true;
}

/** Normalize 2D feature points
 * @param std::vector<Match> vector with matches
 * @param Eigen::Matrix3f& output normalization matrix of features in the previous frame
 * @param Eigen::Matrix3f& output normalization matrix of features in the current frame
 * @return std::vector<Match> vector with normalized matches */
std::vector<Match> MonoOdometer::normalize2DPoints(std::vector<Match> matches, Eigen::Matrix3f &NormTPrev, Eigen::Matrix3f &NormTCurr)
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
Eigen::Matrix3f MonoOdometer::getF(std::vector<Match> matches, std::vector<int> indices)
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
std::vector<int> MonoOdometer::getInliers(std::vector<Match> matches, Eigen::Matrix3f F)
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
Eigen::Matrix3f MonoOdometer::F2E(Eigen::Matrix3f F, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr)
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
void MonoOdometer::E2Rt(Eigen::Matrix3f E, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, std::vector<Match> matches, Eigen::Matrix3f &R, Eigen::Vector3f &t, Eigen::Matrix<float, 4, Eigen::Dynamic> &points3D)
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
int MonoOdometer::triangulate(std::vector<Match> matches, Eigen::Matrix3f KPrev, Eigen::Matrix3f KCurr, Eigen::Matrix3f R, Eigen::Vector3f t, Eigen::Matrix<float, 4, Eigen::Dynamic> &points3D)
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
Eigen::Matrix<float, 3, 4> MonoOdometer::getProjectionMatrix(Eigen::Matrix3f K, Eigen::Matrix3f R, Eigen::Vector3f t)
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
std::vector<Eigen::Vector4f> MonoOdometer::getClosePoints(std::vector<Eigen::Vector4f> points, double &median)
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
double MonoOdometer::getTranslationScale(std::vector<Eigen::Vector4f> points3D, double median, double pitch, double height)
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
