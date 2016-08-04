#include <multicam_vo/utils.h>


/** Get previous and current camera associated to a given matches vector position, assuming that the vector is organized this way:
 * position i   -> matches where the previous camera is cam i and the current camera is cam i (intra-camera matches)
 * position i+1 -> matches where the previous camera is cam i and the current camera is the cam right to i (inter-camera matches)
 * position i+2 -> matches where the previous camera is cam i and the current camera is the cam left to i (inter-camera matches)
 * @param Ladybug2 ladybug object
 * @param int vector position
 * @param int& output previous camera index
 * @param int& output current camera index
 * @return bool true if valid vector position, false otherwise */
 bool getPrevAndCurrCamIndex(Ladybug2 lb2, int position, int &camPrev, int &camCurr)
 {
     if(position < 0 || position >= 15)
     {
         return false;
     }

     camPrev = position/3;
     if(position%3 == 0)
     {
         camCurr = camPrev;
     }
     else
     {
         int camLeft, camRight;
         lb2.getLeftRightCameras(camPrev, camLeft, camRight);
         if(position%3 == 1)
         {
             camCurr = camRight;
         }
         else if(position%3 == 2)
         {
             camCurr = camLeft;
         }
     }
     return true;
 }
 
/** Get vector position given a previous and current camera index, assuming that the vector is organized this way:
 * position i   -> matches where the previous camera is cam i and the current camera is cam i (intra-camera matches)
 * position i+1 -> matches where the previous camera is cam i and the current camera is the cam right to i (inter-camera matches)
 * position i+2 -> matches where the previous camera is cam i and the current camera is the cam left to i (inter-camera matches)
 * @param Ladybug2 ladybug object
 * @param int previous camera index
 * @param int current camera index
 * @param int& output vector index
 * @return bool true if valid camera indices, false otherwise */
bool getVectorPosition(Ladybug2 lb2, int camPrev, int camCurr, int &position)
{
	if(camPrev < 0 || camPrev > 4)
	{
		return false;
	}
	position = 3 * camPrev;
	
	if(camCurr == camPrev)
	{
		return true;
	}
	
	int camLeft, camRight;
	lb2.getLeftRightCameras(camPrev, camLeft, camRight);
	if(camCurr == camRight)
	{
		position += 1;
		return true;
	}
	if(camCurr == camLeft)
	{
		position += 2;
		return true;
	}
	return false;
}

/** Check if a given integer x exists in a vector of integers, vec
 * @param std::vector<int> vector vec with integers
 * @param int integer x to be searched
 * @return bool true if x is in vec, false otherwise */
bool elemInVec(std::vector<int> vec, int x)
{
    for(int i=0; i<vec.size(); i++)
    {
        if(vec[i] == x)
        {
            return true;
        }
    }
    return false;
}

/** Get samplesize random integers between zero and totalNumber-1
 * @param int maximum possibly sectected number + 1
 * @param int number of random integers to be selected
 * @return std::vector<int> vector with the random integers */
std::vector<int> getRandomSample(int totalNumber, int sampleSize)
{
    
    std::vector<int> randomSample;
    for(int i=0; i<sampleSize; i++)
    {
		// if the randomly selected number is in the vector, select another one
        int x = rand()%totalNumber;		
        
        while(elemInVec(randomSample, x))
        {
            
            x = rand()%totalNumber;
        }        
        randomSample.push_back(x);
    }
    return randomSample;
}

/** Convert transform to odometry message.
 * @param Eigen::Matrix4f transform
 * @param int camera index with the most successful motion estimation
 * @return nav_msgs::Odometry odometry message */
nav_msgs::Odometry transform2OdometryMsg(Eigen::Matrix4f T, int bestCamera)
{
    nav_msgs::Odometry msg;

    // translation
    msg.pose.pose.position.x = T(0, 3);
    msg.pose.pose.position.y = T(1, 3);
    msg.pose.pose.position.z = T(2, 3);

    // rotation
    tf::Matrix3x3 R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
    double roll, pitch, yaw;
    R.getRPY(roll, pitch, yaw);
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    quaternionTFToMsg(q, msg.pose.pose.orientation);

    char childFrameId[25];
    if(bestCamera != -1)
    {
        sprintf(childFrameId, "ladybug_cam_%02d", bestCamera);
    }
    else
    {
        sprintf(childFrameId, "none");
    }    
    msg.child_frame_id = std::string(childFrameId);

    return msg;
}

/** Get transform matrix for a given rotation matrix and translation vector.
 * @param Eigen::Matrix3f rotation matrix
 * @param Eigen::Vector3f translation vector
 * @return Eigen::Matrix4f transform */
Eigen::Matrix4f Rt2T(Eigen::Matrix3f R, Eigen::Vector3f t)
{
    Eigen::Matrix4f T;
    T << R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2), t(1), R(2, 0), R(2, 1), R(2, 2), t(2), 0.0, 0.0, 0.0, 1.0;
    return T;
}

/** Extract rotation matrix and translation vector from a given transform matrix.
 * @param Eigen::Matrix4f transform
 * @param Eigen::Matrix3f output rotation matrix
 * @param Eigen::Vector3f output translation vector
 * @return void */
void T2Rt(Eigen::Matrix4f T, Eigen::Matrix3f &R, Eigen::Vector3f &t)
{
    R << T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2);
    t << T(0, 3), T(1, 3), T(2, 3);
}
