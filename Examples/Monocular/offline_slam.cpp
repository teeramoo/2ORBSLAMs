/**
 * ORB-SLAM2 driver for AIT quadrotor SLAM
 * Cloned from mono_tum.cc from the ORB-SLAM2 distribution.
 *
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

class Position
{
public:
    float X;
    float Y;
    float Z;
    float pitch;
    float roll;
    float yaw;
    float timestamp;
};

void readFrameData(const string &strFrameDataFile, vector<double> &vTimestamps,
        vector<string> &vFrameNames, vector<Position> &vPositions,
        float fCameraDelay);

void positionToTransform(Position &position, cv::Mat &mTrw);

void readConfig(string strConfig, cv::Mat &mMavToCamDownward,
        cv::Mat &mMavToCamForward, cv::Mat &mKDownward, float &fCameraDelay, int &iImageMode);

double frameDifference(cv::Mat &matFrameCurrent, cv::Mat &matFramePrevious);

void xyzToUv(float x, float y, float z, cv::Mat &mTcw, cv::Mat &mK, float &u,
        float &v);

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        cerr << endl << "Usage: " << argv[0]
                << " <path_to_vocabulary> <path_to_settings> <path_to_video> <path_to_log_file>"
                << endl;
        return 1;
    }

    // Read log file and MAV configuration

	vector<cv::Mat> vMatBuffer;
	size_t sBufferSize = 50;
	vMatBuffer.reserve(sBufferSize);
    vector<double> vTimestamps;
    vector<string> vFrameNames;
    vector<Position> vPositions;
    string strConfig = string(argv[2]);
    string strVideoFile = string(argv[3]);
    string strFrameDataFile = string(argv[4]);
    int iImageMode = 0;
	int iCurrentState = -1;
	int iBufferLoc = 0;
	int iLastLoadedLoc = 0;
	int iRefSkipFrames = 0;
	bool bBufferLock = false;
	bool bSetReference = false;

    cv::Mat TcrDownward, mTrcForward, mKDownward;
    float fCameraDelay;
    readConfig(strConfig, TcrDownward, mTrcForward, mKDownward, fCameraDelay, iImageMode);
    readFrameData(strFrameDataFile, vTimestamps, vFrameNames, vPositions,
            fCameraDelay);

    int nImages = vTimestamps.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    //ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MAV, true);
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
    // Vector for tracking time statistics
      

    vector<float> vTimesTrack;

    cout << "-------" << endl;
    cout << "Start processing video ..." << flush;

    cv::VideoCapture videoCapture(strVideoFile);
    if (!videoCapture.isOpened())
    {
        ostringstream ostream;
        ostream << "Could not open video file " << strVideoFile;
        throw runtime_error(ostream.str());
    }
	
    // Main loop
// 	nImages = 1000;
    cout << "Processing " << nImages << " images." << endl;

    cv::Mat matFrameComposed;
    cv::Mat matFrameDownward;
    cv::Mat matFrameForward;
    cv::Mat matFrameDownwardLast;
//  	nImages = 8000;
 	int cWaitKeyPress = 0;
	int iSysState = -1;
	double tInc = 16;
	double tFrameTotal = 100.0;
	//cv::namedWindow("Debug",CV_WINDOW_NORMAL);
	
    for (int ni = 0; ni < nImages; ni++)
    {
//  		double tframe = tFrameTotal;
 		double tframe = vTimestamps[ni];
		if(iCurrentState >= 3 || iBufferLoc < vMatBuffer.size())
		{
		  if(iBufferLoc < 0)
		  {
			iBufferLoc = vMatBuffer.size()-1;
		  }
		  matFrameDownward = vMatBuffer.at(iBufferLoc);
		  cout << "Frame " << iBufferLoc << " timestamp " << tframe << endl;
		}
		else
		{
		  // Read image from file
		  if (!videoCapture.read(matFrameComposed))
		  {
			  throw runtime_error("Premature end of video file.");
		  }
		  
		  tFrameTotal += tInc;
		  cout << "-------------" << endl;
		  cout << "Frame " << ni << " timestamp " << tframe << " pts "
				  << videoCapture.get(CV_CAP_PROP_POS_MSEC) << endl;

		  if (ni > 0)
			  matFrameDownwardLast = matFrameDownward.clone();

		  // Split into downward and forward frames

		  if(iImageMode == 0)			//Use full image
		  {
			  matFrameComposed.copyTo(matFrameDownward);
		  }
		  else if(iImageMode == 1)	//Use lower half of image
		  {
			  cv::Mat matFrameDownwardRoi(matFrameComposed,
					  cv::Rect(0, matFrameComposed.rows / 2, matFrameComposed.cols,
							  matFrameComposed.rows / 2));
			  matFrameDownwardRoi.copyTo(matFrameDownward);
		  }
		  else if(iImageMode == 2)	//Use resized image
		  {
			  cv::resize(matFrameComposed,matFrameDownward,cv::Size(1280,720));		
		  }
		  else if(iImageMode == 3)	//Use upper half of image
		  {
			  cv::Mat matFrameDownwardRoi(matFrameComposed,
					  cv::Rect(0, 0, matFrameComposed.cols,
							  matFrameComposed.rows / 2));
			  matFrameDownwardRoi.copyTo(matFrameDownward);
		  }
		
//         cv::Mat matFrameForwardRoi(matFrameComposed,
//                 cv::Rect(0, 0, matFrameComposed.cols,
//                         matFrameComposed.rows / 2));
//         matFrameForwardRoi.copyTo(matFrameForward);

		  if(vMatBuffer.size() >= sBufferSize)
		  {
			vMatBuffer.erase(vMatBuffer.begin());
			iBufferLoc--;
		  }
		  cv::Mat bufferFrame = matFrameDownward.clone();
		  vMatBuffer.push_back(bufferFrame);
		}
        std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now();

        if (ni==0) imwrite("frame0.png",matFrameDownward);
        if (ni==1) imwrite("frame1.png",matFrameDownward);
        if (ni==2) imwrite("frame2.png",matFrameDownward);
        if (ni==3) imwrite("frame3.png",matFrameDownward);
        if (ni==4) imwrite("frame4.png",matFrameDownward);
        if (ni==5) imwrite("frame5.png",matFrameDownward);
        if (ni==6) imwrite("frame6.png",matFrameDownward);
        if (ni==7) imwrite("frame7.png",matFrameDownward);
        if (ni==8) imwrite("frame8.png",matFrameDownward);
        if (ni==9) imwrite("frame9.png",matFrameDownward);

        double frameDiff = 0;
        if (ni > 0)
        {
            frameDiff = frameDifference(matFrameDownward, matFrameDownwardLast);
        }

        if (frameDiff == 0 && ni > 0 && (iCurrentState < 3))
        {
            cout << "SKIPPING DUPLICATE FRAME" << endl;
        }
        /*
		else if (bSetReference) //Skipping for ref frame parallax
		{
			cout << "SKIPPING FOR REFERENCE FRAME PARALLAX" << endl;
			iRefSkipFrames++;
			if(iRefSkipFrames > 3)
			{
			  bSetReference = false;
			  iRefSkipFrames = 0;
			}
		}
		
		else if(iSysState != 2)
		{
			cout << "SKIPPING FOR not OK state" << endl;
			iRefSkipFrames++;
			if(iRefSkipFrames > 3)
			{
			  iSysState = 2;
			  iRefSkipFrames = 0;
			}
		}*/
        else
        {
            // Pass the downward image to the SLAM system

//             cout << "Frame " << vFrameNames[ni] << " position ("
//                     << vPositions[ni].X << ", " << vPositions[ni].Y << ", "
//                     << vPositions[ni].Z << ", " << vPositions[ni].roll << ", "
//                     << vPositions[ni].pitch << ", " << vPositions[ni].yaw << ")"
//                     << endl;
//             cv::Mat Twr;
//             positionToTransform(vPositions[ni], Twr);
//             cout << "Twr from position: " << Twr << endl;
//             cout << "Tcr from config: " << TcrDownward << endl;
//             cv::Mat Tcw = TcrDownward * Twr.inv();
//             cout << "World-to-Cam Transform: " << Tcw << endl;
//             {
//                 float u, v;
//                 xyzToUv(-5, 0, 0, Tcw, mKDownward, u, v);
//                 cout << "Transform [-5, 0, 0] to camera coords: (" << u << ", "
//                         << v << ")" << endl;
//             }
            //SLAM.TrackMAV(matFrameDownward, tframe, Tcw, true);
//             SLAM.TrackMonocular(matFrameDownward, tframe);
// 		  cv::imshow("Debug",matFrameDownward);
		  SLAM.TrackMonocular(vMatBuffer.at(iBufferLoc), tframe);
// 		  cv::waitKey(1);
			iCurrentState = 1;
			iSysState = SLAM.GetTrackerState();
			bSetReference = SLAM.GetInitializerState();
			
			if (iCurrentState >= 3 && !bBufferLock)
			{
			  iLastLoadedLoc = iBufferLoc;
// 			  if(iBufferLoc > 9)
// 				iBufferLoc -= 10;
// 			  else
// 				iBufferLoc = 0;
			  bBufferLock = true;
			}
			if (iCurrentState < 3 && bBufferLock)
			{
			  cout<<"Reinitialize successful at " << iBufferLoc << endl;
			  bBufferLock = false;
			}
// 			cout << "Current System mState = "<<iCurrentState<<" ::::::::::::::::::::::::::::::::::::::"<<endl;
        }

        //if (ni >= 7) sleep(2);
        if(iBufferLoc < sBufferSize && !bBufferLock)
		  iBufferLoc++;
		
		if(bBufferLock)
		  iBufferLoc--;

        std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now();

        double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double> >(
                        t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        // Wait to load the next frame

//         double T = 0;
//         if (ni < nImages - 1)
//         {
//             T = vTimestamps[ni + 1] - tframe;
//         }
//         else if (ni > 0)
//         {
//             T = tframe - vTimestamps[ni - 1];
//         }
// 
//         if (ttrack < T)
//         {
//             usleep((T - ttrack) * 1e3);
//         }
        
#define PAUSE_BETWEEN_FRAMESoff
#ifdef PAUSE_BETWEEN_FRAMES
		cWaitKeyPress = cvWaitKey(10);
		cout<<"Wait Key:"<<cWaitKeyPress<<endl;
#endif
    }

    // Stop all threads

    SLAM.Shutdown();

    // Tracking time statistics

    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void xyzToUv(float x, float y, float z, cv::Mat &Tcw, cv::Mat &mK, float &u,
        float &v)
{
    assert(Tcw.type() == CV_32F && mK.type() == CV_32F);
    cv::Mat Xw = cv::Mat::zeros(4, 1, CV_32F);
    Xw.at<float>(0, 0) = x;
    Xw.at<float>(0, 1) = y;
    Xw.at<float>(0, 2) = z;
    Xw.at<float>(0, 3) = 1;
    cv::Mat Xc = Tcw * Xw;

    u = mK.at<float>(0, 0) * Xc.at<float>(0, 0)
            + mK.at<float>(0, 1) * Xc.at<float>(1, 0)
            + mK.at<float>(0, 2) * Xc.at<float>(2, 0);
    v = mK.at<float>(1, 0) * Xc.at<float>(0, 0)
            + mK.at<float>(1, 1) * Xc.at<float>(1, 0)
            + mK.at<float>(1, 2) * Xc.at<float>(2, 0);
    float w = mK.at<float>(2, 0) * Xc.at<float>(0, 0)
            + mK.at<float>(2, 1) * Xc.at<float>(1, 0)
            + mK.at<float>(2, 2) * Xc.at<float>(2, 0);
    u /= w;
    v /= w;
}

void readConfig(string strConfig, cv::Mat &mMavToCamDownward,
        cv::Mat &mMavToCamForward, cv::Mat &mKDownward, float &fCameraDelay, int &iImageMode)
{
    cv::Mat Rvec, Tvec;
    cv::FileStorage fSettings(strConfig, cv::FileStorage::READ);

    fSettings["rot_downward"] >> Rvec;
    Rvec.convertTo(Rvec, CV_32F);
    cv::Mat Rmat(3, 3, CV_32F);
    cv::Rodrigues(Rvec, Rmat);
    fSettings["trans_downward"] >> Tvec;
    Tvec.convertTo(Tvec, CV_32F);
    mMavToCamDownward = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat mTmp = mMavToCamDownward(cv::Rect(0, 0, 3, 3));
    Rmat.copyTo(mTmp);
    mTmp = mMavToCamDownward(cv::Rect(3, 0, 1, 3));
    Tvec.copyTo(mTmp);

    fSettings["rot_forward"] >> Rvec;
    Rvec.convertTo(Rvec, CV_32F);
    cv::Rodrigues(Rvec, Rmat);
    fSettings["trans_forward"] >> Tvec;
    Tvec.convertTo(Tvec, CV_32F);
    mMavToCamForward = cv::Mat::eye(4, 4, CV_32F);
    mTmp = mMavToCamForward(cv::Rect(0, 0, 3, 3));
    Rmat.copyTo(mTmp);
    mTmp = mMavToCamForward(cv::Rect(3, 0, 1, 3));
    Tvec.copyTo(mTmp);

    fSettings["camera_matrix_downward"] >> mKDownward;
    mKDownward.convertTo(mKDownward, CV_32F);

    fSettings["camera_delay"] >> fCameraDelay;
    fSettings["image_mode"] >> iImageMode;
}

void positionToTransform(Position &position, cv::Mat &mTrw)
{
    float aaRotMatrix[3][3];
    // mavlink_euler_to_dcm gives right handed point rotations
    // so if r/p/y is the rotation of the robot in the (NED) world coordinate
    // system, we will get a robot to (NED) world rotation matrix
    mavlink_euler_to_dcm(position.roll, position.pitch, position.yaw,
            aaRotMatrix);
#define DEBUG_DCM
#ifdef DEBUG_DCM
    float roll, pitch, yaw;
    mavlink_dcm_to_euler(aaRotMatrix, &roll, &pitch, &yaw);
    assert(fabs(yaw-position.yaw)<1e-6);
#endif
    mTrw = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat mRrobotToNed(3, 3, CV_32F);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            mRrobotToNed.at<float>(i, j) = aaRotMatrix[i][j];
        }
    }
    // Convert NED rotation to ENU
    cv::Mat RNed2Enu = cv::Mat::zeros(3, 3, CV_32F);
    RNed2Enu.at<float>(0, 0) = 0;
    RNed2Enu.at<float>(0, 1) = 1;
    RNed2Enu.at<float>(0, 2) = 0;
    RNed2Enu.at<float>(1, 0) = 1;
    RNed2Enu.at<float>(1, 1) = 0;
    RNed2Enu.at<float>(1, 2) = 0;
    RNed2Enu.at<float>(2, 0) = 0;
    RNed2Enu.at<float>(2, 1) = 0;
    RNed2Enu.at<float>(2, 2) = -1;
    cv::Mat Rrw = mTrw(cv::Rect(0, 0, 3, 3));
    Rrw = RNed2Enu * mRrobotToNed;
    cv::Mat matC(3, 1, CV_32F);
    mTrw.at<float>(0, 3) = position.X;
    mTrw.at<float>(1, 3) = position.Y;
    mTrw.at<float>(2, 3) = position.Z;
}

void tokenize(const string &str, vector<string> &vTokens)
{
    int iPos = 0;
    int iTokBeg = 0;
    while (iPos < (int) str.length())
    {
        if (str[iPos] == ' ')
        {
            if (iTokBeg < iPos)
            {
                vTokens.push_back(str.substr(iTokBeg, iPos - iTokBeg));
                iTokBeg = iPos + 1;
            }
        }
        iPos++;
    }
    if (iTokBeg < (int) str.length())
        vTokens.push_back(str.substr(iTokBeg));
}

#define EARTH_RADIUS 6371000

void latLonAltToXYZ(int latOr, int lonOr, int altOr, int lat, int lon, int alt,
        float &X, float &Y, float &Z)
{
    int latRel = lat - latOr;
    int lonRel = lon - lonOr;
    int altRel = alt - altOr;
    Y = 2 * EARTH_RADIUS * sin(latRel / 2 / 1e+7 / 180 * M_PI);
    X = 2 * EARTH_RADIUS * cos((latOr + lat) / 2 / 1e+7 / 180 * M_PI)
            * sin(lonRel / 2 / 1e+7 / 180 * M_PI);
    Z = altRel / 1e+3;
}

// Search for first position in given list with timestamp greater than or equal to given timestamp.
// Begin search assuming that there are POSITION_FREQ positions per second, approximately. This makes
// the search constant time rather than linear or log time.

#define POSITION_FREQ 150
Position pastPosition(vector<Position> &vPositions, float timestamp)
{
    int n = (int) vPositions.size();
    float timestampLast = vPositions[n - 1].timestamp;
    float timestampDiff = timestampLast - timestamp;
    int i = n - 1 - (int) (timestampDiff * POSITION_FREQ);
    if (i < 0)
        i = 0;
    if (vPositions[i].timestamp < timestamp)
    {
        while (i < n && vPositions[i].timestamp < timestamp)
            i++;
        if (i >= n)
            i = n - 1;
    }
    else
    {
        while (i >= 0 && vPositions[i].timestamp >= timestamp)
            i--;
        if (i < 0)
            i = 0;
        else if (i < n - 1)
            i++;
    }
    return vPositions[i];
}

void readFrameData(const string &strFrameDataFile, vector<double> &vTimestamps,
        vector<string> &vFrameNames, vector<Position> &vPositions,
        float fCameraDelay)
{
    ifstream f;
    f.open(strFrameDataFile.c_str());
    if (!f.is_open())
    {
        ostringstream ostream;
        ostream << "Could not open frame data file " << strFrameDataFile;
        throw runtime_error(ostream.str());
    }

    bool bFirstGps = false;
    int latOr = 0, lonOr = 0, altOr = 0;
    Position positionLatest;
    positionLatest.X = 0;
    positionLatest.Y = 0;
    positionLatest.Z = 0;
    positionLatest.roll = 0;
    positionLatest.pitch = 0;
    positionLatest.yaw = 0;
    positionLatest.timestamp = 0;
    vector<Position> vPositionsAll;
    while (f.is_open() && !f.eof())
    {
        string s;
        getline(f, s);
        if (s.length() == 0)
            continue;
        if (s[0] == '#')
            continue;
        vector<string> vTokens;
        tokenize(s, vTokens);
        if (vTokens[0] == "Output")
        {
            // Frame descriptor
            vTimestamps.push_back(std::stod(vTokens[8]));
            vFrameNames.push_back(vTokens[2]);
            if (vPositionsAll.size() > 0)
            {
                vPositions.push_back(
                        pastPosition(vPositionsAll,
                                positionLatest.timestamp - fCameraDelay));
            }
            else
            {
                vPositions.push_back(positionLatest);
            }
        }
        else if (vTokens[1] == "lat" && vTokens[3] == "lon")
        {
            // GLOBAL_POSITION_INT
            positionLatest.timestamp = std::stof(vTokens[0]);
            int lat = std::stod(vTokens[2]);
            int lon = std::stod(vTokens[4]);
            int alt = std::stod(vTokens[6]);
#if 0
            int relalt = std::stod(vTokens[8]);
            int vx = std::stod(vTokens[10]);
            int vy = std::stod(vTokens[12]);
            int vz = std::stod(vTokens[14]);
            int hdg = std::stod(vTokens[16]);
#endif
            if (!bFirstGps)
            {
                latOr = lat;
                lonOr = lon;
                altOr = alt;
                bFirstGps = true;
            }
            latLonAltToXYZ(latOr, lonOr, altOr, lat, lon, alt, positionLatest.X,
                    positionLatest.Y, positionLatest.Z);
            vPositionsAll.push_back(positionLatest);
        }
        else if (vTokens[1] == "roll" && vTokens[3] == "pitch")
        {
            // ATTITUDE
            positionLatest.roll = std::stof(vTokens[2]);
            positionLatest.pitch = std::stof(vTokens[4]);
            positionLatest.yaw = std::stof(vTokens[6]);
        }
    }
}

double frameDifference(cv::Mat &matFrameCurrent, cv::Mat &matFramePrevious)
{
    double diff = 0.0;
    assert(matFrameCurrent.rows > 0 && matFrameCurrent.cols > 0);
    assert(
            matFrameCurrent.rows == matFramePrevious.rows
                    && matFrameCurrent.cols == matFramePrevious.cols);
    assert(
            matFrameCurrent.type() == CV_8UC3 && matFramePrevious.type() == CV_8UC3);
    for (int i = 0; i < matFrameCurrent.rows; i++)
    {
        for (int j = 0; j < matFrameCurrent.cols; j++)
        {
            cv::Vec3b cur;
            cv::Vec3b prev;
            cur = matFrameCurrent.at<cv::Vec3b>(i, j);
            prev = matFramePrevious.at<cv::Vec3b>(i, j);
            for (int k = 0; k < 3; k++)
                diff += fabs(cur[k] - prev[k]);
        }
    }
    return diff;
}
