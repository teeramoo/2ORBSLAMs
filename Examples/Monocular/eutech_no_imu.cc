/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include "../../src/IMU/imudata.h"
#include "../../src/IMU/configparam.h"


using namespace std;

class Position
{
public:
    double X;
    double Y;
    double Z;
    double pitch;
    double roll;
    double yaw;
    double timestamp;
};

void tokenize(const string &str, vector<string> &vTokens);

void tokenize_space(const string &str, vector<string> &vTokens);

void readFrameData(const string &strFrameDataFile, vector<double> &vTimestamps,
        vector<int> &vFrameNames, vector<Position> &vIMUPositions);

void LoadImages(const string &strPathTimes, vector<double> &vTimeStamps);
                
void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions);

int main(int argc, char **argv)
{
   	if (argc != 7)
    {
        cerr << endl << "Usage: " << argv[0]
                << " <path_to_vocabulary> <path_to_settings> <path_to_video> <path_to_vid_log_file> <path_to_imu_log_file> <path_to_output_folder>"
                << endl;
        return 1;
    }
    
    string strVideoFile = string(argv[3]);
    string strFrameDataFile = string(argv[4]);
    string strIMUDataFile = string(argv[5]);
    string sPathToOutputFolder = string(argv[6]);

    // Retrieve paths to images
    vector<double> vTimestamps;
    LoadImages(strFrameDataFile, vTimestamps);

	vector<Position> vIMUPositions;
	LoadIMUData(strIMUDataFile, vIMUPositions);
	

    int nImages = vTimestamps.size();
	int nIMU = vIMUPositions.size();
	cout << "Images: " << nImages << endl;
	cout << "IMU: " << nIMU << endl;

	cv::VideoCapture videoCapture(strVideoFile);
    if (!videoCapture.isOpened())
    {
        ostringstream ostream;
        ostream << "Could not open video file " << strVideoFile;
        throw runtime_error(ostream.str());
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,sPathToOutputFolder, true);

	ORB_SLAM2::ConfigParam config(argv[2]);

	double imageMsgDelaySec = config.GetImageDelayToIMU();
	
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
	cv::Mat im;
	cv::Mat imUn;
    
    double epsilon = 1e-6;
    
    int nIMUc = 0;
    int skip = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        if (!videoCapture.read(imUn))
		{
		  throw runtime_error("Premature end of video file.");
		}
		cv::Mat roi(imUn, cv::Rect(0, imUn.rows/2, imUn.cols, imUn.rows/2));
		roi.copyTo(im);
        double tframe = vTimestamps[ni];	//Time is in millisecond so divide by 1000 
        double tempFrame = tframe/1e6;
		if(ni < skip)
		{
			nIMUc++;
			continue;
		}
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  ni << endl;
            return 1;
        }
        
		double tempIMU = vIMUPositions[nIMUc].timestamp/1e6;
        std::vector<ORB_SLAM2::IMUData> vimuData;

        cout << "Frame: "<< ni << endl;

		Position currentPos = vIMUPositions[nIMUc-1];
    	double ax = 1;
    	double ay = 1;
    	double az = 1;
    	double wx = 1;
    	double wy = 1;
    	double wz = 1;
    	double dTime = tframe/1e6;
    	ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);
       	vimuData.push_back(imudata);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // imageMsg->header.stamp == image time
        //SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
        SLAM.SetFrameNumber(ni);
        SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec);
        
        //SLAM.TrackMonocular(im,tempFrame);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        bool bstop = false;
		while(0)//!SLAM.bLocalMapAcceptKF())
        {
            if(false)
            {
                bstop=true;
            }
        };
        if(bstop)
            break;
    }
	
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory and keyframe info
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.CallTrackingSavers();

    return 0;
}


void tokenize(const string &str, vector<string> &vTokens)
{
    int iPos = 0;
    int iTokBeg = 0;
    while (iPos < (int) str.length())
    {
        if (str[iPos] == ',')
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


void LoadImages(const string &strPathTimes, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    string s;
    getline(fTimes,s);
    getline(fTimes,s);
    getline(fTimes,s);
    getline(fTimes,s);
    cout<<"Read vid timestamp:" << strPathTimes<<endl;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
       
        if(!s.empty())
        {
        	vector<string> vTokens;
	        
	        /*
	        stringstream ss;
	        ss.str(s);
	        string token;
	        
	        while(getline(ss,token,delim))
	        {
	        	vTokens.push_back(token);
	        }
	        */
        	
        	tokenize(s, vTokens);
        	if(vTokens.size() == 2)
        	{
        		double t = std::stod(vTokens[0]);
        		vTimeStamps.push_back(t);
        	}
        }
    }
}

void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions)
{
	ifstream fTimes;
    fTimes.open(strIMUPath.c_str());
    // Skip first line
    string s;
    getline(fTimes,s);
    
    Position positionLatest;
    positionLatest.X = 0;
    positionLatest.Y = 0;
    positionLatest.Z = 0;
    positionLatest.roll = 0;
    positionLatest.pitch = 0;
    positionLatest.yaw = 0;
    positionLatest.timestamp = 0;
    
    double offsetAcl = 9.8/8192.0;
    double offsetGy = 131.0/250.0;
    
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
        	vector<string> vTokens;
        	tokenize(s, vTokens);
        	if(vTokens.size() == 8)
        	{
		    	positionLatest.timestamp = std::stod(vTokens[0]);

		        //positionLatest.timestamp /= 1e9;

				positionLatest.X = std::stod(vTokens[1]);
		    	positionLatest.Y = std::stod(vTokens[2]);
		    	positionLatest.Z = std::stod(vTokens[3]);
		    	
		    	positionLatest.roll = std::stod(vTokens[4]);
		    	positionLatest.pitch = std::stod(vTokens[5]);
		    	positionLatest.yaw = std::stod(vTokens[6]);
		    	
		    	positionLatest.X *= offsetAcl;
		    	positionLatest.Y *= offsetAcl;
		    	positionLatest.Z *= offsetAcl;
		    	
		    	positionLatest.roll *= offsetGy;
		    	positionLatest.pitch *= offsetGy;
		    	positionLatest.yaw *= offsetGy;
		    			    	
		    	vIMUPositions.push_back(positionLatest);
		    }
        }
    }
}


void tokenize_space(const string &str, vector<string> &vTokens)
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


void readFrameData(const string &strFrameDataFile, vector<double> &vTimestamps,
        vector<int> &vFrameNames, vector<Position> &vIMUPositions)
{
    ifstream f;
    f.open(strFrameDataFile.c_str());
    if (!f.is_open())
    {
        ostringstream ostream;
        ostream << "Could not open frame data file " << strFrameDataFile;
        throw runtime_error(ostream.str());
    }

    Position positionLatest;
    positionLatest.X = 0;
    positionLatest.Y = 0;
    positionLatest.Z = 0;
    positionLatest.roll = 0;
    positionLatest.pitch = 0;
    positionLatest.yaw = 0;
    positionLatest.timestamp = 0;
    while (f.is_open() && !f.eof())
    {
        string s;
        getline(f, s);
        if (s.length() == 0)
            continue;
        if (s[0] == '#')
            continue;
        vector<string> vTokens;
        tokenize_space(s, vTokens);
        if (vTokens[0] == "Output")
        {
            // Frame descriptor
            vTimestamps.push_back(std::stod(vTokens[8]));
            vFrameNames.push_back(vIMUPositions.size());
        }
        else if (vTokens[1] == "xacc" && vTokens[3] == "yacc")
        {
            // linear acceleration
            positionLatest.timestamp = std::stof(vTokens[0]);
            positionLatest.X = std::stof(vTokens[2]);
        	positionLatest.Y = std::stof(vTokens[4]);
        	positionLatest.Z = std::stof(vTokens[6]);
        	vIMUPositions.push_back(positionLatest);
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


