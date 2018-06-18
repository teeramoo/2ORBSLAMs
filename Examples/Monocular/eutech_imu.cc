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
    float X;
    float Y;
    float Z;
    float pitch;
    float roll;
    float yaw;
    double timestamp;
};

void tokenize(const string &str, vector<string> &vTokens);

void tokenize_space(const string &str, vector<string> &vTokens);

void readFrameData(const string &strFrameDataFile, vector<double> &vTimestamps,
        vector<int> &vFrameNames, vector<Position> &vIMUPositions);

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);
                
void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions);

int main(int argc, char **argv)
{
   	if (argc != 6)
    {
        cerr << endl << "Usage: " << argv[0]
                << " <path_to_vocabulary> <path_to_settings> <path_to_video> <path_to_log_file> <path_to_output_folder>"
                << endl;
        return 1;
    }
    
    string strVideoFile = string(argv[3]);
    string strFrameDataFile = string(argv[4]);
    string strOutputFolder = string(argv[5]);

    // Retrieve paths to images
    vector<int> vFrameNames;
    vector<double> vTimestamps;
    //LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

	vector<Position> vIMUPositions;
	//LoadIMUData(string(argv[5]), vIMUPositions);
	
	readFrameData(strFrameDataFile, vTimestamps, vFrameNames, vIMUPositions);

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
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,strOutputFolder,true);

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
	cv::Mat matFrameComposed;
    
    int nIMUc = 0;
    int skip = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        if (!videoCapture.read(matFrameComposed))
		{
		  throw runtime_error("Premature end of video file.");
		}
        cv::Mat matFrameDownwardRoi(matFrameComposed,
					  cv::Rect(0, matFrameComposed.rows / 2, matFrameComposed.cols,
					  matFrameComposed.rows / 2));
		matFrameDownwardRoi.copyTo(im);
        double tframe = vTimestamps[ni];	//Time is in millisecond so divide by 1000 
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
        
        std::vector<ORB_SLAM2::IMUData> vimuData;
        //This check should be use fabs() < epsilon
        cout << "Check for IMU at " << nIMUc << endl;
        cout << "IMU time: " << setprecision(18) << vIMUPositions[nIMUc].timestamp << endl;
        cout << "Frame time: " << setprecision(18) << tframe << endl;
        cout << "Time diff: " << setprecision(18) << tframe - vIMUPositions[nIMUc].timestamp << endl;
        while(nIMUc < vFrameNames[ni])
        {
        	cout << nIMUc << " IMU time: "<< setprecision(18) << vIMUPositions[nIMUc].timestamp << endl;
        	cout << nIMUc << " Frame time: " << setprecision(18) <<  tframe << endl;
        	Position currentPos = vIMUPositions[nIMUc];
        	double ax = currentPos.X;
        	double ay = currentPos.Y;
        	double az = currentPos.Z;
        	double wx = currentPos.roll;
        	double wy = currentPos.pitch;
        	double wz = currentPos.yaw;
        	double dTime = currentPos.timestamp*1e6;
        	ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);
           	vimuData.push_back(imudata);
           	nIMUc++;
        }
		cout<<"Done aggregating imu... " << endl;
		if(vimuData.size() == 0)
		{
			cout<<"Hit blank IMU slot ###############################" << endl;
			Position currentPos = vIMUPositions[nIMUc-1];
        	double ax = currentPos.X;
        	double ay = currentPos.Y;
        	double az = currentPos.Z;
        	double wx = currentPos.roll;
        	double wy = currentPos.pitch;
        	double wz = currentPos.yaw;
        	double dTime = currentPos.timestamp*1e6;
        	ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);
           	vimuData.push_back(imudata);
		}
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // imageMsg->header.stamp == image time
        //SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
        
        SLAM.TrackMonoVI(im, vimuData, (tframe/1e3) - imageMsgDelaySec);
        
        //SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        bool bstop = false;
		while(!SLAM.bLocalMapAcceptKF())
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

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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


void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    string s;
    getline(fTimes,s);
    cout<<"Load images:" << strImagePath<<endl;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
       // char delim = ',';
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
            vstrImages.push_back(strImagePath+"/"+vTokens[1].substr(0,vTokens[1].length()-1));
            double t = std::stod(vTokens[0]);
            vTimeStamps.push_back(t);///1e9);
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
    vector<Position> vPositionsAll;
    
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
        	vector<string> vTokens;
        	tokenize(s, vTokens);
        	positionLatest.timestamp = std::stod(vTokens[0]);

            //positionLatest.timestamp /= 1e9;

        	positionLatest.roll = std::stof(vTokens[1]);
        	positionLatest.pitch = std::stof(vTokens[2]);
        	positionLatest.yaw = std::stof(vTokens[3]);
        	
    		positionLatest.X = std::stof(vTokens[4]);
        	positionLatest.Y = std::stof(vTokens[5]);
        	positionLatest.Z = std::stof(vTokens[6]);
        	
        	vIMUPositions.push_back(positionLatest);
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


