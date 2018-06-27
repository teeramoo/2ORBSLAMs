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
#include<string>
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

void readFrameData(const string &strFrameFile, vector<double> &vTimestamps,
        vector<int> &vFrameNames, vector<Position> &vIMUPositions);

void LoadImages(const string &strPathTimes, vector<string> &vTimeStamps);
                
void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions);

cv::Mat calculateMatCam1to2(cv::Mat &mCam1, cv::Mat &mCam2, bool &bDebug);

int runSLAM(string &strVoc, string &strCameraSettings, string &strInputFormat, string &strInputFile,
            vector<string> &vTimestamps, vector<Position> &vIMUPositions,ORB_SLAM2::System::eSensor &eSensor, bool &isVideo,
            string &sPathToOutputFolder, bool &bDebug, bool &bUseViewer, bool &bSecondSLAM,
            vector<cv::Mat> &vCamPoses, vector<ORB_SLAM2::KeyFrame*> &vKeyFrames,
            vector<int> &vKeyFrameID, cv::Mat &mTrcBotCam, cv::Mat &mTrcTopCam);



void readInputFile(string inputFile, string &strVoc, string &strDownCamSettings, string &strUpCamSettings, string &strDownInput,
                   string &strUpInput, string &strInputFormat, bool &isVideo, string &strFrameFile, string &strIMUFile, string &strDownOutput,
                   string &strUpOutput, bool &bDebug);

int main(int argc, char **argv)
{
    string strVoc, strDownCamSettings, strUpCamSettings, strDownInput, strUpInput, strInputFormat, strFrameFile, strIMUFile, strDownOutput, strUpOutput;
    bool isVideo, bDebug;
    bool bViewer =true;
    bool isSecondSLAM = false;
    
    if (argc != 2)
    {
        cerr << endl << "Usage: " << argv[0] << "<input_file>" << endl;
        return 1;
    }
    
    readInputFile(string(argv[1]), strVoc, strDownCamSettings, strUpCamSettings, strDownInput, strUpInput, strInputFormat, isVideo, strFrameFile, strIMUFile, strDownOutput, strUpOutput, bDebug);
    
    ORB_SLAM2::System::eSensor bSensorType = ORB_SLAM2::System::MONOCULAR;

    // Retrieve paths to images
    vector<string> vTimestamps;
    LoadImages(strFrameFile, vTimestamps);

	vector<Position> vIMUPositions;
	if(strIMUFile == "")
	    std::cout << "Skip loading IMU data" << std::endl;
    else
        LoadIMUData(strIMUFile, vIMUPositions);

    vector<cv::Mat> vCamPoses;
    vector<ORB_SLAM2::KeyFrame*> vKeyFrames;
    vector<int> vKeyFrameID;
    cv::Mat mTrcBotCam;
    cv::Mat mTrcTopCam;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
/*
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Total images in the sequence: " << nImages << endl << endl;

    // Main loop
	cv::Mat im;
    
    double epsilon = 1e-6;
    
    int nIMUc = 0;
    int skip = 0;
    string mTransform = "TransformMatrix/transform.yaml" ;
    bool bInitMat = false;
    std::vector<cv::Mat> vCamPoses;
    vector<ORB_SLAM2::KeyFrame*> vKeyFrames;
    vector<int> vKeyFrameID;
    cv::Mat blankMat = cv::Mat();
    cv::Mat mTrcBotCam;
    cv::Mat mTrcTopCam;
    {

        ORB_SLAM2::System SLAM(strVoc,strDownCamSettings,ORB_SLAM2::System::MONOCULAR,strDownOutput, bDebug, true);
//        ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,strDownOutput, true, bDebug);
        double imageMsgDelaySec = SLAM.imageDelaySec;
        mTrcBotCam = SLAM.GetInitCamPose();

        if(bDebug)
            cout << "mTrcBotCam is : " << mTrcBotCam << endl;
        

    for(int ni=0; ni< nImages; ni++)         // Start point of 1st SLAM //    for(int ni=0; ni<nImages; ni++)
    {

        if (!videoCaptureBottom.read(im))         // Read image from file
		{
		  throw runtime_error("Premature end of bottom video file.");
		}

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

        cout << endl;
        cout << "--------------------------------------" << endl;
        cout << "Frame: "<< ni << " of " <<  nImages << endl;

        cout << "Checking IMU data at frame "  << ni << endl;
        while((tempFrame-tempIMU) > epsilon)
        {

        	Position currentPos = vIMUPositions[nIMUc];
        	double ax = currentPos.X;
        	double ay = currentPos.Y;
        	double az = currentPos.Z;
        	double wx = currentPos.roll;
        	double wy = currentPos.pitch;
        	double wz = currentPos.yaw;
        	double dTime = currentPos.timestamp/1e6;

        	ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);

           	vimuData.push_back(imudata);

           	nIMUc++;
            if(nIMUc <= vIMUPositions.size())
            {
                tempIMU = vIMUPositions[nIMUc].timestamp/1e6;
            }
            else
                break;

        }
        cout << "Finish checking IMU data at frame :" << ni<< endl;
        cout << "Size of vimuData is " << vimuData.size() << endl;

		if(vimuData.size() == 0)
		{
			cout<<"Hit blank IMU slot ###############################" << endl;
			cout<<"Skipping this frame (Specially if before initializing)!" << endl;
            vCamPoses.push_back(blankMat); // Add empty Matrix
           	continue;
		}


        if(ni < 200)
            continue;
        //Fast skip
        if(ni <0)
        {
            vCamPoses.push_back(blankMat); // Add empty Matrix
            continue;
        }



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

       SLAM.SetFrameNumber(ni);

       cv::Mat mCamPoseCurrent = SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec, blankMat );
       vCamPoses.push_back(mCamPoseCurrent); 


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
      
    }
	
    // Stop all threads
        cout << "Executing shutdown process SLAM.Shutdown() " << endl;

        vKeyFrames = SLAM.GetAllKeyFrames();
        cout << "Total vKeyframe " << vKeyFrames.size() << endl;


        for(int i = 0; i < vKeyFrames.size(); i++)
        {
            vKeyFrameID.push_back(vKeyFrames[i]->iFrameNumber);
        }
        cout << "Total vKeyFrameID " << vKeyFrameID.size() << endl;

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

*/
/*
        cout << "Total number of KeyFrame : " << vKeyFrames.size() << endl;

        cout << "First KeyFrame is : " << vKeyFrames[0]->iFrameNumber << endl;
        cout << "with Matrix " << vKeyFrames[0]->GetPose() << endl;
        cout << "Second KeyFrame is : " << vKeyFrames[1]->iFrameNumber << endl;
        cout << "with Matrix " << vKeyFrames[1]->GetPose() << endl;

        cout << "Size of vCamposes is : " << vCamPoses.size() <<endl;
        cout << "Matrix of Frame according to 1st keyframe is : " << vCamPoses[vKeyFrames[0]->iFrameNumber] << endl;
        cout << "Matrix of Frame according to 2nd keyframe is : " << vCamPoses[vKeyFrames[1]->iFrameNumber] << endl;
        cout << "Matrix of Frame according to 2nd keyframe +1 is : " << vCamPoses[vKeyFrames[1]->iFrameNumber +1] << endl;
        cout << "Matrix of Frame according to 2nd keyframe -1 is : " << vCamPoses[vKeyFrames[1]->iFrameNumber -1 ] << endl;

        int EmptyFrame = 0;
        vector<int> TestCamPoses;
        for(int i=0; i < vCamPoses.size(); i++)
        {
            if(vCamPoses[i].empty())
            {
                EmptyFrame++;
                TestCamPoses.push_back(i);
            }

        }
        cout << "Total empty frame is : " << EmptyFrame << endl;
        cout << "First Empty frame is : " << TestCamPoses.front() << endl;
        cout << "Last emptry frame is : " << TestCamPoses.back() << endl;

    //    cout << "SLAM 1 was initialized at : " << vKeyFrames[1]->iFrameNumber << endl;

    }
*/
    // Save camera trajectory and keyframe info


    //Run first SLAM
    
    runSLAM(strVoc, strDownCamSettings, strInputFormat, strDownInput, vTimestamps,vIMUPositions,
            bSensorType, isVideo, strDownOutput, bDebug, bViewer, isSecondSLAM,
            vCamPoses, vKeyFrames, vKeyFrameID, mTrcBotCam, mTrcTopCam);

    cout << "----------------------------------------------------------------------------" << endl;

    cout << "First SLAM process ended." << endl;

 //   return 0 ;
    sleep(1);

    
    cout << " Starting second SLAM...." << endl;
    
    isSecondSLAM = true;

    runSLAM(strVoc, strUpCamSettings, strInputFormat, strUpInput,vTimestamps,vIMUPositions,
            bSensorType, isVideo, strUpOutput, bDebug, bViewer, isSecondSLAM,
            vCamPoses, vKeyFrames, vKeyFrameID, mTrcBotCam, mTrcTopCam);

    cout << "----------------------------------------------------------------------------" << endl;

    cout << "Second SLAM process ended." << endl;

/*
    nIMUc = 0;
    skip = 0;
    cout << "Images: " << nImages << endl;
    cout << "IMU: " << nIMU << endl;

    {
        ORB_SLAM2::System SLAM(argv[1],argv[3],ORB_SLAM2::System::MONOCULAR,strUpOutput, true);
        double imageMsgDelaySec = SLAM.imageDelaySec;
        SLAM.SetSecondSLAM(true); // Set 2nd SLAM
        mTrcTopCam = SLAM.GetInitCamPose();

        cout << "mTrcTopCam is : " << mTrcTopCam << endl;

        int FirstFrameCounter = 0;
        cv::Mat mForcedKF;


        cv::Mat mCamBotToTop = cv::Mat::eye(4,4,CV_32F);
        cv::Mat t1, t2, R1, R2;

        mTrcBotCam(cv::Rect(3,0,1,3)).copyTo(t1);
        mTrcBotCam(cv::Rect(0,0,3,3)).copyTo(R1);

        mTrcTopCam(cv::Rect(3,0,1,3)).copyTo(t2);
        mTrcTopCam(cv::Rect(0,0,3,3)).copyTo(R2);

        cv::Mat tempMat1 = R2.inv()*R1;
        cv::Mat tempMat2 = R2.inv()* (t1 - t2);
        cv::Mat tempMat3 = t1-t2;


        tempMat1.copyTo(mCamBotToTop(cv::Rect(0,0,3,3)));
        tempMat2.copyTo(mCamBotToTop(cv::Rect(3,0,1,3)));


        cout << " R2.inv()*R1 is : " << R2.inv()*R1 << endl;

        cout << "  R2.inv()* (t1 - t2) is : " <<  R2.inv()* (t1 - t2) << endl;

        cout << "t1-t2 is : " << t1-t2 << endl;

        cout << "R2 is : " << R2 << endl;

        cout << "t2 is : " << t2 << endl;

        cout << "R1 is : " << R1 << endl;

        cout << "t1 is : " << t1 << endl;

        cout << "R2.inv() is : " << R2.inv() << endl;

        cout << "mCamBotToTop is : " << mCamBotToTop << endl;

        cout << "Check mTrcTop * mTrcBot.inv() " << mTrcTopCam * mTrcBotCam.inv()<< endl;

        sleep(10);
        SLAM.SetCamTopToBot(mCamBotToTop);

        for(int ni=0; ni< nImages; ni++)  //     for(int ni=0; ni<nImages; ni++)
        {
            // Read image from file
            if (!videoCaptureTop.read(im)) {
                throw runtime_error("Premature end of top video file.");
            }

            
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


            if(!SLAM.CheckInitialization())  //Check initialization
            {
                if(ni > vKeyFrameID[FirstFrameCounter])
                {
                    while(vKeyFrameID[FirstFrameCounter] < ni)
                    {
                        FirstFrameCounter++;
                    }

                    if(FirstFrameCounter < vKeyFrameID.size())
                        mForcedKF = vKeyFrames[FirstFrameCounter]->GetPose();
                    else
                        mForcedKF = vKeyFrames[FirstFrameCounter-1]->GetPose();


                    SLAM.SetForceKeyFrame(vKeyFrameID[FirstFrameCounter], mForcedKF);

                }
            }


            double tempIMU = vIMUPositions[nIMUc].timestamp/1e6;
            std::vector<ORB_SLAM2::IMUData> vimuData;

            cout << endl;
            cout << "--------------------------------------" << endl;
            cout << "Frame: "<< ni << " of " <<  nImages << endl;

            while((tempFrame-tempIMU) > epsilon)
            {

                Position currentPos = vIMUPositions[nIMUc];
                double ax = currentPos.X;
                double ay = currentPos.Y;
                double az = currentPos.Z;
                double wx = currentPos.roll;
                double wy = currentPos.pitch;
                double wz = currentPos.yaw;
                double dTime = currentPos.timestamp/1e6;
                cout << "Checking IMU data at frame "  << ni << endl;
                ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);
                cout << "Finish checking IMU data at frame :" << ni<< endl;

                vimuData.push_back(imudata);
                cout << "Size of vimuData is " << vimuData.size() << endl;


                nIMUc++;
                if(nIMUc <= vIMUPositions.size())
                {
                    tempIMU = vIMUPositions[nIMUc].timestamp/1e6;
                    cout << "Current IMU data size is : " << nIMUc << " of "<< vIMUPositions.size() << endl;
                }
                else
                    break;

            }

            if(vimuData.size() == 0)
            {
                cout<<"Hit blank IMU slot ###############################" << endl;
                cout<<"Skipping this frame (Specially if before initializing)!" << endl;
                continue;
            }


#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Quick skip to Initialized frame - 20
            if (ni < vKeyFrameID[FirstFrameCounter] - 1)
                continue;
            
            SLAM.SetFrameNumber(ni);
//            cv::Mat mCamPoseCurrent = SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec);

            cout << "SLAM_1 was initialized at frame : " << vKeyFrameID[FirstFrameCounter] << endl;

            cv::Mat mCamPoseCurrent = SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec, vCamPoses[ni]);



            if(!mCamPoseCurrent.empty())
            {
                cout << "Frame number : " << ni << endl;
                cout << "current camera pose is : " << endl << mCamPoseCurrent << endl;
           }



#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            vTimesTrack[ni]=ttrack;

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

    }
    */
    return 0;
}

int runSLAM(string &strVoc, string &strCameraSettings, string &strInputFormat, string &strInputFile,
            vector<string> &vTimestamps, vector<Position> &vIMUPositions,ORB_SLAM2::System::eSensor &eSensor, bool &isVideo,
            string &sPathToOutputFolder, bool &bDebug, bool &bUseViewer, bool &bSecondSLAM,
            vector<cv::Mat> &vCamPoses, vector<ORB_SLAM2::KeyFrame*> &vKeyFrames,
            vector<int> &vKeyFrameID, cv::Mat &mTrcBotCam, cv::Mat &mTrcTopCam)
{

    cv::VideoCapture videoCapture;
    cv::Mat imSample;

    //Check input type and video file
    if(isVideo) {
        if(!videoCapture.open(strInputFile))
            throw runtime_error("Cannot open video file at " + strInputFile );
    }
    else {
        string sampleImPath = strInputFile + vTimestamps.front() + "." + strInputFormat;
        imSample = cv::imread(sampleImPath,CV_LOAD_IMAGE_ANYDEPTH);
        if(imSample.empty())
            throw runtime_error("Cannot open a sample image at " + sampleImPath );
    }

    //Initialize ORB-SLAM
    ORB_SLAM2::System SLAM(strVoc,strCameraSettings, eSensor,sPathToOutputFolder, bDebug, bUseViewer);
    double imageMsgDelaySec = SLAM.imageDelaySec;

    //Set 2nd SLAM
    if(bSecondSLAM) {
        cout << "enabling 2nd SLAM" << endl;
        SLAM.SetSecondSLAM(true);
        mTrcTopCam = SLAM.GetInitCamPose();

        if(bDebug)
            cout << "mTrcTopCam is : " << mTrcTopCam << endl;

        //Change matCam1to2
        cv::Mat mCamBotToTop = calculateMatCam1to2(mTrcBotCam, mTrcTopCam, bDebug);
        SLAM.SetCamBotToTop(mCamBotToTop);

    } else {

        SLAM.SetSecondSLAM(false);
        mTrcBotCam = SLAM.GetInitCamPose();
        if(bDebug)
            cout << "mTrcBotCam is : " << mTrcBotCam << endl;
    }


    int nImages = vTimestamps.size();
    int nIMUs = vIMUPositions.size();
    int skip = 0;
    int nIMUc = 0;
    double epsilon = 1e-6;

    cv::Mat im;
    cv::Mat blankMat = cv::Mat();
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    int FirstFrameCounter = 0;
    cv::Mat mForcedKF;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Total images in the sequence: " << nImages << endl << endl;


    // Main loop

    for(int ni=0; ni< nImages; ni++) {        // Start point of 1st SLAM

        cout << "--------------------------------------" << endl;
        cout << "Frame: "<< ni << " of " <<  nImages << endl;


        //Load image
        if(isVideo) {

            if(!videoCapture.read(im)) {
                cout << "Cannot read frame number " << ni+1 << endl;
                break;
            }

        } else {
            cout << "reading image number "<< ni << endl;
            string imPath = strInputFile + vTimestamps[ni] + "." + strInputFormat;
            im = cv::imread(imPath, CV_LOAD_IMAGE_GRAYSCALE);

            if(im.empty()) {
                cout << "Cannot read frame number " << ni+1 << " at " << imPath << endl;
                break;
            }

            cout << "image number " << ni  << " loaded."<< endl;
        }

        if(bSecondSLAM && ni == 0)
            continue;


        double tframe = std::stod(vTimestamps[ni]);	//Time is in millisecond so divide by 1000
        double tempFrame = tframe/1e6;

 //       cout << "read timestamp in double format" << endl;
        if(ni < skip) {
            nIMUc++;
            continue;
        }
/*
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  ni << endl;

            return -1;
        }
*/

        //Check initialization

        if(!SLAM.CheckInitialization() && bSecondSLAM) {


            if(ni > vKeyFrameID[FirstFrameCounter]) {

                while(vKeyFrameID[FirstFrameCounter] < ni)
                    FirstFrameCounter++;

                if(FirstFrameCounter < vKeyFrameID.size())
                    mForcedKF = vKeyFrames[FirstFrameCounter]->GetPose();
                else
                    mForcedKF = vKeyFrames[FirstFrameCounter-1]->GetPose();


                SLAM.SetForceKeyFrame(vKeyFrameID[FirstFrameCounter], mForcedKF);

            }

        }

        cout << endl;


        cout << "size of vIMU is : " << vIMUPositions.size() << endl;

        std::vector<ORB_SLAM2::IMUData> vimuData;


        if(vIMUPositions.size() > 0) {

            double tempIMU = vIMUPositions[nIMUc].timestamp/1e9;



            if(bDebug && vIMUPositions.size() != 0)
                cout << "Checking IMU data at frame "  << ni << endl;

            while((tempFrame-tempIMU) > epsilon) {

                Position currentPos = vIMUPositions[nIMUc];
                double ax = currentPos.X;
                double ay = currentPos.Y;
                double az = currentPos.Z;
                double wx = currentPos.roll;
                double wy = currentPos.pitch;
                double wz = currentPos.yaw;
                double dTime = currentPos.timestamp/1e9;

                ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);

                vimuData.push_back(imudata);

                nIMUc++;
                if(nIMUc <= vIMUPositions.size()) {
                    tempIMU = vIMUPositions[nIMUc].timestamp/1e9;

                } else
                    break;

            }

            if(bDebug) {
                cout << "Finish checking IMU data at frame :" << ni<< endl;
                cout << "Size of vimuData is " << vimuData.size() << endl;
            }

            if(vimuData.size() == 0) {
                cout<<"Hit blank IMU slot ###############################" << endl;
                cout<<"Skipping this frame (Specially if before initializing)!" << endl;
                if(!bSecondSLAM)
                    vCamPoses.push_back(blankMat); // Add empty Matrix
                continue;
            }

        }
        else {
            double ax = 0.0;
            double ay = 0.0;
            double az = 0.0;
            double wx = 0.0;
            double wy = 0.0;
            double wz = 0.0;
            double dTime = tempFrame;

            ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);


        vimuData.push_back(imudata);
        }


        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif


        if(bSecondSLAM) {
            cout << "FirstFrameCounter is : " << FirstFrameCounter << endl;
            cout << "vKeyFrameID[FirstFrameCounter] : " << vKeyFrameID[FirstFrameCounter] << endl;
            if (ni < vKeyFrameID[FirstFrameCounter] - 1) {
//                cout << "FirstFrameCounter is : " << FirstFrameCounter << endl;
                cout << "skip this frame b/c of ni < vKeyframeID " << endl;
                im.release();
                continue;
            }

        }



        SLAM.SetFrameNumber(ni);


        if(bSecondSLAM) {
            cout << "vCamPose.size() : " << vCamPoses.size() << endl;
            cout << "current ni is : " << ni << endl;
            cout << "vCamPoses[ni] is : " << vCamPoses[ni]  << endl;
            cout << "vCamPoses[ni+1] is : " << vCamPoses[ni+1]  << endl;
            cout << "mTrcTopCam is : " << mTrcTopCam  << endl;
            if(vCamPoses[ni+1].empty())
                continue;
            cv::Mat mTempCamPose = mTrcTopCam * vCamPoses[ni+1].clone();
            cv::Mat mCamPoseCurrent = SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec, mTempCamPose );
            mTempCamPose.release();

        }

        /*
        else {

            if(ni < 200) {
                vCamPoses.push_back(blankMat);
                continue;
            }
*/

            cv::Mat mCamPoseCurrent = SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec, blankMat );
            vCamPoses.push_back(mCamPoseCurrent);
            cout << "vCamPose.size()" << vCamPoses.size();


        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif


        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        im.release();
        }


    // Stop all threads
    cout << "Executing shutdown process SLAM.Shutdown() " << endl;

    vKeyFrames = SLAM.GetAllKeyFrames();
    if(bDebug)
        cout << "Total vKeyframe " << vKeyFrames.size() << endl;


    for(int i = 0; i < vKeyFrames.size(); i++) {

        vKeyFrameID.push_back(vKeyFrames[i]->iFrameNumber);
    }

    if(bDebug)
        cout << "Total vKeyFrameID " << vKeyFrameID.size() << endl;

    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++) {

        totaltime+=vTimesTrack[ni];
    }

    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
/*
        cout << "Total number of KeyFrame : " << vKeyFrames.size() << endl;

        cout << "First KeyFrame is : " << vKeyFrames[0]->iFrameNumber << endl;
        cout << "with Matrix " << vKeyFrames[0]->GetPose() << endl;
        cout << "Second KeyFrame is : " << vKeyFrames[1]->iFrameNumber << endl;
        cout << "with Matrix " << vKeyFrames[1]->GetPose() << endl;

        cout << "Size of vCamposes is : " << vCamPoses.size() <<endl;
        cout << "Matrix of Frame according to 1st keyframe is : " << vCamPoses[vKeyFrames[0]->iFrameNumber] << endl;
        cout << "Matrix of Frame according to 2nd keyframe is : " << vCamPoses[vKeyFrames[1]->iFrameNumber] << endl;
        cout << "Matrix of Frame according to 2nd keyframe +1 is : " << vCamPoses[vKeyFrames[1]->iFrameNumber +1] << endl;
        cout << "Matrix of Frame according to 2nd keyframe -1 is : " << vCamPoses[vKeyFrames[1]->iFrameNumber -1 ] << endl;

        int EmptyFrame = 0;
        vector<int> TestCamPoses;
        for(int i=0; i < vCamPoses.size(); i++)
        {
            if(vCamPoses[i].empty())
            {
                EmptyFrame++;
                TestCamPoses.push_back(i);
            }

        }
        cout << "Total empty frame is : " << EmptyFrame << endl;
        cout << "First Empty frame is : " << TestCamPoses.front() << endl;
        cout << "Last emptry frame is : " << TestCamPoses.back() << endl;
*/
    //    cout << "SLAM 1 was initialized at : " << vKeyFrames[1]->iFrameNumber << endl;

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


void LoadImages(const string &strPathTimes, vector<string> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    string s;

    cout<<"Read input timestamp:" << strPathTimes<<endl;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
       
        if(!s.empty())
        {
        	vector<string> vTokens;

        	tokenize(s, vTokens);
        	if(vTokens.size() == 2)
        	{
        		//double t = std::stod(vTokens[0]);
        		vTimeStamps.push_back(vTokens[0]);
        	}
        }
    }
}

void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions)
{
    cout << "read IMU data" << endl;
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
    
//    double offsetAcl = 9.8/8192.0;
//    double offsetGy = 131.0/250.0;
//    double offsetGy = 1.0/131.0;
      double offsetAcl = 0.0;
      double offsetGy = 0.0;
//      double offsetGy = 1.0*180/M_PI;



    int countLine = 0;
    while(!fTimes.eof())
    {   countLine++;
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
        	vector<string> vTokens;
        	tokenize(s, vTokens);
        	if(vTokens.size() >= 7)
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

                //Conversion
	//	    	positionLatest.Z -= 9.81;

		    	positionLatest.roll *= offsetGy;
		    	positionLatest.pitch *= offsetGy;
		    	positionLatest.yaw *= offsetGy;
		    			    	
		    	vIMUPositions.push_back(positionLatest);
		    }

            if(vTokens.size() > 0 && vTokens.size() < 8)
                cout << "Skip line number from token < 8: " << countLine << endl;
            if(vTokens.size() > 8)
                cout << "Skip line number from token > 8: " << countLine << endl;
        }
    }
    cout << "Total line read : " << countLine << endl;
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


void readFrameData(const string &strFrameFile, vector<double> &vTimestamps,
        vector<int> &vFrameNames, vector<Position> &vIMUPositions)
{
    ifstream f;
    f.open(strFrameFile.c_str());
    if (!f.is_open())
    {
        ostringstream ostream;
        ostream << "Could not open frame data file " << strFrameFile;
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

cv::Mat calculateMatCam1to2(cv::Mat &mCam1, cv::Mat &mCam2, bool &bDebug)
{


    cv::Mat mCamBotToTop = cv::Mat::eye(4,4,CV_32F);
    mCamBotToTop = mCam2*mCam1*mCamBotToTop;

/*
    cout << "Calculating camera 1 to 2" << endl;
    cv::Mat _mCam1 = mCam1.inv();
    cv::Mat _mCam2 = mCam2.inv();
    cv::Mat mCamBotToTop = cv::Mat::eye(4,4,CV_32F);
    cv::Mat t1, t2, R1, R2;

    _mCam1(cv::Rect(3,0,1,3)).copyTo(t1);
    _mCam1(cv::Rect(0,0,3,3)).copyTo(R1);

    _mCam2(cv::Rect(3,0,1,3)).copyTo(t2);
    _mCam2(cv::Rect(0,0,3,3)).copyTo(R2);

    cv::Mat tempMat1 = R2.inv()*R1;
    cv::Mat tempMat2 = R2.inv()* (t1 - t2);
    cv::Mat tempMat3 = t1-t2;


    tempMat1.copyTo(mCamBotToTop(cv::Rect(0,0,3,3)));

    // No translation involved   tempMat2.copyTo(mCamBotToTop(cv::Rect(3,0,1,3)));

if(bDebug)
{
    cout << " R2.inv()*R1 is : " << R2.inv()*R1 << endl;

    cout << "  R2.inv()* (t1 - t2) is : " <<  R2.inv()* (t1 - t2) << endl;

    cout << "t1-t2 is : " << t1-t2 << endl;

    cout << "R2 is : " << R2 << endl;

    cout << "t2 is : " << t2 << endl;

    cout << "R1 is : " << R1 << endl;

    cout << "t1 is : " << t1 << endl;

    cout << "R2.inv() is : " << R2.inv() << endl;

    cout << "mCamBotToTop is : " << mCamBotToTop << endl;

    cout << "Check mCam2 * mCam1.inv() " << mCam1 * mCam2.inv()<< endl;

}
*/
    return mCamBotToTop;
}

void readInputFile(string inputFile,string &strVoc,string &strDownCamSettings,string &strUpCamSettings,string &strDownInput,
                   string &strUpInput, string &strInputFormat, bool &isVideo, string &strFrameFile, string &strIMUFile, string &strDownOutput,
                   string &strUpOutput, bool &bDebug) {

    cv::FileStorage fSettings;
    fSettings.open(inputFile, cv::FileStorage::READ);

    if(!fSettings.isOpened()) {
        cerr << "Cannot open input file from " << inputFile << endl;
        return;
    }

    std::cout << "opening " << inputFile << endl;

    std::cout<<std::endl<<std::endl<<"Parameters: "<<std::endl;

    fSettings["vocab"] >> strVoc;
    std::cout<<"Get vocab from "<< strVoc <<std::endl;

    fSettings["downward_cam_settings"] >> strDownCamSettings;
    std::cout<<"get downward camera settings from "<< strDownCamSettings <<std::endl;

    fSettings["upward_cam_settings"] >> strUpCamSettings;
    std::cout<<"get upward camera settings from "<< strUpCamSettings <<std::endl;

    fSettings["downward_input"] >> strDownInput;
    std::cout<<"get downward camera input from "<< strDownInput <<std::endl;

    fSettings["upward_input"] >> strUpInput;
    std::cout<<"get upward camera input from "<< strUpInput <<std::endl;

    fSettings["input_format"] >> strInputFormat;
    
    fSettings["isVideo"] >> isVideo;
    if(isVideo)
        std::cout<<"input type is a video." <<std::endl;
    else {
        
        std::cout<<"input type is a sequence of image in " <<strInputFormat << " format."<<std::endl;
        
    }


    
    
    fSettings["timestamp_file"] >> strFrameFile;
    std::cout<<"get frame timestamp from "<< strFrameFile <<std::endl;

    fSettings["IMU"] >> strIMUFile;
    if(strIMUFile == "")
        std::cout<<"No IMU file added. ORB-SLAM will run in non-IMU mode." <<std::endl;
    else
        std::cout<<"get IMU file from "<< strIMUFile <<std::endl;

    fSettings["output_downward"] >> strDownOutput;
    std::cout<<"recording SLAM output from downward camera to "<< strDownOutput <<std::endl;

    fSettings["output_upward"] >> strUpOutput;
    std::cout<<"recording SLAM output from downward camera to "<< strUpOutput <<std::endl;

    fSettings["debug"] >> bDebug;
    if(bDebug)
        std::cout<<"Debug mode : ON "<<std::endl;
    else
        std::cout<<"Debug mode : OFF "<<std::endl;


}

