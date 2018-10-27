/**
* This file is a controller for a Lego NXT that uses ORB-SLAM2 and OpenCV
* 
*/


#ifndef CONTROLLERNXT_H
#define CONTROLLERNXT_H

#include <string>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>
#include"../../../include/System.h"

namespace ORB_SLAM2
{

class Plane
{
public:
    Plane(const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcw);
    Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);

    void Recompute();

    //normal
    cv::Mat n;
    //origin
    cv::Mat o;
    //arbitrary orientation along normal
    float rang;
    //transformation from world to the plane
    cv::Mat Tpw;
    pangolin::OpenGlMatrix glTpw;
    //MapPoints that define the plane
    std::vector<MapPoint*> mvMPs;
    //camera pose when the plane was first observed (to compute normal direction)
    cv::Mat mTcw, XC;
};

class ControllerNXT
{
public:
    ControllerNXT();

    void SetFPS(const float fps){
        mFPS = fps;
        mT=1e3/fps;
    }

    void SetSLAM(ORB_SLAM2::System* pSystem){
        mpSystem = pSystem;
    }

    // Close bluetooth socket
    void CloseSocket();

    // Initialize bluetooth connection
    int InitBluetooth(char *btAddress);

    // Get battery level of the NXT
    int GetBattery();

    // Get distance and angle to obstacle from SLAM map
    int GetDistance();
    int GetAngle();

    // Motor control
    int SpeedController();
    int TurnController();

    // Test Viewer
    void MyViewer();

    // Main thread function. 
    void Run();

    // Thread control
    void RequestFinish();
    

    // OLD: Main thread function. 
    void OldRun();


    void SetCameraCalibration(const float &fx_, const float &fy_, const float &cx_, const float &cy_){
        fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    }

    void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status,
                      const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs);

    void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status,
                      std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs);

private:

    // Socket for the bluetooth connection
    int nxtSocket;

    // Set output state of the motors
    int SetOutput(unsigned char powerLM, unsigned char powerRM);

    // Thread control
    bool CheckFinish();
    bool mbFinishRequested;
    std::mutex mMutexFinish;

    //SLAM
    ORB_SLAM2::System* mpSystem;

    void PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im);
    void AddTextToImage(const std::string &s, cv::Mat &im, const int r=0, const int g=0, const int b=0);
    void LoadCameraPose(const cv::Mat &Tcw);
    void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im);
    void DrawCube(const float &size, const float x=0, const float y=0, const float z=0);
    void DrawPlane(int ndivs, float ndivsize);
    void DrawPlane(Plane* pPlane, int ndivs, float ndivsize);
    void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs, cv::Mat &im);

    Plane* DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations=50);

    // frame rate
    float mFPS, mT;
    float fx,fy,cx,cy;

    // Last processed image and computed pose by the SLAM
    std::mutex mMutexPoseImage;
    cv::Mat mTcw;
    cv::Mat mImage;
    int mStatus;
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<MapPoint*> mvMPs;

};


}


#endif // CONTROLLERNXT_H
	

