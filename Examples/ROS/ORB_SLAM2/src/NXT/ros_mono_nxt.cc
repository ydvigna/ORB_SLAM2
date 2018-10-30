/**
* This file is a modification of ros_mono.cc from ORB_SLAM2.
*
* For more information on ORB-SLAM2 see <https://github.com/raulmur/ORB_SLAM2>
*
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

// ROS headers in /opt/ros/melodic/include/
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV header
#include <opencv2/core/core.hpp>

// ORB_SLAM2 headers
#include "../../../include/System.h"
#include "ControllerNXT.h"

using namespace std;


ORB_SLAM2::ControllerNXT controllerNXT;

bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};


int main(int argc, char **argv)
{
    // Initialize ROS node (argc, argv, name of node)
    ros::init(argc, argv, "NXT");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 NXT path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // - bool bUseViewer : windows for map and current frame (DONT use with cv window from ControllerNXT)
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    controllerNXT.SetSLAM(&SLAM);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    // Initialize camera settings
    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    controllerNXT.SetFPS(fps);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    controllerNXT.SetCameraCalibration(fx,fy,cx,cy);

    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    // Create a new thread for navigation
    thread tController = thread(&ORB_SLAM2::ControllerNXT::Run,&controllerNXT);

    // Enter a loop processing callbacks, uses NodeHandle
    ros::spin();

    // Stop controller thread
    controllerNXT.RequestFinish();
    tController.join();

    // Stop all threads
    SLAM.Shutdown();
    cout << "SLAM SHUTDOWN" << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    ros::shutdown();
    cout << "ROS SHUTDOWN" << endl;

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat im = cv_ptr->image.clone();
    cv::Mat imu;
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    int state = mpSLAM->GetTrackingState();
    vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

    cv::undistort(im,imu,K,DistCoef);

    // if(!bRGB)
        cv::cvtColor(imu,imu,CV_BGR2RGB);
    controllerNXT.SetImagePose(imu,Tcw,state,vKeys,vMPs);
}

