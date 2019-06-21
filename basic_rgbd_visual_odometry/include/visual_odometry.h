#ifndef _VISUAL_ODOMETRY_H_
#define _VISUAL_ODOMETRY_H_

#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace omni_visual_odometry
{

class visual_odometry
{
private:
    struct CameraIntrinsics
    {
        double cx; //x posiiton of the optical center
        double cy; //y position of the optical center
        double fx; //focal length in x direction
        double fy; //focal length in y direction

    };
    
    cv::Mat current_rgb_frame;
    cv::Mat current_d_frame;
    cv::Mat previous_rgb_frame;
    cv::Mat previous_d_frame;
    bool first_frame = true;

    // ORB Object initialization
    cv::Ptr<cv::ORB> orb_detector;

    // Feature Matcher
    cv::Ptr<cv::DescriptorMatcher> orb_matcher;

    // Initializing the old image descriptor and keypoint vector
    std::vector<cv::KeyPoint> previous_frame_keypoints;
    cv::Mat previous_frame_descriptor;

    //Initializing the new image descriptor and keypoint vector
    std::vector<cv::KeyPoint> current_frame_keypoints;
    cv::Mat current_frame_descriptor;

    // Initializing matched points container
    std::vector<std::vector<cv::DMatch>> matched_points;
    
    //Initializing container for filtered matched points (filtering will be done as in lowe's paper through the use of threshold)
    std::vector<cv::DMatch> good_matched_points;
    std::vector<cv::KeyPoint> matched_points_current;
    std::vector<cv::KeyPoint> matched_points_previous;

    // Lowe's filtering threshold
    const float threshold;

    // PointClouds for previous and current frames
    std::vector<cv::Point3f> current_pointcloud;
    std::vector<cv::Point3f> previous_pointcloud;
    
    CameraIntrinsics rgbd_camera_intrinsics;

public:
    visual_odometry(int);
    ~visual_odometry();
    void ComputeOdometry(cv::Mat& rgb_image, cv::Mat& depth_image);
    void ComputeMatchedFeatures();
    void TransitionToNextTimeStep();
    void ComputePointCloud(const cv::Mat&, const std::vector<cv::KeyPoint>&, std::vector<cv::Point3f>&);
    void ComputePreviousPointCloud();
    void ComputeCurrentPointCloud();
    void SetIntrinsicParams(double cx, double cy, double fx, double fy);
    void InitializeFirstFrame();
    void RemoveDepthlessMatches();

};

}
#endif