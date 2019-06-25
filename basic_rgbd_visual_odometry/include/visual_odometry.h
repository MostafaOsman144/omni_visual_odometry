#ifndef _VISUAL_ODOMETRY_H_
#define _VISUAL_ODOMETRY_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include"visual_odometry_helpers.h"

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

    // initialize a visual_odometry_helper object
    visual_odometry_helpers helper_class;

    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_32F);

    cv::Mat incremental_rot;
    cv::Mat incremental_trans;


    Eigen::MatrixXd camera_transform = Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd incremental_transform;

public:
    visual_odometry(int);
    ~visual_odometry();

    // Class Functionality starts at this function
    void ComputeOdometry(cv::Mat& rgb_image, cv::Mat& depth_image, Eigen::MatrixXd& camera_transform_out);
    
    // Computes the Matches between the current and previous frames, only keeps those which have depth values
    void ComputeMatchedFeatures();

    // Make current frame previous frame, remove all the data from this frame
    void TransitionToNextTimeStep();
    void TransitionToNextTimeStepKeepFrame();

    // COmputes the 3D position of the matched points in the current and previous frames
    void ComputePointCloud(const cv::Mat&, const std::vector<cv::KeyPoint>&, std::vector<cv::Point3f>&);

    // Sets the intrinsic parameters for the camera which is being used
    void SetIntrinsicParams(double cx, double cy, double fx, double fy);

    // First frame initialization, before the visual odometry process starts
    void InitializeFirstFrame();
};

}
#endif