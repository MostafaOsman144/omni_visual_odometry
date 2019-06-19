#ifndef _VISUAL_ODOMETRY_H_
#define _VISUAL_ODOMETRY_H_

#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <vector>
#include <iostream>

namespace omni_visual_odometry
{

class visual_odometry
{
private:
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

    // Lowe's filtering threshold
    const float threshold;

    

public:
    visual_odometry(int);
    ~visual_odometry();
    void ComputeOdometry(cv::Mat& rgb_image, cv::Mat& depth_image);
    void ComputeMatchedFeatures();
    void MakeCurrentPrevious();
};

}
#endif