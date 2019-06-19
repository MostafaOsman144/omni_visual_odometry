#include "visual_odometry.h"

namespace omni_visual_odometry
{
visual_odometry::visual_odometry(int threshold_input)
:threshold(threshold_input)
{
    orb_detector = cv::ORB::create();
    orb_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    if(threshold > 1 || threshold < 0)
    {
        std::cerr << "The value used for the threshold is incorrect, please use a value between 0 and 1" << std::endl;
    }
}

visual_odometry::~visual_odometry()
{
    
}

void visual_odometry::ComputeOdometry(cv::Mat& rgb_image, cv::Mat& depth_image)
{
    if(first_frame)
    {
        previous_rgb_frame = rgb_image;
        previous_d_frame = depth_image;

        orb_detector->detectAndCompute(previous_rgb_frame, cv::noArray(), previous_frame_keypoints, previous_frame_descriptor);
        first_frame = false;

    }
    else
    {
        current_rgb_frame = rgb_image;
        current_d_frame = depth_image;

        ComputeMatchedFeatures();

        MakeCurrentPrevious();

    }

}

void visual_odometry::ComputeMatchedFeatures()
{
    orb_detector->detectAndCompute(current_rgb_frame, cv::noArray(), current_frame_keypoints, current_frame_descriptor);
    orb_matcher->knnMatch(current_frame_descriptor, previous_frame_descriptor, matched_points, 2);

    if(!matched_points.empty())
    {
        for(size_t i = 0; i < matched_points.size(); i++)
        {
            //if(matched_points[i][0].distance < threshold*matched_points[i][1].distance)
            //{
                good_matched_points.push_back(matched_points[i][0]);
                matched_points_current.push_back(current_frame_keypoints[good_matched_points[i].queryIdx]);
                matched_points_previous.push_back(previous_frame_keypoints[good_matched_points[i].trainIdx]);
            //}

        }
    }

    if(good_matched_points.empty())
    {
        std::cout << "failed" << std::endl;
    }

    //cv::Mat featuresShowing;
    //cv::drawKeypoints(current_rgb_frame, current_frame_keypoints, featuresShowing);

    cv::Mat img_matches;
    cv::drawMatches(current_rgb_frame, current_frame_keypoints, previous_rgb_frame, previous_frame_keypoints,
                    good_matched_points, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::imshow("Good Matches", img_matches);

    //cv::imshow("Features",featuresShowing);
    cv::waitKey(10);


}

void visual_odometry::MakeCurrentPrevious()
{
    previous_rgb_frame = current_rgb_frame;
    previous_d_frame = current_d_frame;

    previous_frame_descriptor = current_frame_descriptor;
    previous_frame_keypoints = current_frame_keypoints;

    matched_points.clear();
    good_matched_points.clear();

}

}



