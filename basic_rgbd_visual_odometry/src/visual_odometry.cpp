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
        std::cout << std::endl;
    }

}

visual_odometry::~visual_odometry()
{
    
}

void visual_odometry::ComputeOdometry(cv::Mat& rgb_image, cv::Mat& depth_image)
{
    if(first_frame)
    {   
        previous_rgb_frame = rgb_image.clone();
        previous_d_frame = depth_image.clone();

        InitializeFirstFrame();

    }
    else
    {
        current_rgb_frame = rgb_image.clone();
        current_d_frame = depth_image.clone();

        // Step 1 -> Compute the feature matches between the two successive frames
        ComputeMatchedFeatures();

        // Step 2 -> Compute the 3D location of the matched points
        if(!matched_points_previous.empty() && !matched_points_current.empty())
        {
            ComputePointCloud(previous_d_frame, matched_points_previous, previous_pointcloud);
            ComputePointCloud(current_d_frame, matched_points_current, current_pointcloud);
        }
        else
        {
            std::cout << "No Matching points are there to compute the 3D positions" << std::endl;
            std::cout << std::endl;
        }
        
        ComputeTransformation();

        TransitionToNextTimeStep();

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
        std::cout << "Could not find good matches in this two consecutive frames" << std::endl;
        std::cout << std::endl;
    }

    //cv::Mat featuresShowing;
    //cv::drawKeypoints(current_rgb_frame, current_frame_keypoints, featuresShowing);

    RemoveDepthlessMatches();

    cv::Mat img_matches;
    cv::drawMatches(current_rgb_frame, current_frame_keypoints, previous_rgb_frame, previous_frame_keypoints,
                    good_matched_points, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::imshow("Good Matches", img_matches);

    //cv::imshow("Features",featuresShowing);
    cv::waitKey(10);

}

void visual_odometry::RemoveDepthlessMatches()
{   
    std::vector<cv::KeyPoint> previous_points_with_depth;
    std::vector<cv::KeyPoint> current_points_with_depth;
    std::vector<cv::DMatch> matches_with_depth;

    for(size_t i = 0; i < matched_points_current.size(); i++)
    {   
        cv::Point2f previous_position = matched_points_previous[i].pt;
        cv::Point2f current_position = matched_points_current[i].pt;

        if(!std::isnan(current_d_frame.at<float>(current_position)) && 
           !std::isnan(previous_d_frame.at<float>(previous_position)) &&
            current_d_frame.at<float>(current_position) != 0 && 
            previous_d_frame.at<float>(previous_position) != 0)
        {
           previous_points_with_depth.push_back(matched_points_previous[i]);
           current_points_with_depth.push_back(matched_points_current[i]); 
           matches_with_depth.push_back(good_matched_points[i]);
        }
    }

    matched_points_previous = previous_points_with_depth;
    matched_points_current = current_points_with_depth;
    good_matched_points = matches_with_depth;
}

void visual_odometry::TransitionToNextTimeStep()
{
    previous_rgb_frame = current_rgb_frame;
    previous_d_frame = current_d_frame;
    previous_frame_descriptor = current_frame_descriptor;
    previous_frame_keypoints = current_frame_keypoints;
    
    matched_points.clear();
    good_matched_points.clear();

    matched_points_current.clear();
    matched_points_previous.clear();

    current_pointcloud.clear();
    previous_pointcloud.clear();

}

void visual_odometry::ComputePointCloud(const cv::Mat& depth_image, const std::vector<cv::KeyPoint>& matched_keypoints,
                                        std::vector<cv::Point3f>& point_cloud)
{
    for(size_t i = 0; i < matched_keypoints.size(); i++)
    {   
        double z = depth_image.at<float>(matched_keypoints[i].pt);
        if(z != 0 && !std::isnan(z))
        {
            double x = (matched_keypoints[i].pt.x - rgbd_camera_intrinsics.cx) * z / rgbd_camera_intrinsics.fx;
            double y = (matched_keypoints[i].pt.y - rgbd_camera_intrinsics.cy) * z / rgbd_camera_intrinsics.fy;

            point_cloud.push_back(cv::Point3f(x, y, z));
        }
        else
        {
            std::cout << "Why is that ?? " << std::endl;
        }
        
    }
}


void visual_odometry::SetIntrinsicParams(double cx, double cy, double fx, double fy)
{
    rgbd_camera_intrinsics.cx = cx;
    rgbd_camera_intrinsics.cy = cy;
    rgbd_camera_intrinsics.fx = fx;
    rgbd_camera_intrinsics.fy = fy;

    std::cout << "The intrinsics were defined successfully. The values are:" << std::endl;
    std::cout << "cx = " << rgbd_camera_intrinsics.cx << std::endl;
    std::cout << "cy = " << rgbd_camera_intrinsics.cy << std::endl;
    std::cout << "fx = " << rgbd_camera_intrinsics.fx << std::endl;
    std::cout << "fy = " << rgbd_camera_intrinsics.fy << std::endl;
    std::cout << std::endl;

}

void visual_odometry::InitializeFirstFrame()
{
    orb_detector->detectAndCompute(previous_rgb_frame, cv::noArray(), previous_frame_keypoints, previous_frame_descriptor);

    if(previous_frame_keypoints.empty())
    {
        std::cerr << "Failed to initialize, no keypoints were found." << std::endl;
        std::cout << std::endl;
    }

    for(size_t i = 0; i < previous_frame_keypoints.size(); i++)
    {
        double z = previous_d_frame.at<float>(previous_frame_keypoints[i].pt);
        
        if(z != 0 || std::isnan(z))
        {
            double x = (previous_frame_keypoints[i].pt.x - rgbd_camera_intrinsics.cx) * z / rgbd_camera_intrinsics.fx;
            double y = (previous_frame_keypoints[i].pt.y - rgbd_camera_intrinsics.cy) * z / rgbd_camera_intrinsics.fy;

            previous_pointcloud.push_back(cv::Point3f(x, y, z));
        }
        
    }
    
    if(!previous_pointcloud.empty())
    {
        std::cout << "Visual Odometry was Initialized successfully" << std::endl;
        std::cout << std::endl;
        first_frame = false;

    }

}

void visual_odometry::ComputeTransformation()
{
    cv::Mat transformation, inlier_points;

    //cv::estimateAffine3D(previous_pointcloud, current_pointcloud, transformation, inlier_points);
}

}