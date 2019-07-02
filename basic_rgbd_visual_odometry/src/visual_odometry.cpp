#include "visual_odometry.h"

namespace omni_visual_odometry
{
visual_odometry::visual_odometry(int threshold_input)
:threshold(threshold_input)
{
    orb_detector = cv::ORB::create();
    orb_detector->setScaleFactor(1.5f);
    orb_detector->setMaxFeatures(2000);
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

void visual_odometry::ComputeOdometry(cv::Mat& rgb_image, cv::Mat& depth_image, Eigen::MatrixXd& camera_transform_out)
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
        if(!(matched_points_previous.size() <= 4) && !(matched_points_current.size() <= 4))
        {   
            std::vector<cv::Point2f> matched_current;
            for(int i = 0; i < matched_points_current.size(); i++)
            {
                matched_current.push_back(matched_points_current[i].pt);
            }

            ComputePointCloud(previous_d_frame, matched_points_previous, previous_pointcloud);
            
            if(previous_pointcloud.size() == matched_current.size())
            {
                cv::solvePnPRansac(previous_pointcloud, matched_current, camera_matrix, cv::noArray(), 
                               incremental_rot, incremental_trans, false, 2000, 0.1, 0.99,cv::noArray(), CV_P3P);
                
                cv::Mat rotation_matrix = cv::Mat::zeros(3,3, CV_32F);
                cv::Rodrigues(incremental_rot, rotation_matrix);

                Eigen::MatrixXd increment = Eigen::MatrixXd::Zero(4,4);
                cv::Mat increment_cv = cv::Mat::zeros(4,4,CV_32F);
                helper_class.buildTransformationMatFromRotAndTrans(rotation_matrix, incremental_trans, increment_cv);
                
                increment = helper_class.Convert4x4FromMatToEigen(increment_cv);
                
                incremental_transform = increment.inverse();
                camera_transform = camera_transform * incremental_transform;

                camera_transform_out = camera_transform;
            }
            else
            {
                camera_transform_out = camera_transform;
            }
            

        }
        else
        {
            std::cout << "No Matching points are there to compute the 3D positions" << std::endl;
            std::cout << std::endl;
        }

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
                good_matched_points.push_back(matched_points[i][0]);
                matched_points_current.push_back(current_frame_keypoints[good_matched_points[i].queryIdx]);
                matched_points_previous.push_back(previous_frame_keypoints[good_matched_points[i].trainIdx]);

        }
    }

    if(good_matched_points.empty())
    {
        std::cout << "Could not find good matches in this two consecutive frames" << std::endl;
        std::cout << std::endl;
    }

    helper_class.RemoveDepthlessMatches(good_matched_points, 
                                        matched_points_current,
                                        matched_points_previous,
                                        current_d_frame,
                                        previous_d_frame);

    cv::Mat img_matches;
    cv::drawMatches(current_rgb_frame, current_frame_keypoints, previous_rgb_frame, previous_frame_keypoints,
                    good_matched_points, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::imshow("Good Matches", img_matches);
    cv::waitKey(10);
    cv::imshow("Depth Map", current_d_frame);
    cv::waitKey(10);

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

void visual_odometry::TransitionToNextTimeStepKeepFrame()
{   
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
            // It is very unlikely that we reach this code.
            std::cout << " Even after passing over RemoveDepthlessMatches,";
            std::cout << " there is still some keypoints which does not have depth " << std::endl;
            std::cout << std::endl;
        }
        
    }
}


void visual_odometry::SetIntrinsicParams(double cx, double cy, double fx, double fy, std::string camera_name)
{
    rgbd_camera_intrinsics.cx = cx;
    rgbd_camera_intrinsics.cy = cy;
    rgbd_camera_intrinsics.fx = fx;
    rgbd_camera_intrinsics.fy = fy;
    rgbd_camera_intrinsics.camera_name = camera_name;

    std::cout << "Reading the parameters of an " << camera_name << " camera" << std::endl;
    std::cout << "The intrinsics were defined successfully. The values are:" << std::endl;
    std::cout << "cx = " << rgbd_camera_intrinsics.cx << std::endl;
    std::cout << "cy = " << rgbd_camera_intrinsics.cy << std::endl;
    std::cout << "fx = " << rgbd_camera_intrinsics.fx << std::endl;
    std::cout << "fy = " << rgbd_camera_intrinsics.fy << std::endl;
    std::cout << std::endl; 

    camera_matrix.at<float>(0,0) = rgbd_camera_intrinsics.fx;
    camera_matrix.at<float>(1,1) = rgbd_camera_intrinsics.fy;
    camera_matrix.at<float>(0,2) = rgbd_camera_intrinsics.cx;
    camera_matrix.at<float>(1,2) = rgbd_camera_intrinsics.cy;

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

}