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
        
        bool success = ComputeTransformation();

        if(success)
        {
            camera_transform = camera_transform * incremental_transform.inverse();
            std::cout << camera_transform << std::endl << std::endl;
        }
        else
        {
            std::cout << "Failed" << std::endl;
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

    RemoveDepthlessMatches();

    cv::Mat img_matches;
    cv::drawMatches(current_rgb_frame, current_frame_keypoints, previous_rgb_frame, previous_frame_keypoints,
                    good_matched_points, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::imshow("Good Matches", img_matches);
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

bool visual_odometry::ComputeTransformation()
{   
    // select four point pairs from the pointclouds
    Eigen::MatrixXd points_current = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd points_previous = Eigen::MatrixXd::Zero(4, 4);

    // Solving the equation Ax = b to get the rigid body transformation
    Eigen::MatrixXd constraints_matrix = Eigen::MatrixXd::Zero(12, 12); // A

    // Solution of the linear problem
    // Initializing the containers
    Eigen::MatrixXd transformation = Eigen::MatrixXd::Zero(4, 4);
    Eigen::VectorXd transformation_as_vector = Eigen::VectorXd::Zero(12, 1);

    int number_of_iterations = 1000;

    for(int i = 0 ; i < number_of_iterations; i++)
    {
        //TODO: RANSAC to be added to remove outliers, for more robust estimation fo the transformation.
        TakeRandom3DPairs(points_current, points_previous, current_pointcloud, previous_pointcloud);

        // TODO: Replace this hard code with a for-loop 
        //Filling the constraint matrix
        constraints_matrix.block(0, 0, 1, 4) = points_previous.row(0);
        constraints_matrix.block(1, 4, 1, 4) = points_previous.row(0);
        constraints_matrix.block(2, 8, 1, 4) = points_previous.row(0);
        
        constraints_matrix.block(3, 0, 1, 4) = points_previous.row(1);
        constraints_matrix.block(4, 4, 1, 4) = points_previous.row(1);
        constraints_matrix.block(5, 8, 1, 4) = points_previous.row(1);

        constraints_matrix.block(6, 0, 1, 4) = points_previous.row(2);
        constraints_matrix.block(7, 4, 1, 4) = points_previous.row(2);
        constraints_matrix.block(8, 8, 1, 4) = points_previous.row(2);

        constraints_matrix.block(9, 0, 1, 4) = points_previous.row(3);
        constraints_matrix.block(10, 4, 1, 4) = points_previous.row(3);
        constraints_matrix.block(11, 8, 1, 4) = points_previous.row(3);

        // TODO: Replace this hard code with a for-loop
        //Filling the current vector
        Eigen::VectorXd vector_after_transformation = Eigen::VectorXd::Zero(12, 1);
        vector_after_transformation.segment(0, 3) = points_current.block(0, 0, 3, 1);
        vector_after_transformation.segment(3, 3) = points_current.block(0, 1, 3, 1);
        vector_after_transformation.segment(6, 3) = points_current.block(0, 2, 3, 1);
        vector_after_transformation.segment(9, 3) = points_current.block(0, 3, 3, 1);

        // Check whether the constraint matrix is singular or not, to avoid an exception.
        // Constraint matrix is only singular if two of the points are collinear
        if(!CheckDeterminantValue(constraints_matrix, 0, 1e-2))
        {
            Eigen::MatrixXd inverse_of_constraints = constraints_matrix.inverse();
            transformation_as_vector = inverse_of_constraints * vector_after_transformation;
            
            // TODO: Make this a for-loop or a function, first search for a reshape function in Eigen.
            transformation(0,0) = transformation_as_vector(0);
            transformation(0,1) = transformation_as_vector(1);
            transformation(0,2) = transformation_as_vector(2);
            transformation(0,3) = transformation_as_vector(3);
            
            transformation(1,0) = transformation_as_vector(4);
            transformation(1,1) = transformation_as_vector(5);
            transformation(1,2) = transformation_as_vector(6);
            transformation(1,3) = transformation_as_vector(7);
            
            transformation(2,0) = transformation_as_vector(8);
            transformation(2,1) = transformation_as_vector(9);
            transformation(2,2) = transformation_as_vector(10);
            transformation(2,3) = transformation_as_vector(11);
            
            transformation(3,0) = 0;
            transformation(3,1) = 0;
            transformation(3,2) = 0;
            transformation(3,3) = 1;

            if(CheckIfSO3(transformation.block(0, 0, 3, 3), 1e-1))
            {
                //std::cout << "transformation is SO(3)" << std::endl;
                incremental_transform = transformation;
                return true;
            }
        }

    }
    return false;   
}

bool visual_odometry::CheckDeterminantValue(Eigen::MatrixXd input_matrix, double target_value, double epsilon)
{
    double determinant = input_matrix.determinant();
    double delta = determinant - target_value;
    if(fabs(delta) <= fabs(epsilon) && !std::isnan(determinant))
    {
        return true;
    }

    return false;
}

bool visual_odometry::CheckIfSO3(Eigen::MatrixXd matrix, double epsilon)
{
    if(matrix.rows() != 3 || matrix.cols() != 3)
    {
        return false;
    }
    
    if(!CheckDeterminantValue(matrix, 1, epsilon))
    {
        return false;
    }

    Eigen::MatrixXd diffference = matrix * matrix.transpose() - Eigen::MatrixXd::Identity(3, 3);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            if(fabs(diffference(i, j)) >= fabs(epsilon))
            {
                return false;
            }    
        }
    }

    return true;
}


// TODO: Fix this to be generic, now it will break with any dimension other than 4
void visual_odometry::TakeRandom3DPairs(Eigen::MatrixXd& points_current, 
                                        Eigen::MatrixXd& points_previous,
                                        const std::vector<cv::Point3f>& current_pointcloud, 
                                        const std::vector<cv::Point3f>& previous_pointcloud)
{
    for(size_t i = 0; i < points_current.cols(); i++)
    {
        int random_number = std::rand() % (current_pointcloud.size() + 1) ;

        points_current(0, i) = current_pointcloud[random_number].x;
        points_current(1, i) = current_pointcloud[random_number].y;
        points_current(2, i) = current_pointcloud[random_number].z;
        points_current(3, i) = 1; // Homogeneous representation factor

        points_previous(i, 0) = previous_pointcloud[random_number].x;
        points_previous(i, 1) = previous_pointcloud[random_number].y;
        points_previous(i, 2) = previous_pointcloud[random_number].z;
        points_previous(i, 3) = 1; // Homogeneous representation factor
        
    }
}

}