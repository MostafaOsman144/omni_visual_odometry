#include"visual_odometry_helpers.h"

namespace omni_visual_odometry
{

visual_odometry_helpers::visual_odometry_helpers()
{
}

visual_odometry_helpers::~visual_odometry_helpers()
{
}

bool visual_odometry_helpers::CheckDeterminantValue(Eigen::MatrixXd input_matrix, double target_value, double epsilon)
{
    double determinant = input_matrix.determinant();
    double delta = determinant - target_value;
    if(fabs(delta) <= fabs(epsilon) && !std::isnan(determinant))
    {
        return true;
    }

    return false;
}

bool visual_odometry_helpers::CheckIfSO3(Eigen::MatrixXd matrix, double epsilon)
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
void visual_odometry_helpers::TakeRandom3DPairs(Eigen::MatrixXd& points_current, 
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

void visual_odometry_helpers::RemoveDepthlessMatches(std::vector<cv::DMatch>& good_matched_points,
                                                     std::vector<cv::KeyPoint>& matched_points_current,
                                                     std::vector<cv::KeyPoint>& matched_points_previous,
                                                     cv::Mat& current_d_frame,
                                                     cv::Mat& previous_d_frame)
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

}