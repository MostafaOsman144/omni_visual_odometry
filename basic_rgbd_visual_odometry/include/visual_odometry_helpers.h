#ifndef _VISUAL_ODOMETRY_HELPERS_H_
#define _VISUAL_ODOMETRY_HELPERS_H_

#include <eigen3/Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>

namespace omni_visual_odometry
{

class visual_odometry_helpers
{   
public:
    visual_odometry_helpers(/* args */);
    ~visual_odometry_helpers();

    void RemoveDepthlessMatches(std::vector<cv::DMatch>& good_matched_points,
                                                        std::vector<cv::KeyPoint>& matched_points_current,
                                                        std::vector<cv::KeyPoint>& matched_points_previous,
                                                        cv::Mat& current_d_frame,
                                                        cv::Mat& previous_d_frame);

    // Checks whether the determinant of a matrix is of a given value
    bool CheckDeterminantValue(Eigen::MatrixXd input_matrix, double target_value, double epsilon);

    // Select random 3D points from both current and previous pointclouds and return them in points_current 
    //and points_previous
    void TakeRandom3DPairs(Eigen::MatrixXd& points_current, 
                                        Eigen::MatrixXd& points_previous,
                                        const std::vector<cv::Point3f>& current_pointcloud, 
                                        const std::vector<cv::Point3f>& previous_pointcloud);

    // This function checks whether or not a given matrix is an element of the SO(3) group
    bool CheckIfSO3(Eigen::MatrixXd matrix, double epsilon);

    Eigen::MatrixXd Convert4x4FromMatToEigen(cv::Mat& input);

    void buildTransformationMatFromRotAndTrans(const cv::Mat& rot, const cv::Mat& trans, cv::Mat& output);
};

}

#endif