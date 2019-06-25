#ifndef ODOMETRY_NODE_RGBD_NODE_H_
#define ODOMETRY_NODE_RGBD_NODE_H_

#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>

#include<message_filters/subscriber.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<message_filters/synchronizer.h>

#include<cv_bridge/cv_bridge.h>
#include<opencv-3.3.1-dev/opencv2/opencv.hpp>

#include<eigen3/Eigen/Dense>

#include"visual_odometry.h"

class OdometryNodeRGBD
{
    private:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> rgb_sub;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> d_sub;
        std::unique_ptr<message_filters::Synchronizer<sync_policy>> sync;

        omni_visual_odometry::visual_odometry* rgbdOdometryObject;

        ros::NodeHandle* node_handle;
        ros::NodeHandle* private_node_handle;

        ros::Publisher odometry_publisher;

    public:
        OdometryNodeRGBD(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle,
                         omni_visual_odometry::visual_odometry*);
        ~OdometryNodeRGBD();

        void ImagesCallbackFunction(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);

        void ReadIntrinicsFromParamterFile();

        void PublishOdometry(Eigen::MatrixXd&);
};


#endif