#ifndef RGBD_VISUAL_ODOMETRY_NODE_H_
#define RGBD_VISUAL_ODOMETRY_NODE_H_

#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>

#include<message_filters/subscriber.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<message_filters/synchronizer.h>

#include<cv_bridge/cv_bridge.h>
#include<opencv-3.3.1-dev/opencv2/core.hpp>

class VisualOdometry
{
    public:
        VisualOdometry();
        ~VisualOdometry();
};


#endif