#include<RGBDVisualOdometryNode.h>

void visualOdometryCallback(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr& d_map)
{
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(rgb_image);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(d_map);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_visual_odometry");
    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_subscriber(node_handler);

    // Subscribes to the RGB-D Camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy; 
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/rgb_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> d_sub(node_handler, "depth_image", 1); 
    message_filters::Synchronizer<sync_policy> sync(sync_policy(10), rgb_sub, d_sub);
    sync.registerCallback(boost::bind(&visualOdometryCallback, _1, _2));

    // Read the Camera Parameter file
    

    ros::spin();
    ros::shutdown();
    
    return 0;
}

VisualOdometry::VisualOdometry()
{

}