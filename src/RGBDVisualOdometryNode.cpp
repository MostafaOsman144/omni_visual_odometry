#include "RGBDVisualOdometryNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_visual_odometry");
    ros::NodeHandle node_handle;
    
    omni_visual_odometry::visual_odometry odometryObject;

    OdometryNodeRGBD rgbdOdometryNode(node_handle, &odometryObject);

    ros::spin();
    ros::shutdown();
    
    return 0;
}

/// The visualOdometry Class constructor, initializes the synchronized subscription to the rgbd camera through the 
/// use of the node_handle of the node.
OdometryNodeRGBD::OdometryNodeRGBD(ros::NodeHandle& node_handle, omni_visual_odometry::visual_odometry* odometryObject)
{
  rgbdOdometryObject = odometryObject;

  rgb_sub = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
    new  message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/camera/rgb/image_color", 1));
  
  d_sub = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
    new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/camera/depth/image", 1));
  
  sync = std::unique_ptr<message_filters::Synchronizer<sync_policy>>(
    new message_filters::Synchronizer<sync_policy>(sync_policy(10), *rgb_sub, *d_sub));
  
  sync->registerCallback(boost::bind(&OdometryNodeRGBD::ImagesCallbackFunction, this, _1, _2));
}

/// VisualOdometry Class destructor, not used here, all points are unique_ptrs
OdometryNodeRGBD::~OdometryNodeRGBD()
{
  delete rgbdOdometryObject;
}

/// The images callback function, here the images are received through from the image messages, and through cv_bridge the
/// images are converted opencv Mat instances so that further processing can be done through opencv.
void OdometryNodeRGBD::ImagesCallbackFunction(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr& d_map)
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

  cv::Mat cv_rgb_image = cv_ptrRGB->image;
  cv::Mat cv_depth_image = cv_ptrD->image;

  cv::namedWindow("Checking if Working");
  cv::imshow("Checking if Working",cv_rgb_image);
  cv::waitKey(10);

  GetOrbFeatures(cv_rgb_image, cv_depth_image);
}


void OdometryNodeRGBD::GetOrbFeatures(cv::Mat& rgb_image, cv::Mat& depth_image)
{
  cv::Ptr<cv::ORB> orb_detector = cv::ORB::create();

}