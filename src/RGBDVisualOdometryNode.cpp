#include "RGBDVisualOdometryNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_visual_odometry");
    ros::NodeHandle node_handle, private_node_handle("~");
    
    omni_visual_odometry::visual_odometry odometryObject(0.8f);

    OdometryNodeRGBD rgbdOdometryNode(node_handle, private_node_handle, &odometryObject);

    ros::spin();

    ros::shutdown();
  
    return 0;
}

/// The visualOdometry Class constructor, initializes the synchronized subscription to the rgbd camera through the 
/// use of the node_handle of the node.
OdometryNodeRGBD::OdometryNodeRGBD(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle,
                                   omni_visual_odometry::visual_odometry* odometryObject)
{
  rgbdOdometryObject = odometryObject;
  this->node_handle = &node_handle;
  this->private_node_handle = &private_node_handle;

  ReadIntrinicsFromParamterFile();

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
//  delete rgbdOdometryObject;
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

  cv::Mat cv_rgb_image, cv_depth_image;

  cv_rgb_image = cv_ptrRGB->image;
  cv_depth_image = cv_ptrD->image;

  cv::cvtColor(cv_rgb_image, cv_rgb_image, cv::COLOR_BGR2GRAY);

  //cv::namedWindow("Checking if Working");
  //cv::imshow("Checking if Working",cv_rgb_image);
  //cv::waitKey(10);

  rgbdOdometryObject->ComputeOdometry(cv_rgb_image, cv_depth_image);

}

void OdometryNodeRGBD::ReadIntrinicsFromParamterFile()
{ 
  double cx;
  double cy;
  double fx;
  double fy;

  if(!private_node_handle->getParam("cx", cx) ||
     !private_node_handle->getParam("cy", cy) ||
     !private_node_handle->getParam("fx", fx) ||
     !private_node_handle->getParam("fy", fy))
     {
       std::string error_message = ros::this_node::getName() + ": Failed to load the local parameters, please check the path";
       ROS_ERROR("%s \n", error_message.c_str());
       ros::shutdown();
       return;
     }

  rgbdOdometryObject->SetIntrinsicParams(cx, cy, fx, fy);

}