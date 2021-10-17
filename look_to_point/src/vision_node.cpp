// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string windowName      = "Inside of TIAGo's head";
// static const std::string cameraFrame     = "/xtion_rgb_optical_frame";   #will be important later 
static const std::string imageTopic      = "/xtion/rgb/image_raw";
static const std::string cameraInfoTopic = "/xtion/rgb/camera_info";

// Intrinsic parameters of the camera
cv::Mat cameraIntrinsics;

ros::Time latestImageStamp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  latestImageStamp = imgMsg->header.stamp;
  cv_bridge::CvImagePtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  cv::imshow(windowName, cvImgPtr->image);
  cv::waitKey(15);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "Vision");

  ROS_INFO("Starting Vision application ...");
 
 //1st NodeHandle does the initialization,last one will cleanup any resources the node was using.   
 
 ros::NodeHandle nh;

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Get the camera intrinsic parameters from the appropriate ROS topic
  ROS_INFO("Waiting for camera intrinsics ... ");
  sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage
      <sensor_msgs::CameraInfo>(cameraInfoTopic, ros::Duration(10.0));
  if(msg.use_count() > 0)
  {
    cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
    cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
    cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
    cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
    cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
    cameraIntrinsics.at<double>(2, 2) = 1;
  }

  // Create the window to show TIAGo's camera images
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

  // Define ROS topic from where TIAGo publishes images
  
  image_transport::ImageTransport it(nh);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
  image_transport::Subscriber sub = it.subscribe(imageTopic, 1,
                                                 imageCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  cv::destroyWindow(windowName);

  return EXIT_SUCCESS;s
}
