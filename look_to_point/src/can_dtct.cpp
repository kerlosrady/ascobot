/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */

/**
 * @file
 *
 * @brief example on how to subscribe to an image topic and how to make the robot look towards a given direction
 *
 * How to test this application:
 *
 * 1) Launch the application:
 *
 *   $ rosrun tiago_tutorials look_to_point
 *
 * 2) Click on image pixels to make TIAGo look towards that direction
 *
 */

// C++ standard headers
#include <exception>
#include <string>
#include <array>
#include <iostream>
#include <math.h>

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

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

using namespace cv;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string graywindowName  = "Gray Image";
static const std::string cameraFrame     = "/xtion_rgb_optical_frame";   
static const std::string imageTopic      = "/xtion/rgb/image_raw";
static const std::string depthImageTopic = "/xtion/depth/image_raw";
static const std::string cameraInfoTopic = "/xtion/rgb/camera_info";

// Camera images
cv_bridge::CvImagePtr cvImgPtr;
sensor_msgs::ImageConstPtr depthImg;
// Intrinsic parameters of the camera
cv::Mat cameraIntrinsics;
//Processing images
cv::Mat grayImg;
cv::Mat medianImg;
cv::Mat cannyOutput;
cv::Mat output;
cv::Mat h;
cv::Mat g;
cv::Mat fil;

ros::Time latestImageStamp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// My Function of detecting the top of cans


int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{
    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -1;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4) 
    {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) || ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) 
        { 
          for (i = 0; i < 4; i++)
              depth_data.byte_data[i] = depth_image->data[index + i];

          if (depth_data.float_data == depth_data.float_data)
              return int(depth_data.float_data*1000);

          return -1;  // If depth data invalid
        }

        // else, one little endian, one big endian
        for (i = 0; i < 4; i++) 
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
   int temp_val;
   // If big endian
   if (depth_image->is_bigendian)
       temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
   // If little endian
   else
       temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
   // Make sure data is valid (check if NaN)
   if (temp_val == temp_val)
       return temp_val;
   return -1;  // If depth data invalid
}

void detectcircles (cv::Mat img, sensor_msgs::ImageConstPtr ros_img)
{
  
  //Covert to gray image
  cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY,2);

  // //Apply Median Filter to eliminate noise 
  cv::medianBlur(grayImg,medianImg,19);
  // cv::imshow("medianImg",medianImg);
  cv::threshold(medianImg,medianImg,120,255,cv::THRESH_TOZERO);
  // cv::imshow("medianImgafter",medianImg);

  //Contour Detection
  cv::Canny(medianImg,cannyOutput,90,120,3,0);
  cv::imshow("Canny",cannyOutput);


  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(cannyOutput,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::RotatedRect> minRect( contours.size() );

  output = img;

  int centerX [contours.size()];
  int centerY [contours.size()];
  double Co_x [contours.size()];
  double Co_y [contours.size()];
  double Co_z [contours.size()]; 

  for( size_t i = 0; i< contours.size(); i++ )
  {
    //Apply minAreaRect function to get the fitted rectangles for each contour
    minRect[i] = cv::minAreaRect( contours[i] );
    cv::Point2f rect_points[4];
    minRect[i].points( rect_points );

    // Filter contours by their length not to get small contours(noisy contours)

      //Get the center of fitted recttangles
      centerX[i] = (rect_points[0].x + rect_points[2].x)/2;
      centerY[i] = (rect_points[0].y + rect_points[2].y)/2;
      cv::Point2f a(centerX[i],centerY[i]);
      circle( img, a, 1, Scalar(0,100,100), 3, LINE_AA);
      // putText(g, to_string(centerX[i]),a , FONT_HERSHEY_DUPLEX,1, Scalar(0,143,143), 1);
      // putText(h, to_string(centerY[i]),a , FONT_HERSHEY_DUPLEX,1, Scalar(0,143,143), 1);
    
      geometry_msgs::PointStamped pointStamped;
      pointStamped.header.frame_id = cameraFrame;
      pointStamped.header.stamp    = latestImageStamp;

      //compute normalized coordinates of the selected pixel
      Co_x[i] = ( centerX[i]  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
      Co_y[i] = ( centerY[i]  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
      ROS_INFO("[%d,%d,%d,%d]", cameraIntrinsics.at<double>(0,2),cameraIntrinsics.at<double>(0,0),cameraIntrinsics.at<double>(1,2),cameraIntrinsics.at<double>(1,1));

      float temp_z = ReadDepthData(centerX[i] , centerY[i], ros_img);
      ROS_INFO("[%d]",temp_z);
      if (temp_z == -1 )
         Co_z[i] = 1; 
      else
        Co_z[i] =temp_z;

      pointStamped.point.x = Co_x[i] * Co_z[i];
      pointStamped.point.y = Co_y[i] * Co_z[i];
      pointStamped.point.z = Co_z[i];   

  }
  cv::imshow("FINAL",img);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS call back for every new image received
void imageCallback1(const sensor_msgs::ImageConstPtr& imgMsg)
{
  ROS_INFO_STREAM("Entering Callback1");
  latestImageStamp = imgMsg->header.stamp;
  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  ROS_INFO_STREAM("Exiting Callback1");
}

void imageCallback2(const sensor_msgs::ImageConstPtr& image) 
{
  ROS_INFO_STREAM("Entering callback2");
  depthImg = image;
  detectcircles(cvImgPtr->image,depthImg);
  ROS_INFO_STREAM("Exiting callback2");

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
 ros::NodeHandle n;
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

  // Define ROS topic from where TIAGo publishes images
  
  image_transport::ImageTransport it1(nh);
  image_transport::ImageTransport it2(n);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
  image_transport::Subscriber sub1 = it1.subscribe(imageTopic, 1,
                                                 imageCallback1);
  
  ROS_INFO_STREAM("Subscribing to " << depthImageTopic << " ...");
  image_transport::Subscriber sub2 = it2.subscribe(depthImageTopic, 1, imageCallback2);
  ROS_INFO_STREAM("Done Subscribing");

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  return EXIT_SUCCESS;
}
