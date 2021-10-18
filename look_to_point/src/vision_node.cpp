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


using namespace cv;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string originalwindowName      = "Inside of TIAGo's head";
static const std::string graywindowName      = "Gray Image";
static const std::string cameraFrame     = "/xtion_rgb_optical_frame";   //will be important later 
static const std::string imageTopic      = "/xtion/rgb/image_raw";
static const std::string cameraInfoTopic = "/xtion/rgb/camera_info";

// Intrinsic parameters of the camera
cv::Mat cameraIntrinsics;
cv::Mat grayImg;
cv::Mat medianImg;
cv::Mat cannyOutput;
cv::Mat output;

ros::Time latestImageStamp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// My Function of detecting the top of cans
void detectcircles (cv::Mat img)
{
  //Covert to gray image
  cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);

  // //Apply Median Filter to eliminate noise 
  cv::medianBlur(grayImg,medianImg,3);

  //Contour Detection
  cv::Canny(medianImg,cannyOutput,60,180,3,0);
  cv::imshow("Canny",cannyOutput);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(cannyOutput,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::RotatedRect> minRect( contours.size() );

  int counter = 1;
  output = img;
  for( size_t i = 0; i< contours.size(); i++ )
  {
    //Apply minAreaRect function to get the fitted rectangles for each contour
    minRect[i] = cv::minAreaRect( contours[i] );
    cv::Point2f rect_points[4];
    minRect[i].points( rect_points );


    // Filter contours by their length not to get small contours(noisy contours)
    if(contours[i].size()>30)
    {
      //Get the center of fitted recttangles
      int centerX = (rect_points[0].x + rect_points[2].x)/2;
      int centerY = (rect_points[0].y + rect_points[2].y)/2;
      cv::Point2f a(centerX,centerY);

      int sum = 0;
      std::vector<int> storeLength;
      for(int j=0; j<(int)contours[i].size(); j++)
      {
          cv::Point2f b(contours[i][j].x,contours[i][j].y);
          int res = cv::norm(cv::Mat(a),cv::Mat(b));
          sum += res;
          storeLength.push_back(res);
      }
      int meanLength = sum / (int)storeLength.size();

        for ( int j = 0; j < 4; j++ )
        {
            line( output, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255),3 );
            cv::putText(output,std::to_string(centerX,centerY),cv::Point(centerX,centerY),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,255,255),3);
        }
        counter++;
        
    }
      imshow("detected circles", output);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  latestImageStamp = imgMsg->header.stamp;
  cv_bridge::CvImagePtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  cv::imshow(originalwindowName, cvImgPtr->image);
  detectcircles(cvImgPtr->image);
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
  cv::namedWindow(originalwindowName, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(graywindowName, cv::WINDOW_AUTOSIZE);

  // Define ROS topic from where TIAGo publishes images
  
  image_transport::ImageTransport it(nh);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
  image_transport::Subscriber sub = it.subscribe(imageTopic, 1,
                                                 imageCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  cv::destroyWindow(originalwindowName);
  cv::destroyWindow(graywindowName);


  return EXIT_SUCCESS;
}
