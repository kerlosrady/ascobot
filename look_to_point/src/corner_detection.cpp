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
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
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
using namespace sensor_msgs;
using namespace message_filters;

typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string imageTopic      = "/xtion/rgb/image_raw";
static const std::string depthImageTopic = "/xtion/depth_registered/image_raw";

// Camera images
cv_bridge::CvImagePtr cvImgPtr1;
cv_bridge::CvImagePtr cvImgPtr2;
// Intrinsic parameters of the camera

ros::Time latestImageStamp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// My Function of detecting the top of cans

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS call back for every new image received
void callback(const sensor_msgs::ImageConstPtr& imgMsg, const sensor_msgs::ImageConstPtr& depthImgMsg) 
{
  latestImageStamp = imgMsg->header.stamp;
  cvImgPtr1 = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  cvImgPtr2 = cv_bridge::toCvCopy(depthImgMsg, sensor_msgs::image_encodings::32FC1);
  cv::imshow("RGB",cvImgPtr1);
  cv::imshow("Depth",cvImgPtr2);
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
 topic from where TIAGo publishes images
  // use compressed image transport to use less network bandwidth
  ROS_INFO_STREAM("Subscribing ");

  message_filters::Subscriber<Image> image_sub(nh,imageTopic, 1);
  message_filters::Subscriber<Image> depth_sub(nh,depthImageTopic, 1);
  TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> sync(image_sub, depth_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ROS_INFO_STREAM("Done Subscribing");

  ros::spin();

  return EXIT_SUCCESS;
}
