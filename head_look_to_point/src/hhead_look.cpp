/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */


/** \author Alessandro Di Fava. */

/**
 * @file
 *
 * @brief example on how to subscribe to an image topic and how to make the robot look towards a given direction
 *
 * How to test this application:
 *
 * 1) Launch the application:
 *
 *   $ rosrun tiago_dual_tutorials head_look_to_point
 *
 * 2) Click on image pixels to make TIAGo++ look towards that direction
 *
 */

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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string windowName      = "Inside of TIAGo++'s head";
static const std::string cameraFrame     = "/xtion_rgb_optical_frame";
static const std::string imageTopic      = "/xtion/rgb/image_raw";
static const std::string cameraInfoTopic = "/xtion/rgb/camera_info";


// Our Action interface type for moving TIAGo++'s head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;

ros::Time latestImageStamp;

// OpenCV callback function for mouse events on a window
void onMouse( int event, int u, int v, int, void* )
{

  //build the action goal
  control_msgs::PointHeadGoal goal;
  //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  goal.pointing_frame = cameraFrame;
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.25;
  goal.target = pointStamped;

  pointHeadClient->sendGoal(goal);
  ros::Duration(0.5).sleep();
}

// Create a ROS action client to move TIAGo's head
void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "head_look_to_point");

  ROS_INFO("Starting look_to_point application ...");
 
  // Create a point head action client to move the TIAGo++'s head
  createPointHeadClient( pointHeadClient );


  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  return EXIT_SUCCESS;
}
