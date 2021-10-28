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
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <ros/topic.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

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

static const std::string graywindowName  = "Gray Image";
static const std::string cameraFrame     = "/xtion_rgb_optical_frame";   
static const std::string imageTopic      = "/xtion/rgb/image_raw";
static const std::string depthImageTopic = "/xtion/depth_registered/image_raw";
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
cv::Mat fil;

int done = 0;
nav_msgs::Path points;

ros::Time latestImageStamp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// My Function of detecting the top of cans
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class SubscribeAndPublish
{
  public:
    SubscribeAndPublish()
    {
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
      // use compressed image transport to use less network bandwidth
      ROS_INFO_STREAM("Subscribing ");
      message_filters::Subscriber<Image> image_sub(nh,imageTopic, 1);
      message_filters::Subscriber<Image> depth_sub(nh,depthImageTopic, 1);
      TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> sync(image_sub, depth_sub, 10);
      sync.registerCallback(boost::bind(&SubscribeAndPublish::callback, this, _1, _2));
      ROS_INFO_STREAM("Done Subscribing");
      pub = nh.advertise<nav_msgs::Path>("cansPos", 10);
      ros::spin();

    }
    double ReadDepthData(unsigned int x, unsigned int y, sensor_msgs::ImageConstPtr depth_image)
    {
      // If position is invalid
      if ((x >= depth_image->width) || (y >= depth_image->height))
      {
        cout<< "Out of range"<<endl;
        return 0;      
      }  

      int index = (y*depth_image->step) + (x*(depth_image->step/depth_image->width));
      
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
            return double(depth_data.float_data);
          }

          // else, one little endian, one big endian
          for (i = 0; i < 4; i++) 
              depth_data.byte_data[i] = depth_image->data[3 + index - i];
          return double(depth_data.float_data);
      }
      int temp_val;
      // If big endian
      if (depth_image->is_bigendian)
          temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
      // If little endian
      else
          temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);

      return temp_val;
    }
    // ROS call back for every new image received
    void callback(const sensor_msgs::ImageConstPtr& imgMsg, const sensor_msgs::ImageConstPtr& depthImgMsg) 
    {
      ROS_INFO_STREAM("Entering Call Back");
      latestImageStamp = imgMsg->header.stamp;
      cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
      cv::Mat img = cvImgPtr->image;
      sensor_msgs::ImageConstPtr ros_img = depthImgMsg;

      // cv::imshow("img",img);

      //Covert to gray image
      cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY,2);
      // //Apply Median Filter to eliminate noise 
      cv::medianBlur(grayImg,medianImg,19);
      // cv::imshow("medianBlur",medianImg);

      cv::imshow("medianImg",medianImg);
      cv::threshold(medianImg,medianImg,120,255,cv::THRESH_TOZERO);
      cv::imshow("threshold",medianImg);

      //Contour Detection
      cv::Canny(medianImg,cannyOutput,90,120,3,0);
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(cannyOutput,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

      //Min Rec fit
      std::vector<cv::RotatedRect> minRect( contours.size() );
      img.copyTo(output);

      int centerX [contours.size()];
      int centerY [contours.size()];
      double Co_x [contours.size()];
      double Co_y [contours.size()];
      double Co_z [contours.size()]; 
      std::vector<geometry_msgs::PoseStamped> posesTemp(contours.size());
      
      cout << contours.size()<< endl;
      for( size_t i = 0; i< contours.size(); i++ )
      {
        //Apply minAreaRect function to get the fitted rectangles for each contour
        minRect[i] = cv::minAreaRect( contours[i] );
        cv::Point2f rect_points[4];
        minRect[i].points( rect_points );
        //Get the center of fitted recttangles
        centerX[i] = (rect_points[0].x + rect_points[2].x)/2;
        centerY[i] = (rect_points[0].y + rect_points[2].y)/2;
        cv::Point2f a(centerX[i],centerY[i]);
        circle( img, a, 1, Scalar(0,100,100), 3, LINE_AA);
        cv::putText(output,std::to_string(i+1),cv::Point(centerX[i],centerY[i]),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,255,255),3);
        
        posesTemp[i].header.frame_id = cameraFrame;
        //compute normalized coordinates of the selected pixel
        Co_x[i] = ( centerX[i]  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
        Co_y[i] = ( centerY[i]  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
        Co_z[i]= ReadDepthData(centerX[i] , centerY[i], ros_img);
        cout<< "The co of the "<< i+1<< "contour is x:  "<< Co_x[i] << "  Y:   "<< Co_y[i]<<"   Z:  "<< Co_z[i]<<endl;

        posesTemp[i].pose.position.x = Co_x[i] * Co_z[i];
        posesTemp[i].pose.position.y = Co_y[i] * Co_z[i];
        posesTemp[i].pose.position.z = Co_z[i];  
      }

      points.header.frame_id = cameraFrame;
      points.poses = posesTemp;

      pub.publish(points);
      cv::imshow("FINAL",img);

      cv::waitKey(15);
      ROS_INFO_STREAM("Exiting Call Back");
    }

  private:
    ros::NodeHandle nh; 
    ros::Publisher pub;

};//End of class SubscribeAndPublish



// Entry point
int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "Vision");

  ROS_INFO("Starting Vision application ...");
  
  SubscribeAndPublish SAPObject;
  return 0;
  
}
