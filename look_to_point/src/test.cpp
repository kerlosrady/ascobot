// C++ standard headers
#include <exception>
#include <string>
#include <array>
#include <iostream>
#include <math.h>

// Boost headers
#include <boost/shared_ptr.hpp>
// OpenCV headers

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>


#ifdef _DEBUG
#pragma comment(lib,"opencv_world400d.lib")
#else
#pragma comment(lib,"opencv_world400.lib")
#endif

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

cv::Mat grayTmpl = imread("tmp.bmp", 0);
