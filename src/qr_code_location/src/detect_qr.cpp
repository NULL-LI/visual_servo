#include "ros/ros.h"
#include "std_msgs/String.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <sstream>
using namespace std;
using namespace cv;
int main(int argc, char** argv)

{
  ros::init(argc, argv, "Image_publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub_0 = n.advertise<cv_bridge::CvImage>("ImagePublish", 1000);

  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  int ros_loop_rate = 100;
  ros::Rate loop_rate(ros_loop_rate);
  int cnt = 0;
  ROS_INFO("Start publishing");

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 787.422776, 0, 327.190656, 0, 793.961826, 253.265647, 0, 0, 1);
  cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.115669, -0.034292, -0.000843, 0.000460, 0.000000);
  Ptr<aruco::Dictionary> dictionary =
      aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_4X4_50));

  VideoCapture capture(1), capture0(0);
  namedWindow("hhh", CV_WINDOW_AUTOSIZE);
  int count = 0;
  while (ros::ok())
  {
    Mat frame, frameCopy;
    capture >> frame;
    frame.copyTo(frameCopy);

    if (frame.empty())
    {
      ROS_INFO("frame.empty");
      continue;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);

    if (ids.size() > 0)
    {
      ROS_INFO("Markers detected : %d", (int)ids.size());
      cv::aruco::drawDetectedMarkers(frameCopy, corners, ids);
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
      // draw axis for each marker
      for (int i = 0; i < ids.size(); i++)
        cv::aruco::drawAxis(frameCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
    }
    cv::imshow("out", frameCopy);

    //    imshow("hhh", frame);
    //    count++;
    //    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    //    pub.publish(msg);
    waitKey(10);
    //    ros::spinOnce();
    //    ROS_INFO("Frames captured: %d, size %d, %d", count, frame.cols, frame.rows);
  }
  return 0;
}
