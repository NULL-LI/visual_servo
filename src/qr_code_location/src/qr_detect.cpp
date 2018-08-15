#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <numeric>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
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

  VideoCapture capture(1);
  namedWindow("hhh", CV_WINDOW_AUTOSIZE);
  int count = 0;
  while (ros::ok())
  {
    Mat frame;
    capture >> frame;
    imshow("hhh", frame);

    //        imwrite(image_index.str(), frame);
    //        loop_rate.sleep();
    /*char key_board =*/
    count++;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    waitKey(10);
    ros::spinOnce();
    ROS_INFO("Frames captured: %d, size %d, %d", count, frame.cols, frame.rows);
  }
  return 0;
}
