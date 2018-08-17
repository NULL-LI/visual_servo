#include "ros/ros.h"
#include "std_msgs/String.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include <tf/transform_broadcaster.h>
using namespace std;
using namespace cv;

#define TARGET_QR_ID 3

int main(int argc, char** argv)

{
  ros::init(argc, argv, "Image_publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub_0 =
      n.advertise<cv_bridge::CvImage>("ImagePublish", 1000);

  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  int ros_loop_rate = 100;
  ros::Rate loop_rate(ros_loop_rate);
  int cnt = 0;

  tf::TransformBroadcaster tf_broadcaster;
  ROS_INFO("Start publishing");

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 787.422776, 0, 327.190656,
                          0, 793.961826, 253.265647, 0, 0, 1);
  cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.115669, -0.034292,
                        -0.000843, 0.000460, 0.000000);
  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_4X4_50));

  VideoCapture  capture0(0),capture(1);
//  namedWindow("hhh", CV_WINDOW_AUTOSIZE);
  int count = 0;
  while (ros::ok()) {
    Mat frame, frameCopy;
    capture >> frame;
    frame.copyTo(frameCopy);

    if (frame.empty()) {
      ROS_INFO("frame.empty");
      continue;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);

    if (ids.size() > 0) {
      ROS_INFO("Markers detected : %d", (int)ids.size());
      if (ids.size() == 1 && (int)ids[0] == TARGET_QR_ID) {
        //      ROS_INFO("Marker id : %d", (int)ids[0]);
        cv::aruco::drawDetectedMarkers(frameCopy, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.053, cameraMatrix,
                                             distCoeffs, rvecs, tvecs);
        // draw axis for each marker
        for (int i = 0; i < ids.size(); i++) {
          cv::aruco::drawAxis(frameCopy, cameraMatrix, distCoeffs, rvecs[i],
                              tvecs[i], 0.1);
        }

        tf::Vector3 trans(tvecs[0][0], tvecs[0][1], tvecs[0][2]);
        tf::Vector3 rot_axis(rvecs[0][0], rvecs[0][1], rvecs[0][2]);
        tfScalar rot_angle = rot_axis.length();
        tf::Quaternion rot_q;
        rot_q.setRotation(rot_axis, rot_angle);
        //        ROS_INFO("tvec detected : %.4f %.4f %.4f", tvecs[0][0],
        //        tvecs[0][1],                 tvecs[0][2]);
        ROS_INFO("rvec detected : %.4f %.4f %.4f", rvecs[0][0], rvecs[0][1],
                 rvecs[0][2]);

        tf::Transform transform_from_aruco;
        transform_from_aruco.setOrigin(trans);
        transform_from_aruco.setRotation(rot_q);
        tf_broadcaster.sendTransform(tf::StampedTransform(
            transform_from_aruco, ros::Time::now(), "camera_link", "qr_code"));

//        tf::Transform transform_from_aruco;
//        transform_from_aruco.setOrigin(tf::Vector3(0.0, 0.0, 0.5));
//        rot_q.setRotation(tf::Vector3(1.0, 0.0, 0.0), tfScalar(M_PI));
//        transform_from_aruco.setRotation(rot_q);
//        tf_broadcaster.sendTransform(tf::StampedTransform(
//            transform_from_aruco, ros::Time::now(), "camera_link", "qr_code"));



        tf::Transform qr_code_flip;
        qr_code_flip.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        rot_q.setRotation(tf::Vector3(1.0, 0.0, 0.0), tfScalar(M_PI));
        qr_code_flip.setRotation(rot_q);
        tf_broadcaster.sendTransform(tf::StampedTransform(
            qr_code_flip, ros::Time::now(), "qr_code", "qr_code_flip"));

        tf::Transform ee_target;
        ee_target.setOrigin(tf::Vector3(0.0, 0.0, -0.25));
        rot_q.setRotation(tf::Vector3(1.0, 1.0, 1.0), 0);
        ee_target.setRotation(rot_q);
        tf_broadcaster.sendTransform(tf::StampedTransform(
            ee_target, ros::Time::now(), "qr_code_flip", "camera_target"));


        tf::Transform ee_to_camera;
        ee_to_camera.setOrigin(tf::Vector3(0.0, 0.0, -0.0));
        rot_q.setRotation(tf::Vector3(0.0, 1.0, 0.0), 0);
        ee_to_camera.setRotation(rot_q);
        tf_broadcaster.sendTransform(tf::StampedTransform(
            ee_to_camera, ros::Time::now(), "m1n6s300_end_effector", "camera_link"));


        tf::Transform camt_to_eet=ee_to_camera.inverse();
//        camt_to_eet.setOrigin(tf::Vector3(0.0, 0.0, -0.15));
//        rot_q.setRotation(tf::Vector3(1.0, 1.0, 1.0), 0);
//        camt_to_eet.setRotation(rot_q);
        tf_broadcaster.sendTransform(tf::StampedTransform(
            camt_to_eet, ros::Time::now(), "camera_target", "ee_target"));

//        tf::Transform link6_to_ee;
//        ee_to_camera.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//        rot_q.setRotation(tf::Vector3(1.0, 1.0, 1.0), 0);
//        ee_to_camera.setRotation(rot_q);
//        tf_broadcaster.sendTransform(tf::StampedTransform(
//            ee_to_camera, ros::Time::now(), "m1n6s300_end_effector", "ee_link"));

      }
    }
    cv::imshow("out", frameCopy);

    //    imshow("hhh", frame);
    //    count++;
    //    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
    //    "bgr8", frame).toImageMsg();
    //    pub.publish(msg);
    waitKey(10);
    //    ros::spinOnce();
    //    ROS_INFO("Frames captured: %d, size %d, %d", count, frame.cols,
    //    frame.rows);
  }
  return 0;
}
