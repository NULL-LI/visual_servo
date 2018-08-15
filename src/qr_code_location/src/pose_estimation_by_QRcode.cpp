#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom");
  ros::NodeHandle node;
  ros::Rate loop_rate(10);
  ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("/odom", 1000, callback_odom);
  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("/odom_new", 1000);
  cv::VideoCapture inputVideo;
  inputVideo.open(1);
  cv::Mat cameraMatrix(3, 3, CV_64F), distCoeffs(5, 1, CV_64F);
  // define the QR code position
  cv::Mat QRcode_pose[4];
  QRcode_pose[0] = cv::Mat::eye(4, 4, CV_64F);
  QRcode_pose[1] = cv::Mat::eye(4, 4, CV_64F);
  QRcode_pose[1].at<double>(0, 3) = 1.0;
  QRcode_pose[1].at<double>(1, 3) = 0.0;
  QRcode_pose[2] = cv::Mat::eye(4, 4, CV_64F);
  QRcode_pose[2].at<double>(0, 3) = 1.0;
  QRcode_pose[2].at<double>(1, 3) = 1.0;
  QRcode_pose[3] = cv::Mat::eye(4, 4, CV_64F);
  QRcode_pose[3].at<double>(0, 3) = 0.0;
  QRcode_pose[3].at<double>(1, 3) = 1.0;
  // camera parameters are read from somewhere
  cameraMatrix.at<double>(0, 0) = 8.1614e+02;
  cameraMatrix.at<double>(0, 1) = 0.0;
  cameraMatrix.at<double>(0, 2) = 3.1950e+02;
  cameraMatrix.at<double>(1, 0) = 0.0;
  cameraMatrix.at<double>(1, 1) = 8.1614e+02;
  cameraMatrix.at<double>(1, 2) = 2.3950e+02;
  cameraMatrix.at<double>(2, 0) = 0.0;
  cameraMatrix.at<double>(2, 1) = 0.0;
  cameraMatrix.at<double>(2, 2) = 1.0;

  distCoeffs.at<double>(0, 0) = -3.1056e-02;
  distCoeffs.at<double>(1, 0) = 1.5698e+00;
  distCoeffs.at<double>(2, 0) = 0.0;
  distCoeffs.at<double>(3, 0) = 0.0;
  distCoeffs.at<double>(4, 0) = -6.2398e+00;
  // end
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  while (inputVideo.grab() && ros::ok())
  {
    ros::spinOnce();
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);
    image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0)
    {
      cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
      std::vector<cv::Vec3d> rvecs, tvecs;

      cv::aruco::estimatePoseSingleMarkers(corners, 0.053, cameraMatrix, distCoeffs, rvecs, tvecs);
      // draw axis for each marker
      for (int i = 0; i < ids.size(); i++)
      {
        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        cv::Mat rotation_Matrix(3, 3, CV_64F);
        cv::Mat transform_Matrix(4, 4, CV_64F), camera_to_base(4, 4, CV_64F), base_to_world(4, 4, CV_64F);
        cout << "test" << endl;
        cv::Rodrigues(rvecs[i], rotation_Matrix);
        transform_Matrix.at<double>(0, 0) = rotation_Matrix.at<double>(0, 0);
        transform_Matrix.at<double>(0, 1) = rotation_Matrix.at<double>(0, 1);
        transform_Matrix.at<double>(0, 2) = rotation_Matrix.at<double>(0, 2);
        transform_Matrix.at<double>(0, 3) = tvecs[i][0];
        transform_Matrix.at<double>(1, 0) = rotation_Matrix.at<double>(1, 0);
        transform_Matrix.at<double>(1, 1) = rotation_Matrix.at<double>(1, 1);
        transform_Matrix.at<double>(1, 2) = rotation_Matrix.at<double>(1, 2);
        transform_Matrix.at<double>(1, 3) = tvecs[i][1];
        transform_Matrix.at<double>(2, 0) = rotation_Matrix.at<double>(2, 0);
        transform_Matrix.at<double>(2, 1) = rotation_Matrix.at<double>(2, 1);
        transform_Matrix.at<double>(2, 2) = rotation_Matrix.at<double>(2, 2);
        transform_Matrix.at<double>(2, 3) = tvecs[i][2];
        transform_Matrix.at<double>(3, 0) = 0.0;
        transform_Matrix.at<double>(3, 1) = 0.0;
        transform_Matrix.at<double>(3, 2) = 0.0;
        transform_Matrix.at<double>(3, 3) = 1.0;
        // input the matric (camera_to_base)
        camera_to_base.at<double>(0, 0) = 0.999;
        camera_to_base.at<double>(0, 1) = -0.0311;
        camera_to_base.at<double>(0, 2) = -0.016;
        camera_to_base.at<double>(0, 3) = 0.0298;
        camera_to_base.at<double>(1, 0) = -0.003;
        camera_to_base.at<double>(1, 1) = -0.545;
        camera_to_base.at<double>(1, 2) = 0.838;
        camera_to_base.at<double>(1, 3) = 0.120;
        camera_to_base.at<double>(2, 0) = -0.035;
        camera_to_base.at<double>(2, 1) = -0.838;
        camera_to_base.at<double>(2, 2) = -0.545;
        camera_to_base.at<double>(2, 3) = 0.258;
        camera_to_base.at<double>(3, 0) = 0.0;
        camera_to_base.at<double>(3, 1) = 0.0;
        camera_to_base.at<double>(3, 2) = 0.0;
        camera_to_base.at<double>(3, 3) = 1.0;
        cout << "here is ok" << endl;
        int index = ids[i] - 1;
        cout << QRcode_pose[index] << endl;
        cout << transform_Matrix.inv() << endl;
        cout << camera_to_base.inv() << endl;
        base_to_world = QRcode_pose[index] * transform_Matrix.inv() * camera_to_base.inv();
        cout << "here is ok111" << endl;
        //                cout<<"tvecs[i]"<<endl;
        //                cout<<tvecs[i]<<endl;
        //                cout<<"transform_Matrix"<<endl;q
        //                cout<<transform_Matrix<<endl;
        //                cout<<"QRcode_pose"<<endl;
        //                cout<<QRcode_pose<<endl;
        cout << "ids[i]" << endl;
        cout << ids[i] << endl;
        cout << "base_to_world" << endl;
        cout << base_to_world << endl;
      }
    }
    cv::imshow("out", imageCopy);

    char key = (char)cv::waitKey(30);
    if (key == 27)
      break;

    loop_rate.sleep();
  }
  return 0;
}
