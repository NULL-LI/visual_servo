#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sstream>


#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
//#include <kdl/chain.hpp>
//#include <kdl/tree.hpp>
//#include <kdl/chainfksolver.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/frames_io.hpp>
//#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/chainiksolver.hpp>
//#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_nr.hpp>
//#include <kdl/chainjnttojacsolver.hpp>
//#include <kdl/jacobian.hpp>
//#include <sensor_msgs/JointState.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "../../../devel/include/keyboard/Key.h"

#define PI 3.14159265359

using namespace std;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "jaco_pub_test");
    ros::NodeHandle n;
    ros::Publisher chatter_pub_0 = n.advertise<sensor_msgs::JointState>("joint_states", 1000);    
    int ros_loop_rate=1000;
    ros::Rate loop_rate(ros_loop_rate);
    sensor_msgs::JointState JointStatePub;

    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);

    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
