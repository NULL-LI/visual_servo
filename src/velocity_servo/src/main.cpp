#include <sstream>
#include "../../../devel/include/kinova_msgs/JointVelocity.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <math.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <../../../devel/include/keyboard/Key.h>

#include <key_def.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#define PI 3.14159265359
#define stop_time 0.1
using namespace std;
using namespace KDL;
using namespace Eigen;

// string robotype="m1n6s300";
string robotype = "m1n6s300";
Frame F_dest;
JntArray q_now;
// JntArray q_tmp;
double d_v = 0.001;

double v_cart = 0.03;

Twist twist_end;

int control_cnt = 0;

double trans_kp = 0.4;
double trans_ki = 0.0005;
double trans_kd = 0.01;

double rot_kp = trans_kp;
double rot_ki = trans_ki;
double rot_kd = trans_kd;

//double rot_kp = 0.02;
//double rot_ki = 0.0002;
//double rot_kd = 0.005;

class CommandReceiver_keyboard {
 public:
  CommandReceiver_keyboard() {
    chatter_pub_0 = n.advertise<keyboard::Key>("keyup", 1000);
    sub = n.subscribe("keyboard/keydown", 1000,
                      &CommandReceiver_keyboard::callback, this);
  }
  void callback(const keyboard::Key& keymsg) {
    ROS_INFO("KEYBOARD_MSG_RECEIVED");
    //        printf("Q: %f %f %f %f %f %f
    //        \n",q_tmp(0),q_tmp(1),q_tmp(2),q_tmp(3),q_tmp(4),q_tmp(5));
    int code = keymsg.code;
    control_cnt = 0;
    switch (code) {
      case KEY_w:  // w
        break;
      default:
        break;

        ROS_INFO("KEYBOARD_MSG_HANDLED");
    }
  }

 private:
  ros::NodeHandle n;
  ros::Publisher chatter_pub_0;
  ros::Subscriber sub;
};

class CommandReceiver_robo_pos {
 public:
  CommandReceiver_robo_pos() {
    // chatter_pub_0 = n.advertise<sensor_msgs::JointState>("joint_states_test",
    // 1000);
    sub = n.subscribe("m1n6s300_driver/out/joint_state", 1000,
                      &CommandReceiver_robo_pos::callback, this);
  }
  void callback(const sensor_msgs::JointState& kinova_states) {
    for (int i = 0; i < 6; i++) {
      q_now(i) = kinova_states.position[i];
      //            cout<<kinova_states<<endl;
    }
  }

 private:
  ros::NodeHandle n;
  ros::Publisher chatter_pub_0;
  ros::Subscriber sub;
};

Vector3f trans_velocity = Vector3f::Zero();
Vector3f rot_omega = Vector3f::Zero();
Vector3f translation_vector_pre_1 = Vector3f::Zero();
Vector3f rotation_vector_pre_1 = Vector3f::Zero();
Vector3f translation_vector_pre_2 = Vector3f::Zero();
Vector3f rotation_vector_pre_2 = Vector3f::Zero();

int vec_out_cnt_max=20;
int vec_out_cnt=vec_out_cnt_max;

int robot_end_servo(const tf::StampedTransform transform, Twist& twist) {
  tf::Vector3 translation = transform.getOrigin();
  tf::Quaternion rotation = transform.getRotation();
  tfScalar rotation_angle = rotation.getAngle();
  tf::Vector3 rotation_axis = rotation.getAxis();

  Vector3f translation_vector(translation.getX(), translation.getY(),
                              translation.getZ());
  Vector3f rotation_vector(rotation_axis.getX(), rotation_axis.getY(),
                           rotation_axis.getZ());
  rotation_vector *= rotation_angle;

  Vector3f translation_error_p = translation_vector - translation_vector_pre_1;
  Vector3f translation_error_i = translation_vector;
  Vector3f translation_error_d = translation_vector -
                                 2 * translation_vector_pre_1 +
                                 translation_vector_pre_2;

  Vector3f rotation_error_p = rotation_vector - rotation_vector_pre_1;
  Vector3f rotation_error_i = rotation_vector;
  Vector3f rotation_error_d =
      rotation_vector - 2 * rotation_vector_pre_1 + rotation_vector_pre_2;

  trans_velocity += trans_kp * translation_error_p +
                    trans_ki * translation_error_i +
                    trans_kd * translation_error_d;
  rot_omega += rot_kp * rotation_error_p + rot_ki * rotation_error_i +
               rot_kd * rotation_error_d;

  translation_vector_pre_2 = translation_vector_pre_1;
  rotation_vector_pre_2 = rotation_vector_pre_1;
  translation_vector_pre_1 = translation_vector;
  rotation_vector_pre_1 = rotation_vector;
//   rot_omega=Vector3f::Zero();

  SetToZero(twist);
  twist(0) = trans_velocity(0);
  twist(1) = trans_velocity(1);
  twist(2) = trans_velocity(2);

  twist(3) = rot_omega(0);
  twist(4) = rot_omega(1);
  twist(5) = rot_omega(2);


  vec_out_cnt--;
  if(vec_out_cnt<0)
  {
  cout << "trans_vect :" << translation_vector.transpose()
       << "  rot_vect: " << rotation_vector.transpose() << endl;
//  ROS_INFO("twist_cart: %f %f %f %f %f %f ", twist(0), twist(1),
//           twist(2), twist(3), twist(4), twist(5));
  vec_out_cnt=vec_out_cnt_max;
}
  return 0;
}

int speed_translate(const tf::StampedTransform transform, Twist& twist) {
    tf::Quaternion end_to_base_q_tf=transform.getRotation();
Eigen::Quaterniond end_to_base_q(end_to_base_q_tf);
Eigen::Matrix3d end_to_base_m(end_to_base_q);

Eigen::Vector3d speed(twist(0), twist(1),twist(2));
Eigen::Vector3d omega(twist(3), twist(4), twist(5));

speed=end_to_base_m*speed;
omega=end_to_base_m*omega;

twist(0) = speed(0);
twist(1) = speed(1);
twist(2) = speed(2);

twist(3) = omega(0);
twist(4) = omega(1);
twist(5) = omega(2);
vec_out_cnt--;
if(vec_out_cnt<0)
{
//  cout << "trans_vect :" << translation_vector.transpose()
//       << "  rot_vect: " << rotation_vector.transpose() << endl;
//ROS_INFO("twist_cart: %f %f %f %f %f %f ", twist(0), twist(1),
//         twist(2), twist(3), twist(4), twist(5));
vec_out_cnt=vec_out_cnt_max;
}
return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jaco_vel_move_real");

  ros::NodeHandle n;

  ros::Publisher robot_joint_velocity_publisher =
      n.advertise<kinova_msgs::JointVelocity>(
          "m1n6s300_driver/in/joint_velocity", 1000);
  tf::TransformListener tf_listener;

  int ros_loop_rate = 100;
  int control_cnt_MAX = ros_loop_rate * stop_time;
  ros::Rate loop_rate(ros_loop_rate);

  kinova_msgs::JointVelocity JointVelPub;

  SetToZero(twist_end);

  double cnt = 0;

  int cycleperiod = 2;

  KDL::Tree my_tree;
  KDL::Chain my_chain;
  unsigned int joint_num;
  KDL::JntArray jointpositions;
  KDL::JntArray jointvelocitys;
  if (!kdl_parser::treeFromFile(
          "./src/kinova-ros-master/kinova_description/urdf/" + robotype +
              "_standalone.urdf",
          my_tree)) {
    ROS_ERROR("Failed to construct kdl tree");
  }
  if (!my_tree.getChain(robotype + "_link_base", robotype + "_end_effector",
                        my_chain)) {
    ROS_ERROR("Failed to convert to chain");
  }

  joint_num = my_chain.getNrOfJoints();

  jointpositions = KDL::JntArray(joint_num);
  jointvelocitys = KDL::JntArray(joint_num);

  ChainFkSolverPos_recursive fksolver1(my_chain);  // Forward position solver
  ChainIkSolverVel_pinv iksolver1v(my_chain);      // Inverse velocity solver
  ChainIkSolverPos_NR iksolver1(
      my_chain, fksolver1, iksolver1v, 100,
      1e-6);  // Maximum 100 iterations, stop at accuracy 1e-6

  // Creation of jntarrays:
  JntArray q(joint_num);
  JntArray q_init(joint_num);

  JntArray q_next(joint_num);
  JntArray q_vel(joint_num);
  JntArray q_vel_0(joint_num);
  Frame F_init;
  Frame F_now;

  q_init(0) = 1;
  q_init(1) = 1;
  q_init(2) = 1;
  q_init(3) = 1;
  q_init(4) = 1;
  q_init(5) = 1;

  q_now.resize(joint_num);

  if (fksolver1.JntToCart(q_init, F_init) != 0) {
    ROS_ERROR("Failed to solve init cart");
  }

  cout << F_init.p << endl << F_init.M << endl;
  F_dest = F_init;

  CommandReceiver_keyboard command_receiver_keyboard;  //监视Topic中命令
  CommandReceiver_robo_pos command_receiver_robo_pos;  // refresh q_now
  while (ros::ok()) {
    tf::StampedTransform transform;
    try {
      tf_listener.waitForTransform( robotype + "_end_effector","ee_target",
                                   ros::Time(0), ros::Duration(10.0));
      tf_listener.lookupTransform(robotype + "_end_effector","ee_target",
                                  ros::Time(0), transform);

    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }

    robot_end_servo(transform, twist_end);
    try {
      tf_listener.waitForTransform( robotype + "_end_effector",robotype + "_link_base",
                                   ros::Time(0), ros::Duration(10.0));
      tf_listener.lookupTransform(robotype + "_end_effector",robotype + "_link_base",
                                  ros::Time(0), transform);

    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }

speed_translate(transform, twist_end);



    if (iksolver1v.CartToJnt(q_now, twist_end, q_vel_0) != 0) {
      ROS_ERROR("Failed to solve jnt");
    } else {
        double q_abs_vel_max=0;
      for (int i = 0; i < 6; i++) {
//        q_vel(i) = q_vel_0(i) * 180.0 / M_PI;
        if(q_abs_vel_max<fabs(q_vel(i)))
        {
            q_abs_vel_max=fabs(q_vel(i));
        }
      }
      for (int i = 0; i < 6; i++) {
        q_vel(i) = q_vel_0(i) * 180.0 / max(M_PI,q_abs_vel_max*12) ;
      }
      JointVelPub.joint1 = q_vel(0);
      JointVelPub.joint2 = q_vel(1);
      JointVelPub.joint3 = q_vel(2);
      JointVelPub.joint4 = q_vel(3);
      JointVelPub.joint5 = q_vel(4);
      JointVelPub.joint6 = q_vel(5);

      //      ROS_INFO("q_vel: %f %f %f %f %f %f ", q_vel(0), q_vel(1),
      //      q_vel(2),
      //               q_vel(3), q_vel(4), q_vel(5));

      robot_joint_velocity_publisher.publish(JointVelPub);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
