#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#include "../../../devel/include/kinova_msgs/JointVelocity.h"

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
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "../../../devel/include/keyboard/Key.h"

#define PI 3.14159265359

using namespace std;
using namespace KDL;


Frame F_dest;
    JntArray q_tmp;
double d_v=0.001;

class CommandReceiver
{
public:
    CommandReceiver()
    {
        chatter_pub_0 = n.advertise<keyboard::Key>("keyup", 1000);
        sub = n.subscribe("keyboard/keydown", 1000, &CommandReceiver::callback, this);
    }

    void callback(const keyboard::Key& keymsg)
    {
        ROS_INFO("KEYBOARD_MSG_RECEIVED");
        printf("Q: %f %f %f %f %f %f \n",q_tmp(0),q_tmp(1),q_tmp(2),q_tmp(3),q_tmp(4),q_tmp(5));
        int code=keymsg.code;
        switch (code) {
        case 119://w
            F_dest.p(0)+=d_v;
            break;
        case 115://KEY_s:
            F_dest.p(0)-=d_v;
            break;
        case 97://KEY_a:
            F_dest.p(1)+=d_v;
            break;
        case 100://KEY_d:
            F_dest.p(1)-=d_v;
            break;
        case 273://KEY_UP:
            F_dest.p(2)+=d_v;
            break;
        case 274://KEY_DOWN:
            F_dest.p(2)-=d_v;
            break;
        default:
            break;
        }
    }
private:
    ros::NodeHandle n;
    ros::Publisher chatter_pub_0;
    ros::Subscriber sub;
};



int main(int argc, char **argv)
{

    ros::init(argc, argv, "jaco_pub_test");

    ros::NodeHandle n;

    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Comm_Recv", 1000);


    //FollowJointTrajectoryAction followJointTrajectory("follow_joint_trajectory");//moveit

    ros::Publisher chatter_pub_0 = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    //   ros::Publisher chatter_pub_1 = n.advertise<can_driver::orig_new>("Comm_Orig", 1000);
    //   ros::Publisher chatter_pub_0 = n.advertise<can_driver::DownToUp>("down_to_up", 1000);

    //CommandReceiver command_receiver;//监视Topic中命令

    //ros::Publisher comm_recv_pub = n.advertise<beginner_tutorials::recv>("Comm_Recv", 1000);
    //ros::Subscriber sub = n.subscribe("up_to_down", 1000, chatterCallback);


    int ros_loop_rate=1000;
    ros::Rate loop_rate(ros_loop_rate);


    sensor_msgs::JointState JointStatePub;



    JointStatePub.name.resize(9);
    JointStatePub.position.resize(9);


    JointStatePub.name[0]="j2n6s300_joint_1";
    JointStatePub.name[1]="j2n6s300_joint_2";
    JointStatePub.name[2]="j2n6s300_joint_3";
    JointStatePub.name[3]="j2n6s300_joint_4";
    JointStatePub.name[4]="j2n6s300_joint_5";
    JointStatePub.name[5]="j2n6s300_joint_6";
    JointStatePub.name[6]="j2n6s300_joint_finger_1";
    JointStatePub.name[7]="j2n6s300_joint_finger_2";
    JointStatePub.name[8]="j2n6s300_joint_finger_3";

    for(int i=0;i<9;i++)
    {
        JointStatePub.position[i]=0;
    }

    double cnt=0;

    int cycleperiod=2;

    KDL::Tree my_tree;
    KDL::Chain my_chain;
    unsigned int joint_num;
    KDL::JntArray jointpositions;
    KDL::JntArray jointvelocitys;
    if (!kdl_parser::treeFromFile("./src/kinova-ros-master/kinova_description/urdf/j2n6s300_standalone.urdf", my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    if(!my_tree.getChain("j2n6s300_link_base", "j2n6s300_end_effector", my_chain))
    {
        ROS_ERROR("Failed to convert to chain");
    }


    joint_num=my_chain.getNrOfJoints();

    jointpositions = KDL::JntArray(joint_num);
    jointvelocitys = KDL::JntArray(joint_num);

    ChainFkSolverPos_recursive fksolver1(my_chain);//Forward position solver
    ChainIkSolverVel_pinv iksolver1v(my_chain);//Inverse velocity solver
    ChainIkSolverPos_NR iksolver1(my_chain,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

    //Creation of jntarrays:
    JntArray q(joint_num);
    JntArray q_init(joint_num);

    JntArray q_now(joint_num);
    JntArray q_next(joint_num);


    Frame F_init;
    //    Frame F_dest=...;

    //    int ret = iksolverpos.CartToJnt(q_init,F_dest,q);

//    for(int i=0;i<6;i++)
//    {
//        q_init(i)=0;
//        q_now(i)=0;
//    }


    q_init(0)=1;
    q_init(1)=1;
    q_init(2)=1 ;
    q_init(3)=1 ;
    q_init(4)=1 ;
    q_init(5)=1;

//q_init(0)=26188.316285;
//q_init(1)=-24565.820627;
//q_init(2)=40093.660688 ;
//q_init(3)=-102626.646365 ;
//q_init(4)=27251.082089 ;
//q_init(5)=-2661.168053;

q_now=q_init;


    if(fksolver1.JntToCart(q_init,F_init)!=0)
    {
        ROS_ERROR("Failed to solve init cart");
    }

    cout<<F_init.p<<endl<<F_init.M<<endl;
    F_dest=F_init;


    CommandReceiver command_receiver;//监视Topic中命令

    while (ros::ok())
    {
        //        int ret = iksolver1.CartToJnt(q_init,F_dest,q);

        //        F_dest.p(0)+=0.0001;
        if(iksolver1.CartToJnt(q_now,F_dest,q_next)!=0)
        {
             ROS_ERROR("Failed to solve jnt");
        }else{
        q_now=q_next;
q_tmp=q_next;
        JointStatePub.header.stamp = ros::Time::now();
        double angle=sin(cnt/ros_loop_rate/cycleperiod*2*PI);
        for(int i=0;i<6;i++)
        {
            JointStatePub.position[i]=q_now(i);
        }
        //        cout<<F_dest.p<<endl;
        //JointStatePub.position[1]=;
        //        ROS_INFO("%d",nj);
        //        ROS_INFO("%f",angle);

        chatter_pub_0.publish(JointStatePub);
}
        cnt+=1;

        ros::spinOnce();
        loop_rate.sleep();
    }

    //    while (ros::ok())
    //    {
    //        JointStatePub.header.stamp = ros::Time::now();


    //        double angle=sin(cnt/ros_loop_rate/cycleperiod*2*PI);

    //        for(int i=0;i<9;i++)
    //        {
    //            JointStatePub.position[i]=angle;
    //        }

    //        //JointStatePub.position[1]=;
    //        ROS_INFO("%f",angle);

    //        chatter_pub_0.publish(JointStatePub);

    //        cnt+=1;
    //        ros::spinOnce();
    //        loop_rate.sleep();
    //    }

    return 0;
}
