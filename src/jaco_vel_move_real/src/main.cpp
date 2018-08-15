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


#include <../../../devel/include/keyboard/Key.h>

#include <key_def.h>

#define PI 3.14159265359
#define stop_time 0.1
using namespace std;
using namespace KDL;

//string robotype="m1n6s300";
string robotype="j2n6s300";
Frame F_dest;
JntArray q_now;
//JntArray q_tmp;
double d_v=0.001;

double v_cart=0.03;

Twist twist_cart;

int control_cnt=0;



class CommandReceiver_keyboard
{
public:
    CommandReceiver_keyboard()
    {
        chatter_pub_0 = n.advertise<keyboard::Key>("keyup", 1000);
        sub = n.subscribe("keyboard/keydown", 1000, &CommandReceiver_keyboard::callback, this);
    }
    void callback(const keyboard::Key& keymsg)
    {
        ROS_INFO("KEYBOARD_MSG_RECEIVED");
//        printf("Q: %f %f %f %f %f %f \n",q_tmp(0),q_tmp(1),q_tmp(2),q_tmp(3),q_tmp(4),q_tmp(5));
        int code=keymsg.code;
        control_cnt=0;

        SetToZero(twist_cart);

        switch (code) {
        case KEY_w://w
            F_dest.p(0)+=d_v;
            twist_cart(0)=v_cart;
            break;
        case KEY_s:
            F_dest.p(0)-=d_v;
            twist_cart(0)=-v_cart;
            break;
        case KEY_a:
            F_dest.p(1)+=d_v;
            twist_cart(1)=v_cart;
            break;
        case KEY_d:
            F_dest.p(1)-=d_v;
            twist_cart(1)=-v_cart;
            break;
        case KEY_UP:
            F_dest.p(2)+=d_v;
            twist_cart(2)=v_cart;
            break;
        case KEY_DOWN:
            F_dest.p(2)-=d_v;
            twist_cart(2)=-v_cart;
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

class CommandReceiver_robo_vel
{
public:
    CommandReceiver_robo_vel()
    {
        //chatter_pub_0 = n.advertise<sensor_msgs::JointState>("joint_states_test", 1000);
        sub= n.subscribe("m1n6s300_driver/out/joint_state", 1000, &CommandReceiver_robo_vel::callback, this);
    }
    void callback(const sensor_msgs::JointState& kinova_states)
    {
        for(int i=0;i<6;i++)
        {
            q_now(i)=kinova_states.position[i];
//            cout<<kinova_states<<endl;
        }
    }
private:
    ros::NodeHandle n;
    ros::Publisher chatter_pub_0;
    ros::Subscriber sub;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "jaco_vel_move_real");

    ros::NodeHandle n;

    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Comm_Recv", 1000);cnt


    //FollowJointTrajectoryAction followJointTrajectory("follow_joint_trajectory");//moveit

    ros::Publisher chatter_pub_0 = n.advertise<kinova_msgs::JointVelocity>("m1n6s300_driver/in/joint_velocity", 1000);
    //   ros::Publisher chatter_pub_1 = n.advertise<can_driver::orig_new>("Comm_Orig", 1000);
    //   ros::Publisher chatter_pub_0 = n.advertise<can_driver::DownToUp>("down_to_up", 1000);

    //CommandReceiver command_receiver;//监视Topic中命令

    //ros::Publisher comm_recv_pub = n.advertise<beginner_tutorials::recv>("Comm_Recv", 1000);
    //ros::Subscriber sub = n.subscribe("up_to_down", 1000, chatterCallback);


    int ros_loop_rate=100;
    int control_cnt_MAX=ros_loop_rate*stop_time;
    ros::Rate loop_rate(ros_loop_rate);


    kinova_msgs::JointVelocity JointVelPub;



    //    JointStatePub.name.resize(9);
    //    JointStatePub.position.resize(9);


    //    JointStatePub.name[0]=robotype+"_joint_1";
    //    JointStatePub.name[1]=robotype+"_joint_2";
    //    JointStatePub.name[2]=robotype+"_joint_3";
    //    JointStatePub.name[3]=robotype+"_joint_4";
    //    JointStatePub.name[4]=robotype+"_joint_5";
    //    JointStatePub.name[5]=robotype+"_joint_6";
    //    JointStatePub.name[6]=robotype+"_joint_finger_1";
    //    JointStatePub.name[7]=robotype+"_joint_finger_2";
    //    JointStatePub.name[8]=robotype+"_joint_finger_3";

    //    for(int i=0;i<9;i++)
    //    {
    //        JointStatePub.position[i]=0;
    //    }

//    for(int i=0;i<6;i++)
//    {
//        twist_cart(i)=0;
//    }
        SetToZero(twist_cart);
//twist_cart.Zero();

    double cnt=0;

    int cycleperiod=2;

    KDL::Tree my_tree;
    KDL::Chain my_chain;
    unsigned int joint_num;
    KDL::JntArray jointpositions;
    KDL::JntArray jointvelocitys;
    if (!kdl_parser::treeFromFile("./src/kinova-ros-master/kinova_description/urdf/"+robotype+"_standalone.urdf", my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    if(!my_tree.getChain(robotype+"_link_base",robotype+"_end_effector", my_chain))
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


    JntArray q_next(joint_num);
    JntArray q_vel(joint_num);
    JntArray q_vel_0(joint_num);
    Frame F_init;
    Frame F_now;
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

    q_now.resize(joint_num);


    //q_init(0)=26188.316285;
    //q_init(1)=-24565.820627;
    //q_init(2)=40093.660688 ;
    //q_init(3)=-102626.646365 ;
    //q_init(4)=27251.082089 ;
    //q_init(5)=-2661.168053;

    //    q_now=q_init;


    if(fksolver1.JntToCart(q_init,F_init)!=0)
    {
        ROS_ERROR("Failed to solve init cart");
    }

    cout<<F_init.p<<endl<<F_init.M<<endl;
    F_dest=F_init;


    CommandReceiver_keyboard command_receiver_keyboard;//监视Topic中命令
    CommandReceiver_robo_vel command_receiver_robo_vel;
    while (ros::ok())
    {
        //        int ret = iksolver1.CartToJnt(q_init,F_dest,q);

        //        F_dest.p(0)+=0.0001;
        //        if(fksolver1.JntToCart(q_now,F_now)!=0)
        //        {
        //            ROS_ERROR("Failed to solve init cart");
        //        }

//        ROS_INFO("JS: %f %f %f %f %f %f ",q_now(0),q_now(1),q_now(2),q_now(3),q_now(4),q_now(5));
ROS_INFO("twist_cart: %f %f %f %f %f %f ",twist_cart(0),twist_cart(1),twist_cart(2),twist_cart(3),twist_cart(4),twist_cart(5));

        if(iksolver1v.CartToJnt(q_now,twist_cart,q_vel_0)!=0)
        {
                        ROS_ERROR("Failed to solve jnt");
        }
        else{
            //            q_now=q_next;
            //            q_tmp=q_next;
            //            JointVelPub.header.stamp = ros::Time::now();
            //            double angle=sin(cnt/ros_loop_rate/cycleperiod*2*PI);
            //            for(int i=0;i<6;i++)
            //            {
            //                JointVelPub.position[i]=q_now(i);
            //            }

            for(int i=0;i<6;i++)
            {
                q_vel(i) = q_vel_0(i)*180/3.14159;
            }
            if(control_cnt>control_cnt_MAX)
            {
                  SetToZero(twist_cart);
                for(int i=0;i<6;i++)
                {
                    q_vel(i) = 0;
                }
            }

            JointVelPub.joint1=q_vel(0);
            JointVelPub.joint2=q_vel(1);
            JointVelPub.joint3=q_vel(2);
            JointVelPub.joint4=q_vel(3);
            JointVelPub.joint5=q_vel(4);
            JointVelPub.joint6=q_vel(5);

            //        cout<<F_dest.p<<endl;
            //JointStatePub.position[1]=;
            //        ROS_INFO("%d",nj);
            //        ROS_INFO("%f",angle);

            chatter_pub_0.publish(JointVelPub);
        }
        control_cnt+=1;
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
