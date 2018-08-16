
//OPENCV INCLUDES
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <unistd.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//STD C++ INCLUDES
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>



//CALIBRAITON BASE
#include "calibrationbase.h"
#define KEYCODE_SPACE 0x20
#define KEYCODE_ESC   0x1B
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}
using namespace std;
using namespace cv;
const string  IMAGE_WINDOW_NAME="image_viewer";
bool movetoPoseJointstate_local(String jointgroup,String referenceFrame,Mat jointstate,int nJoint);

class JointStateRecorder
{
  ros::NodeHandle nh_;
  ros::Subscriber sub ;

public:
    JointStateRecorder()
//    : it_(nh_)
  {
    sub = nh_.subscribe("joint_states", 1, &JointStateRecorder::JointstateCallback,this);
    initJointname();
    nJoint=7;
    allowance_error= 0.0001;
    gotTarget = false;
  }
public :
  vector <double> current_jointstate ;
  vector <double> target_jointstate;
  vector <string> jn;
  int nJoint;
  double  allowance_error ;
  bool   gotTarget;
   void initJointname()
   {
       jn.push_back("arm_left_joint_1_s");
       jn.push_back("arm_left_joint_2_l");
       jn.push_back("arm_left_joint_3_e");
       jn.push_back("arm_left_joint_4_u");
       jn.push_back("arm_left_joint_5_r");
       jn.push_back("arm_left_joint_6_b");
       jn.push_back("arm_left_joint_7_t");

   }

  ~JointStateRecorder()
  {

  }

    void JointstateCallback(const sensor_msgs::JointState& jointstate)
   {

        current_jointstate.clear();

        int namesize = jointstate.name.size();
        int index = 0;

        for (int i = 0 ;i<namesize;i++)
        {
              if(jointstate.name[i].compare(jn[index])==0)
             {
                current_jointstate.push_back(jointstate.position[i]);
                index ++;
                if(index==nJoint) break;
             }

        }

       if(target_jointstate.size()==nJoint)
       {
           gotTarget= isGotTarget();
       }
    }
   bool setTarget(Mat jointvalue)
    {
        if((jointvalue.rows!=1)||(jointvalue.cols!=nJoint))
        {
          ROS_ERROR("Bad jointvalue");
          return false;
        }

        target_jointstate.clear();

        for (int i = 0;i< nJoint;i++)
        {
            target_jointstate.push_back(jointvalue.at<double>(0,i));

        }
        return true;
    }
    bool isGotTarget()
    {

           for (int i = 0 ;i <nJoint;i++)
           {
                    if(fabs(current_jointstate[i]-target_jointstate[i])>allowance_error) return false;

           }
           return true;
    }
};




class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : it_(nh_)
  {
      image_sub_ = it_.subscribe("/kinect2/hd/image_color", 1,
      &ImageConverter::imageCb, this);

    cv::namedWindow(IMAGE_WINDOW_NAME);
  }
  ~ImageConverter()
  {
    cv::destroyWindow(IMAGE_WINDOW_NAME);
  }

  cv::Mat currentframe;

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
     Mat image=cv_ptr->image;
     currentframe=image;
     Mat image_resized;
     resize(image,image_resized,Size(640,480),0,0,CV_INTER_LINEAR);
     cv::imshow(IMAGE_WINDOW_NAME, image_resized);
     cv::waitKey(10);
  }
};
void saveData( Mat image,int count,tf::StampedTransform pose,Mat *posefile)
{
   char imagename[100];
   sprintf(imagename,"frame%d.jpg",count);
   imwrite(imagename,image);
   cout<<"Saving image "<<imagename<<endl;


   //save pose
   Mat mat_pose = stampedTransform2Mat(pose);
   Mat rotation = mat_pose(Range(0,3),Range(0,3));
   Mat quat     = dcm2quat(rotation);
   Mat mat_pose_1_7(1,7,CV_64F);
       for (int i=0;i<4;i++)
       {
           mat_pose_1_7.at<double>(0,i)=quat.at<double>(0,i);
       }
   mat_pose_1_7.at<double>(0,4)=mat_pose.at<double>(0,3);
   mat_pose_1_7.at<double>(0,5)=mat_pose.at<double>(1,3);
   mat_pose_1_7.at<double>(0,6)=mat_pose.at<double>(2,3);


   for (int i = 0;i<7;i++)
   {
      posefile->at<double>(count-1,i)=mat_pose_1_7.at<double>(0,i);

   }
   cout<<"---------------"<<endl;
   cout<<mat_pose_1_7<<endl;
   cout<<" --------------"<<endl;

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "move_sortjoint_node");
  ros::start();
  ros::NodeHandle n;
     if(argc!=2)
    {

        cerr<<"args error : Number of pose "<<endl;
    }

    int nRows=atoi(argv[1]);
    cout<<"Number of pose : "<<nRows<<endl;
    string move_group_name="arm_left";
    string referenceFrame="base_link";
    string file = "sortJoint.txt";
    string tmp;
    ifstream fin(file.c_str());



    Mat Jointstate(nRows,8,CV_64F);
    Mat mPose(nRows,7,CV_64F);

    vector<int> mIndex;


    double j1,j2,j3,j4,j5,j6,j7,dindex;
    int index;
    for (int i = 0 ;i<nRows;i++)
    {
        getline(fin,tmp);
        sscanf(tmp.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf ",&j1,&j2,&j3,&j4,&j5,&j6,&j7,&dindex);
        index= static_cast<int> (dindex);
        Jointstate.at<double>(i,0)=j1;
        Jointstate.at<double>(i,1)=j2;
        Jointstate.at<double>(i,2)=j3;
        Jointstate.at<double>(i,3)=j4;
        Jointstate.at<double>(i,4)=j5;
        Jointstate.at<double>(i,5)=j6;
        Jointstate.at<double>(i,6)=j7;
        mIndex.push_back(index);

    }
   cout<<"done "<<endl;

   cout<<"index size : "<<mIndex.size()<<endl;
   // JointStateRecorder  jointrecorder;
     ImageConverter ic;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate loop(10);
    for (int i = 0; i<nRows;i++)
    {
       int current_index = mIndex[i];
       cout<<"......................."<<current_index<<"............................."<<endl;
                if(movetoPoseJointstate_local(move_group_name,referenceFrame,Jointstate(Range(i,i+1),Range(0,7)),7))
                {
                  bool iscontinue=true;
                  while(iscontinue)
                  {
                      if( kbhit())   //space down
                       {
                         char c=getchar();
                         if(c==KEYCODE_SPACE)
                         {
                           iscontinue = false;
                         }

                      }
                      loop.sleep();
                  }
                  ros::spinOnce();

                         tf::StampedTransform pose;

                          if(findPoseture("/arm_left_link_7_t","/base_link",&pose))
                           {
                                            saveData(ic.currentframe,current_index,pose,&mPose);
                           }

                cout<<" Done ! current count  "<<i+1<<endl;
                }
                else
                {

                    ROS_ERROR("!!!  MOVE TO JOINTVALUE FAILED");

                }
      }

    cout<<mPose<<endl;
    FileStorage fs("sortpose.xml",FileStorage::WRITE);
    fs<<"sortpose"<<mPose;
    fs.release();

}


bool movetoPoseJointstate_local(String jointgroup,String referenceFrame,Mat jointstate,int nJoint)
{

if((jointstate.cols==nJoint && jointstate.rows==1 ))
{
        ROS_INFO("JOINTSTATE VALUE IS GOOD ");
 }
 else
{
        ROS_ERROR("JOINTSTATE VAULE IS WRONG");
        return false;
}

   std::vector<double> pose;
    pose.resize(nJoint);
    cout<<"NUMBER OF JOINT "<<nJoint<<endl;
    for(int i=0;i<nJoint;i++)
    {
        pose[i] = jointstate.at<double>(0,i);

    }

      moveit::planning_interface::MoveGroup::Plan my_plan;
      moveit::planning_interface::MoveGroup Jointgroup(jointgroup);
      Jointgroup.setPoseReferenceFrame(referenceFrame);
      Jointgroup.setPlannerId("RRTConnectkConfigDefault");

      Jointgroup.setJointValueTarget(pose);
      bool success =Jointgroup.plan(my_plan);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      cout<<"!!!!!!!!!!!!!!"<<endl;
      sleep(10.0);
     if(success)
     { Jointgroup.execute(my_plan);

         return true;
     }
     else
     {
     return false ;
     }
      ros::spinOnce();

}

