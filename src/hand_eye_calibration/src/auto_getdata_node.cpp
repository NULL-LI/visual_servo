
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
using namespace std;
using namespace cv;

const string JOINTSTATE_FILENAME="jointstate.xml";

//PARAMETERS

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
int image_count=1;
bool iscontinue=true;
string  image_topic_name;
string  output_path;
string  robot_end_link;
string robot_base_link;
const string  IMAGE_WINDOW_NAME="image_viewer";
const string   ROBOT_POSE_FILENAME="robotpose.xml";
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : it_(nh_)
  {
      image_sub_ = it_.subscribe(image_topic_name, 3,
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
     cout<<"image size "<< image.rows<<endl;
    cv::imshow(IMAGE_WINDOW_NAME, image_resized);
    cv::waitKey(10);
  }
};
void saveData( Mat image,tf::StampedTransform pose)
{
   char imagename[100];
   sprintf(imagename,"frame%d.jpg",image_count);
   imwrite(imagename,image);
   cout<<"save image "<<imagename<<endl;

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

    if(image_count==1)  //first time
        {
            FileStorage fs(ROBOT_POSE_FILENAME,FileStorage::WRITE);
            fs<<"robotpose"<<mat_pose_1_7;
            fs.release();
        }
    else
        {
             Mat temp;
             FileStorage fsread(ROBOT_POSE_FILENAME,FileStorage::READ);
             fsread["robotpose"]>>temp;
             fsread.release();

             Mat robot_pose(image_count,7,CV_64F);
             for(int i=0;i<image_count-1;i++)
             {
                 for (int j=0;j<7;j++) robot_pose.at<double>(i,j)=temp.at<double>(i,j);
             }
             for (int j=0;j<7;j++)  robot_pose.at<double>(image_count-1,j)= mat_pose_1_7.at<double>(0,j);
             FileStorage fs(ROBOT_POSE_FILENAME,FileStorage::WRITE);
             fs<<"robotpose"<<robot_pose;
             fs.release();
        }

  image_count++;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "auto_getdata_node");
  ros::start();
  ros::AsyncSpinner spinner(4);
  spinner.start();



    if(argc != 6)
         {
             cerr<<"args error "<<endl;
             cerr<<"Usage rosrun Hand_Eye_calibration getdata_node  image_topic_name   robot_end_link  robot_base_link move_group_name referenceFrame"<<endl;
             return -1;
         }
    cout<<" =================================" <<endl;
    cout<< "image topic name :"<<endl;
    cout<< "     "<<argv[1]<<endl;

    Mat Jointstate;

    FileStorage fs(JOINTSTATE_FILENAME,FileStorage::READ);
    fs["robot_jointstate"]>>Jointstate;
    fs.release();
    if(Jointstate.empty())
    {
       cerr<<"Jointstate file empty "<<endl;
       return -1;
    }
    image_topic_name=argv[1];
    robot_end_link=argv[2];
    robot_base_link=argv[3];
    string move_group_name=argv[4];
    string referenceFrame=argv[5];  //base_link


    ros::NodeHandle n;
    ImageConverter ic;   //show image



    int nImages=Jointstate.rows;
    int nJoint =Jointstate.cols;


     for(int i=0;i<nImages;i++)
     {
        //MOVE TO POSE
         Mat mJointstate=Jointstate(Range(i,i+1),Range(0,nJoint));
         if(movetoPoseJointstate(move_group_name,referenceFrame,mJointstate,nJoint))
           {
            //SAVE IMAGE AND POSE
             ros::spinOnce();

             tf::StampedTransform pose;
                           if(findPoseture(robot_end_link,robot_base_link,&pose))
                           {
                                saveData(ic.currentframe,pose);
                           }
           }

         cout<<"..............................................."<<endl;
         sleep(1);


     }

     return 0;

}


