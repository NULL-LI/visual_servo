#include"calibrationbase.h"

#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sensor_msgs/JointState.h>
using namespace std;
using namespace cv;
const string JOINTSTATE_FILENAME="jointstate.xml";
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

class JointStateRecorder
{
  ros::NodeHandle nh_;
  ros::Subscriber sub ;

public:
    JointStateRecorder(int JointNumber,string firstJointName)
//    : it_(nh_)
  {
    sub = nh_.subscribe("joint_states", 1, &JointStateRecorder::JointstateCallback,this);
    mJointNum=JointNumber;
    count =0;
    mFirstJointname=firstJointName;
  }


  ~JointStateRecorder()
  {
   destroyWindow("shadow");
  }

// sensor_msgs::JointState::Ptr current_jointstate_ptr;
 vector<double>mJointstate;
 int  mJointNum;
 int  count ;
 string mFirstJointname;
 bool recorddata()
 {
      cout<<"Jointstate record  Success "<<"count = "<<count<<endl;
      count=count+1;
      Mat jointstateMat(count,mJointNum,CV_64F);
     if(count!=1)
          {
              FileStorage fs1(JOINTSTATE_FILENAME,FileStorage::READ);
              Mat temp;
              fs1["robot_jointstate"]>>temp;

          for(int i=0;i<count-1;i++)
              {
                 for(int j=0;j<mJointNum;j++)
                  {
                     jointstateMat.at<double>(i,j)=temp.at<double>(i,j);
                  }
              }
              fs1.release();
           }

          FileStorage fs(JOINTSTATE_FILENAME,FileStorage::WRITE);
           for(int i=0;i<mJointNum;i++)  {
                 jointstateMat.at<double>(count-1,i)=mJointstate[i];

              }
          fs<<"robot_jointstate"<<jointstateMat;
          fs.release();

 }

    void JointstateCallback(const sensor_msgs::JointState& jointstate)
   {
        if(jointstate.name[0]==mFirstJointname)
        {
            mJointstate.clear();
            for (int i=0;i<mJointNum;i++)
            {
              mJointstate.push_back(jointstate.position[i]);
            }
        }
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_robot_jointstate_node");
    ros::start();
    if(argc!=3)
    {
        cerr<<"Error : argc error "<<endl;
        cout<<"Usage :rosrun Hand_Eye_calibration get_robot_jointstate_node  number_of_joint first_joint_name"<<endl;
        return -1;
    }

   int jointNum=atoi(argv[1]);
   string firstJointName=argv[2];

    JointStateRecorder  jointstate(jointNum,firstJointName);
    ros::Rate loop(60);

    while(true)
       {
           ros::spinOnce();
           loop.sleep();
           if( kbhit())   //space down
            {
              char c=getchar();
              if(c==KEYCODE_ESC)
              {
                 cout<<"shuting down..."<<endl;
                 break;
              }
              jointstate.recorddata();
           }

       }



    return 0;
}
