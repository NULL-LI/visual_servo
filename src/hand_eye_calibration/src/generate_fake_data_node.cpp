#include "calibrationbase.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <iostream>
#include <fstream>
//OPENCV INCLUDES
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "generate_fake_data_node");
  ros::start();


  if(argc!=2)
  {
      cerr<<" args : number of pose "<<endl;
      return -1;
  }
//robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model_frame: %s ",kinematic_model->getModelFrame().c_str());   //model frame : base_link
 //Planning Scene

  planning_scene::PlanningScene planning_scene(kinematic_model);

 // kinematic state
  robot_state::RobotState& Current_state= planning_scene.getCurrentStateNonConst();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm_left");
  const std::vector<std::string> &joint_names  = joint_model_group->getJointModelNames();
  int nJoint=joint_names.size();
  for (int i=0 ;i<nJoint;i++)
  {
      cout<<joint_names[i]<<endl;
  }
  ROS_INFO("Generate_fake_data transform arm_left_link_7_t to /base_link ");

  int PoseNumber=atoi(argv[1]);

  char filename_pose[] = "test_robotpose.m";
  char filename_jointstate[] = "test_jointstate.m";
  ofstream fout(filename_pose);
  ofstream fout_joint(filename_jointstate);
  fout<<"robotpose"<<"=[ ";
  fout_joint<<"robot_jointstate"<<"= [";


  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  for (int i=0;i<PoseNumber;i++)
      {
               collision_result.clear();
               Current_state.setToRandomPositions(joint_model_group);
            //   kinematic_state->setToRandomPositions(joint_model_group);
               planning_scene.checkSelfCollision(collision_request, collision_result);

               if(collision_result.collision)
               {
                 cout<<"Bad joint value"<<endl;
                 continue;
               }
               const Eigen::Affine3d &end_effector_state = Current_state.getGlobalLinkTransform("arm_left_link_7_t");
               Mat rotation(3,3,CV_64F);
                   for (int x=0;x<3 ;x++)
                   {
                       for(int y=0;y<3;y++)  rotation.at<double>(x,y)=end_effector_state(x,y);  }
               Mat translate(1,3,CV_64F);
                   for (int x=0;x<3  ;x++) translate.at<double>(0,x)=end_effector_state(x,3);

                  Mat quat     = dcm2quat(rotation);
           fout<<quat.at<double>(0,0)<<" "<<quat.at<double>(0,1)<<" "<<quat.at<double>(0,2)<<" "<<quat.at<double>(0,3)<<" ";
           fout<<translate.at<double>(0,0)*1000<<" "<<translate.at<double>(0,1)*1000<<" "<<translate.at<double>(0,2)*1000<<" ; "<<endl;

                 //get joint value
            std::vector<double> joint_values;
            Current_state.copyJointGroupPositions(joint_model_group, joint_values);

            fout_joint<<joint_values[0]<<" "<<joint_values[1]<<" "<<joint_values[2]<<" "<<joint_values[3]<<" "<<joint_values[4]<<" ";
            fout_joint<<joint_values[5]<<" "<<joint_values[6]<<" ;"<<endl;

      }
  fout<<"]"<<endl;
  fout.close();

  fout_joint<<"]"<<endl;
  fout_joint.close();

}


