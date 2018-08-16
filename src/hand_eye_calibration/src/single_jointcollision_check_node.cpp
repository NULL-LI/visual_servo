#include "calibrationbase.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
//OPENCV INCLUDES


using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "single_jointcollision_check_node");
  ros::start();

//robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model_frame: %s ",kinematic_model->getModelFrame().c_str());   //model frame : base_link
 //Planning Scene
  planning_scene::PlanningScene planning_scene(kinematic_model);

  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm_left");
  const std::vector<std::string> &joint_names  = joint_model_group->getJointModelNames();

//set joint value

      std::vector<double> joint_values;
      joint_values.push_back(0.0);
      joint_values.push_back(-0.1823999999999999);
      joint_values.push_back( -0.0005899999999998684);
      joint_values.push_back(-0.856);
      joint_values.push_back(0.0);
      joint_values.push_back(-1.81);
      joint_values.push_back(-0.57592);
      joint_values.push_back(0);

     for(int i = 0 ;i<joint_names.size();i++)
     {
      cout<<joint_values[i]<<endl;
     }


//set position
     robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
     current_state.setJointGroupPositions(joint_model_group,joint_values);

     current_state.copyJointGroupPositions(joint_model_group, joint_values);
     for(std::size_t i = 0; i < joint_names.size(); ++i)
     {
       ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);

     }

 //check
     collision_detection::CollisionRequest collision_request;
     collision_detection::CollisionResult collision_result;


     robot_state::RobotState current_robot_state =planning_scene.getCurrentState();
     cout<<planning_scene.isStateColliding(current_robot_state,"arm_left")<<endl;

     collision_request.contacts=true;


               planning_scene.checkCollision(collision_request, collision_result);
               if(collision_result.collision)
               {
                 cout<<"Bad joint value"<<endl;

               }
               else
                  {

                   cout<<"good "<<endl;
               }


}






