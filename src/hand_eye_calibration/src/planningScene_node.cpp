
#include "calibrationbase.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "planningScene_node");
  ros::start();
  ros::NodeHandle node_handle;
  if(argc!=8)
  {
     cout<<"args error "<<endl;
     cout<<"tx ty tz qw qx qy qz"<<endl;
     return -1;
  }

  double qw ,qx,qy,qz;
  double tx,ty,tz;
  tx = atof(argv[1]);
  ty = atof(argv[2]);
  tz = atof(argv[3]);


  qw = atof(argv[4]);
  qx = atof(argv[5]);
  qy = atof(argv[6]);
  qz = atof(argv[7]);


  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

   ros::WallDuration sleep_t(0.5);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {

    sleep_t.sleep();
  }
 //get diff subscribe

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "arm_left_link_7_t";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "arm_left_link_7_t";
  /* The id of the object */
  attached_object.object.id = "calibration_board";

  /* A default pose */
  cout<<tx <<endl;
  cout<<ty<<endl;
  geometry_msgs::Pose pose;
  pose.orientation.w = -qw;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.position.x = 0.001*tx;
  pose.position.y = 0.001*ty;
  pose.position.z = 0.001*tz;



  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.2;
  primitive.dimensions[1] = 0.2;
  primitive.dimensions[2] = 0.002;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  planning_scene.world.collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);


  sleep_t.sleep();

}
