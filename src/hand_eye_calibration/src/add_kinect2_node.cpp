
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

  ros::init(argc, argv, "add_kinect2_node");
  ros::start();
  ros::NodeHandle node_handle;


  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

   ros::WallDuration sleep_t(0.5);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {

    sleep_t.sleep();
  }
 //get diff subscribe

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "base_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "base_link";
  /* The id of the object */
  attached_object.object.id = "kinect2";


  geometry_msgs::Pose pose;


  pose.orientation.w = -0.3401;
  pose.orientation.x = 0.6269;
  pose.orientation.y = -0.6175 ;
  pose.orientation.z = 0.3318;
  pose.position.x = 0.2621;
  pose.position.y = -0.0007;
  pose.position.z = 1.0582;



  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.26;
  primitive.dimensions[1] = 0.07;
  primitive.dimensions[2] = 0.07;

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
