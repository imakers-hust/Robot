#include <moveit/move_group_interface/move_group_interface.h>   // replace the old version "move_group.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/Float64.h>
#include <kinova_driver/kinova_ros_types.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <iostream>
#include <vector>
#include <tf2_ros/transform_listener.h>
//手指控制
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
// 消息定义
#include <std_msgs/Int8.h>
#include "kinova_arm_moveit_demo/targetsVector.h"   //自定义消息类型，所有识别定位结果
#include "kinova_arm_moveit_demo/targetState.h"     //自定义消息类型，单个识别定位结果
#include "rviz_teleop_commander/targets_tag.h"      //自定义消息类型，传递要抓取的目标标签
#include "rviz_teleop_commander/grab_result.h"      //自定义消息类型，传递当前抓取的目标标签和抓取次数 -- 用于在rviz界面显示当前抓取编号和抓取次数
#include <Eigen/Eigen>
#include <sensor_msgs/JointState.h>                 //关节位置信息

using namespace std;
using namespace Eigen;


void getJointState(sensor_msgs::JointState curState);
std_msgs::Float64 joint1;
std_msgs::Float64 joint2;
std_msgs::Float64 joint3;
std_msgs::Float64 joint4;
std_msgs::Float64 joint5;
std_msgs::Float64 joint6;
std_msgs::Float64 joint7;
std_msgs::Float64 finger1;
std_msgs::Float64 finger2;
std_msgs::Float64 finger3;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_pos_pub");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/j2s7s300/joint_states", 1, getJointState);// Subscribing the joint_states and record them.
  ros::Publisher pub1 = n.advertise<std_msgs::Float64>("/joint1",1);
  ros::Publisher pub2 = n.advertise<std_msgs::Float64>("/joint2",1);
  ros::Publisher pub3 = n.advertise<std_msgs::Float64>("/joint3",1);
  ros::Publisher pub4 = n.advertise<std_msgs::Float64>("/joint4",1);
  ros::Publisher pub5 = n.advertise<std_msgs::Float64>("/joint5",1);
  ros::Publisher pub6 = n.advertise<std_msgs::Float64>("/joint6",1);
  ros::Publisher pub7 = n.advertise<std_msgs::Float64>("/joint7",1);
  ros::Publisher pub8 = n.advertise<std_msgs::Float64>("/finger1",1);
  ros::Publisher pub9 = n.advertise<std_msgs::Float64>("/finger2",1);
  ros::Publisher pub10 = n.advertise<std_msgs::Float64>("/finger3",1);


  usleep(1000000);//Leave 1s for building the subscribers and publishers

  while(ros::ok())
  {
    pub1.publish(joint1);
    pub2.publish(joint2);
    pub3.publish(joint3);
    pub4.publish(joint4);
    pub5.publish(joint5);
    pub6.publish(joint6);
    pub7.publish(joint7);
    pub8.publish(finger1);
    pub9.publish(finger2);
    pub10.publish(finger3);

  };

  return 0;
}

void getJointState(sensor_msgs::JointState curState)
{
  joint1.data = curState.position[0];
  joint2.data = curState.position[1];
  joint3.data = curState.position[2];
  joint4.data = curState.position[3];
  joint5.data = curState.position[4];
  joint6.data = curState.position[5];
  joint7.data = curState.position[6];
  finger1.data = curState.position[7];
  finger2.data = curState.position[8];
  finger3.data = curState.position[9];

}
