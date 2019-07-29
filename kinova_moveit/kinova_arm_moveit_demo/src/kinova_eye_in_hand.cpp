#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <std_msgs/Int8.h>
#include "kinova_arm_moveit_demo/toolposeChange.h"
#include <geometry_msgs/PoseStamped.h>

#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"

#define TF_EULER_DEFAULT_ZYX
//手指client类型自定义
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;

using namespace std;
using namespace Eigen;

const float deg2rad = M_PI / 180.0;//用于将角度转化为弧度。rad = deg*deg2rad
const float rad2deg = 180.0 / M_PI;//用于将弧度转化为角度。deg = rad*rad2deg
moveit::planning_interface::MoveGroup* gripper_group_=NULL;
geometry_msgs::Pose target_pose1;

geometry_msgs::Pose tempPose;
//定义机器人类型，手指控制 added by yang 20180418
string kinova_robot_type = "j2s7s300";
string Finger_action_address = "/" + kinova_robot_type + "_driver/fingers_action/finger_positions";    //手指控制服务器的名称
const double FINGER_MAX = 6400;	//手指开合程度：0完全张开，6400完全闭合
//定义手指控制client added by yang 20180418
Finger_actionlibClient* client=NULL;

//获取当前位姿 
geometry_msgs::Pose get_current_pose(moveit::planning_interface::MoveGroup &group)
{
	geometry_msgs::PoseStamped getPoseMsg;
  getPoseMsg = group.getCurrentPose();

  geometry_msgs::Pose current_pose;
  current_pose = getPoseMsg.pose;

	ROS_INFO("Current position :\n x[%f], y[%f], z[%f]", current_pose.position.x, current_pose.position.y, current_pose.position.z);
	ROS_INFO("Current orientation :\n x[%f], y[%f], z[%f], w[%f]", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
  return current_pose;
}
geometry_msgs::Pose get_current_pose()
{
  geometry_msgs::Pose current_pose;
  current_pose = tempPose;
  ROS_INFO("Current position :\n x[%f], y[%f], z[%f]", current_pose.position.x, current_pose.position.y, current_pose.position.z);
  ROS_INFO("Current orientation :\n x[%f], y[%f], z[%f], w[%f]", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
  return current_pose;
}

//当前位姿geometry_msgs::Pose转为MatrixXd齐次变换矩阵的格式
MatrixXd pose2matrix(const geometry_msgs::Pose &pose_)
{
  VectorXd pose_t= MatrixXd::Zero(3, 1);		//位置向量
	Matrix3d pose_r;	//旋转矩阵
  //四元数转旋转矩阵
	Quaterniond pose_r_q(pose_.orientation.w,
											 pose_.orientation.x,
											 pose_.orientation.y,
											 pose_.orientation.z);
	pose_r=pose_r_q.matrix();
  //当前位姿的位置向量
	pose_t<<pose_.position.x, pose_.position.y, pose_.position.z;
  //转换为4X4矩阵
  MatrixXd pose_m = MatrixXd::Zero(4, 4);
  for(int j=0; j<3; j++)
  {
    for(int k=0; k<3; k++)	
	  {
	    pose_m(j,k)=pose_r(j,k);
	  }		
  }
	for(int j=0; j<3; j++)
	{
	  pose_m(j,3)=pose_t(j);
	}
	pose_m(3,3)=1.0;

  return pose_m;
}

//获取old位姿坐标系下，从old到new的位置和姿态增量
MatrixXd getPoseIncreasement(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &old_pose)
{
	VectorXd poseCur_t= MatrixXd::Zero(3, 1);		//当前位姿位置
	Matrix3d poseCur_r;	//当前位姿旋转矩阵
	VectorXd poseOld_t= MatrixXd::Zero(3, 1);	//之前位姿位置
	Matrix3d poseOld_r;	//之前位姿旋转矩阵
	//当前位姿的四元数转旋转矩阵
	Quaterniond poseCur_r_q(current_pose.orientation.w,
													current_pose.orientation.x,
													current_pose.orientation.y,
													current_pose.orientation.z);
	poseCur_r=poseCur_r_q.matrix();
	//当前位姿的位置向量
	poseCur_t<<current_pose.position.x, current_pose.position.y, current_pose.position.z;
	//之前位姿的四元数转旋转矩阵
	Quaterniond poseOld_r_q(old_pose.orientation.w,
													old_pose.orientation.x,
													old_pose.orientation.y,
													old_pose.orientation.z);
	poseOld_r=poseOld_r_q.matrix();
	//之前位姿的位置向量
	poseOld_t<<old_pose.position.x, old_pose.position.y, old_pose.position.z;

	//位姿增量计算
	Matrix3d R_old_new;
  VectorXd t_old_new(3);
  R_old_new = poseOld_r.transpose()*poseCur_r;
  t_old_new = poseOld_r.transpose()*poseCur_t-poseOld_r.transpose()*poseOld_t;

	//转换为4X4矩阵
  MatrixXd old_new = MatrixXd::Zero(4, 4);
  for(int j=0; j<3; j++)
  {
    for(int k=0; k<3; k++)	
	{
	  old_new(j,k)=R_old_new(j,k);
	}		
  }
	for(int j=0; j<3; j++)
	{
	  old_new(j,3)=t_old_new(j);
	}
	old_new(3,3)=1.0;

	cout<<"old_new :"<<endl<<old_new<<endl;

  return old_new;
}

//ZYX欧拉角(角度)转四元数
Quaterniond  Euler_to_Quaterniond(double yaw, double pitching, double droll)
{
	//角度转弧度
	yaw=yaw*deg2rad;
  pitching=pitching*deg2rad;
  droll=droll*deg2rad;

  Vector3d ea0(yaw, pitching, droll);
  Matrix3d R;
  R = AngleAxisd(ea0[0], Vector3d::UnitZ())
      * AngleAxisd(ea0[1], Vector3d::UnitY())
      * AngleAxisd(ea0[2], Vector3d::UnitX());

  //cout << R << endl << endl;  

  //RotationMatrix to Quaterniond  
  Quaterniond q;
  q = R;
  cout << "x:" << q.x() << "\t" << "y:" << q.y() << "\t" << "z:" << q.z() << "\t" << "w:" << q.w() << endl;
  return q;
}

void tool_pose_CB(const geometry_msgs::PoseStamped &msg)
{
	tempPose=msg.pose;
}

//手抓控制函数，输入0-1之间的控制量，控制手抓开合程度，0完全张开，1完全闭合 added by yang 20180418
bool fingerControl(double finger_turn)
{
    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, 1.0);
    }
    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn * FINGER_MAX;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    client->sendGoal(goal);
    if (client->waitForResult(ros::Duration(5.0)))
    {
        client->getResult();
        return true;
    }
    else
    {
        client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eye2hand_calib_control");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("arm");		//这句话的作用？
	gripper_group_ = new moveit::planning_interface::MoveGroup("gripper");
  // Add by Petori in 2018/4/8
  group.setEndEffectorLink( "j2s7s300_end_effector");	//这句话的作用？

  //　实物控制的话，得删掉这两句－－删掉是为了减少Rviz的使用所占用的时间
  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;
  

  //发布的消息
  ros::Publisher signal_pub = node_handle.advertise<std_msgs::Int8>("hand_eye_arm_signal", 1);  //到达位姿后，发送标志位
  ros::Publisher pose_change_pub = node_handle.advertise<kinova_arm_moveit_demo::toolposeChange>("hand_eye_arm_pose_change", 10);//发送末端位姿变化
  ros::Publisher pose_pub = node_handle.advertise<kinova_arm_moveit_demo::toolposeChange>("hand_eye_arm_pose", 10);//发送末端位姿齐次变换矩阵

   //订阅话题
  ros::Subscriber too_pose_sub = node_handle.subscribe("/j2s7s300_driver/out/tool_pose", 1, tool_pose_CB);    //接收visual_detect检测结果

	client = new Finger_actionlibClient(Finger_action_address, true);
	sleep(3.0);
  //抓紧手指
	fingerControl(0.1);//open
	//gripper_group_->setNamedTarget("Open");
	//gripper_group_->move();
	//sleep(5.0);
	//std::vector< double > jointValues_finger(3);
	//jointValues_finger[0]=1.4;
	//jointValues_finger[1]=1.4;
	//jointValues_finger[2]=1.4;
	//gripper_group_->setJointValueTarget(jointValues_finger);
	//gripper_group_->move();
	sleep(5.0);
	fingerControl(1);//open


  std_msgs::Int8 flag;	//到达位姿后，发送标志位
  flag.data=1;
  Quaterniond quater;
  tf::Quaternion quaternion;        //相对于基座的旋转角
  tf::Quaternion quaternion_add;		//相对于当前末端的旋转角增量
  vector<tfScalar> rpy(3);   //设置相对于基座标的旋转，采用ZYX固定轴角的方式
  vector<double> zyx(3);     //相对于基座的位移
	/**************************************************************************************************************************/
	/****************************************************位姿0*****************************************************************/
	/**************************************************************************************************************************/
	ROS_INFO("STATE 0");

	std::vector< double > jointValues(7);
	jointValues[0]=-139.0 * deg2rad;
	jointValues[1]=286.36 * deg2rad;
	jointValues[2]=351.48 * deg2rad;
	jointValues[3]=103.39 * deg2rad;
	jointValues[4]=231.31 * deg2rad;
	jointValues[5]=263.31 * deg2rad;
	jointValues[6]=356.26 * deg2rad;
    group.setJointValueTarget(jointValues);
	group.move();

	//geometry_msgs::Pose current_pose1;	//获取当前末端位姿
	//current_pose1=get_current_pose(group);

	//return 1;	
/*
	tf::Pose random_pose;
  zyx[0]=-0.481602;			//-0.481602  -0.3
  zyx[1]=-0.546735;			//-0.546735  -0.25
  zyx[2]=0.224044;			//0.224044   0.4
  random_pose.setOrigin(tf::Vector3(zyx[0], zyx[1],  zyx[2]));
  //quater=Euler_to_Quaterniond(90,0,-90);
  //random_pose.setRotation(tf::Quaternion(quater.x(), quater.y(), quater.z(), quater.w()));
  rpy[0]=100*deg2rad;	//90    -95*deg2rad
  rpy[1]=0*deg2rad;	//0     -70*deg2rad
  rpy[2]=95*deg2rad;	//90	-100*deg2rad
  quaternion.setRPY(rpy[0], rpy[1], rpy[2]);   //设置初始值
  quaternion_add.setRPY(0,0,10*deg2rad);     //设置增量大小
  quaternion*=quaternion_add;
  random_pose.setRotation(quaternion);
  geometry_msgs::Pose target_pose;
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
*/
	int k=0;
	while(k<15)
	{
      if(!ros::ok())
      {
         return -1;
      }
			sleep(1.0);
			k++;
	}

	geometry_msgs::Pose current_pose, old_pose;	//获取当前末端位姿
	//current_pose=get_current_pose(group);
	current_pose=get_current_pose();
	old_pose=current_pose;

  //发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=2;
	
	/**************************************************************************************************************************/
	/****************************************************位姿1*****************************************************************/
	/**************************************************************************************************************************/
	ROS_INFO("STATE 1");

	jointValues[0]=-135.32 * deg2rad;
	jointValues[1]=286.36 * deg2rad;
	jointValues[2]=352.45 * deg2rad;
	jointValues[3]=107.24 * deg2rad;
	jointValues[4]=229.87 * deg2rad;
	jointValues[5]=261.48 * deg2rad;
	jointValues[6]=376.85 * deg2rad;
    group.setJointValueTarget(jointValues);
	group.move();


/*
  zyx[0]+=0.1;
  zyx[1]-=0.1;
	random_pose.setOrigin(tf::Vector3(zyx[0], zyx[1],  zyx[2]));
  quaternion_add.setRPY(10*deg2rad,10*deg2rad,0);     //设置增量大小
  quaternion*=quaternion_add;
  random_pose.setRotation(quaternion);
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
*/
	
	k=0;
	while(k<15)
	{
      if(!ros::ok())
      {
         return -1;
      }
			sleep(1.0);
			k++;
	}
	//current_pose=get_current_pose(group);	//获取当前末端位姿
	current_pose=get_current_pose();
	MatrixXd d_pose=MatrixXd::Zero(4, 4);	//位姿增量
	d_pose=getPoseIncreasement(current_pose,old_pose);
	old_pose=current_pose;

  MatrixXd pose_m=MatrixXd::Zero(4, 4);   //当前位置，基坐标系下末端位姿齐次变换矩阵
  kinova_arm_moveit_demo::toolposeChange poseChange;   //末端位姿齐次变换矩阵
	kinova_arm_moveit_demo::toolposeChange pose;   //基坐标系下末端位姿
  //当前末端位姿
  pose_m=pose2matrix(current_pose); 
  //发送运动前到运动后末端位姿的变换矩阵
	for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)	
	  {
	    poseChange.pose_change[4*i+j]=d_pose(i,j);
		  pose.pose_change[4*i+j]=pose_m(i,j);
	  }		
  }
  pose_change_pub.publish(poseChange);
	pose_pub.publish(pose);
  //发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=3;

	/**************************************************************************************************************************/
	/****************************************************位姿2*****************************************************************/
	/**************************************************************************************************************************/
	ROS_INFO("STATE 2");

	jointValues[0]=-139.77 * deg2rad;
	jointValues[1]=288.32 * deg2rad;
	jointValues[2]=351.96 * deg2rad;
	jointValues[3]=104.8 * deg2rad;
	jointValues[4]=215.88 * deg2rad;
	jointValues[5]=267.98 * deg2rad;
	jointValues[6]=381.77 * deg2rad;

    group.setJointValueTarget(jointValues);
	group.move();

/*
  zyx[0]+=0.05;
  zyx[2]+=0.1;
	random_pose.setOrigin(tf::Vector3(zyx[0], zyx[1],  zyx[2]));
  quaternion_add.setRPY(0,-10*deg2rad,15*deg2rad);     //设置增量大小
  quaternion*=quaternion_add;
  random_pose.setRotation(quaternion);
  
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
*/
	k=0;
	while(k<15)
	{
      if(!ros::ok())
      {
         return -1;
      }
			sleep(1.0);
			k++;
	}

	//current_pose=get_current_pose(group);	//获取当前末端位姿
	current_pose=get_current_pose();
    d_pose=getPoseIncreasement(current_pose,old_pose);
	old_pose=current_pose;

  //当前末端位姿
  pose_m=pose2matrix(current_pose); 
  //发送运动前到运动后末端位姿的变换矩阵
	for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)	
	  {
	    poseChange.pose_change[4*i+j]=d_pose(i,j);
		  pose.pose_change[4*i+j]=pose_m(i,j);
	  }		
  }
  pose_change_pub.publish(poseChange);
	pose_pub.publish(pose);
  //发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=4;

	/**************************************************************************************************************************/
	/****************************************************位姿3*****************************************************************/
	/**************************************************************************************************************************/
	ROS_INFO("STATE 3");

	jointValues[0]=-140.34 * deg2rad;
	jointValues[1]=279.01 * deg2rad;
	jointValues[2]=350.85 * deg2rad;
	jointValues[3]=93.77 * deg2rad;
	jointValues[4]=209.85 * deg2rad;
	jointValues[5]=255.85 * deg2rad;
	jointValues[6]=357.56 * deg2rad;

    group.setJointValueTarget(jointValues);
	group.move();
/*
  zyx[0]-=0.1;
	random_pose.setOrigin(tf::Vector3(zyx[0], zyx[1],  zyx[2]));
  quaternion_add.setRPY(5*deg2rad,0,-10*deg2rad);     //设置增量大小
  quaternion*=quaternion_add;
  random_pose.setRotation(quaternion);
  
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
*/
	k=0;
	while(k<15)
	{
      if(!ros::ok())
      {
         return -1;
      }
			sleep(1.0);
			k++;
	}
	//current_pose=get_current_pose(group);
	current_pose=get_current_pose();
  d_pose=getPoseIncreasement(current_pose,old_pose);
	old_pose=current_pose;

  //当前末端位姿
  pose_m=pose2matrix(current_pose); 
  //发送运动前到运动后末端位姿的变换矩阵
	for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)	
	  {
	    poseChange.pose_change[4*i+j]=d_pose(i,j);
		  pose.pose_change[4*i+j]=pose_m(i,j);
	  }		
  }
  pose_change_pub.publish(poseChange);
	pose_pub.publish(pose);
  //发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=5;

	/**************************************************************************************************************************/
	/****************************************************位姿4*****************************************************************/
	/**************************************************************************************************************************/
	ROS_INFO("STATE 4");

	jointValues[0]=-134.93 * deg2rad;
	jointValues[1]=283.84 * deg2rad;
	jointValues[2]=352.48 * deg2rad;
	jointValues[3]=104.42 * deg2rad;
	jointValues[4]=230.28 * deg2rad;
	jointValues[5]=257.60 * deg2rad;
	jointValues[6]=347.59 * deg2rad;

    group.setJointValueTarget(jointValues);
	group.move();
/*
  zyx[0]-=0.05;
  zyx[1]+=0.1;
  zyx[2]+=0.1;
	random_pose.setOrigin(tf::Vector3(zyx[0], zyx[1],  zyx[2]));
  quaternion_add.setRPY(5*deg2rad,10*deg2rad,-5*deg2rad);     //设置增量大小
  quaternion*=quaternion_add;
  random_pose.setRotation(quaternion);
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
*/

	k=0;
	while(k<15)
	{
      if(!ros::ok())
      {
         return -1;
      }
			sleep(1.0);
			k++;
	}

	//current_pose=get_current_pose(group);	//获取当前末端位姿
	current_pose=get_current_pose();
  d_pose=getPoseIncreasement(current_pose,old_pose);
	old_pose=current_pose;


  //当前末端位姿
  pose_m=pose2matrix(current_pose); 
  //发送运动前到运动后末端位姿的变换矩阵
	for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)	
	  {
	    poseChange.pose_change[4*i+j]=d_pose(i,j);
		  pose.pose_change[4*i+j]=pose_m(i,j);
	  }		
  }
  pose_change_pub.publish(poseChange);
	pose_pub.publish(pose);
  //发送标志位
	signal_pub.publish(flag);
	ros::Duration(2).sleep();
	flag.data=6;
  
  //释放标定板
	sleep(10.0);
	//gripper_group_->setNamedTarget("Open");
	//gripper_group_->move();

  sleep(1.0);
  ros::shutdown();  
  return 0;
}
