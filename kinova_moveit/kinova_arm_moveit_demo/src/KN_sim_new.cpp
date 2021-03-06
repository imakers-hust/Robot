#include <moveit/move_group_interface/move_group_interface.h>   // replace the old version "move_group.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
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

//-------------------------------------------------全局变量--------------------------------------------------
const int N_MAX=70;                                 //循环抓取允许最大识别不到的次数，超出此次数识别结束
vector<kinova_arm_moveit_demo::targetState> targets;//视觉定位结果
geometry_msgs::Pose startPose;                    //机械臂初始识别位置
geometry_msgs::Pose startPose1;                   //姿态备选
geometry_msgs::Pose startPose2;
geometry_msgs::Pose startPose3;
geometry_msgs::Pose startPose4;
geometry_msgs::Pose placePose;                      //机械臂抓取放置位置
sensor_msgs::JointState kinovaState;                //机械臂当前状态
vector<int> targetsTag;                           	//需要抓取的目标物的标签
bool getTargets=0;                                	//当接收到视觉定位结果时getTargets置1，执行完放置后置0
bool getTargetsTag=0;                             	//当接收到需要抓取的目标物的标签时置1
int poseChangeTimes=1;                              //当检测不到目标物体时,变换姿态重新检测的次数
double minimumDistance = 0.3;                       //允许距离目标物的最小距离,单位米
double servoCircle = 0.5;                           //伺服运动周期,单位秒

//-------------------------------------------------相机相关--------------------------------------------------
//相机参数和深度信息用于计算
#define Fxy 692.97839
#define UV0 400.5
#define Zw 0.77
//手眼关系定义--赋值在main函数中
Eigen::Matrix3d hand2eye_r;
Eigen::Vector3d hand2eye_t;
Eigen::Quaterniond hand2eye_q;

//-------------------------------------------------机器人相关------------------------------------------------
//定义机器人类型
string kinova_robot_type = "j2s7s300";
string Finger_action_address = "/" + kinova_robot_type + "_driver/fingers_action/finger_positions";    //手指控制服务器的名称
string base_frame = "base_link";// tf中基坐标系的名称
string tool_frame = "wrist_3_link";// tf中工具坐标系的名称
string camera_frame = "camera_color_optical_frame";// tf中相机坐标系的名称
string world_frame = "world";// tf中世界坐标系的名称
//手指client类型自定义
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;
//定义手指控制client
Finger_actionlibClient* client=NULL;
//爪子开闭程度
float openVal=0.4;
float closeVal=0.9;
float highVal=0.02;
const double FINGER_MAX = 6400;	//手指开合程度：0完全张开，6400完全闭合

//最合适的抓取参数--old        1       2      3      4      5      6      7      8      9      10
//float closeVals[10]=    {1.200, 0.900, 1.050, 1.150, 1.200, 1.050, 0.960, 1.300, 0.950, 1.200};// 爪子闭合程度
//float highVals[10]=     {0.065, 0.065, 0.050, 0.025, 0.040, 0.030, 0.020, 0.065, 0.050, 0.030};// 抓取高度
//float openVals[10]=     {0.900, 0.400, 0.400, 0.800, 1, 0.400, 0.400, 0.400, 0.800, 0.400};// 爪子张开程度

//2019仿真训练
//                        1      2      3       4     5       6     7       8      9     10
//最合适的抓取参数         三棱锥   圆锥   正方体  四棱锥  球     三棱柱   四棱台  圆柱   六棱柱  长方体
float highVals[10]=     {0.030, 0.050, 0.055, 0.050, 0.050, 0.045, 0.050, 0.030, 0.030, 0.025};// 抓取高度 0.05
float openVals[10]=     {1.100, 0.900, 0.600, 0.800, 0.900, 0.900, 0.800, 0.800, 0.800, 0.900};// 爪子张开程度  0.08
float closeVals[10]=    {1.300, 1.200, 1.100, 1.200, 1.200, 1.300, 1.200, 1.300, 1.200, 1.200};// 爪子闭合程度 0.950

// -------------------------------------------------函数定义-------------------------------------------------
//接收相机节点发过来的识别结果,更新全局变量targets
void detectResultCB(const kinova_arm_moveit_demo::targetsVector &msg);

//接收teleop指定的抓取序列号，更新全局变量targetsTag
void tagsCB(const rviz_teleop_commander::targets_tag &msg);

//手抓控制函数，输入0-1之间的控制量，控制手抓开合程度，0完全张开，1完全闭合
bool fingerControl(double finger_turn);

//机械臂运动控制函数
void pickAndPlace(kinova_arm_moveit_demo::targetState curTargetPoint, const tf2_ros::Buffer& tfBuffer_);

//抓取插值函数
std::vector<geometry_msgs::Pose> pickInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose);

//放置插值函数
std::vector<geometry_msgs::Pose> placeInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose);

//设置机械臂放置位置
void setPlacePose();
void setStartPose1();
void setStartPose2();
void setStartPose3();
void setStartPose4();

//前往视觉识别初始位置
void goStartPose();

//获取机器人当前信息
void getRobotInfo(sensor_msgs::JointState curState);

//判断目标物体是否存在
bool judgeIsExist(int tag, vector<kinova_arm_moveit_demo::targetState> targetAll);

//判断目标物体是否被遮挡
bool judgeIsHinder(int tag, vector<kinova_arm_moveit_demo::targetState> targetAll);

//获取遮挡物体的位置
kinova_arm_moveit_demo::targetState judgeTheObstacle(int tag, vector<kinova_arm_moveit_demo::targetState> targetAll);

//拾取遮挡物体
void pickTheObstacle(kinova_arm_moveit_demo::targetState targetNow, const tf2_ros::Buffer& tfBuffer_);

//放置遮挡物体
void placeTheObstacle(int tag, kinova_arm_moveit_demo::targetState targetNow, vector<kinova_arm_moveit_demo::targetState> targetAll, const tf2_ros::Buffer& tfBuffer_);

//计算与目标物体的绝对距离
double calcDistance(kinova_arm_moveit_demo::targetState targetNow);

//获取当前目标对象的位置
kinova_arm_moveit_demo::targetState getTargetPoint(int tag, vector<kinova_arm_moveit_demo::targetState> targetAll);

//接近目标物体
void approachTarget(int tag,kinova_arm_moveit_demo::targetState targetNow, sensor_msgs::JointState robotState);

//相机识别到的物体位姿转换到机器人基座标系下
kinova_arm_moveit_demo::targetState transTarget(const tf2_ros::Buffer& tfBuffer_, kinova_arm_moveit_demo::targetState targetNow);



/*************************************/
/********函数定义*********************/
/*************************************/

//接收到detect_result消息的回调函数，将消息内容赋值到全局变量targets里面
void detectResultCB(const kinova_arm_moveit_demo::targetsVector &msg)
{
	int num = msg.targets.size();
  targets.clear();
	targets.resize(num);
	for(int i=0; i<num; i++)
	{
    targets[i]=msg.targets[i];
  }
	getTargets=1;	//接收到视觉定位结果getTargets置1
}
//接收targets_tag消息的回调函数，将接收到的消息更新到targetsTag里面
void tagsCB(const rviz_teleop_commander::targets_tag &msg)
{
	int num=msg.targetsTag.size();
  targetsTag.clear();
  targetsTag.resize(num);
  ROS_INFO("Amount of targets = %d",num);
	for(int i=0; i<num; i++)
	{
		targetsTag[i]=msg.targetsTag[i];
		ROS_INFO(" [%d]", targetsTag[i]);
	}
	getTargetsTag=1;	//接收到需要抓取的目标物的标签
}


void pickAndPlace(kinova_arm_moveit_demo::targetState curTargetPoint, const tf2_ros::Buffer& tfBuffer_)
{
//流程介绍
//1--获得目标点并对路径进行插值
//2--执行插值后的路径
//3--到达目标点抓取物体
//4--从目标点到放置点进行插值
//5--执行插值后的路径
//6--放置物体
//7--等待下一个目标点
  moveit::planning_interface::MoveGroupInterface arm_group("arm");	// replace the old version "moveit::planning_interface::MoveGroupInterface"
  moveit::planning_interface::MoveGroupInterface *finger_group;
  finger_group = new moveit::planning_interface::MoveGroupInterface("gripper");
  geometry_msgs::Pose targetPose;	//定义抓取位姿
  geometry_msgs::Point point;
  geometry_msgs::Quaternion orientation;

  moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
  moveit::planning_interface::MoveGroupInterface::Plan place_plan;

  //抓取动作
  std::vector< double > jointValues;
  jointValues.push_back(openVal);
  jointValues.push_back(openVal);
  jointValues.push_back(openVal);
  finger_group->setJointValueTarget(jointValues);
  finger_group->move();
  jointValues.clear();

  //**************key*******************************
  curTargetPoint = transTarget(tfBuffer_, curTargetPoint);
  //**************key*******************************


  point.x = curTargetPoint.x;//获取抓取位姿
  point.y = curTargetPoint.y;
  point.z = highVal;


//  orientation.x = curTargetPoint.qx;//方向由视觉节点给定－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－Petori
//  orientation.y = curTargetPoint.qy;
//  orientation.z = curTargetPoint.qz;
//  orientation.w = curTargetPoint.qw;

    orientation.x = -0.988;
    orientation.y = 0.156;
    orientation.z = 0;
    orientation.w = 0;

  cout<<"The target position is x:["<<point.x<<"], y:["<<point.y<<"], z:["<<point.z<<"].";
  targetPose.position = point;// 设置好目标位姿为可用的格式
  targetPose.orientation = orientation;


  //抓取插值
  std::vector<geometry_msgs::Pose> pickWayPoints;
  pickWayPoints = pickInterpolate(placePose, targetPose);

  //前往抓取点
  moveit_msgs::RobotTrajectory trajectory1;
  arm_group.computeCartesianPath(pickWayPoints,
                                 0.02,  // eef_step
                                 0.0,   // jump_threshold
                                 trajectory1);
  pick_plan.trajectory_ = trajectory1;
  ROS_INFO("Prepare for picking ...");
  arm_group.execute(pick_plan);

  //抓取动作
  jointValues.push_back(closeVal);
  jointValues.push_back(closeVal);
  jointValues.push_back(closeVal);
  finger_group->setJointValueTarget(jointValues);
  finger_group->move();
  jointValues.clear();
  //抓取完毕

  ros::Duration(1).sleep();

  //放置插值
  std::vector<geometry_msgs::Pose> placeWayPoints;
  placeWayPoints = placeInterpolate(targetPose, placePose);

  //前往放置点
  moveit_msgs::RobotTrajectory trajectory2;
  arm_group.computeCartesianPath(placeWayPoints,
                                 0.02,  // eef_step
                                 0.0,   // jump_threshold
                                 trajectory2);
  place_plan.trajectory_ = trajectory2;
  ROS_INFO("Prepare for placing. ");
  arm_group.execute(place_plan);

  //松开爪子
  jointValues.push_back(0);
  jointValues.push_back(0);
  jointValues.push_back(0);
  finger_group->setJointValueTarget(jointValues);
  finger_group->move();
  jointValues.clear();
  //松开完毕

}
//抓取插值函数
std::vector<geometry_msgs::Pose> pickInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose)
{
  //从放置位置前往抓取位置
  //插值后路径为＂----|＂形（先平移，后下落）
  std::vector<geometry_msgs::Pose> pickWayPoints;

  geometry_msgs::Pose midPose4;
  geometry_msgs::Point startPoint;
  geometry_msgs::Point targetPoint;
  geometry_msgs::Point midPoint;

  startPoint = startPose.position;
  targetPoint = targetPose.position;

  // midPose4
  midPoint.x = targetPoint.x;
  midPoint.y = targetPoint.y;
  midPoint.z = startPoint.z;

  midPose4.position = midPoint;
  midPose4.orientation = targetPose.orientation;

  pickWayPoints.push_back(midPose4);

  // Give targetPose
  pickWayPoints.push_back(targetPose);

  return pickWayPoints;
}

//放置插值函数
std::vector<geometry_msgs::Pose> placeInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose)
{
  //从放置位置前往抓取位置
  //插值后路径为＂|----＂形（先抬升，后平移）
  std::vector<geometry_msgs::Pose> placeWayPoints;
  geometry_msgs::Pose midPose1;

  geometry_msgs::Point startPoint;
  geometry_msgs::Point targetPoint;
  geometry_msgs::Point midPoint;

  startPoint = startPose.position;
  targetPoint = targetPose.position;

  // midPose1
  midPoint.x = startPoint.x;
  midPoint.y = startPoint.y;
  midPoint.z = targetPoint.z;

  midPose1.position = midPoint;
  midPose1.orientation = targetPose.orientation;

  placeWayPoints.push_back(midPose1);

  // Give targetPose
  placeWayPoints.push_back(targetPose);

  return placeWayPoints;
}


void goStartPose()
{
  //前往视觉识别位置
  moveit::planning_interface::MoveGroupInterface arm_group("arm");
  std::vector<geometry_msgs::Pose> targetWayPoints;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan move_plan;


  targetWayPoints.push_back(startPose);
  arm_group.computeCartesianPath(targetWayPoints,
                                 0.02,  // eef_step
                                 0.0,   // jump_threshold
                                 trajectory);
  move_plan.trajectory_ = trajectory;

  arm_group.execute(move_plan);
}

void setPlacePose()
{
  placePose.position.x = -0.15;
  placePose.position.y = 0.43;
  placePose.position.z = 0.25;
  placePose.orientation.x = -0.988;
  placePose.orientation.y = 0.156;
  placePose.orientation.z = 0;
  placePose.orientation.w = 0;
}
void setStartPose1()
{
  startPose1.position.x = 0.25;
  startPose1.position.y = 0.4;
  startPose1.position.z = 0.25;
  startPose1.orientation.x = -0.988;;
  startPose1.orientation.y = 0.156;
  startPose1.orientation.z = 0;
  startPose1.orientation.w = 0;
}
void setStartPose2()
{
    startPose2.position.x = 0.25;
    startPose2.position.y = 0.4;
    startPose2.position.z = 0.25;
    startPose2.orientation.x = -0.988;
    startPose2.orientation.y = 0;
    startPose2.orientation.z = 0;
    startPose2.orientation.w = 0;
}
void setStartPose3()
{
    startPose3.position.x = 0.25;
    startPose3.position.y = 0.4;
    startPose3.position.z = 0.25;
    startPose3.orientation.x = -0.988;
    startPose3.orientation.y = 0.156;
    startPose3.orientation.z = 0;
    startPose3.orientation.w = 0;
}
void setStartPose4()
{
    startPose4.position.x = 0.25;
    startPose4.position.y = 0.4;
    startPose4.position.z = 0.25;
    startPose4.orientation.x = -0.988;
    startPose4.orientation.y = 0.156;
    startPose4.orientation.z = 0;
    startPose4.orientation.w = 0;
}

void getRobotInfo(sensor_msgs::JointState curState)
{
  kinovaState = curState;
}

//判断目标物体是否存在
bool judgeIsExist(int tag, vector<kinova_arm_moveit_demo::targetState> targetAll)
{
  int num = targetAll.size();
  bool result = false;

  for(int i=0;i<num;i++)
  {
    if(tag==targetAll[i].tag)
    {
      result = true;
      break;
    }
  }
  return result;
}

//判断目标物体是否被遮挡
bool judgeIsHinder(int tag, vector<kinova_arm_moveit_demo::targetState> targetAll)
{
  // 根据目标物道相机的距离判断
  // 基于targetAll中的坐标在相机坐标系下的前提
  int num = targetAll.size();
  bool result = false;
  double distance[num];
  double targetDistance;

  for(int i=0;i<num;i++)
  {
    distance[i] = targetAll[i].x*targetAll[i].x + targetAll[i].y*targetAll[i].y + targetAll[i].z*targetAll[i].z;
    if(tag==targetAll[i].tag)
    {
      targetDistance = distance[i];
    }
  }

  for(int i=0;i<num;i++)
  {
    if(distance[num]<targetDistance)
    {
      result = true;
    }
  }
  return result;
}


//计算与目标物体的绝对距离
double calcDistance(kinova_arm_moveit_demo::targetState targetNow)
{
  double distance = 0;
  distance = targetNow.x*targetNow.x + targetNow.y*targetNow.y + targetNow.z*targetNow.z;
  return distance;
}

//获取当前目标对象的位置
kinova_arm_moveit_demo::targetState getTargetPoint(int tag, vector<kinova_arm_moveit_demo::targetState> targetAll)
{
  int num = targetAll.size();
  int targetTag;

  for(int i=0;i<num;i++)
  {
    if(tag==targetAll[i].tag)
    {
      targetTag = i;
    }
  }
  return targetAll[targetTag];
}

//接近目标物体
void approachTarget(int tag, kinova_arm_moveit_demo::targetState targetNow, sensor_msgs::JointState robotState)
{
  // 速度控制--------------------------------------------------------------------------------------待补充
}

kinova_arm_moveit_demo::targetState transTarget(const tf2_ros::Buffer& tfBuffer_, kinova_arm_moveit_demo::targetState targetNow)
{
  kinova_arm_moveit_demo::targetState transResult;
  Eigen::Vector3d item_center3d;

  item_center3d(0)=targetNow.x;
  item_center3d(1)=targetNow.y;
  item_center3d(2)=targetNow.z;

  cout<<"targetNow: "<<endl;
  cout<<targetNow.x<<","<<targetNow.y<<","<<targetNow.z<<endl;

  Eigen::Quaterniond quater(targetNow.qw,targetNow.qx,targetNow.qy,targetNow.qz);

  //用tf来计算base2hand--------------------------------------
  Eigen::Matrix3d base2cam_r;
  Eigen::Vector3d base2cam_t;
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform(world_frame, camera_frame, ros::Time(0),ros::Duration(0.5));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  Quaterniond rotQ(transformStamped.transform.rotation.w,
                   transformStamped.transform.rotation.x,
                   transformStamped.transform.rotation.y,
                   transformStamped.transform.rotation.z);

  base2cam_r=rotQ.matrix();

  cout<<"base2cam_r: "<<endl;
  cout<<base2cam_r<<endl;


  base2cam_t<<transformStamped.transform.translation.x,
           transformStamped.transform.translation.y,
           transformStamped.transform.translation.z;

  cout<<"base2cam_t: "<<endl;
  cout<<base2cam_t<<endl;

  // 计算完毕-------------------------------------------------

  //目标物从工具坐标系转到基坐标系下
  item_center3d=base2cam_r*item_center3d+base2cam_t;
  quater=rotQ*quater;

  //获取当前抓取物品的位置
  transResult.x=item_center3d(0);
  transResult.y=item_center3d(1);
  //transResult.z=item_center3d(2);
  transResult.z = highVal;

  transResult.qx=quater.x();
  transResult.qy=quater.y();
  transResult.qz=quater.z();
  transResult.qw=quater.w();

//  ROS_INFO("curTargetPoint: %f %f %f",transResult.x,transResult.y,transResult.z);
//  ROS_INFO("curTargetPose: %f %f %f %f",transResult.qx,transResult.qy,transResult.qz,transResult.qw);

  return transResult;
}

// -------------------------------------------------主程序入口-----------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "KN_sim");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(3);
    spinner.start();

  //发布消息和订阅消息
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher detectTarget_pub = node_handle.advertise<std_msgs::Int8>("detect_target", 10);  //让visual_detect节点检测目标
    ros::Publisher grab_result_pub = node_handle.advertise<rviz_teleop_commander::grab_result>("grab_result", 1);  //发布抓取状态
    ros::Subscriber detectResult_sub = node_handle.subscribe("detect_result", 1, detectResultCB);    //接收visual_detect检测结果
    ros::Subscriber tags_sub = node_handle.subscribe("targets_tag", 1, tagsCB);				//接收目标序列
    ros::Subscriber joints_sub = node_handle.subscribe("/j2s7s300/joint_states", 1, getRobotInfo);				//接收目标序列
    moveit::planning_interface::MoveGroupInterface *finger_group;
    finger_group = new moveit::planning_interface::MoveGroupInterface("gripper");

    moveit_msgs::DisplayTrajectory display_trajectory;
    rviz_teleop_commander::grab_result grabResultMsg;
    std_msgs::Int8 detectTarget;

    //获取工具坐标系
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);  //获取机械臂末端在基坐标系下的位姿

    //手眼关系赋值
    hand2eye_r<<-1, 0, 0,
            0, 1, 0,
            0, 0, -1;
    hand2eye_t<<0.321,0.43,0.8;
    hand2eye_q=hand2eye_r;
    client = new Finger_actionlibClient(Finger_action_address, true);

  //全局变量赋初值
  setStartPose1();
  setStartPose2();
  setStartPose3();
  setStartPose4();
  setPlacePose();

  startPose = startPose1;
  goStartPose();


  cout<<"/*************************************/"<<endl;
  /************输入目标标签***************/
  /*************************************/

  ROS_INFO("waiting for tags of targets input in GUI");
  while(getTargetsTag!=1)				//等待抓取目标输入
    {
    ros::Duration(0.5).sleep();
        if(!ros::ok())
        {
            detectTarget.data=2;		//让visual_detect节点退出
            detectTarget_pub.publish(detectTarget);
      ros::Duration(1.0).sleep();
            return 1;
        }
    }

  detectTarget.data=1;		//让visual_detect节点检测目标
  detectTarget_pub.publish(detectTarget);
  ros::Duration(1.0).sleep();
  ROS_INFO("All ready, waiting for goal.");

  cout<<"/*************************************/"<<endl;
  /*************************************/
  /***********目标检测与抓取**************/
  /*************************************/
  while(ros::ok())
  {
    //goStartPose();   //there is one in line 839
    kinova_arm_moveit_demo::targetState curTargetPoint;
    bool isExist = 0;                                   //是否存在
    int curTag = 0;                                     //当前抓取对象
    int number=targetsTag.size();                       //目标个数

    for(int i=0;i<number;i++)
    {
      curTag = targetsTag[i];
      //手抓闭合程度，抓取高度
      closeVal=closeVals[curTag-1];
      highVal=highVals[curTag-1];
      openVal=openVals[curTag-1];

      ROS_INFO("Start to detect target [%d].", curTag);
      bool targetFinish = false;

      while(!targetFinish)
      {
        isExist = judgeIsExist(curTag,targets);

        int times = 1;
        bool nextTarget = false;
        while((!isExist)&&(times<(poseChangeTimes)))
        {
          ROS_INFO("There is no target [%d], change pose to detect again.", curTag);
          if(times==0) {startPose = startPose2;goStartPose();}
          if(times==1) {startPose = startPose3;goStartPose();}
          if(times==2) {startPose = startPose4;goStartPose();}
          if(times==poseChangeTimes)
          {
            ROS_INFO("Detection failed, go to pick the next target.");
            startPose = startPose1;
            goStartPose();
            nextTarget = true;
          }
          ros::Duration(1.0).sleep();//这里延时一秒够吗
          isExist = judgeIsExist(curTag,targets);
          times++;
        }

        while(isExist)
        {
          ROS_INFO("Got the target [%d].", curTag);


//**************key*******************************
          curTargetPoint = targets[i];
          pickAndPlace(curTargetPoint,tfBuffer);
//**************key*******************************


          //判断放置框中是否有目标物体
          isExist = judgeIsExist(curTag,targets);
          if(isExist)
          {
            targetFinish = true;
            ROS_INFO("Target [%d] succeeds.", curTag);
          }

          //判断抓取框中是否已经没有目标物体
          goStartPose();
          isExist = judgeIsExist(curTag,targets);
          if(!isExist)
          {
            targetFinish = true;
            ROS_INFO("Target [%d] succeeds.", curTag);
            isExist = false;
          }
          else
          {
            targetFinish = false;
            ROS_INFO("Target [%d] failed, pick again.", curTag);
            isExist = true;
          }
        }
        //if(nextTarget) break;
        targetFinish = true;
      }
    }
  }

    //退出程序
    detectTarget.data=2;		//让visual_detect节点退出
    detectTarget_pub.publish(detectTarget);
    //发布抓取状态
    grabResultMsg.now_target=-1;
    grabResultMsg.grab_times=-1;
    grab_result_pub.publish(grabResultMsg);
    ros::Duration(1.0).sleep();
    return 0;
}
