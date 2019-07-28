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
#include <unistd.h>
//手指控制
#include "robotiq_2f_gripper_control/gripperControl.h"  //robotiq二指手
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
geometry_msgs::Pose startPose;                      //机械臂初始识别位置
geometry_msgs::Pose startPose1;                     //姿态备选
geometry_msgs::Pose startPose2;
geometry_msgs::Pose startPose3;
geometry_msgs::Pose startPose4;
geometry_msgs::Pose placePose;                      //机械臂抓取放置位置
sensor_msgs::JointState urState;                    //机械臂当前状态
vector<int> targetsTag;                           	//需要抓取的目标物的标签
bool getTargets=0;                                	//当接收到视觉定位结果时getTargets置1，执行完放置后置0
bool getTargetsTag=0;                             	//当接收到需要抓取的目标物的标签时置1
int poseChangeTimes=0;                              //当检测不到目标物体时,变换姿态重新检测的次数
double minimumDistance = 0.3;                       //允许距离目标物的最小距离,单位米
double servoCircle = 0.5;                           //伺服运动周期,单位秒
std_msgs::Int8 detectMsg;
double destHeight = 0.15;

//-------------------------------------------------相机相关--------------------------------------------------
//相机参数和深度信息用于计算
#define Fxy 692.97839
#define UV0 400.5
#define Zw 0.77
//手眼关系定义--赋值在main函数中
Eigen::Matrix3f hand2eye_r;
Eigen::Vector3f hand2eye_t;
Eigen::Quaternionf hand2eye_q;

//-------------------------------------------------机器人相关------------------------------------------------
//定义机器人类型
string base_frame = "base_link";// tf中基坐标系的名称
string tool_frame = "tool0";// tf中工具坐标系的名称
//手指client类型自定义
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;
//定义手指控制client
Finger_actionlibClient* client=NULL;
// Robotiq手爪能自动调节闭合程度
float highVal=0.05;
float closeVal=50;
//                   芒果， 蛇果，青苹果， 香蕉,  山竹，  橙子，  柠檬， 草莓，  葡萄，  杨桃
float highVals[10]= {0.05, 0.07, 0.07,  0.1, 0.040,  0.07,  0.05, 0.045, 0.045, 0.050};// 抓取高度
float closeVals[10] = {65,  45,    45,   40,    60,   45,    100,  150,    140,    50};//爪子闭合

// -------------------------------------------------函数定义-------------------------------------------------
//接收相机节点发过来的识别结果,更新全局变量targets
void detectResultCB(const kinova_arm_moveit_demo::targetsVector &msg);

//接收teleop指定的抓取序列号，更新全局变量targetsTag
void tagsCB(const rviz_teleop_commander::targets_tag &msg);

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

// 转换位姿，用于控制UR实物
geometry_msgs::Pose changePoseForUR(geometry_msgs::Pose pose);


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
  moveit::planning_interface::MoveGroupInterface arm_group("manipulator");	// replace the old version "moveit::planning_interface::MoveGroupInterface"
  geometry_msgs::Pose targetPose;	//定义抓取位姿
  geometry_msgs::Point point;
  geometry_msgs::Quaternion orientation;
  Eigen::Quaternionf cameraQuater;
  Eigen::Matrix3f cameraRot;

  //ur5张开手爪
  sendGripperMsg(0);//open

  //cout<<"curTargetPoint's position: "<<curTargetPoint.x<<","<<curTargetPoint.y<<","<<curTargetPoint.z<<endl;
  cout<<"curTargetPoint's quater: "<<curTargetPoint.qw<<","<<curTargetPoint.qx<<","<<curTargetPoint.qy<<","<<curTargetPoint.qz<<endl;
  curTargetPoint = transTarget(tfBuffer_, curTargetPoint);

  point.x = curTargetPoint.x;//获取抓取位姿
  point.y = curTargetPoint.y;
  point.z = curTargetPoint.z + highVal + destHeight;

  moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
  moveit::planning_interface::MoveGroupInterface::Plan place_plan;

  orientation.x = curTargetPoint.qx;//方向由视觉节点给定－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－Petori
  orientation.y = curTargetPoint.qy;
  orientation.z = curTargetPoint.qz;
  orientation.w = curTargetPoint.qw;

  orientation.w = 0;
  orientation.x = 0;//方向先固定（调试
  orientation.y = 1;
  orientation.z = 0;

  cameraQuater.x() = orientation.x;
  cameraQuater.y() = orientation.y;
  cameraQuater.z() = orientation.z;
  cameraQuater.w() = orientation.w;

  cameraRot = cameraQuater.toRotationMatrix();
  cout<<"Pose from camera -- initial: "<<endl;
  cout<<cameraRot<<endl;

  targetPose.position = point;// 设置好目标位姿为可用的格式
  targetPose.orientation = orientation;

  cout<<"targetPose z: "<<endl;//targetPose.position.x<<",";
//  cout<<targetPose.position.y<<",";
  cout<<targetPose.position.z<<endl;

  cout<<"cameraQuater: "<<cameraQuater.w()<<","<<cameraQuater.x();
  cout<<cameraQuater.y()<<",";
  cout<<cameraQuater.z()<<endl;

  cout<<"targetPose: "<<targetPose.orientation.w<<","<<targetPose.orientation.x;
  cout<<targetPose.orientation.y<<",";
  cout<<targetPose.orientation.z<<endl;

  targetPose = changePoseForUR(targetPose);

  //抓取插值
  std::vector<geometry_msgs::Pose> pickWayPoints;
  pickWayPoints = pickInterpolate(startPose, targetPose);

  //前往抓取点
  moveit_msgs::RobotTrajectory trajectory1;
  arm_group.computeCartesianPath(pickWayPoints,
                                 0.02,  // eef_step
                                 0.0,   // jump_threshold
                                 trajectory1);

  pick_plan.trajectory_ = trajectory1;
  arm_group.execute(pick_plan);

  ROS_INFO("Prepare for picking .");

  //ur5抓取动作
  sendGripperMsg(closeVal);
  sleep(1);
  //抓取完毕

  ros::Duration(0.5).sleep();

  //放置插值
  geometry_msgs::Pose placePose_tmp;
  placePose_tmp = changePoseForUR(placePose);
  std::vector<geometry_msgs::Pose> placeWayPoints;
  placeWayPoints = placeInterpolate(targetPose, placePose_tmp);

  //前往放置点
  moveit_msgs::RobotTrajectory trajectory2;
  arm_group.computeCartesianPath(placeWayPoints,
                                 0.02,  // eef_step
                                 0.0,   // jump_threshold
                                 trajectory2);
  place_plan.trajectory_ = trajectory2;
  arm_group.execute(place_plan);

  ROS_INFO("Prepare for placing. ");

  //松开爪子
  //ur5张开手爪
  sendGripperMsg(0);//open
  sleep(1.0);
  targets.clear();
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

void setPlacePose()
{
    placePose.position.x = -0.3;
    placePose.position.y = -0.5;
    placePose.position.z = 0.26;
    placePose.orientation.x = 0;
    placePose.orientation.y = 1;
    placePose.orientation.z = 0;
    placePose.orientation.w = 0;
}


void goStartPose()
{
  //前往视觉识别位置
  moveit::planning_interface::MoveGroupInterface arm_group("manipulator");
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

  // zhangkai爪子
  sendGripperMsg(0);
  sleep(1.0);
}

//void setStartPose1()
//{
//  startPose1.clear();
//  startPose1.push_back(-2.33038);
//  startPose1.push_back(2.42892);
//  startPose1.push_back(3.49546);
//  startPose1.push_back(1.81877);
//  startPose1.push_back(2.89536);
//  startPose1.push_back(1.97723);
//  startPose1.push_back(-14.52231);
//}

//注意，UR是0 1 0 0，kinova是1 0 0 0，顺序都是xyzw，kinova这里指仿真中的姿态，实物要到现场才知道
void setStartPose1()
{
    startPose1.position.x = 0.159;
    startPose1.position.y = -0.377;
    startPose1.position.z = 0.446;
    startPose1.orientation.x = 0;
    startPose1.orientation.y = 1;
    startPose1.orientation.z = 0;
    startPose1.orientation.w = 0;
}

void setStartPose2()
{
    startPose2.position.x = 0.159;
    startPose2.position.y = -0.377;
    startPose2.position.z = 0.446;
    startPose2.orientation.x = 0;
    startPose2.orientation.y = 1;
    startPose2.orientation.z = 0;
    startPose2.orientation.w = 0;
}
void setStartPose3()
{
    startPose3.position.x = 0.159;
    startPose3.position.y = -0.377;
    startPose3.position.z = 0.446;
    startPose3.orientation.x = 0;
    startPose3.orientation.y = 1;
    startPose3.orientation.z = 0;
    startPose3.orientation.w = 0;
}
void setStartPose4()
{
    startPose4.position.x = 0.159;
    startPose4.position.y = -0.377;
    startPose4.position.z = 0.446;
    startPose4.orientation.x = 0;
    startPose4.orientation.y = 1;
    startPose4.orientation.z = 0;
    startPose4.orientation.w = 0;
}

void getRobotInfo(sensor_msgs::JointState curState)
{
  urState = curState;
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
  // 根据目标物到相机的距离判断6
  // 基于targetAll中的坐标在相机坐标系下的前提
  int num = targetAll.size();
  bool result = false;
  double distance[num];
  double targetDistance;

  for(int i=0;i<num;i++)
  {
    distance[i] = targetAll[i].x*targetAll[i].x + targetAll[i].y*targetAll[i].y + targetAll[i].z*targetAll[i].z;
    //cout<<"distance: "<<distance[i]<<endl;//test
    if(tag==targetAll[i].tag)
    {
      targetDistance = distance[i];
      //cout<<"targetDistance: "<<targetDistance<<endl;//test
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

//获取遮挡物体的位置
kinova_arm_moveit_demo::targetState judgeTheObstacle(int tag, vector<kinova_arm_moveit_demo::targetState> targetAll)
{
  // 从所有遮挡物中,先取出离相机最近的那个
  int num = targetAll.size();
  int obstacleTag;
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
      targetDistance = distance[num];
      obstacleTag = i;
    }
  }

  return targetAll[obstacleTag];
}

//拾取遮挡物体
void pickTheObstacle(kinova_arm_moveit_demo::targetState targetNow, const tf2_ros::Buffer& tfBuffer_)
{
  moveit::planning_interface::MoveGroupInterface arm_group("manipulator");	//ur
  moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
  kinova_arm_moveit_demo::targetState targetReal;
  geometry_msgs::Pose poseStart;
  geometry_msgs::Pose targetPose;	//定义抓取位姿

  // 把目标物体坐标转换到机器人坐标系下
  targetReal = transTarget(tfBuffer_, targetNow);
  targetPose.position.x = targetReal.x;
  targetPose.position.y = targetReal.y;
  targetPose.position.z = targetReal.z;
  targetPose.orientation.x = targetReal.qx;
  targetPose.orientation.y = targetReal.qy;
  targetPose.orientation.z = targetReal.qz;
  targetPose.orientation.w = targetReal.qw;

  // 转换startPose(vector(double))为poseStart(geometry_msgs::Pose) ---  改了数据类型,直接赋值
  poseStart = startPose;

  // 抓取路径插值并执行抓取
  std::vector<geometry_msgs::Pose> pickWayPoints;
  pickWayPoints = pickInterpolate(poseStart, targetPose);

  //前往抓取障碍物体
  moveit_msgs::RobotTrajectory trajectory1;
  arm_group.computeCartesianPath(pickWayPoints,
                                 0.02,  // eef_step
                                 0.0,   // jump_threshold
                                 trajectory1);

  pick_plan.trajectory_ = trajectory1;
  arm_group.execute(pick_plan);

  //松开爪子
  sendGripperMsg(0);//open
  sleep(1.0);
}

//放置遮挡物体
void placeTheObstacle(int tag, kinova_arm_moveit_demo::targetState targetNow, vector<kinova_arm_moveit_demo::targetState> targetAll, const tf2_ros::Buffer& tfBuffer_)
{
  // 判断将遮挡物体放在哪里比较合适
  // 算法为,选中一个与其他所有物体的距离平方之和最大的物体A
  // 判断距离A物体最近的物体B,若B不是我们本轮的抓取对象,则将障碍物置于AB的中点
  // 若B是本轮的抓取对象,则选取距离A第二近的物体C,障碍物置于AC的中点
  // 若场上仅有障碍物与目标物,则平移五公分
  int num = targetAll.size();
  moveit::planning_interface::MoveGroupInterface arm_group("manipulator");	//ur
  moveit::planning_interface::MoveGroupInterface::Plan place_plan;
  kinova_arm_moveit_demo::targetState targetReal;
  kinova_arm_moveit_demo::targetState targetTemp;
  geometry_msgs::Pose targetPose;	//定义抓取位姿
  geometry_msgs::Pose nowPose;	//定义抓取位姿

  nowPose.position.x = targetNow.x;
  nowPose.position.y = targetNow.y;
  nowPose.position.z = targetNow.z; // 抬升10cm再放下
  nowPose.orientation.x = targetNow.qx;
  nowPose.orientation.y = targetNow.qy;
  nowPose.orientation.z = targetNow.qz;
  nowPose.orientation.w = targetNow.qw;

  double coordinates[num][2];

  // 只有目标物和障碍物的情况
  if(num == 2)
  {
    for(int i=0;i<2;i++)
    {
      if(targetAll[i].tag==tag)continue;
      targetReal = transTarget(tfBuffer_, targetAll[i]);
    }
    targetPose.position.x = targetReal.x + 0.02;
    targetPose.position.y = targetReal.y + 0.02;
    targetPose.position.z = targetReal.z + 0.2; // 抬升10cm再放下
    targetPose.orientation.x = targetReal.qx;
    targetPose.orientation.y = targetReal.qy;
    targetPose.orientation.z = targetReal.qz;
    targetPose.orientation.w = targetReal.qw;
  }

  // 场上存在三个以上物体的情况
  else
  {
    // 选出物体A
    double maxSum = 0;
    double nowSum;
    double fastTag;
    for(int i=0;i<num;i++)
    {
      targetReal = transTarget(tfBuffer_, targetAll[i]);
      coordinates[i][0] = targetReal.x;
      coordinates[i][1] = targetReal.y;
      nowSum = 0;
      for(int j=0;j<(num);j++)
      {
        if(j!=i)
        {
          targetTemp = transTarget(tfBuffer_, targetAll[j]);
          nowSum = (targetTemp.x-coordinates[i][0])*(targetTemp.x-coordinates[i][0]);
          nowSum = nowSum + (targetTemp.y-coordinates[i][1])*(targetTemp.y-coordinates[i][1]);
        }
      }
      if(nowSum>maxSum)
      {
        maxSum = nowSum;
        fastTag = i;
      }
    }

    // 选出物体B/C
    double miniSum = 1000000;
    double nearTag;
    targetReal = transTarget(tfBuffer_, targetAll[fastTag]);
    for(int i=0;i<num;i++)
    {
      if(i==tag) continue;

      nowSum = 0;
      targetTemp = transTarget(tfBuffer_, targetAll[i]);
      nowSum = (targetTemp.x-coordinates[i][0])*(targetTemp.x-coordinates[i][0]);
      nowSum = nowSum + (targetTemp.y-coordinates[i][1])*(targetTemp.y-coordinates[i][1]);
      if(nowSum<miniSum)
      {
        miniSum = nowSum;
        nearTag = i;
      }
    }

    // 计算AB / AC 的中心位置
    kinova_arm_moveit_demo::targetState targetA;
    kinova_arm_moveit_demo::targetState targetB;

    targetA = transTarget(tfBuffer_,targetAll[fastTag]);
    targetB = transTarget(tfBuffer_,targetAll[nearTag]);

    targetPose.position.x = (targetA.x + targetB.x)/2;
    targetPose.position.y = (targetA.y + targetB.y)/2;
    targetPose.position.z = (targetA.z + targetB.z)/2; + 0.2; // 抬升10cm再放下
    targetPose.orientation.x = targetA.qx;
    targetPose.orientation.y = targetA.qy;
    targetPose.orientation.z = targetA.qz;
    targetPose.orientation.w = targetA.qw;
  }

  //前往放置障碍物体--执行targetPose
  std::vector<geometry_msgs::Pose> placeWayPoints;
  placeWayPoints = placeInterpolate(nowPose, targetPose);
  moveit_msgs::RobotTrajectory trajectory1;
  arm_group.computeCartesianPath(placeWayPoints,
                                 0.02,  // eef_step
                                 0.0,   // jump_threshold
                                 trajectory1);

  place_plan.trajectory_ = trajectory1;
  arm_group.execute(place_plan);

  //松开爪子
  sendGripperMsg(0);//open
  sleep(1.0);
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
  Eigen::Vector3f cam_center3f, base_center3f;

  cam_center3f(0)=targetNow.x;
  cam_center3f(1)=targetNow.y;
  cam_center3f(2)=targetNow.z;
  Eigen::Quaternionf camera_quater(targetNow.qw,targetNow.qx,targetNow.qy,targetNow.qz);
  Eigen::Quaternionf object_quater;
  Eigen::Matrix3f camera_rot;
  Eigen::Matrix3f object_rot;
  Eigen::Vector3f object_x_axis;
  Eigen::Vector3f object_y_axis;
  Eigen::Vector3f object_z_axis;
  object_z_axis << 0,0,-1;

  base_center3f=hand2eye_r*cam_center3f+hand2eye_t;

  //用tf来计算base2hand--------------------------------------
  Eigen::Matrix3f base2hand_r;
  Eigen::Vector3f base2hand_t;
  geometry_msgs::TransformStamped transformStamped;

  try
  {
    transformStamped = tfBuffer_.lookupTransform(base_frame, tool_frame, ros::Time(0),ros::Duration(0.5));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
  }

  // ur从tf订阅的位姿也得转换--------------------- ur only
  transformStamped.transform.translation.x = -transformStamped.transform.translation.x;
  transformStamped.transform.translation.y = -transformStamped.transform.translation.y;

  //base2hand_r=rotQ.matrix();

  //相对姿态不变，固定住
  base2hand_r<<-1,0,0,
                0,1,0,
                0,0,-1;

  base2hand_t<<transformStamped.transform.translation.x,
           transformStamped.transform.translation.y,
           transformStamped.transform.translation.z;

  //目标物从工具坐标系转到基坐标系下------位置结果
  base_center3f=base2hand_r*base_center3f+base2hand_t;
  //目标物从工具坐标系转到基坐标系下------姿态结果
  camera_rot = camera_quater.toRotationMatrix();
  object_x_axis = camera_rot.col(0);
  object_x_axis(0) = -object_x_axis(0);//ur的法兰x向和基坐标系x向相反
  object_x_axis(2) = -object_x_axis(2);
  object_y_axis = object_z_axis.cross(object_x_axis);
  object_x_axis = object_y_axis.cross(object_z_axis);
  object_rot << object_x_axis(0),object_y_axis(0),object_z_axis(0),
                object_x_axis(1),object_y_axis(1),object_z_axis(1),
                object_x_axis(2),object_y_axis(2),object_z_axis(2);
  object_quater = object_rot;
  cout<<"object_rot:"<<endl;
  cout<<object_rot<<endl;

  //获取当前抓取物品的位置和姿态
  transResult.x=base_center3f(0);
  transResult.y=base_center3f(1);
  transResult.z=base_center3f(2);
  transResult.qx=object_quater.x();
  transResult.qy=object_quater.y();
  transResult.qz=object_quater.z();
  transResult.qw=object_quater.w();

  return transResult;
}

// 把位姿态转换一下，给UR使用
geometry_msgs::Pose changePoseForUR(geometry_msgs::Pose pose)
{
    double x_init,y_init,z_init,w_init;
    double x_mid,y_mid,z_mid,w_mid;
    double x_trans,y_trans,z_trans,w_trans;
    double x_x180,y_x180,z_x180,w_x180;
    double x_y270,y_y270,z_y270,w_y270;

    pose.position.x = -pose.position.x;
    pose.position.y = -pose.position.y;
    pose.position.z = pose.position.z;

//    // what's the relationship
//    Eigen::Quaternionf quater1(0,0,1,0);
//    Eigen::Quaternionf quatermid(0.707105,0,0.707108,0);
//    Eigen::Quaternionf quater2(0.5,-0.5,0.5,0.5);
//    Eigen::Matrix3f wtfRot1;
//    Eigen::Matrix3f wtfRotMid;
//    Eigen::Matrix3f wtfRot2;

//    wtfRot1 = quater1.toRotationMatrix();
//    wtfRot2 = quater2.toRotationMatrix();
//    wtfRotMid = quatermid.toRotationMatrix();

//    cout<<"wtfRot1: "<<endl;
//    cout<<wtfRot1<<endl;
//    cout<<"wtfRot2: "<<endl;
//    cout<<wtfRot2<<endl;
//    cout<<"wtfRotMid: "<<endl;
//    cout<<wtfRotMid<<endl;


    // calculate the transformation for ur
    Eigen::Matrix3f originalRot;
    Eigen::Matrix3f resultRot;
    Eigen::Matrix3f midRot;
    Eigen::Matrix3f transRot1;
    Eigen::Matrix3f transRot2;

    midRot << 0,0,1,
              0,1,0,
             -1,0,0;

    originalRot << -1,0,0,
                   0,1,0,
                  0,0,-1;

    resultRot <<  0,-1,0,
                  0,0,1,
                 -1,0,0;

    transRot1 = midRot*originalRot.inverse();
    transRot2 = resultRot*midRot.inverse();
    // calculation over

    Eigen::Quaternionf originalPose;
    Eigen::Matrix3f inputRot;
    Eigen::Matrix3f outputRot;
    Eigen::Quaternionf outputQuater;

    originalPose.x() = pose.orientation.x;
    originalPose.y() = pose.orientation.y;
    originalPose.z() = pose.orientation.z;
    originalPose.w() = pose.orientation.w;

    inputRot = originalPose.toRotationMatrix();
    outputRot = transRot2*transRot1*inputRot;

//    cout<<"inputRot: "<<endl;
//    cout<<inputRot<<endl;

//    cout<<"outputRot: "<<endl;
//    cout<<outputRot<<endl;

    outputQuater = outputRot;
    pose.orientation.x = outputQuater.x();
    pose.orientation.y = outputQuater.y();
    pose.orientation.z = outputQuater.z();
    pose.orientation.w = outputQuater.w();

//    pose.orientation.x = -0.5;
//    pose.orientation.y = 0.5;
//    pose.orientation.z = 0.5;
//    pose.orientation.w = 0.5;

    return pose;
}

// -------------------------------------------------主程序入口-----------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "UR_real_new");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3);
  spinner.start();

  //发布消息和订阅消息
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  ros::Publisher detectTarget_pub = node_handle.advertise<std_msgs::Int8>("detect_target", 10);  //让visual_detect节点检测目标
  ros::Publisher grab_result_pub = node_handle.advertise<rviz_teleop_commander::grab_result>("grab_result", 1);  //发布抓取状态
  ros::Subscriber detectResult_sub = node_handle.subscribe("detect_result", 1, detectResultCB);    //接收visual_detect检测结果
  ros::Subscriber tags_sub = node_handle.subscribe("targets_tag", 1, tagsCB);				//接收目标序列
  ros::Subscriber joints_sub = node_handle.subscribe("/joint_states", 1, getRobotInfo);				//接收目标序列
  ros::Publisher gripperPub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 20);
  sleep(1);

  gripperPubPtr = &gripperPub;
  initializeGripperMsg();

  moveit_msgs::DisplayTrajectory display_trajectory;
  rviz_teleop_commander::grab_result grabResultMsg;
  std_msgs::Int8 detectTarget;

  //获取工具坐标系
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);  //获取机械臂末端在基坐标系下的位姿

  //手眼关系赋值
  hand2eye_r<<0.9996059286630877, 0.02635426162256812, -0.009666451051709242,
          -0.02637844622662881, 0.9996491879441952, -0.002382984930695774,
          0.009600258135837264, 0.002637031823908145, 0.9999504393253122;
  hand2eye_t<<-0.05960843140159527,-0.08985859631884774,0.06895877317853209;
  hand2eye_q = hand2eye_r;
  //cout<<"hand2eye_q: "<<hand2eye_q.x()<<","<<hand2eye_q.y()<<","<<hand2eye_q.z()<<","<<hand2eye_q.w()<<endl;

  //全局变量赋初值
  setStartPose1();
  setStartPose2();
  setStartPose3();
  setStartPose4();
  setPlacePose();

  startPose = startPose1;

  startPose = changePoseForUR(startPose);
  goStartPose();

  /*************************************/
  /************输入目标标签***************/
  /*************************************/

//  detectMsg.data = 1;		//让visual_detect节点检测目标
//  detectTarget_pub.publish(detectMsg);sleep(3);

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

  /*************************************/
  /***********目标检测与抓取**************/
  /*************************************/

  // 初始化UR手爪


  while(ros::ok())
  {
    kinova_arm_moveit_demo::targetState curTargetPoint;

    bool isExist = 0;                                   //是否存在
    bool isHinder = 1;                                  //是否有遮挡
    int curTag = 0;                                     //当前抓取对象
    int number=targetsTag.size();                       //目标个数

    for(int i=0;i<number;i++)
    {
      curTag = targetsTag[i];

      //查表选择抓取高度
      highVal=highVals[curTag-1];
      closeVal=closeVals[curTag-1];

      ROS_INFO("Start to detect target [%d].", curTag);
      bool targetFinish = false;

      while(!targetFinish)
      {
        isExist = judgeIsExist(curTag,targets);

        int times = 0;
        bool nextTarget = false;
        while((!isExist)&&(times<(poseChangeTimes)))
        {
          ROS_INFO("There is no target [%d], change pose to detect again.", curTag);
          if(times==0) {startPose = startPose2;startPose = changePoseForUR(startPose);goStartPose();}
          if(times==1) {startPose = startPose3;startPose = changePoseForUR(startPose);goStartPose();}
          if(times==2) {startPose = startPose4;startPose = changePoseForUR(startPose);goStartPose();}
          if(times==poseChangeTimes)
          {
            ROS_INFO("Detection failed, go to pick the next target.");

            startPose = startPose1;startPose = changePoseForUR(startPose);
            goStartPose();
            nextTarget = true;
          }
          isExist = judgeIsExist(curTag,targets);
          times++;
        }

        isHinder = judgeIsHinder(curTag,targets);
        isHinder = false;

        cout<<"isExist:"<<isExist<<endl;
        cout<<"isHinder"<<isHinder<<endl;

        if(isExist&&isHinder)//暂时不设置解决遮挡的次数
        {
          ROS_INFO("The target [%d] is blocked by others, try to pick up the obstacle.", curTag);
          curTargetPoint = judgeTheObstacle(curTag,targets);
          pickTheObstacle(curTargetPoint,tfBuffer);
          placeTheObstacle(curTag,curTargetPoint,targets,tfBuffer);
          goStartPose();
          ROS_INFO("Try to detect the target [%d] again.", curTag);
          isExist = 0;//存在位isExist置0,以跳出本循环和跳过下一个while循环
        }

        if(isExist&&(!isHinder))
        {
          ROS_INFO("Got the unblocked target [%d].", curTag);
          curTargetPoint = getTargetPoint(curTag,targets);

//          double distance;
//          distance = calcDistance(curTargetPoint);

//          while(isExist&&(distance>minimumDistance))
//          {
//            curTargetPoint = getTargetPoint(curTag,targets);
//            approachTarget(curTag,curTargetPoint,urState);
//            ROS_INFO("Approaching the target [%d] ...", curTag);
//            ros::Duration(servoCircle).sleep();
//            isExist = judgeIsExist(curTag,targets);
//            distance = calcDistance(curTargetPoint);
//          }

          pickAndPlace(curTargetPoint,tfBuffer);

          //判断抓取框中是否已经没有目标物体
          goStartPose();
          //targetFinish = true;

//          detectTarget_pub.publish(detectMsg);sleep(3);
//          isExist = judgeIsExist(curTag,targets);
//          if(!isExist)
//          {
//            targetFinish = true;
//            ROS_INFO("Target [%d] succeeds.", curTag);
//          }
        }
        //if(nextTarget) targetFinish = true;
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
