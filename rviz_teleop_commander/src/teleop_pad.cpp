#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "teleop_pad.h"

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{

  // 创建一个输入目标物标签的窗口
  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget( new QLabel( "tags of targets:" ));
  output_tags_editor_ = new QLineEdit;
  topic_layout->addWidget( output_tags_editor_ );

  //创建一个按钮
  start_push_button=new QPushButton( "start" );
  topic_layout->addWidget(start_push_button);
  
  //创建当前抓取目标物标签显示框
  topic_layout->addWidget( new QLabel( "grab target:" ));
  grabTagShow = new QLineEdit;
  topic_layout->addWidget( grabTagShow );
  //grabTagShow->setText("yi");
  
  //创建当前抓取目标物次数显示框
  topic_layout->addWidget( new QLabel( "grab times:" ));
  grabTimesShow = new QLineEdit;
  topic_layout->addWidget( grabTimesShow );

  //show=new QLineEdit;
  //topic_layout->addWidget( show );
  //show->setText("yi");

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );

  //多线程订阅消息
  ros::AsyncSpinner spinner(1);

  //发布的话题
  tags_publisher_ = nh_.advertise<rviz_teleop_commander::targets_tag>( "targets_tag", 1 );
  
  //接收的话题
  grab_result_sub = nh_.subscribe("grab_result", 1, &TeleopPanel::grabResultCB, this);	

  // 设置信号与槽的连接
  connect( output_tags_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));             // 输入topic命名，回车后，调用updateTopic()
  
  connect(start_push_button, SIGNAL(clicked()), this, SLOT(pubTagsTopic()));  //发布目标标签

  spinner.start();
}


// 记录标签值到消息类型中
void TeleopPanel::updateTopic()
{
  QString tags = output_tags_editor_->text();
  QList<QString> tagsList;
  tagsList=tags.split(' ');
  tagsMsg.targetsTag.clear();
  tagsMsg.targetsTag.resize(tagsList.size());
  for(int i=0; i<tagsList.size(); i++)
  {
    tagsMsg.targetsTag[i]=tagsList[i].toInt();
  }
}

// 发布目标标签消息
void TeleopPanel::pubTagsTopic()
{
  tags_publisher_.publish(tagsMsg);
}


// 重载父类的功能
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// 重载父类的功能，加载配置数据
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_tags_editor_->setText( topic );
    updateTopic();
  }
}

//消息回调函数，当前抓取的目标标签和抓取次数
void TeleopPanel::grabResultCB(const rviz_teleop_commander::grab_result::ConstPtr &msg)
{
  grabTagShow->setText(QString::number(msg->now_target));
  grabTimesShow->setText(QString::number(msg->grab_times));
}

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel,rviz::Panel )
// END_TUTORIAL
