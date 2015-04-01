#include "rviz/config.h"
#include "drc_teleop_interface.h"
#include "ros/time.h"
#include <ros/package.h>
#include "ui_drc_teleop_interface.h"
#include "dynamic_reconfigure/Reconfigure.h"

using namespace rviz;
namespace drc_task_common
{
  DRCTeleopInterfaceAction::DRCTeleopInterfaceAction( QWidget* parent )
    : rviz::Panel( parent )
  {
    ui_ = new Ui::DRCTeleopInterface();
    ui_->setupUi(this);
    ui_->verticalLayout->setAlignment(Qt::AlignLeft);

    ros::NodeHandle nh("~");
    std::string reset_pose_button_icon_name,
      reset_manip_pose_button_icon_name,
      hand_reset_pose_button_icon_name,
      hand_hook_pose_button_icon_name,
      hand_grasp_pose_button_icon_name,
      hrpsys_start_abc_button_icon_name,
      hrpsys_start_st_button_icon_name,
      hrpsys_start_imp_button_icon_name,
      hrpsys_stop_abc_button_icon_name,
      hrpsys_stop_st_button_icon_name,
      hrpsys_stop_imp_button_icon_name,
      display_manip_icon_name,
      hide_manip_icon_name;
    
    nh.param<std::string>("/reset_pose_icon", reset_pose_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/reset-pose.jpg"));
    nh.param<std::string>("/reset_manip_pose_icon", reset_manip_pose_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/reset-manip-pose.jpg"));
    nh.param<std::string>("/hand_reset_pose_icon", hand_reset_pose_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/hand-reset-pose.jpg"));
    nh.param<std::string>("/hand_hook_pose_icon", hand_hook_pose_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/hand-hook-pose.jpg"));
    nh.param<std::string>("/hand_grasp_pose_icon", hand_grasp_pose_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/hand-grasp-pose.jpg"));
    nh.param<std::string>("/start_abc_icon", hrpsys_start_abc_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/start-abc.png"));
    nh.param<std::string>("/start_st_icon", hrpsys_start_st_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/start-st.png"));
    nh.param<std::string>("/start_imp_icon", hrpsys_start_imp_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/start-imp.png"));
    nh.param<std::string>("/stop_abc_icon", hrpsys_stop_abc_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/stop-abc.png"));
    nh.param<std::string>("/stop_st_icon", hrpsys_stop_st_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/stop-st.png"));
    nh.param<std::string>("/stop_imp_icon", hrpsys_stop_imp_button_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/stop-imp.png"));
    nh.param<std::string>("/display_manip", display_manip_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/display_6dof.png"));
    nh.param<std::string>("/hide_manip", hide_manip_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/hide_6dof.png"));


    ui_->reset_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->reset_pose_button->setIcon(QIcon(QPixmap(QString(reset_pose_button_icon_name.c_str()))));
    ui_->reset_manip_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->reset_manip_pose_button->setIcon(QIcon(QPixmap(QString(reset_manip_pose_button_icon_name.c_str()))));
    ui_->hand_reset_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hand_reset_pose_button->setIcon(QIcon(QPixmap(QString(hand_reset_pose_button_icon_name.c_str()))));
    ui_->hand_hook_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hand_hook_pose_button->setIcon(QIcon(QPixmap(QString(hand_hook_pose_button_icon_name.c_str()))));
    ui_->hand_hook_pose_after_5sec_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hand_hook_pose_after_5sec_button->setIcon(QIcon(QPixmap(QString(hand_hook_pose_button_icon_name.c_str()))));
    ui_->hand_grasp_pose_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hand_grasp_pose_button->setIcon(QIcon(QPixmap(QString(hand_grasp_pose_button_icon_name.c_str()))));
    ui_->hrpsys_start_abc_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_start_abc_button->setIcon(QIcon(QPixmap(QString(hrpsys_start_abc_button_icon_name.c_str()))));
    ui_->hrpsys_start_st_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_start_st_button->setIcon(QIcon(QPixmap(QString(hrpsys_start_st_button_icon_name.c_str()))));
    ui_->hrpsys_start_imp_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_start_imp_button->setIcon(QIcon(QPixmap(QString(hrpsys_start_imp_button_icon_name.c_str()))));
    ui_->hrpsys_stop_abc_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_stop_abc_button->setIcon(QIcon(QPixmap(QString(hrpsys_stop_abc_button_icon_name.c_str()))));
    ui_->hrpsys_stop_st_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_stop_st_button->setIcon(QIcon(QPixmap(QString(hrpsys_stop_st_button_icon_name.c_str()))));
    ui_->hrpsys_stop_imp_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hrpsys_stop_imp_button->setIcon(QIcon(QPixmap(QString(hrpsys_stop_imp_button_icon_name.c_str()))));
    ui_->display_manip_button->setIcon(QIcon(QPixmap(QString(display_manip_icon_name.c_str()))));
    ui_->display_manip_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    ui_->hide_manip_button->setIcon(QIcon(QPixmap(QString(hide_manip_icon_name.c_str()))));
    ui_->hide_manip_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    connect( ui_->reset_pose_button, SIGNAL( clicked() ), this, SLOT( callRequestResetPose()));
    connect( ui_->reset_manip_pose_button, SIGNAL( clicked() ), this, SLOT( callRequestManipPose()));

    connect( ui_->hand_reset_pose_button, SIGNAL( clicked() ), this, SLOT(  callRequestResetGripperPose()));
    connect( ui_->hand_hook_pose_button, SIGNAL( clicked() ), this, SLOT(  callRequestHookGrippePose()));
    connect( ui_->hand_hook_pose_after_5sec_button, SIGNAL( clicked() ), this, SLOT(  callRequestHookGrippePoseAfter5sec()));
    connect( ui_->hand_grasp_pose_button, SIGNAL( clicked() ), this, SLOT(  callRequestGraspGrippePose()));

    connect( ui_->hrpsys_start_abc_button, SIGNAL( clicked() ), this, SLOT(  callRequestStartABC()));
    connect( ui_->hrpsys_start_st_button, SIGNAL( clicked() ), this, SLOT(  callRequestStartST()));
    connect( ui_->hrpsys_start_imp_button, SIGNAL( clicked() ), this, SLOT(  callRequestStartIMP()));

    connect( ui_->hrpsys_stop_abc_button, SIGNAL( clicked() ), this, SLOT(  callRequestStopABC ()));
    connect( ui_->hrpsys_stop_st_button, SIGNAL( clicked() ), this, SLOT(  callRequestStopST()));
    connect( ui_->hrpsys_stop_imp_button, SIGNAL( clicked() ), this, SLOT(  callRequestStopIMP()));

    connect( ui_->display_manip_button, SIGNAL( clicked() ), this, SLOT(  callRequestDisplayManip ()));
    connect( ui_->hide_manip_button, SIGNAL( clicked() ), this, SLOT(  callRequestHideManip ()));

  }

  void DRCTeleopInterfaceAction::callRequestResetPose(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::RESET_POSE);
  };

  void DRCTeleopInterfaceAction::callRequestManipPose(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::RESET_MANIP_POSE);
  };

  void DRCTeleopInterfaceAction::callRequestResetGripperPose(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HAND_RESET_POSE);
  };

  void DRCTeleopInterfaceAction::callRequestHookGrippePose(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HAND_HOOK_POSE);
  };
  void DRCTeleopInterfaceAction::callRequestHookGrippePoseAfter5sec(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HAND_HOOK_AFTER_POSE);
  };

  void DRCTeleopInterfaceAction::callRequestGraspGrippePose(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HAND_GRASP_POSE);
  };

  void DRCTeleopInterfaceAction::callRequestStartABC(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HRPSYS_START_ABC);
  };

  void DRCTeleopInterfaceAction::callRequestStartST(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HRPSYS_START_ST);
  };

  void DRCTeleopInterfaceAction::callRequestStartIMP(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HRPSYS_START_IMP);
  };

  void DRCTeleopInterfaceAction::callRequestStopABC(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HRPSYS_STOP_ABC);
  };

  void DRCTeleopInterfaceAction::callRequestStopST(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HRPSYS_STOP_ST);
  };

  void DRCTeleopInterfaceAction::callRequestStopIMP(){
    callRequestUint8Request(drc_com_common::OCS2FCSmall::HRPSYS_STOP_IMP);
  };

  void DRCTeleopInterfaceAction::callRequestDisplayManip(){
    ros::ServiceClient client = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/transformable_interactive_server/set_parameters", true);
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::BoolParameter bool_param;
    bool_param.name = std::string("display_interactive_manipulator");
    bool_param.value = true;
    srv.request.config.bools.push_back(bool_param);
    if(client.call(srv))
      {
        ROS_INFO("Call Success");
      }
    else{
      ROS_ERROR("Service call FAIL");
    };
  };

  void DRCTeleopInterfaceAction::callRequestHideManip(){
    ros::ServiceClient client = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/transformable_interactive_server/set_parameters", true);
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::BoolParameter bool_param;
    bool_param.name = std::string("display_interactive_manipulator");
    bool_param.value = false;
    srv.request.config.bools.push_back(bool_param);
    if(client.call(srv))
      {
        ROS_INFO("Call Success");
      }
    else{
      ROS_ERROR("Service call FAIL");
    };
  };

  void DRCTeleopInterfaceAction::callRequestUint8Request(uint type){
    ros::ServiceClient client = nh_.serviceClient<drc_task_common::Uint8Request>("/uint8_command", true);
    drc_task_common::Uint8Request srv;
    srv.request.type = type;
    if(client.call(srv))
      {
        ROS_INFO("Call Success");
      }
    else{
      ROS_ERROR("Service call FAIL");
    };
  }

  void DRCTeleopInterfaceAction::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
  }

  void DRCTeleopInterfaceAction::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(drc_task_common::DRCTeleopInterfaceAction, rviz::Panel )
