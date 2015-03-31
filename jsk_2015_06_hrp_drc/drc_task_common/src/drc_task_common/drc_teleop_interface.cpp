#include "rviz/config.h"
#include "drc_teleop_interface.h"
#include "ros/time.h"
#include <ros/package.h>

#include "ui_drc_teleop_interface.h"
#include "drc_task_common/GetIKArm.h"
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
    nh.param<std::string>("/display_manip", display_manip_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/recog-param-valve.png"));
    nh.param<std::string>("/hide_manip", hide_manip_icon_name, ros::package::getPath("drc_task_common")+std::string("/icons/recog-param-valve.png"));


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
    std::string command("(send *ri* :angle-vector (send *robot* :reset-pose) 5000))");
    callRequestEusCommand(command);
  };

  void DRCTeleopInterfaceAction::callRequestManipPose(){
    std::string command("(send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)");
    callRequestEusCommand(command);
  };

  void DRCTeleopInterfaceAction::callRequestResetGripperPose(){
    std::string command("(progn (send *robot* :hand :arms :reset-pose) (send *ri* :hand-angle-vector (apply #\'concatenate float-vector (send *robot* :hand :arms :angle-vector))))");
    callRequestEusCommand(command);
  };

  void DRCTeleopInterfaceAction::callRequestHookGrippePose(){
    std::string command("(progn (send *robot* :hand :arms :hook-pose) (send *ri* :hand-angle-vector (apply #\'concatenate float-vector (send *robot* :hand :arms :angle-vector))))");
    callRequestEusCommand(command);
  };
  void DRCTeleopInterfaceAction::callRequestHookGrippePoseAfter5sec(){
    std::string command("(progn (unix::sleep 5) (send *robot* :hand :arms :hook-pose) (send *ri* :hand-angle-vector (apply #\'concatenate float-vector (send *robot* :hand :arms :angle-vector))))");
    callRequestEusCommand(command);
  };

  std::string DRCTeleopInterfaceAction::getIKArm(){
    ros::ServiceClient client = nh_.serviceClient<drc_task_common::GetIKArm>("/get_ik_arm", true);
    drc_task_common::GetIKArm srv;
    if(client.call(srv)){
      ROS_INFO("Get Arm Call Success (%s)", srv.response.ik_arm.c_str());
      return srv.response.ik_arm;
    }
    else{
      ROS_ERROR("Get Arm Service call FAIL");
      return std::string(":arms");
    }
  }
  void DRCTeleopInterfaceAction::callRequestGraspGrippePose(){
    char command_str[512];
    std::string arm_string = getIKArm();
    sprintf(command_str, 
            "(progn (send *robot* :hand %s :extension-pose) (send *ri* :hand-angle-vector (apply #\'concatenate float-vector (send *robot* :hand :arms :angle-vector))) (send *ri* :hand-wait-interpolation) (send *robot* :hand %s :grasp-pose) (send *ri* :hand-angle-vector (apply #\'concatenate float-vector (send *robot* :hand :arms :angle-vector))))"
            , arm_string.c_str(), arm_string.c_str()
      );
    callRequestEusCommand(std::string(command_str));
  };

  void DRCTeleopInterfaceAction::callRequestStartABC(){
    std::string command("(send *ri* :start-auto-balancer)");
    callRequestEusCommand(command);
  };

  void DRCTeleopInterfaceAction::callRequestStartST(){
    std::string command("(send *ri* :start-st)");
    callRequestEusCommand(command);
  };

  void DRCTeleopInterfaceAction::callRequestStartIMP(){
    std::string command("(send *ri* :start-impedance :arms :moment-gain #f(0 0 0) :k-p 1000 :d-p 400)");
    callRequestEusCommand(command);
  };

  void DRCTeleopInterfaceAction::callRequestStopABC(){
    std::string command("(send *ri* :stop-auto-balancer)");
    callRequestEusCommand(command);
  };

  void DRCTeleopInterfaceAction::callRequestStopST(){
    std::string command("(send *ri* :stop-st)");
    callRequestEusCommand(command);
  };

  void DRCTeleopInterfaceAction::callRequestStopIMP(){
    std::string command("(send *ri* :stop-impedance :arms)");
    callRequestEusCommand(command);
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

  void DRCTeleopInterfaceAction::callRequestEusCommand(std::string command){
    ros::ServiceClient client = nh_.serviceClient<jsk_rviz_plugins::EusCommand>("/uint8_command", true);
    jsk_rviz_plugins::EusCommand srv;
    srv.request.command = command;
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
