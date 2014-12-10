#ifndef DRC_TELEOP_INTERFACE_H
#define DRC_TELEOP_INTERFACE_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QtGui>
#include <jsk_rviz_plugins/EusCommand.h>

namespace Ui
{
  class DRCTeleopInterface;
}

namespace drc_task_common
{
  class DRCTeleopInterfaceAction: public rviz::Panel
  {
    Q_OBJECT
    public:
    DRCTeleopInterfaceAction( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  protected Q_SLOTS:

    void callRequestResetPose();
    void callRequestManipPose();
    void callRequestInitPose();

    void callRequestResetGripperPose();
    void callRequestHookGrippePose();
    void callRequestGraspGrippePose();

    void callRequestStartABC();
    void callRequestStartST();
    void callRequestStartIMP();
    void callRequestStartIMPforDrill();

    void callRequestStopABC();
    void callRequestStopST();
    void callRequestStopIMP();

    void callRequestRecogParamDrill();
    void callRequestRecogParamHandle();
    void callRequestRecogParamValve();

    void callRequestEusCommand(std::string command);

  protected:
    // The ROS node handle.
    ros::NodeHandle nh_;
    Ui::DRCTeleopInterface *ui_;
  };

}

#endif // TELEOP_PANEL_H
