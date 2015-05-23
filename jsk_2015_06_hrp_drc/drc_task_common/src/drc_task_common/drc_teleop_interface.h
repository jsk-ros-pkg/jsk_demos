#ifndef DRC_TELEOP_INTERFACE_H
#define DRC_TELEOP_INTERFACE_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QtGui>
#include <drc_task_common/Uint8Request.h>
#include <drc_com_common/OCS2FCSmall.h>
#include <stdio.h>

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
    void callRequestResetManipPose();
    void callRequestDrillResetPose();
    void callRequestWatchDrillPose();
    void callRequestDoorThroughPose();

    void callRequestResetGripperPose();
    void callRequestHookGrippePose();
    void callRequestHookGrippePoseAfter5sec();

    void callRequestGraspGrippePose();

    void callRequestStartABC();
    void callRequestStartST();
    void callRequestStartIMP();
    void callRequestStartIMPSoft();
    void callRequestStartIMPHard();

    /* void callRequestStopABC(); */
    /* void callRequestStopST(); */
    void callRequestStopIMP();

    void callRequestHandCalib();

    void callRequestDisplayManip();
    void callRequestHideManip();
    
    void callRequestUint8Request(uint type);
    
  protected:
    // The ROS node handle.
    ros::NodeHandle nh_;
    Ui::DRCTeleopInterface *ui_;
  };

}

#endif // TELEOP_PANEL_H
