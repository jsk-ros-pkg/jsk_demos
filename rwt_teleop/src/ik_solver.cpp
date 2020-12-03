// ros
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
//boost
#include <boost/function.hpp>
#include <boost/program_options.hpp>
// //hrpsys
// #include <hrpModel/Body.h>
// #include <hrpModel/Link.h>
// #include <hrpModel/JointPath.h>
// #include <hrpModel/ModelLoaderUtil.h>
// #include <cmath>

// void onTgtPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, StaticSizedPoseStamed* ret_ptr){
//     msg->header.frame_id.copy(ret_ptr->header.frame_id, MAX_FRAME_ID_SIZE);
//     ret_ptr->header.seq     = msg->header.seq;
//     ret_ptr->header.stamp   = msg->header.stamp;
//     ret_ptr->pose           = msg->pose;
// }
// void onEEWrenchCB(const geometry_msgs::WrenchStamped::ConstPtr& msg, StaticSizedWrenchStamped* ret_ptr){
//     msg->header.frame_id.copy(ret_ptr->header.frame_id, MAX_FRAME_ID_SIZE);
//     ret_ptr->header.seq     = msg->header.seq;
//     ret_ptr->header.stamp   = msg->header.stamp;
//     ret_ptr->wrench         = msg->wrench;
// }
// void onMasterDelayCheckPacketCB (const std_msgs::Time::ConstPtr& msg, ros_shm_t* ret_ptr){  ret_ptr->master_rcv_time = msg->data;}
// void onSlaveDelayCheckPacketCB  (const std_msgs::Time::ConstPtr& msg, ros_shm_t* ret_ptr){  ret_ptr->slave_rcv_time  = msg->data;}

// #include <iomanip>
// #include <rtm/CorbaNaming.h>
// #include <hrpModel/Link.h>
// #include <hrpModel/JointPath.h>
// #include <hrpUtil/Eigen3d.h>
// #include <hrpUtil/Eigen4d.h>
// #include <hrpCollision/ColdetModel.h>
// #include <rtm/idl/BasicDataType.hh>
// // #include "hrpsys/idl/HRPDataTypes.hh"
// #include <rtm/Manager.h>
// #include <rtm/DataFlowComponentBase.h>
// #include <rtm/CorbaPort.h>
// #include <rtm/DataInPort.h>
// #include <rtm/DataOutPort.h>
// #include <rtm/idl/BasicDataTypeSkel.h>
// #include <hrpModel/Body.h>
// #include <hrpModel/ColdetLinkPair.h>
// #include <hrpModel/ModelLoaderUtil.h>
// // #include "TimedPosture.h"
// // #include "interpolator.h"
// // #include "VclipLinkPair.h"
// // #include "CollisionDetectorService_impl.h"
// // #include "../SoftErrorLimiter/beep.h"
// #include <rtm/Manager.h>
// #include <rtm/DataFlowComponentBase.h>
// #include <rtm/CorbaPort.h>
// #include <rtm/DataInPort.h>
// #include <rtm/DataOutPort.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ik_solver");
    ros::NodeHandle n;
    ros::Rate rate(100);



    // hrp::BodyPtr robot_for_ik;
    // // OpenHRP::BodyInfo_ptr bodyInfo = loadBodyInfo("aaaa.dae");
    // RTC::Manager& rtcManager = RTC::Manager::instance();
    // // std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    // std::string nameServer = "localhost:2809";
    // int comPos = nameServer.find(",");
    // if (comPos < 0){
    //     comPos = nameServer.length();
    // }
    // std::cerr<<nameServer<<std::endl;
    // nameServer = nameServer.substr(0, comPos);
    // std::cerr<<nameServer<<std::endl;
    // RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    // OpenHRP::BodyInfo_var binfo;
    // binfo = hrp::loadBodyInfo("/home/leus/catkin_ws/kuka/src/openhrp3/sample/model/PA10/pa10.main.wrl", CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    // std::cerr<<binfo->name()<<std::endl;

    ///// setup ros pub sub
    // const std::vector<std::string> tgt_names = {"larm", "rarm", "lleg", "rleg", "head"};
    // const std::vector<std::string> ee_names = {"larm", "rarm", "lleg", "rleg"};
    // std::vector<ros::Subscriber> masterTgtPoses_sub, masterEEWrenches_sub;
    // std::vector<ros::Publisher> slaveEEWrenches_pub, slaveTgtPoses_pub;
    // for(int i=0; i<tgt_names.size(); i++){
    //     std::string topic = "master_"+tgt_names[i]+"_pose";
    //     ROS_INFO_STREAM(pname << " register subscriber " << topic);
    //     masterTgtPoses_sub.push_back( n.subscribe<geometry_msgs::PoseStamped>(topic, 1,
    //         boost::bind(onTgtPoseCB, _1, &shmaddr->masterTgtPoses[i]),
    //         ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay()));
    // }
    // for(int i=0; i<ee_names.size(); i++){
    //     std::string topic = "master_"+ee_names[i]+"_wrench";
    //     ROS_INFO_STREAM(pname << " register subscriber " << topic);
    //     masterEEWrenches_sub.push_back( n.subscribe<geometry_msgs::WrenchStamped>(topic, 1,
    //         boost::bind(onEEWrenchCB, _1, &shmaddr->masterEEWrenches[i]),
    //         ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay()));
    // }
    // for(int i=0; i<ee_names.size(); i++){
    //     std::string topic = "slave_"+ee_names[i]+"_wrench";
    //     ROS_INFO_STREAM(pname << " register publisher " << topic);
    //     slaveEEWrenches_pub.push_back( n.advertise<geometry_msgs::WrenchStamped>(topic, 1));
    // }
    // for(int i=0; i<tgt_names.size(); i++){
    //     std::string topic = "slave_"+tgt_names[i]+"_pose";
    //     ROS_INFO_STREAM(pname << " register publisher " << topic);
    //     slaveTgtPoses_pub.push_back( n.advertise<geometry_msgs::PoseStamped>(topic, 1));
    // }
    // ros::Publisher  master_delay_ans_pub    = n.advertise<std_msgs::Float64>("master_delay_ans",    1);
    // ros::Publisher  slave_delay_ans_pub     = n.advertise<std_msgs::Float64>("slave_delay_ans",     1);
    // ros::Publisher  delay_check_packet_pub  = n.advertise<std_msgs::Time>   ("delay_check_packet_inbound",  1);
    // ros::Subscriber delay_check_packet_sub  = n.subscribe<std_msgs::Time>   ("delay_check_packet_outbound", 1,
                                    // boost::bind(onMasterDelayCheckPacketCB, _1, shmaddr),
                                    // ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay());

    std::vector<ros::Publisher> joint_angle_pubs;
    const std::vector<std::string> LR_str = {"L", "R"};
    const std::vector<std::string> joint_names = {"J1", "J2", "J3", "J4", "J5", "J6", "J7"};
    for(auto lr : LR_str){
        for(auto jn : joint_names){
            const std::string topic = "/iiwa/PositionJointInterface_"+lr+jn+"_controller/command";
            ROS_INFO_STREAM("Register publisher " << topic);
            joint_angle_pubs.push_back( n.advertise<std_msgs::Float64>(topic, 1));
        }
    }

    ///// main loop
    // std::vector<geometry_msgs::WrenchStamped>   latest_w(ee_names.size());
    // std::vector<geometry_msgs::PoseStamped>     latest_p(tgt_names.size());
    while (ros::ok()) {
        ros::Time ros_time_now = ros::Time(ros::WallTime::now().sec, ros::WallTime::now().nsec);
        int test = 1;
        for(auto jap : joint_angle_pubs){
            std_msgs::Float64 val;
            val.data = 3.14 * sin(2 * M_PI * ros::Time::now().toSec() * test++ * 0.01);
            jap.publish(val);
        }
        // for(int i=0; i<ee_names.size(); i++){
        //     if(now.slaveEEWrenches[i].header.stamp != latest_w[i].header.stamp){ // update only if new data
        //         latest_w[i].header.frame_id  = now.slaveEEWrenches[i].header.frame_id;
        //         latest_w[i].header.stamp     = now.slaveEEWrenches[i].header.stamp;
        //         latest_w[i].header.seq       = now.slaveEEWrenches[i].header.seq;
        //         latest_w[i].wrench           = now.slaveEEWrenches[i].wrench;
        //         slaveEEWrenches_pub[i].publish(latest_w[i]);
        //     }
        // }
        // for(int i=0; i<tgt_names.size(); i++){
        //     if(now.slaveTgtPoses[i].header.stamp != latest_p[i].header.stamp){ // update only if new data
        //         latest_p[i].header.frame_id  = now.slaveTgtPoses[i].header.frame_id;
        //         latest_p[i].header.stamp     = now.slaveTgtPoses[i].header.stamp;
        //         latest_p[i].header.seq       = now.slaveTgtPoses[i].header.seq;
        //         latest_p[i].pose             = now.slaveTgtPoses[i].pose;
        //         slaveTgtPoses_pub[i].publish(latest_p[i]);
        //     }
        // }
        // ////////////// calc and pub delay answer
        // ros::Time ros_time_now  = ros::Time(ros::WallTime::now().sec, ros::WallTime::now().nsec);
        // std_msgs::Float64 master_delay_ans, slave_delay_ans;
        // shmaddr->master_delay   = ros_time_now - now.master_rcv_time;
        // shmaddr->slave_delay    = ros_time_now - now.slave_rcv_time;
        // master_delay_ans.data   = shmaddr->master_delay.toSec();
        // slave_delay_ans.data    = shmaddr->slave_delay.toSec();
        // master_delay_ans_pub.publish(master_delay_ans);
        // slave_delay_ans_pub.publish(slave_delay_ans);
        // ////////////// send timestamp for delay calc
        // std_msgs::Time abs_time_now;
        // abs_time_now.data = ros::Time(ros::WallTime::now().sec, ros::WallTime::now().nsec);
        // delay_check_packet_pub.publish(abs_time_now);
        ROS_INFO_THROTTLE(3,"test");
        ros::spinOnce();
        rate.sleep();
    }
}