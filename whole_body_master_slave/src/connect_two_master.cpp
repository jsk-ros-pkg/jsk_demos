#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <string.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/function.hpp>
#include <ncurses.h>
#include <boost/program_options.hpp>

int shmid;
#define NUM_EES 4
#define NUM_TGTS 10
const int RATE = 1000;
const int MONITOR_RATE = 10;
#define MAX_FRAME_ID_SIZE 1024
std::vector<std::string> ee_names, tgt_names;
std::string master_uri = "http://tablis:11311"; // default
std::string slave_uri = "http://jaxonred:11311"; // default

struct StaticSizedHeader{
    char frame_id[MAX_FRAME_ID_SIZE];// header.frame_id is varid length and unsuitable for shm
    uint32_t seq;
    ros::Time stamp;
};
struct StaticSizedPoseStamed{
    StaticSizedHeader header;
    geometry_msgs::Pose pose;
};
struct StaticSizedWrenchStamped{
    StaticSizedHeader header;
    geometry_msgs::Wrench wrench;
};
struct ros_shm_t{
    StaticSizedPoseStamed masterTgtPoses[NUM_TGTS];
    StaticSizedPoseStamed slaveTgtPoses[NUM_TGTS];
    StaticSizedWrenchStamped masterEEWrenches[NUM_EES];
    StaticSizedWrenchStamped slaveEEWrenches[NUM_EES];
    bool master_side_process_ready;
    bool slave_side_process_ready;
    ros::Duration master_delay, slave_delay;
    ros::Time master_rcv_time, slave_rcv_time;
};

void onTgtPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, StaticSizedPoseStamed* ret_ptr){
    msg->header.frame_id.copy(ret_ptr->header.frame_id, MAX_FRAME_ID_SIZE);
    ret_ptr->header.seq     = msg->header.seq;
    ret_ptr->header.stamp   = msg->header.stamp;
    ret_ptr->pose           = msg->pose;
}
void onEEWrenchCB(const geometry_msgs::WrenchStamped::ConstPtr& msg, StaticSizedWrenchStamped* ret_ptr){
    msg->header.frame_id.copy(ret_ptr->header.frame_id, MAX_FRAME_ID_SIZE);
    ret_ptr->header.seq     = msg->header.seq;
    ret_ptr->header.stamp   = msg->header.stamp;
    ret_ptr->wrench         = msg->wrench;
}

void onMasterDelayCheckPacketCB (const std_msgs::Time::ConstPtr& msg, ros_shm_t* ret_ptr){  ret_ptr->master_rcv_time = msg->data;}
void onSlaveDelayCheckPacketCB  (const std_msgs::Time::ConstPtr& msg, ros_shm_t* ret_ptr){  ret_ptr->slave_rcv_time  = msg->data;}

void master_side_process(int argc, char** argv) {
    std::string ros_mater_uri_str = "ROS_MASTER_URI=" + master_uri;
    putenv( const_cast<char*>(ros_mater_uri_str.c_str()));
    ros_shm_t* shmaddr;
    std::string pname = "master_side_process_node";
    ros::init(argc, argv, pname);
    ros::NodeHandle n;
    ros::Rate rate(RATE);
    ROS_INFO_STREAM(pname << " start with " << ros_mater_uri_str);

    ///// get shm
    if ((shmaddr = (ros_shm_t*)shmat(shmid, NULL, 0)) == (void *) -1) {
        ROS_ERROR_STREAM(pname << " shmat error");
        exit(EXIT_FAILURE);
    }else{
        ROS_INFO_STREAM(pname << " shmat success");
    }

    ///// setup ros pub sub (both delay answer will be published in master side for compare plot)
    std::vector<ros::Subscriber> masterTgtPoses_sub, masterEEWrenches_sub;
    std::vector<ros::Publisher> slaveEEWrenches_pub, slaveTgtPoses_pub;
    for(int i=0; i<tgt_names.size(); i++){
        std::string topic = "master_"+tgt_names[i]+"_pose";
        ROS_INFO_STREAM(pname << " register subscriber " << topic);
        masterTgtPoses_sub.push_back( n.subscribe<geometry_msgs::PoseStamped>(topic, 1,
            boost::bind(onTgtPoseCB, _1, &shmaddr->masterTgtPoses[i]),
            ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay()));
    }
    for(int i=0; i<ee_names.size(); i++){
        std::string topic = "master_"+ee_names[i]+"_wrench";
        ROS_INFO_STREAM(pname << " register subscriber " << topic);
        masterEEWrenches_sub.push_back( n.subscribe<geometry_msgs::WrenchStamped>(topic, 1,
            boost::bind(onEEWrenchCB, _1, &shmaddr->masterEEWrenches[i]),
            ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay()));
    }
    for(int i=0; i<ee_names.size(); i++){
        std::string topic = "slave_"+ee_names[i]+"_wrench";
        ROS_INFO_STREAM(pname << " register publisher " << topic);
        slaveEEWrenches_pub.push_back( n.advertise<geometry_msgs::WrenchStamped>(topic, 1));
    }
    for(int i=0; i<tgt_names.size(); i++){
        std::string topic = "slave_"+tgt_names[i]+"_pose";
        ROS_INFO_STREAM(pname << " register publisher " << topic);
        slaveTgtPoses_pub.push_back( n.advertise<geometry_msgs::PoseStamped>(topic, 1));
    }
    ros::Publisher  master_delay_ans_pub    = n.advertise<std_msgs::Float64>("master_delay_ans",    1);
    ros::Publisher  slave_delay_ans_pub     = n.advertise<std_msgs::Float64>("slave_delay_ans",     1);
    ros::Publisher  delay_check_packet_pub  = n.advertise<std_msgs::Time>   ("delay_check_packet_inbound",  1);
    ros::Subscriber delay_check_packet_sub  = n.subscribe<std_msgs::Time>   ("delay_check_packet_outbound", 1,
                                    boost::bind(onMasterDelayCheckPacketCB, _1, shmaddr),
                                    ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay());
    shmaddr->master_side_process_ready = true;
    ROS_INFO_STREAM(pname << " ready, enter loop");

    ///// main loop
    std::vector<geometry_msgs::WrenchStamped>   latest_w(ee_names.size());
    std::vector<geometry_msgs::PoseStamped>     latest_p(tgt_names.size());
    while (ros::ok()) {
        ros_shm_t now = *shmaddr; // copy from shm as soon as possible TODO mutex?
        for(int i=0; i<ee_names.size(); i++){
            if(now.slaveEEWrenches[i].header.stamp != latest_w[i].header.stamp ||
	       now.slaveEEWrenches[i].header.seq   != latest_w[i].header.seq){ // update only if new data
                latest_w[i].header.frame_id  = now.slaveEEWrenches[i].header.frame_id;
                latest_w[i].header.stamp     = now.slaveEEWrenches[i].header.stamp;
                latest_w[i].header.seq       = now.slaveEEWrenches[i].header.seq;
                latest_w[i].wrench           = now.slaveEEWrenches[i].wrench;
                slaveEEWrenches_pub[i].publish(latest_w[i]);
            }
        }
        for(int i=0; i<tgt_names.size(); i++){
            if(now.slaveTgtPoses[i].header.stamp != latest_p[i].header.stamp ||
	       now.slaveTgtPoses[i].header.seq   != latest_p[i].header.seq){ // update only if new data
                latest_p[i].header.frame_id  = now.slaveTgtPoses[i].header.frame_id;
                latest_p[i].header.stamp     = now.slaveTgtPoses[i].header.stamp;
                latest_p[i].header.seq       = now.slaveTgtPoses[i].header.seq;
                latest_p[i].pose             = now.slaveTgtPoses[i].pose;
                slaveTgtPoses_pub[i].publish(latest_p[i]);
            }
        }
        ////////////// calc and pub delay answer
        ros::Time ros_time_now  = ros::Time(ros::WallTime::now().sec, ros::WallTime::now().nsec);
        std_msgs::Float64 master_delay_ans, slave_delay_ans;
        shmaddr->master_delay   = ros_time_now - now.master_rcv_time;
        shmaddr->slave_delay    = ros_time_now - now.slave_rcv_time;
        master_delay_ans.data   = shmaddr->master_delay.toSec();
        slave_delay_ans.data    = shmaddr->slave_delay.toSec();
        master_delay_ans_pub.publish(master_delay_ans);
        slave_delay_ans_pub.publish(slave_delay_ans);
        ////////////// send timestamp for delay calc
        std_msgs::Time abs_time_now;
        abs_time_now.data = ros::Time(ros::WallTime::now().sec, ros::WallTime::now().nsec);
        delay_check_packet_pub.publish(abs_time_now);
        ros::spinOnce();
        rate.sleep();
    }

    ///// exit
    if(shmdt(shmaddr) == 0){    ROS_INFO_STREAM(pname << " shmdt success"); }
    else{                       ROS_ERROR_STREAM(pname << " shmdt fail"); exit(EXIT_FAILURE);}
    ROS_INFO_STREAM(pname << " EXIT_SUCCESS");
    exit(EXIT_SUCCESS);
}

void slave_side_process(int argc, char** argv) {
    std::string ros_mater_uri_str = "ROS_MASTER_URI=" + slave_uri;
    putenv( const_cast<char*>(ros_mater_uri_str.c_str()));
    ros_shm_t* shmaddr;
    std::string pname = "slave_side_process_node";
    ros::init(argc, argv, pname);
    ros::NodeHandle n;
    ros::Rate rate(RATE);
    sleep(1); // ??????????????????
    ROS_INFO_STREAM(pname << " start with " << ros_mater_uri_str);

    ///// get shm
    if ((shmaddr = (ros_shm_t*)shmat(shmid, NULL, 0)) == (void *) -1) {
        ROS_ERROR_STREAM(pname << " shmat error");
        exit(EXIT_FAILURE);
    }else{
        ROS_INFO_STREAM(pname << " shmat success");
    }

    ///// setup ros pub sub
    std::vector<ros::Publisher> masterTgtPoses_pub, masterEEWrenches_pub;
    std::vector<ros::Subscriber> slaveEEWrenches_sub, slaveTgtPoses_sub;
    for(int i=0; i<tgt_names.size(); i++){
        std::string topic = "master_"+tgt_names[i]+"_pose";
        ROS_INFO_STREAM(pname << " register publisher " << topic);
        masterTgtPoses_pub.push_back( n.advertise<geometry_msgs::PoseStamped>(topic, 1));
    }
    for(int i=0; i<ee_names.size(); i++){
        std::string topic = "master_"+ee_names[i]+"_wrench";
        ROS_INFO_STREAM(pname << " register publisher " << topic);
        masterEEWrenches_pub.push_back( n.advertise<geometry_msgs::WrenchStamped>(topic, 1));
    }
    for(int i=0; i<ee_names.size(); i++){
        std::string topic = "slave_"+ee_names[i]+"_wrench";
        ROS_INFO_STREAM(pname << " register subscriber " << topic);
        slaveEEWrenches_sub.push_back( n.subscribe<geometry_msgs::WrenchStamped>(topic, 1,
            boost::bind(onEEWrenchCB, _1, &shmaddr->slaveEEWrenches[i]),
            ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay()));
    }
    for(int i=0; i<tgt_names.size(); i++){
        std::string topic = "slave_"+tgt_names[i]+"_pose";
        ROS_INFO_STREAM(pname << " register subscriber " << topic);
        slaveTgtPoses_sub.push_back( n.subscribe<geometry_msgs::PoseStamped>(topic, 1,
            boost::bind(onTgtPoseCB, _1, &shmaddr->slaveTgtPoses[i]),
            ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay()));
    }
    ros::Publisher  master_delay_ans_pub    = n.advertise<std_msgs::Float64>("master_delay_ans", 1);
    ros::Publisher  slave_delay_ans_pub     = n.advertise<std_msgs::Float64>("slave_delay_ans", 1);
    ros::Publisher  delay_check_packet_pub  = n.advertise<std_msgs::Time>("delay_check_packet_inbound", 1);
    ros::Subscriber delay_check_packet_sub  = n.subscribe<std_msgs::Time>("delay_check_packet_outbound", 1,
                                boost::bind(onSlaveDelayCheckPacketCB, _1, shmaddr),
                                ros::VoidConstPtr(), ros::TransportHints().unreliable().reliable().tcpNoDelay());
    shmaddr->slave_side_process_ready = true;
    ROS_INFO_STREAM(pname << " ready, enter loop");

    ///// main loop
    std::vector<geometry_msgs::PoseStamped>     latest_p(tgt_names.size());
    std::vector<geometry_msgs::WrenchStamped>   latest_w(ee_names.size());
    while (ros::ok()) {
        ros_shm_t now = *shmaddr; // copy from shm as soon as possible TODO mutex?
        for(int i=0; i<tgt_names.size(); i++){
            if(tgt_names[i] == "lhand" || tgt_names[i] == "rhand" || tgt_names[i] == "head"){ continue; }// disable tablis unused topics
            if(now.masterTgtPoses[i].header.stamp != latest_p[i].header.stamp ||
	       now.masterTgtPoses[i].header.seq   != latest_p[i].header.seq){ // update only if new data
                latest_p[i].header.frame_id = now.masterTgtPoses[i].header.frame_id;
                latest_p[i].header.stamp    = now.masterTgtPoses[i].header.stamp;
                latest_p[i].header.seq      = now.masterTgtPoses[i].header.seq;
                latest_p[i].pose            = now.masterTgtPoses[i].pose;
                masterTgtPoses_pub[i].publish(latest_p[i]);
            }
        }
        for(int i=0; i<ee_names.size(); i++){
            if(now.masterTgtPoses[i].header.stamp != latest_w[i].header.stamp ||
	       now.masterTgtPoses[i].header.seq   != latest_w[i].header.seq){ // update only if new data
                latest_w[i].header.frame_id = now.masterEEWrenches[i].header.frame_id;
                latest_w[i].header.stamp    = now.masterEEWrenches[i].header.stamp;
                latest_w[i].header.seq      = now.masterEEWrenches[i].header.seq;
                latest_w[i].wrench          = now.masterEEWrenches[i].wrench;
                masterEEWrenches_pub[i].publish(latest_w[i]);
            }
        }
        ////////////// calc and pub delay answer
        std_msgs::Float64 master_delay_ans, slave_delay_ans;
        master_delay_ans.data   = now.master_delay.toSec();
        slave_delay_ans.data    = now.slave_delay.toSec();
        master_delay_ans_pub.publish(master_delay_ans);
        slave_delay_ans_pub.publish(slave_delay_ans);
        ////////////// send timestamp for delay calc
        std_msgs::Time abs_time_now;
        abs_time_now.data = ros::Time(ros::WallTime::now().sec, ros::WallTime::now().nsec);
        delay_check_packet_pub.publish(abs_time_now);
        ros::spinOnce();
        rate.sleep();
    }

    ///// exit
    if(shmdt(shmaddr) == 0){    ROS_INFO_STREAM(pname << " shmdt success"); }
    else{                       ROS_ERROR_STREAM(pname << " shmdt fail"); exit(EXIT_FAILURE);}
    ROS_INFO_STREAM(pname << " EXIT_SUCCESS");
    exit(EXIT_SUCCESS);
}

int main(int argc, char** argv) {
    int child_cnt;
    ee_names.push_back("lleg");
    ee_names.push_back("rleg");
    ee_names.push_back("larm");
    ee_names.push_back("rarm");
    tgt_names = ee_names;
    tgt_names.push_back("com");
    tgt_names.push_back("head");
    tgt_names.push_back("lhand");
    tgt_names.push_back("rhand");
    tgt_names.push_back("lfloor");
    tgt_names.push_back("rfloor");
    assert(ee_names.size()  == NUM_EES);
    assert(tgt_names.size() == NUM_TGTS);

    ///// create shm
    if ((shmid = shmget(IPC_PRIVATE, sizeof(ros_shm_t), 0600)) == -1) {
        ROS_ERROR("shmget error");
        exit(EXIT_FAILURE);
    }{
        ROS_INFO("shmget success");
    }

    ///// parse options
    boost::program_options::options_description op("target_uri_info");
    op.add_options()
            ("help,h",                                                      "show help.")
            ("master_uri,m",boost::program_options::value<std::string>(),   "master hostname or ip.")
            ("slave_uri,s", boost::program_options::value<std::string>(),   "slave hostname or ip.");
    boost::program_options::variables_map argmap;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, op), argmap);
    boost::program_options::notify(argmap);
    if( argmap.count("help") ){         std::cerr << op << std::endl; return 1; }
    if( argmap.count("master_uri") ){   master_uri  = argmap["master_uri"].as<std::string>();   }
    if( argmap.count("slave_uri") ){    slave_uri   = argmap["slave_uri"].as<std::string>();    }
    std::cerr << "master_uri is set as = "  << master_uri   << std::endl;
    std::cerr << "slave_uri is set as = "   << slave_uri    << std::endl;

    ///// start fork
    if(fork() == 0){ master_side_process(argc, argv); }
    if(fork() == 0){ slave_side_process(argc, argv); }

    ///// below is main draw process
    ros::init(argc, argv, "main_process_node");
    ros::NodeHandle n;
    ros::Rate rate(MONITOR_RATE);

    ///// get shm
    ros_shm_t* shmaddr;
    if ((shmaddr = (ros_shm_t*)shmat(shmid, NULL, 0)) == (void *) -1) {
        std::cerr << " shmat error"<< std::endl; exit(EXIT_FAILURE);
    }else{
        std::cerr << " shmat success"<< std::endl;
    }
    ///// wait for both child process
    while(ros::ok()){
        if(shmaddr->master_side_process_ready && shmaddr->slave_side_process_ready){
            ROS_INFO("both process ready, enter ncurses"); break;
        }else{
            ROS_WARN("wait for both process ready"); sleep(1);
        }
    }

    ///// setup ncurses
    initscr();
    start_color();
    use_default_colors();
    init_pair(1, COLOR_WHITE, COLOR_GREEN);
    init_pair(2, COLOR_WHITE, COLOR_YELLOW);
    init_pair(3, COLOR_WHITE, COLOR_RED);
    clear();

    ///// main loop
    ros_shm_t prev_data = *shmaddr;
    for(unsigned int draw_cnt = 0, line = 1; ros::ok(); line = 1, draw_cnt++){
        ros_shm_t now = *shmaddr; // copy from shm as soon as possible TODO mutex?
        ros::Time ros_time_now = ros::Time(ros::WallTime::now().sec, ros::WallTime::now().nsec);
        erase();

        ///// draw common info
        printw("Drawn frame count %d", draw_cnt);
        move(line++, 0);
        printw("Master = %s", master_uri.c_str());
        move(line++, 0);
        printw("Slave  = %s", slave_uri.c_str());
        move(line++, 0);
        printw("Master side communication delay %8.3f [ms] ( now %12.6f [s] / rcv %12.6f [s] )",
            now.master_delay.toSec()*1e3, ros_time_now.toSec(), now.master_rcv_time.toSec());
        move(line++, 0);
        printw("Slave  side communication delay %8.3f [ms] ( now %12.6f [s] / rcv %12.6f [s] )",
            now.slave_delay.toSec()*1e3, ros_time_now.toSec(), now.slave_rcv_time.toSec());
        move(line++, 0);
        printw("=============================================================================================================================");
        move(line++, 0);

        ///// draw master side info
        for(int i=0; i<NUM_TGTS; i++){
            const int fps = (now.masterTgtPoses[i].header.seq - prev_data.masterTgtPoses[i].header.seq) * MONITOR_RATE;
            const bool is_moving = fabs(now.masterTgtPoses[i].pose.position.x != prev_data.masterTgtPoses[i].pose.position.x) > FLT_EPSILON;
            if(fps>0){
                if(is_moving){  attrset(COLOR_PAIR(1)); printw("%8s", "OK");}
                else{           attrset(COLOR_PAIR(2)); printw("%8s", "NOT_MOVE");} }
            else{               attrset(COLOR_PAIR(3)); printw("%8s", "NOT_RECV");}
            attrset(0);
            std::string topic = "master_"+tgt_names[i]+"_pose";
            printw(" %-18s ",topic.c_str());
            printw("[%9df @ %4d fps] %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ", now.masterTgtPoses[i].header.seq, fps,
                now.masterTgtPoses[i].pose.position.x,
                now.masterTgtPoses[i].pose.position.y,
                now.masterTgtPoses[i].pose.position.z,
                now.masterTgtPoses[i].pose.orientation.x,
                now.masterTgtPoses[i].pose.orientation.y,
                now.masterTgtPoses[i].pose.orientation.z,
                now.masterTgtPoses[i].pose.orientation.w
                );
            move(line++, 0);
        }
        printw("-----------------------------------------------------------------------------------------------------------------------------");
        move(line++, 0);
        for(int i=0; i<NUM_EES; i++){
            const int fps = (now.masterEEWrenches[i].header.seq - prev_data.masterEEWrenches[i].header.seq) * MONITOR_RATE;
            const bool is_moving = fabs(now.masterEEWrenches[i].wrench.force.x - prev_data.masterEEWrenches[i].wrench.force.x) > FLT_EPSILON;
            if(fps>0){
                if(is_moving){  attrset(COLOR_PAIR(1)); printw("%8s", "OK");}
                else{           attrset(COLOR_PAIR(2)); printw("%8s", "NOT_MOVE");} }
            else{               attrset(COLOR_PAIR(3)); printw("%8s", "NOT_RECV");}
            attrset(0);
            std::string topic = "master_"+ee_names[i]+"_wrench";
            printw(" %-18s ",topic.c_str());
            printw("[%9df @ %4d fps] %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ", now.masterEEWrenches[i].header.seq, fps,
                now.masterEEWrenches[i].wrench.force.x,
                now.masterEEWrenches[i].wrench.force.y,
                now.masterEEWrenches[i].wrench.force.z,
                now.masterEEWrenches[i].wrench.torque.x,
                now.masterEEWrenches[i].wrench.torque.y,
                now.masterEEWrenches[i].wrench.torque.z
                );
            move(line++, 0);
        }
        printw("=============================================================================================================================");
        move(line++, 0);

        ///// draw slave side info
        for(int i=0; i<NUM_TGTS; i++){
            const int fps = (now.slaveTgtPoses[i].header.seq - prev_data.slaveTgtPoses[i].header.seq) * MONITOR_RATE;
            const bool is_moving = fabs(now.slaveTgtPoses[i].pose.position.x != prev_data.slaveTgtPoses[i].pose.position.x) > FLT_EPSILON;
            if(fps>0){
                if(is_moving){  attrset(COLOR_PAIR(1)); printw("%8s", "OK");}
                else{           attrset(COLOR_PAIR(2)); printw("%8s", "NOT_MOVE");} }
            else{               attrset(COLOR_PAIR(3)); printw("%8s", "NOT_RECV");}
            attrset(0);
            std::string topic = "slave_"+tgt_names[i]+"_pose";
            printw(" %-18s ",topic.c_str());
            printw("[%9df @ %4d fps] %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ", now.slaveTgtPoses[i].header.seq, fps,
                now.slaveTgtPoses[i].pose.position.x,
                now.slaveTgtPoses[i].pose.position.y,
                now.slaveTgtPoses[i].pose.position.z,
                now.slaveTgtPoses[i].pose.orientation.x,
                now.slaveTgtPoses[i].pose.orientation.y,
                now.slaveTgtPoses[i].pose.orientation.z,
                now.slaveTgtPoses[i].pose.orientation.w
                );
            move(line++, 0);
        }
        printw("-----------------------------------------------------------------------------------------------------------------------------");
        move(line++, 0);
        for(int i=0; i<NUM_EES; i++){
            const int fps = (now.slaveEEWrenches[i].header.seq - prev_data.slaveEEWrenches[i].header.seq) * MONITOR_RATE;
            const bool is_moving = fabs(now.slaveEEWrenches[i].wrench.force.x - prev_data.slaveEEWrenches[i].wrench.force.x) > FLT_EPSILON;
            if(fps>0){
                if(is_moving){  attrset(COLOR_PAIR(1)); printw("%8s", "OK");}
                else{           attrset(COLOR_PAIR(2)); printw("%8s", "NOT_MOVE");} }
            else{               attrset(COLOR_PAIR(3)); printw("%8s", "NOT_RECV");}
            attrset(0);
            std::string topic = "slave_"+ee_names[i]+"_wrench";
            printw(" %-18s ",topic.c_str());
            printw("[%9df @ %4d fps] %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ", now.slaveEEWrenches[i].header.seq, fps,
                now.slaveEEWrenches[i].wrench.force.x,
                now.slaveEEWrenches[i].wrench.force.y,
                now.slaveEEWrenches[i].wrench.force.z,
                now.slaveEEWrenches[i].wrench.torque.x,
                now.slaveEEWrenches[i].wrench.torque.y,
                now.slaveEEWrenches[i].wrench.torque.z
                );
            move(line++, 0);
        }

        prev_data = now;
        refresh();
        rate.sleep();
    }

    ///// exit
    endwin();
    for (child_cnt = 0; child_cnt < 2; ++child_cnt) { wait(NULL); }
    if (shmctl(shmid, IPC_RMID, NULL) == -1) { ROS_ERROR("shmid error"); exit(EXIT_FAILURE); }
    else { ROS_INFO("shmid success"); }
    ROS_INFO("EXIT_SUCCESS");
    return EXIT_SUCCESS;
}
