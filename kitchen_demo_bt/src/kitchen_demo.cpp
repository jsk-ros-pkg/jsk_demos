#include <ros/ros.h>
#include <ros/package.h>

#define DEBUG  // rosbridgecpp logging
#include <roseus_bt/eus_nodes.h>
#include <roseus_bt/command_line_argument_mapping.h>
#include <roseus_bt/rosparam_argument_mapping.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <kitchen_demo_bt/AutoDockAction.h>
#include <kitchen_demo_bt/FinishAction.h>
#include <kitchen_demo_bt/GetLightOnAction.h>
#include <kitchen_demo_bt/InitAction.h>
#include <kitchen_demo_bt/InspectSpotAction.h>
#include <kitchen_demo_bt/MoveToSpotAction.h>
#include <kitchen_demo_bt/ReportAutoDockFailureAction.h>
#include <kitchen_demo_bt/ReportFailureAction.h>
#include <kitchen_demo_bt/ReportLightOnAction.h>
#include <kitchen_demo_bt/ReportStartGoToKitchenAction.h>
#include <kitchen_demo_bt/RoomLightOffAction.h>
#include <kitchen_demo_bt/RoomLightOnAction.h>

using namespace BT;


class AutoDock: public EusActionNode<kitchen_demo_bt::AutoDockAction>
{

public:
  AutoDock(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::AutoDockAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/auto_dock", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class Finish: public EusActionNode<kitchen_demo_bt::FinishAction>
{

public:
  Finish(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::FinishAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/finish", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class GetLightOn: public EusActionNode<kitchen_demo_bt::GetLightOnAction>
{

public:
  GetLightOn(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::GetLightOnAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/get_light_on", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class Init: public EusActionNode<kitchen_demo_bt::InitAction>
{

public:
  Init(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::InitAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/init", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class InspectSpot: public EusActionNode<kitchen_demo_bt::InspectSpotAction>
{

public:
  InspectSpot(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::InspectSpotAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/inspect_spot", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)"),
      InputPort<GoalType::_spot_name_type>("spot_name")
    };
  }

  bool sendGoal(GoalType& goal) override
  {
    BT::Result res;
    res = getInput("spot_name", goal.spot_name);
    if (!res) throw BT::RuntimeError(res.error());
    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class MoveToSpot: public EusActionNode<kitchen_demo_bt::MoveToSpotAction>
{

public:
  MoveToSpot(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::MoveToSpotAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/move_to_spot", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)"),
      InputPort<GoalType::_spot_name_type>("spot_name")
    };
  }

  bool sendGoal(GoalType& goal) override
  {
    BT::Result res;
    res = getInput("spot_name", goal.spot_name);
    if (!res) throw BT::RuntimeError(res.error());
    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class ReportAutoDockFailure: public EusActionNode<kitchen_demo_bt::ReportAutoDockFailureAction>
{

public:
  ReportAutoDockFailure(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::ReportAutoDockFailureAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/report_auto_dock_failure", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class ReportFailure: public EusActionNode<kitchen_demo_bt::ReportFailureAction>
{

public:
  ReportFailure(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::ReportFailureAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/report_failure", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)"),
      InputPort<GoalType::_spot_name_type>("spot_name")
    };
  }

  bool sendGoal(GoalType& goal) override
  {
    BT::Result res;
    res = getInput("spot_name", goal.spot_name);
    if (!res) throw BT::RuntimeError(res.error());
    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class ReportLightOn: public EusActionNode<kitchen_demo_bt::ReportLightOnAction>
{

public:
  ReportLightOn(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::ReportLightOnAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/report_light_on", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class ReportStartGoToKitchen: public EusActionNode<kitchen_demo_bt::ReportStartGoToKitchenAction>
{

public:
  ReportStartGoToKitchen(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::ReportStartGoToKitchenAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/report_start_go_to_kitchen", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class RoomLightOff: public EusActionNode<kitchen_demo_bt::RoomLightOffAction>
{

public:
  RoomLightOff(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::RoomLightOffAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/room_light_off", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class RoomLightOn: public EusActionNode<kitchen_demo_bt::RoomLightOnAction>
{

public:
  RoomLightOn(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<kitchen_demo_bt::RoomLightOnAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "kitchen_demo/room_light_on", "name of the Action Server"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {

    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {

    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kitchen_demo_engine");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::map<std::string, std::string> init_variables;
  // please comment in if you want to use command line argument
  // if (!roseus_bt::parse_command_line(argc, argv, "Run the kitchen_demo task.", init_variables)) return 1;
  if (!roseus_bt::parse_rosparam(pnh, init_variables)) return 1;

  BehaviorTreeFactory factory;

  RegisterRosAction<AutoDock>(factory, "AutoDock", nh);
  RegisterRosAction<Finish>(factory, "Finish", nh);
  RegisterRosAction<GetLightOn>(factory, "GetLightOn", nh);
  RegisterRosAction<Init>(factory, "Init", nh);
  RegisterRosAction<InspectSpot>(factory, "InspectSpot", nh);
  RegisterRosAction<MoveToSpot>(factory, "MoveToSpot", nh);
  RegisterRosAction<ReportAutoDockFailure>(factory, "ReportAutoDockFailure", nh);
  RegisterRosAction<ReportFailure>(factory, "ReportFailure", nh);
  RegisterRosAction<ReportLightOn>(factory, "ReportLightOn", nh);
  RegisterRosAction<ReportStartGoToKitchen>(factory, "ReportStartGoToKitchen", nh);
  RegisterRosAction<RoomLightOff>(factory, "RoomLightOff", nh);
  RegisterRosAction<RoomLightOn>(factory, "RoomLightOn", nh);

  auto tree = factory.maybeCreateLayeredTreeFromFile(fmt::format("{0}/models/kitchen_demo.xml", ros::package::getPath("kitchen_demo_bt")));
  roseus_bt::register_blackboard_variables(tree.get(), init_variables);

  std::string timestamp = std::to_string(ros::Time::now().toNSec());
  std::string log_filename(fmt::format("/home/knorth55/.ros/kitchen_demo_engine_{0}.fbl", timestamp));

  StdCoutLogger logger_cout(*tree);
  FileLogger logger_file(*tree, log_filename.c_str());
  PublisherZMQ publisher_zmq(*tree);

  NodeStatus status = NodeStatus::IDLE;

  std::cout << "Writing log to file: " << log_filename << std::endl;

  try {
    while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
      {
        ros::spinOnce();
        status = tree->tickRoot();
        ros::Duration sleep_time(0.005);
        sleep_time.sleep();
      }
  }
  catch(BT::RuntimeError& err) {
    std::cerr << "Behavior Tree execution terminated after throwing an instance of 'BT::RuntimeError'" << "\n  what():  " << err.what() << std::endl;
  }

  std::cout << "Writed log to file: " << log_filename << std::endl;
  std::cout << "Behavior Tree execution finished with " << toStr(status, true).c_str() << std::endl;
  return (status != NodeStatus::SUCCESS);
}
