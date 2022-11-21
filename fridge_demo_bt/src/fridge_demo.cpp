#include <ros/ros.h>

#define DEBUG  // rosbridgecpp logging
#include <roseus_bt/eus_nodes.h>
#include <roseus_bt/command_line_argument_mapping.h>
#include <roseus_bt/rosparam_argument_mapping.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <fridge_demo_bt/CloseFridgeAction.h>
#include <fridge_demo_bt/FinishAction.h>
#include <fridge_demo_bt/FinishFailureAction.h>
#include <fridge_demo_bt/GraspCanAction.h>
#include <fridge_demo_bt/HandOverCanAction.h>
#include <fridge_demo_bt/InitAction.h>
#include <fridge_demo_bt/MoveToFridgeAction.h>
#include <fridge_demo_bt/MoveToInitialPositionAction.h>
#include <fridge_demo_bt/OpenFridgeAction.h>

using namespace BT;


class CloseFridge: public EusActionNode<fridge_demo_bt::CloseFridgeAction>
{

public:
  CloseFridge(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::CloseFridgeAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/close_fridge", "name of the Action Server"),
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


class Finish: public EusActionNode<fridge_demo_bt::FinishAction>
{

public:
  Finish(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::FinishAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/finish", "name of the Action Server"),
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


class FinishFailure: public EusActionNode<fridge_demo_bt::FinishFailureAction>
{

public:
  FinishFailure(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::FinishFailureAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/finish_failure", "name of the Action Server"),
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


class GraspCan: public EusActionNode<fridge_demo_bt::GraspCanAction>
{

public:
  GraspCan(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::GraspCanAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/grasp_can", "name of the Action Server"),
      InputPort<GoalType::_can_name_type>("can_name"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {
    BT::Result res;
    res = getInput("can_name", goal.can_name);
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


class HandOverCan: public EusActionNode<fridge_demo_bt::HandOverCanAction>
{

public:
  HandOverCan(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::HandOverCanAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/hand_over_can", "name of the Action Server"),
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


class Init: public EusActionNode<fridge_demo_bt::InitAction>
{

public:
  Init(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::InitAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/init", "name of the Action Server"),
      InputPort<GoalType::_can_name_type>("can_name"),
      OutputPort<FeedbackType::_can_name_type>("can_name"),
      InputPort<unsigned>("timeout", 60000, "timeout to connect (milliseconds)")
    };
  }

  bool sendGoal(GoalType& goal) override
  {
    BT::Result res;
    res = getInput("can_name", goal.can_name);
    if (!res) throw BT::RuntimeError(res.error());
    return true;
  }

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {
    if (feedback->update_field_name == "can_name") setOutput("can_name", feedback->can_name);
    return;
  }

  NodeStatus onResult(const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

};


class MoveToFridge: public EusActionNode<fridge_demo_bt::MoveToFridgeAction>
{

public:
  MoveToFridge(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::MoveToFridgeAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/move_to_fridge", "name of the Action Server"),
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


class MoveToInitialPosition: public EusActionNode<fridge_demo_bt::MoveToInitialPositionAction>
{

public:
  MoveToInitialPosition(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::MoveToInitialPositionAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/move_to_initial_position", "name of the Action Server"),
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


class OpenFridge: public EusActionNode<fridge_demo_bt::OpenFridgeAction>
{

public:
  OpenFridge(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<fridge_demo_bt::OpenFridgeAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "fridge_demo/open_frirdge", "name of the Action Server"),
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
  ros::init(argc, argv, "fridge_demo_engine");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::map<std::string, std::string> init_variables;
  // please comment in if you want to use command line argument
  // if (!roseus_bt::parse_command_line(argc, argv, "Run the fridge_demo task.", init_variables)) return 1;
  if (!roseus_bt::parse_rosparam(pnh, init_variables)) return 1;

  BehaviorTreeFactory factory;

  RegisterRosAction<CloseFridge>(factory, "CloseFridge", nh);
  RegisterRosAction<Finish>(factory, "Finish", nh);
  RegisterRosAction<FinishFailure>(factory, "FinishFailure", nh);
  RegisterRosAction<GraspCan>(factory, "GraspCan", nh);
  RegisterRosAction<HandOverCan>(factory, "HandOverCan", nh);
  RegisterRosAction<Init>(factory, "Init", nh);
  RegisterRosAction<MoveToFridge>(factory, "MoveToFridge", nh);
  RegisterRosAction<MoveToInitialPosition>(factory, "MoveToInitialPosition", nh);
  RegisterRosAction<OpenFridge>(factory, "OpenFridge", nh);

  auto tree = factory.maybeCreateLayeredTreeFromFile("/home/knorth55/ros/guiga_ws/src/jsk-ros-pkg/jsk_demos/fridge_demo_bt/models/fridge_demo.xml");
  roseus_bt::register_blackboard_variables(tree.get(), init_variables);

  std::string timestamp = std::to_string(ros::Time::now().toNSec());
  std::string log_filename(fmt::format("/home/knorth55/.ros/fridge_demo_engine_{0}.fbl", timestamp));

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
