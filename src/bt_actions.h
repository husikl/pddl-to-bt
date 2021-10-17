#include <actionlib/server/simple_action_server.h>
#include <behaviortree_ros/AddTwoInts.h>
#include <behaviortree_ros/FibonacciAction.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <behaviortree_ros/placeAction.h>
#include <behaviortree_ros/pickAction.h>
#include <behaviortree_ros/approachAction.h>
#include <behaviortree_ros/ExploreAction.h>
#include <behaviortree_ros/ExecutionResult.h>
#include <behaviortree_ros/VisualAssistanceAction.h>
#include <std_msgs/String.h>
#include <ctime>
#include <cstdio>
#include <typeinfo>

using namespace BT;
using namespace std;

class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    int value = 0;
    if( getInput("message", value ) ){
      std::cout << "PrintValue: " << value << std::endl;
      
    }
    else{
      std::cout << "PrintValue FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
    return NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts() {
    return{ BT::InputPort<int>("message") };
  }
};
//// ----
class PlaceProjected: public RosActionNode<behaviortree_ros::placeAction>
{

public:
  PlaceProjected( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosActionNode<behaviortree_ros::placeAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("id"),
      InputPort<std::string>("location"),
      InputPort<geometry_msgs::Point>("at")
      };
  }

  bool sendGoal(GoalType& goal) override
  {
    // std::cout << " goal = " << goal << std::endl;
    if ( !getInput("id", goal.command) || !getInput("location", goal.command) ) 
    {
      ROS_INFO("input wrong  PLACE PROJECTED?");
      
      // abourt the entire action. Result in a FAILURE
      return false;
    }

    // expected_result_ = 0 + 1 + 1 + 2 + 3 + 5 + 8; // supposing order is 5
    getInput("id", goal.id);
    // std::cout << "goal id = " << goal.id << std::endl;
    ROS_INFO("PlaceProjected: sending goal");
    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("PlaceProjected: result received");
    // setOutput<geometry_msgs::Point>("output",goalPose);
    if (res.result == "ok")
    return NodeStatus::SUCCESS;
    else
    return NodeStatus::FAILURE;


  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("PlaceProjected request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("PlaceProjected halted");
      BaseClass::halt();
    }
  }

private:
  int expected_result_;
};

class ApproachProjected: public RosActionNode<behaviortree_ros::approachAction>
{

public:
  ApproachProjected( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosActionNode<behaviortree_ros::approachAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("id"),
      InputPort<geometry_msgs::Point>("pose")
    //   ,
    //   OutputPort<geometry_msgs::Point>("output")
      };
  }

  bool sendGoal(GoalType& goal) override
  {
    // std::cout << goal << std::endl;
    if (!getInput("id", goal.id) || !getInput("pose", goal.pose)) 
    {
    //   ROS_INFO("input wrong ?");
      
      // abourt the entire action. Result in a FAILURE
      return false;
    }

    // expected_result_ = 0 + 1 + 1 + 2 + 3 + 5 + 8; // supposing order is 5
    // ROS_INFO("approachAction: sending goal");
    // getInput("pose",goalPose);
    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("approachAction: result receive");
    setOutput<geometry_msgs::Point>("output",goalPose);
    if (res.result == "ok")
    return NodeStatus::SUCCESS;
    else
    return NodeStatus::FAILURE;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("approachAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("approachAction halted");
      BaseClass::halt();
    }
  }

private:
  int expected_result_;
  geometry_msgs::Point goalPose;
};

// ----------------------------------------------------------------------------------------------------------------------------------


class MoveBaseServer: public RosActionNode<move_base_msgs::MoveBaseAction>
{

public:
  MoveBaseServer( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosActionNode<move_base_msgs::MoveBaseAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      // InputPort<geometry_msgs::PoseStamped>("goal")};
      InputPort<double>("x"),InputPort<double>("y"), InputPort<std::string>("frame")};
  }

  bool sendGoal(GoalType& goal) override
  {
    if (!getInput("x", goal.target_pose.pose.position.x) && !getInput("y", goal.target_pose.pose.position.y)
    && !getInput("frame", goal.target_pose.header.frame_id)) 
    {
      ROS_INFO("input wrong ?");
      std::cout << goal << std::endl;
      // abourt the entire action. Result in a FAILURE
      return false;
    }
    getInput("x", goal.target_pose.pose.position.x);
    getInput("y", goal.target_pose.pose.position.y);
    getInput("frame", goal.target_pose.header.frame_id);
    // goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.orientation.w = 1.0;
    
    // expected_result_ = 0 + 1 + 1 + 2 + 3 + 5 + 8; // supposing order is 5+
    // std::cout << goal << std::endl;
    ROS_INFO("MoveBaseAction: sending goal");
    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("MoveBaseAction: result receive");

    return NodeStatus::SUCCESS;

  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("MoveBaseAction request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("MoveBaseAction halted");
      BaseClass::halt();
    }
  }

private:
  int expected_result_;
};

//-----------------------------------------------------

  // Simple tree, used to execute once each action.

class PickProjected: public RosActionNode<behaviortree_ros::pickAction>
{

public:
  PickProjected( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosActionNode<behaviortree_ros::pickAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("id"),
      InputPort<std::string>("on"),
      InputPort<geometry_msgs::Point>("pose")
    //   ,
    //   OutputPort<std::string>("output")
      };
  }

  bool sendGoal(GoalType& goal) override
  {
    if (!getInput("id", goal.id) || !getInput("on", goal.id) || !getInput("pose", goal.pose) ) 
    {
      ROS_INFO("input wrong ?");
      
      // abourt the entire action. Result in a FAILURE
      return false;
    }
    getInput("pose",goal.pose);
    getInput("id",goal.id);

    ROS_INFO("PickProjected: sending goal");
    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("PickProjected: result receive");
    // setOutput<std::string>("output",id);
    if (res.result == "ok")
        return NodeStatus::SUCCESS;
    else 
      return NodeStatus::FAILURE;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("PickProjected request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("PickProjected halted");
      BaseClass::halt();
    }
  }

private:
  int expected_result_;
  std::string id;
};

// -----------------------------------------------------------------------------------------------------------------------

class SupportReceiver: public RosActionNode<behaviortree_ros::VisualAssistanceAction>
{

public:
  SupportReceiver( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosActionNode<behaviortree_ros::VisualAssistanceAction>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<string>("type"),
      InputPort<geometry_msgs::Point>("at"),
      InputPort<nav_msgs::Path>("path")};
  }

  bool sendGoal(GoalType& goal) override
  {
    if (!getInput("type", goal.type) ) 
    {
      ROS_INFO("type of assist not specified ?");
      
      
      // abourt the entire action. Result in a FAILURE
      return false;
    }
    if (goal.type=="check_path")
      {
        getInput("path", goal.trajectory);
      }
    else if (goal.type=="object")
      {
        getInput("at", goal.location);
      }

    ROS_INFO("SupportReceiver: sending goal");
    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {
    ROS_INFO("SupportReceiver: result received");

    return NodeStatus::SUCCESS;

  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("SupportReceiver request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("SupportReceiver halted");
      BaseClass::halt();
    }
  }

private:
  int expected_result_;
  
};