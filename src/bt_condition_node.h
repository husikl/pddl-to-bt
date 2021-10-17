// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #ifndef BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
// #define BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_ros/bt_conversions.h>
#include <behaviortree_ros/CollCheck.h>
#include <behaviortree_ros/TrajEntropy.h>
#include <behaviortree_ros/SelfExplore.h>
#include <behaviortree_ros/RobotToPose.h>
#include <behaviortree_ros/GripperState.h>
#include <behaviortree_ros/DetectObj.h>
// #include "traj_entropy.h"
#include <nav_msgs/GetPlan.h>




namespace BT
{

// ------------------------------------------------------------------------------------------------------------
class IsGraspedObj : public RosServiceNode<behaviortree_ros::GripperState> 
{
  public:
  IsGraspedObj( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::GripperState>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("id")};
  }
  void sendRequest(RequestType& request) override
  {

    getInput("id", request.state);
    ROS_INFO("IsGraspedObj: sending request");

  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("IsGraspedObj: response received");
    if( rep.grasping )
    {

      return NodeStatus::SUCCESS;
    }
    else{

      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("IsGraspedObj request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  
};

// *--------------------------------------------------**************************************************************

class IsHandFree : public RosServiceNode<behaviortree_ros::GripperState> 
{
  public:
  IsHandFree( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::GripperState>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("id")};
  }
  void sendRequest(RequestType& request) override
  {

    getInput("id", request.state);
    ROS_INFO("IsHandFree: sending request");

  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("IsHandFree: response received");
    if( !rep.grasping )
    {

      return NodeStatus::SUCCESS;
    }
    else{

      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("IsHandFree request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  
};

// ------------------------------------------------------------------------------------------------------------

class IsRobotCloseTo : public RosServiceNode<behaviortree_ros::RobotToPose> 
{
  public:
  IsRobotCloseTo( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::RobotToPose>(handle, node_name, conf)
    {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("location"),
      BT::InputPort<geometry_msgs::Point>("pose")};
  }
  void sendRequest(RequestType& request) override
  {

    getInput("pose", request.targetPose.position);
    ROS_INFO("IsRobotCloseTo: sending request");

  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("IsRobotCloseTo: response received");
    if( rep.distanceToTargePose < 0.65 )
    {

      return NodeStatus::SUCCESS;
    }
    else{

      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("IsRobotCloseTo request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  
};

// ------------------------------------------------------------------------------------------------------------

class IsObjDetected : public RosServiceNode<behaviortree_ros::DetectObj> 
{
  public:
  IsObjDetected( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::DetectObj>(handle, node_name, conf)
    {}

  geometry_msgs::Point loc;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("id"),
      BT::InputPort<std::string>("on"),
      BT::InputPort<bool>("detected"),
      BT::OutputPort<geometry_msgs::Point>("objPose")};
  }
  void sendRequest(RequestType& request) override
  {
    getInput("id", request.name);
    getInput("on", request.locationName);
    loc = request.pose;
    // getInput("at", request.pose);
    request.reqType = "condition";
    // ROS_INFO("IsObjDetected: sending request");

  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    // getInput("pose", loc);
    // std::cout << "loc = " << loc << std::endl;
    // ros::Duration(10.0).sleep();
    bool detected;
    ROS_INFO("IsObjDetected: response received");
    getInput("detected", detected);
    if( detected || rep.detected )
    {
      setOutput("objPose", rep.location);
      return NodeStatus::SUCCESS;
    }
    else{
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("IsObjDetected request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  
};


// ------------------------------------------------------------------------------------------------------------

class DetectObj : public RosServiceNode<behaviortree_ros::DetectObj> 
{
  public:
  DetectObj( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::DetectObj>(handle, node_name, conf)
    {}

  geometry_msgs::Point loc;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("id"),
      BT::InputPort<std::string>("on"),
      BT::OutputPort<bool>("detected"),
      BT::OutputPort<geometry_msgs::Point>("atLoc")};
  }
  void sendRequest(RequestType& request) override
  {
    getInput("id", request.name);
    getInput("on", request.locationName);
    // ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    // std::cout << request.pose << std::endl;
    // ros::Duration(10.0).sleep();
    loc = request.pose;
    request.reqType = "action";
    ROS_INFO("DetectObj: sending request");
    // std::cout << request << std::endl;
    return;
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("DetectObj: response received");
    setOutput<bool>("output",rep.detected);
    setOutput<geometry_msgs::Point>("atLoc",loc);
    if( rep.detected )
    {
      return NodeStatus::SUCCESS;
    }
    else{
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("DetectObj request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  
};

// ------------------------------------------------------------------------------------------------------------

class RosConditionNode : public BT::ConditionNode
{
protected:

  RosConditionNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
   BT::ConditionNode(name, conf), node_(nh) { }

public:

  // using BaseClass    = RosConditionNode<ConditionT>;
  // using ConditionType  = ConditionT;

  RosConditionNode() = delete;

  virtual ~RosConditionNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<unsigned>("timeout", 100, "timeout to connect to server (milliseconds)")
      };
  }


protected:

  // The node that will be used for any ROS operations
  virtual NodeStatus tick() override;
  ros::NodeHandle& node_;

};

// ------------------------------------------------------------------------------------------------------------

class IsObjectAt : public BT::ConditionNode 
{
  public:
  IsObjectAt(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
  BT::ConditionNode(name, conf), node_(nh)
    {
      
      pose_sub = node_.subscribe("/youbot/pose", 1, &IsObjectAt::objectsCB, this);
    }    
    IsObjectAt() = delete;

    BT::NodeStatus tick() override {
      
      dist = sqrt( (r_x - o_x)*(r_x - o_x) - (r_y - o_y)*(r_y - o_y) );
      // ROS_INFO("[ dist = %f ]", dist);
      // if (dist < 0.75)
      //   return BT::NodeStatus::SUCCESS;
      
      return BT::NodeStatus::FAILURE;
    }
  void objectsCB(const geometry_msgs::PoseStamped& msg)
  {
    // robot_pose_ = msg;
    // r_x = msg.pose.position.x;
    // r_y = msg.pose.position.y;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("id"),
      BT::InputPort<geometry_msgs::Point>("object_pose"),
      BT::InputPort<std::string>("location"),
      BT::InputPort<geometry_msgs::Point>("location_pose"),
    };
  }

  private:
    
    geometry_msgs::PoseStamped robot_pose_;
    double o_x, o_y, r_x, r_y, dist;
    // double distance_;
    std::string object_name;
    // std::string robot_base_frame_;
    ros::NodeHandle& node_;
    ros::Subscriber pose_sub;
};

// ------------------------------------------------------------------------------------------------------------

class IsPathValid : public RosServiceNode<behaviortree_ros::TrajEntropy> 
{
  public:
  IsPathValid( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::TrajEntropy>(handle, node_name, conf)
    {}
    //   client = handle.serviceClient<nav_msgs::GetPlan>("/move_base/get_plan");
    // }


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::Point>("pose"),
      BT::OutputPort<nav_msgs::Path>("segment")
    };
  }
  void sendRequest(RequestType& request) override
  {
    // ROS_INFO("[ Sending request ... ]");
    ros::Duration(0.1).sleep();
    getInput("pose", request.targetPose);
    // ROS_INFO("IsPathValid: sending request");

  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("IsPathValid: response received");
    if( rep.valid )
    {
      setOutput<double>("th_value", rep.th_value);
      // setOutput<nav_msgs::Path>("segment", rep.unknownTrajSegment);
      return NodeStatus::SUCCESS;
    }
    else{
      setOutput<double>("th_value", rep.th_value);
      setOutput<nav_msgs::Path>("segment", rep.unknownTrajSegment);
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("IsPathValid request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  private:
    
    // geometry_msgs::msg::PoseStamped start_pose_;

    // double distance_;
    // std::string global_frame_;
    // std::string robot_base_frame_;
    // ros::NodeHandle& node_;
    nav_msgs::GetPlan req;
    ros::ServiceClient client;
};

// --------------------------------------------------------------------------------------------------------------------------

class IsPathCollFree: public RosServiceNode<behaviortree_ros::CollCheck>
{

public:
  IsPathCollFree( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::CollCheck>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<geometry_msgs::Point>("pose"),
      OutputPort<bool>("in_collision") };
  }

  void sendRequest(RequestType& request) override
  {
    ros::Duration(0.1).sleep();
    getInput("pose", request.target);
    ROS_INFO("IsPathCollFree: sending request");
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("IsPathCollFree: response received");
    if( rep.in_collision )
    {
      setOutput<int>("in_collision", rep.in_collision);
      return NodeStatus::FAILURE;
    }
    else{
      setOutput<int>("in_collision", rep.in_collision);
      return NodeStatus::SUCCESS;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("IsPathCollFree request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

private:
  int expected_result_;
};

class ExploreIPP: public RosServiceNode<behaviortree_ros::SelfExplore>
{

public:
  ExploreIPP( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
  RosServiceNode<behaviortree_ros::SelfExplore>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<nav_msgs::Path>("trajectory"),
      OutputPort<bool>("valid") };
  }

  void sendRequest(RequestType& request) override
  {
    ros::Duration(0.1).sleep();
    getInput("trajectory", request.trajectory);
    ROS_INFO("ExploreIPP: sending request");
  }

  NodeStatus onResponse(const ResponseType& rep) override
  {
    ROS_INFO("ExploreIPP: response received");
    if( rep.valid )
    {
      ROS_INFO("not valid");
      setOutput<bool>("valid", rep.valid);
      return NodeStatus::SUCCESS;
    }
    else{
      ROS_INFO("valid");
      setOutput<bool>("valid", rep.valid);
      return NodeStatus::FAILURE;
    }
  }

  virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
  {
    ROS_ERROR("ExploreIPP request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

private:
  int expected_result_;
};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosCondition(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosConditionNode::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

// #endif  // BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
