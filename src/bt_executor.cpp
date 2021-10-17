#include <behaviortree_ros/bt_service_node.h>
#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/controls/parallel_node.h"
#include <behaviortree_ros/GetPddl.h>
#include <behaviortree_ros/SendTree.h>
#include <behaviortree_ros/GetBT.h>
#include <behaviortree_ros/BTExecutorAction.h>
#include "bt_condition_node.h"
#include "bt_actions.h"


using namespace BT;
using namespace std;


//// ---- VARIABLE TO STORE  AND UPDATE TREE IS DEFINED AS GLOBAL VARIABLE HERE
string receivedTree;
//// ---- VARIABLE TO STORE  AND UPDATE TREE IS DEFINED AS GLOBAL VARIABLE HERE
// -----------------------------------------------------------------------------------------------------------------------

bool treeReceived = false;
class BehaviorTreeReceiver
{
private:

  ros::NodeHandle nh_;
  std::string sub_name_;
  std::string pub_name_;
  ros::Subscriber tree_sub_;
  ros::Publisher result_pub_;
  // ros::ServiceServer send_tree;
  ros::ServiceClient update_tree_client;
  ros::ServiceClient pddl_client;
  
  NodeStatus status;

public:
  bool updated;
  int  i;

  BehaviorTreeReceiver(ros::NodeHandle nodeHandle) :
    nh_(nodeHandle)
  {
    readParameters();
    // tree_sub_ = nh_.subscribe(sub_name_, 10, &BehaviorTreeReceiver::treeCallback, this);
    // result_pub_ = nh_.advertise<behaviortree_ros::ExecutionResult>(pub_name_, 1);
    // send_tree = nh_.advertiseService("/send_tree",&BehaviorTreeReceiver::get_tree_cb,this);
    update_tree_client = nh_.serviceClient<behaviortree_ros::GetBT>("/get_bt_service");
    pddl_client = nh_.serviceClient<behaviortree_ros::GetPddl>("/pddl_service");

    i = 0;
    ROS_INFO("initialized ...");
  }

  virtual ~BehaviorTreeReceiver()
  {
    nh_.shutdown();
  }

  bool readParameters()
  {
    nh_.param("sub_topic", sub_name_, std::string("behavior_tree"));
    nh_.param("sub_topic", pub_name_, std::string("bt_result"));
    return true;
  }

  // bool get_tree_cb(behaviortree_ros::SendTree::Request &req, behaviortree_ros::SendTree::Response &res)
  // {

  //   tree = req.tree;
  //   receivedTree = tree.c_str();
  //   // std::cout << " --------------------------" <<std::endl;
  //   // std::cout << receivedTree << endl;
  //   // std::cout << " --------------------------" <<std::endl;
  //   treeReceived = true;
  //   ROS_WARN("got new tree !!!");    
  //   return true;
  // }

  void getUpdateTree(std::string tree , NodeStatus status, string failedNode)
  {
    string updatedTree;
    behaviortree_ros::GetBT srv ;
    srv.request.type = "update";
    srv.request.bt = tree;
    srv.request.goal_condition = failedNode;
    update_tree_client.call(srv);
    updatedTree = srv.response.behavior_tree;
    
    if (!srv.response.success)
    {
      ROS_WARN("request replanning ...");
      updated = false;
      behaviortree_ros::GetPddl pddlSrv;
      pddlSrv.request.type = "replan";
      pddl_client.call(pddlSrv);
      ROS_INFO("1");
    }
    else {
      receivedTree = updatedTree.c_str();
      // std::cout << " ==========================" <<std::endl;
      // std::cout << receivedTree << endl;
      // std::cout << " ==========================" <<std::endl;
      treeReceived = true;
      updated = true;

    }

  }

  std::string tree;
};


std::string xml_text = R"(
 <root >
     <BehaviorTree>
         <Fallback>
          <IsPathValid service_name="gridmap/trajEntropy" pose="-0.0;-2.0;0.0" segment="{traj}" />
          <SupportReceiver server_name="support_receiver" type="check_path" path="{traj}" at="{location}" />
          </Fallback>
     </BehaviorTree>
 </root>
 )";

bool tree_cb(behaviortree_ros::SendTree::Request &req, behaviortree_ros::SendTree::Response &res)
  {
    ROS_WARN("for request test_cb ...");
    receivedTree = req.tree.c_str();
    // std::cout << " --------------------------" <<std::endl;
    // std::cout << receivedTree << endl;
    // std::cout << " --------------------------" <<std::endl;
    treeReceived = true;
    ROS_WARN("got new tree !!!");    
    return true;
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tree_executor");
  ros::NodeHandle nh, nh2;
  ros::ServiceServer service = nh2.advertiseService("/send_tree", tree_cb);
  BehaviorTreeReceiver receiver(nh);
  BehaviorTreeFactory factory;
  // factory.registerNodeType<PrintValue>("PrintValue");
  RegisterRosService<IsPathValid>(factory, "IsPathValid", nh);
  RegisterRosService<IsPathCollFree>(factory, "IsPathCollFree", nh);
  RegisterRosService<ExploreIPP>(factory, "ExploreIPP", nh);

  RegisterRosService<IsGraspedObj>(factory, "IsGraspedObj", nh);
  RegisterRosService<IsHandFree>(factory, "IsHandFree", nh);
  RegisterRosService<IsRobotCloseTo>(factory, "IsRobotCloseTo", nh);
  RegisterRosService<IsObjDetected>(factory, "IsObjDetected", nh);
  RegisterRosService<DetectObj>(factory, "DetectObj", nh);
  // RegisterRosAction<FibonacciServer>(factory, "Fibonacci", nh);
  factory.registerNodeType<PrintValue>("PrintValue");
  // factory.registerSimpleCondition("IsPathValid", std::bind(IsPathValid));
  RegisterRosAction<MoveBaseServer>(factory, "MoveBase", nh);

  BT::RegisterRosCondition<IsObjectAt>(factory,"IsObjectAt",nh);
  RegisterRosAction<PlaceProjected>(factory, "PlaceProjected", nh);
  RegisterRosAction<ApproachProjected>(factory, "ApproachProjected", nh);
  RegisterRosAction<PickProjected>(factory, "PickProjected", nh);
  RegisterRosAction<SupportReceiver>(factory, "SupportReceiver", nh);
  // RegisterRosAction<ExploreIPP>(factory, "ExploreIPP", nh);


  
  ros::Rate r(1);
  bool success = true;
  auto tree = factory.createTreeFromText(xml_text);

  
  time_t now = time(0);
  tm *ltm = localtime(&now);

  string filename1 = "BT_Log_";
  string filename2 = "BT_Log_";
  filename1.append(to_string(ltm->tm_mon));
  filename1.append("_");
  filename1.append(to_string(ltm->tm_mday));
  filename1.append("_");
  filename1.append(to_string(ltm->tm_hour));
  filename1.append("_");
  filename1.append(to_string(ltm->tm_min));
  filename1.append(".fbl");

  filename2.append(to_string(ltm->tm_mon));
  filename2.append("_");
  filename2.append(to_string(ltm->tm_mday));
  // filename2.append("_");
  // filename2.append(to_string(ltm->tm_hour));
  // filename2.append("_");
  // filename2.append(to_string(ltm->tm_min));
  // filename2.append(".json");

  FileLogger logger_file(tree, filename1.c_str());

  // MinitraceLogger logger_minitrace(tree, filename2.c_str());

  
  NodeStatus status = NodeStatus::IDLE;
  
  std::string failedN;
  int i = 0;
  // PublisherZMQ publisher_zmq(tree, 10, 1668, 1669);
  while( ros::ok() )
  {
    // ROS_WARN("looping ");
    // ROS_INFO("waiting ...");
    // while (status == NodeStatus::IDLE || status == NodeStatus::RUNNING)
    //   {
    //     // ROS_INFO("ticking ...");
    //     StdCoutLogger logger(tree);
    //     status = tree.tickRoot();
    //     r.sleep();

    //   }
    
    if (treeReceived)
    {
      ROS_WARN("got new tree?");
      status = NodeStatus::IDLE;
      auto tree1 = factory.createTreeFromText(receivedTree);
      printTreeRecursively(tree1.rootNode());
      filename2.append(to_string(ltm->tm_mon));
      string LogName = "bt_log_";
      LogName.append(to_string(ltm->tm_mon));
      LogName.append("-");
      LogName.append(to_string(ltm->tm_mday));
      LogName.append("-");
      LogName.append(to_string(ltm->tm_hour));
      LogName.append("_");
      LogName.append(to_string(ltm->tm_min));
      LogName.append(".fbl");
      FileLogger logger_file(tree1, LogName.c_str());
      // // PublisherZMQ publisher_zmq(tree1, 10, 1668, 1669);
      // PublisherZMQ publisher_zmq(tree1);
      // cout <<"before stat = " << status <<endl;
      while (status == NodeStatus::IDLE || status == NodeStatus::RUNNING)
      {
        
        // ROS_INFO("ticking ...");
        StdCoutLogger logger(tree1);
        status = tree1.tickRoot();
        // ros::Duration sleep_time(0.5);
        // sleep_time.sleep();
        r.sleep();
        // ros::Duration(15.0).sleep();
        // ros::spinOnce(); // this could be reason why the callbacks are not working ..?
        failedN =logger.failed_Node;
        // r.sleep();
      }
      treeReceived = false;
    }
    if (status == NodeStatus::FAILURE)
    {
      ROS_WARN("requesting tree update");
       receiver.getUpdateTree(receivedTree, status, failedN );
      //  ros::Duration sleep_time(0.1);
    }
    
    ros::spinOnce();
    r.sleep();
    // ROS_INFO("looping ...");
  }

  return 0;
}
