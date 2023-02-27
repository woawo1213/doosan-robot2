/*
 * dsr_control_node2
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2020 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
 */

#include "rclcpp/rclcpp.hpp" //ROS2
#include <signal.h>

#include <memory>

#include "dsr_control2/dsr_hw_interface2.hpp"
#include <controller_manager/controller_manager.hpp>

#include <boost/thread/thread.hpp>

using namespace dsr_control2;

// rclcpp::Node::SharedPtr g_node = nullptr; 

int g_nKill_dsr_control2 = false;
void SigHandler(int sig) {
  // Do some custom action.
  // For example, publish a stop message to some other nodes.

  // All the default sigint handler does is call shutdown()
  RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),
              "shutdown time! (sig = %d)", sig);

  g_nKill_dsr_control2 = true;

  rclcpp::shutdown();
}

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe) {
  exe->spin();
}

int thread_robot_control(rclcpp::Node::SharedPtr control_node, int nPubRate) {
  auto pArm = std::make_shared<DRHWInterface>(control_node);

  if (pArm->init() != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("dsr_control_node2"),
                 "Error initializing robot");
    return -1;
  }

  std::shared_ptr<rclcpp::Executor> executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // start the controller manager with the robot hardware
  std::string manager_node_name = "controller_manager";
  controller_manager::ControllerManager cm(executor, manager_node_name);

  // load the joint state controller.
  // "ros_controllers" is the resource index from where to look for controllers
  // "ros_controllers::JointStateController" is the class we want to load
  // "my_robot_joint_state_controller" is the name for the node to spawn
#ifdef _OLD_ROS2_CONTROL_
  cm.load_controller("dsr_joint_publisher", //"my_robot_joint_state_controller",
                     "joint_state_controller/JointStateController");

  // load the trajectory controller
  /// cm.load_controller(
  ///    "dsr_joint_trajectory_controller",
  ///    //"my_robot_joint_trajectory_controller",
  ///    "joint_trajectory_controller/JointTrajectoryController");

  // there is no async spinner in ROS 2, so we have to put the spin() in its own
  // thread
  auto future_handle = std::async(std::launch::async, spin, executor);

  // we can either configure each controller individually through its services
  // or we use the controller manager to configure every loaded controller
  if (cm.configure() != controller_interface::return_type::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("dsr_control_node2"),
                 "at least one controller failed to configure");
    return -1;
  }
  // and activate all controller
  if (cm.activate() != controller_interface::return_type::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("dsr_control_node2"),
                 "at least one controller failed to activate");
    return -1;
  }
#else

  auto controller = cm.load_controller(
      "dsr_joint_publisher", //"my_robot_joint_state_controller",
      "joint_state_controller/JointStateController");

  // cm.push_back(controller);

  controller->configure();
  controller->activate();

#endif

  RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),
              "controller_manager is updating!");

  rclcpp::Rate r(nPubRate);

  while (rclcpp::ok()) {
    if (pArm)
      pArm->read();
    cm.update();
    if (pArm)
      pArm->write();

    /// RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),
    /// "thread_robot_control running...");
    r.sleep();
  }

  RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),
              "thread_robot_control Good-bye!");
  return 0;
}

int main(int argc, char **argv) {
  //----- init ROS2 ----------------------

  rclcpp::init(argc, argv);
  // g_node = rclcpp::Node::make_shared("dsr_control_node2"); // ??g_node 가왜필요하지? 이름?
  std::shared_ptr<rclcpp::Node> control_node =
      rclcpp::Node::make_shared("dsr_control_node2");
  // Logger
  /// const rclcpp::Logger logger = rclcpp::get_logger("dsr_control_node2");

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  /// for (int i = 0; i < argc; ++i)
  ///    RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),">>>>> argv[%d] =
  ///    %s",i,argv[i]);

  //----- SET PARA form launch file, if not set form config/default.yaml file
  //--------------------------
  control_node->declare_parameter("name", "dsr01");
  control_node->declare_parameter("rate", 100);
  control_node->declare_parameter("standby", 5000);
  control_node->declare_parameter("command", true);
  control_node->declare_parameter("host", "127.0.0.1");
  control_node->declare_parameter("port", 12345);
  control_node->declare_parameter("mode", "virtual");
  control_node->declare_parameter("model", "m1013");
  control_node->declare_parameter("gripper", "none");
  control_node->declare_parameter("mobile", "none");
  /////////////////////////////////////////////////////////////////////////////////////////////////////

  RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"), "control_node = 0x%p", control_node);

  signal(SIGINT, SigHandler);

  //----- get param --------------------- ?? dynamic param으로 안받는데 할 필요가 있나?
  int n_rate = 100; // hz
  control_node->get_parameter("rate", n_rate);

  RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"), "rate is %d", n_rate);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  boost::thread th_robot_ctl =
      boost::thread(boost::bind(&thread_robot_control, control_node, n_rate /*hz*/));
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"),
              "controller_manager is updating!");

  while (rclcpp::ok() && (false == g_nKill_dsr_control2)) {
    rclcpp::spin(control_node);
  }

  th_robot_ctl.join();

  RCLCPP_INFO(rclcpp::get_logger("dsr_control_node2"), "Good-bye!");

  return 0;
}