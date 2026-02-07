#include "stage_four/manager_node.hpp"

namespace manager_node_cpp
{

/* ManagerNode() //{ */
ManagerNode::ManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("manager_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.timer_manager", rclcpp::ParameterValue(1.0));
  declare_parameter("waypoints.points", std::vector<double>{1.0, 1.0, 1.0, 0.0});
  declare_parameter("waypoints.yawpoints", std::vector<double>{1.57, 3.14, -1.57, 0.0});
}
//}


/* ~ManagerNode() //{ */
ManagerNode::~ManagerNode() {
}
//}

/* OnConfigure() //{ */
CallbackReturn ManagerNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();
  configClients();

  return CallbackReturn::SUCCESS;
}
//}

/* OnActivate() //{ */
CallbackReturn ManagerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_trajectory_->on_activate();

  return CallbackReturn::SUCCESS;
}
//}

/* OnDeactivate() //{ */
CallbackReturn ManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_trajectory_->on_deactivate();

  return CallbackReturn::SUCCESS;
}
//}

/* OnClanup() //{ */
CallbackReturn ManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return CallbackReturn::SUCCESS;
}
//}

/* OnShutdown() //{ */
CallbackReturn ManagerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}
//}

/* GetParameters() //{ */
void ManagerNode::getParameters() {
  RCLCPP_INFO(get_logger(), "Loading parameters");

  get_parameter("rate.timer_manager", _rate_tmr_manager_);
  get_parameter("waypoints.points", _waypoints_points_);

  waypoints_qty_points_ = _waypoints_points_.size() / 4;
}
//}

/* ConfigPubSub() //{ */
void ManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  sub_diagnostics_ = create_subscription<laser_msgs::msg::UavControlDiagnostics>(
      "diagnostics_in", 1, std::bind(&ManagerNode::subControlManagerDiagnostics, this, std::placeholders::_1));
  sub_qrcode_reader_ =
      create_subscription<laser_msgs::msg::PointWithString>("qrcode_detection_in", 1, std::bind(&ManagerNode::subQrcodeReader, this, std::placeholders::_1));

  pub_trajectory_ = create_publisher<laser_msgs::msg::TrajectoryPath>("trajectory_path_out", 1);
}
//}

/* ConfigTimers() //{ */
void ManagerNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");
  
  tmr_manager_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_tmr_manager_), std::bind(&ManagerNode::tmrManager, this), nullptr);
}
//}

/* ConfigClients() //{ */
void ManagerNode::configClients() {
  RCLCPP_INFO(get_logger(), "initClients");

  clt_land_    = create_client<std_srvs::srv::Trigger>("land");
  clt_takeoff_ = create_client<std_srvs::srv::Trigger>("takeoff");
}
//}

/* ConfigServices() //{ */
void ManagerNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  srv_start_state_machine_ = this->create_service<std_srvs::srv::Trigger>(
      "start_state_machine", std::bind(&ManagerNode::stateTriggerRequest, this, std::placeholders::_1, std::placeholders::_2));
}
//}

/* StateMachine() //{ */
void ManagerNode::stateTriggerRequest([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response>                       response) {
  response->success = true;
  response->message = "State Machine started";

  takeoff();
}
//}

/* TakeOff() //{ */
void ManagerNode::takeoff() {
  auto request         = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
    RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
  };

  clt_takeoff_->async_send_request(request, callback_result);
}

/* Land() //{ */
void ManagerNode::land() {
  auto request         = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
    RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
  };

  clt_land_->async_send_request(request, callback_result);
}

/* tmrManager() //{ */
void ManagerNode::tmrManager() {

}
//}


/* SubControlManagerDiagnostics() //{ */
void ManagerNode::subControlManagerDiagnostics(const laser_msgs::msg::UavControlDiagnostics &msg) {
  diagnostics_ = msg;
}
//}

/* SubQrcodeReader() //{ */
void ManagerNode::subQrcodeReader(laser_msgs::msg::PointWithString qrcode_reader) {
}
//}

}  // namespace manager_node_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(manager_node_cpp::ManagerNode)
