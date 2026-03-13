#include "stage_four/manager_node.hpp"

namespace manager_node_cpp
{

/* ManagerNode() //{ */
ManagerNode::ManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("manager_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.timer_manager", rclcpp::ParameterValue(1.0));
  declare_parameter("waypoints.points", std::vector<double>{1.0, 1.0, 1.0, 0.0});
  declare_parameter("waypoints.yawpoints", std::vector<double>{1.57, 3.14, -1.57, 0.0});
  declare_parameter("waypoints.speed", rclcpp::ParameterValue(0.3));
  declare_parameter("waypoints.z_offset", rclcpp::ParameterValue(3.0));
  declare_parameter("waypoints.stop_on_waypoints", rclcpp::ParameterValue(true));
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
  get_parameter("waypoints.speed", _speed_);
  get_parameter("waypoints.z_offset", _z_offset_);
  get_parameter("waypoints.stop_on_waypoints", _stop_on_waypoints_);

  waypoints_qty_points_ = _waypoints_points_.size() / 4;


  trajectory_path_.speed             = _speed_;
  trajectory_path_.stop_on_waypoints = _stop_on_waypoints_;

  for (int i = 0; i < waypoints_qty_points_; i++) {
    laser_msgs::msg::PoseWithHeading trajectory_point;

    trajectory_point.position.x = _waypoints_points_[trajectory_count_];
    trajectory_point.position.y = _waypoints_points_[trajectory_count_ + 1];
    trajectory_point.position.z = (_waypoints_points_[trajectory_count_ + 2] + _z_offset_);
    trajectory_point.heading    = _waypoints_points_[trajectory_count_ + 3];

    std::cout << trajectory_point.position.x << std::endl;
    std::cout << trajectory_point.position.y << std::endl;
    std::cout << trajectory_point.position.z << std::endl;
    std::cout << trajectory_point.heading << std::endl;

    trajectory_path_.waypoints.push_back(trajectory_point);

    trajectory_count_ += 4;
  }
}
//}

/* ConfigPubSub() //{ */
void ManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  sub_diagnostics_ = create_subscription<laser_msgs::msg::UavControlDiagnostics>(
      "diagnostics_in", 1, std::bind(&ManagerNode::subControlManagerDiagnostics, this, std::placeholders::_1));
  sub_qrcode_reader_ = create_subscription<laser_msgs::msg::PointWithStringArrayStamped>("qrcode_detection_in", 1,
                                                                                         std::bind(&ManagerNode::subQrcodeReader, this, std::placeholders::_1));

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

  pub_trajectory_->publish(trajectory_path_);

  is_active_ = true;
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
//}

/* Land() //{ */
void ManagerNode::land() {
  auto request         = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
    RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
  };

  clt_land_->async_send_request(request, callback_result);
}
//}

/* tmrManager() //{ */
void ManagerNode::tmrManager() {
  if (!is_active_) {
    return;
  }

  if (!diagnostics_.have_goal && diagnostics_.is_fly && !land_request_) {
    land();
    land_request_ = true;
  }
}
//}

/* SubControlManagerDiagnostics() //{ */
void ManagerNode::subControlManagerDiagnostics(const laser_msgs::msg::UavControlDiagnostics &msg) {
  diagnostics_ = msg;
}
//}

/* SubQrcodeReader() //{ */
void ManagerNode::subQrcodeReader(const laser_msgs::msg::PointWithStringArrayStamped &qrcode_reader) {

  if (!qr_codes_.empty()) {
    for (int i = 0; i < qr_codes_.size(); i++) {
      std::cout << qr_codes_[i] << std::endl;
    }
  }

  if (qrcode_reader.array.empty()) {
    return;
  }

  const std::lock_guard<std::mutex> lock(qrcode_mtx_);

  std::string data       = qrcode_reader.array[0].data;
  bool        new_qrcode = true;

  if (!first_qrcode_) {
    for (int i = 0; i < qr_codes_.size(); i++) {
      if (data == qr_codes_[i]) {
        new_qrcode = false;
      }
    }
  } else {
    qr_codes_.push_back(data);
    first_qrcode_ = false;
  }

  if (new_qrcode)
    qr_codes_.push_back(data);
}
//}

}  // namespace manager_node_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(manager_node_cpp::ManagerNode)
