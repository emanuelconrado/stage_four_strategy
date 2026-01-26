#include "stage_four/manager_node.hpp"

namespace manager_node_cpp
{
ManagerNode::ManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("manager_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.state_machine", rclcpp::ParameterValue(1.0));
  declare_parameter("waypoints.points", std::vector<double>{1.0, 1.0, 1.0});
  declare_parameter("waypoints.yawpoints", std::vector<double>{1.57, 3.14, -1.57, 0.0});
}

ManagerNode::~ManagerNode() {
}

CallbackReturn ManagerNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();
  configClients();

  yawpoints_   = {1.57, 3.14, -1.57, 0.0};
  yaw_control_ = false;

  return CallbackReturn::SUCCESS;
}


CallbackReturn ManagerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_have_goal_->on_activate();
  pub_goto_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_have_goal_->on_deactivate();
  pub_goto_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}

void ManagerNode::getParameters() {
  RCLCPP_INFO(get_logger(), "Loading parameters");

  get_parameter("rate.state_machine", _rate_state_machine_);
  get_parameter("waypoints.points", _waypoints_points_);

  waypoints_qty_points_ = _waypoints_points_.size() / 4;
}

void ManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");

  sub_have_goal_ = create_subscription<std_msgs::msg::Bool>("/uav1/have_goal", 1, std::bind(&ManagerNode::subOpHaveGoal, this, std::placeholders::_1));
  sub_qrcode_reader_ =
      create_subscription<laser_msgs::msg::PointWithString>("/uav1/qrcode/detections", 1, std::bind(&ManagerNode::subQrcodeReader, this, std::placeholders::_1));

  pub_goto_      = create_publisher<laser_msgs::msg::PoseWithHeading>("uav1/control_manager/goto", 1);
  pub_have_goal_ = create_publisher<std_msgs::msg::Bool>("uav1/have_goal", 1);
}

void ManagerNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");
}

void ManagerNode::configClients() {
  RCLCPP_INFO(get_logger(), "initClients");

  clt_arm_     = create_client<std_srvs::srv::Trigger>("/uav1/px4_api/arm");
  clt_land_    = create_client<std_srvs::srv::Trigger>("/uav1/control_manager/land");
  clt_takeoff_ = create_client<std_srvs::srv::Trigger>("/uav1/control_manager/takeoff");
  clt_disarm_  = create_client<std_srvs::srv::Trigger>("/uav1/px4_api/disarm");
}

void ManagerNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  srv_start_state_machine_ = this->create_service<std_srvs::srv::Trigger>(
      "start_state_machine", std::bind(&ManagerNode::stateTriggerRequest, this, std::placeholders::_1, std::placeholders::_2));
}

void ManagerNode::stateTriggerRequest([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response>                       response) {
  response->success = true;
  response->message = "State Machine started";

  takeoff();
}

void ManagerNode::takeoff() {
  auto request         = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
    RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
  };

  clt_takeoff_->async_send_request(request, callback_result);
  tmr_goingto_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_state_machine_), std::bind(&ManagerNode::goingTo, this), nullptr);
}

void ManagerNode::goingTo() {
  if (!have_goal_) {
    if (waypoints_qty_points_ == 0) {
      auto request         = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
        RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
      };
    }

    if (waypoints_qty_points_ > 0) {
      if (yaw_control_ && waypoints_qty_points_ > 1 && start_point_ != 0 && qrcode_vector_.size() < 5) {

        if (goto_pos_.position.z > 0.0) {

          QrcodeState();
          pub_goto_->publish(goto_pos_);
          if (yaw_point_ == 4) {
            yaw_control_ = false;
          }
        } else {
          yaw_control_ = false;
        }
      } else {
        getNextPose();
        pub_goto_->publish(goto_pos_);
        yaw_point_   = 0;
        yaw_control_ = true;
      }
    }
  }
}

void ManagerNode::subOpHaveGoal(std_msgs::msg::Bool have_goal) {
  have_goal_ = have_goal.data;
}

void ManagerNode::subQrcodeReader(laser_msgs::msg::PointWithString qrcode_reader) {
  for (auto it : qrcode_vector_) {
    if (it.data == qrcode_reader.data) {
      return;
    }
  }
  qrcode_vector_.push_back(qrcode_reader);
}

void ManagerNode::getNextPose() {

  start_point_         = _waypoints_points_.size() - (waypoints_qty_points_ * 4);
  goto_pos_.position.x = _waypoints_points_[start_point_];
  goto_pos_.position.y = _waypoints_points_[start_point_ + 1];
  goto_pos_.position.z = _waypoints_points_[start_point_ + 2];

  waypoints_qty_points_--;
}

void ManagerNode::QrcodeState() {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yawpoints_[yaw_point_]);

  yaw_point_++;
}

}  // namespace manager_node_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(manager_node_cpp::ManagerNode)
