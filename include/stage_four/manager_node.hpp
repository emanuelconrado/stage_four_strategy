#ifndef MANAGER_NODE_CPP__MANAGER_NODE_HPP
#define MANAGER_NODE_CPP__MANAGER_NODE_HPP

#include <memory>
#include <mutex>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <laser_msgs/msg/point_with_string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <laser_msgs/msg/pose_with_heading.hpp>
#include <laser_msgs/msg/trajectory_path.hpp>
#include <laser_msgs/msg/uav_control_diagnostics.hpp>
#include <laser_msgs/msg/point_with_string_array_stamped.hpp>
using namespace std::chrono;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace manager_node_cpp
{
class ManagerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ManagerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~ManagerNode() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();
  void configClients();

  // func
  void stateTriggerRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void takeoff();
  void land();

  // Timers
  rclcpp::TimerBase::SharedPtr tmr_manager_;
  void                         tmrManager();

  // Variables
  int                             trajectory_count_{0};
  int                             start_point_;
  double                          _rate_tmr_manager_;
  double                          _speed_;
  double                          _z_offset_;
  int                             waypoints_qty_points_;
  std::vector<double>             _waypoints_points_;
  laser_msgs::msg::TrajectoryPath trajectory_path_;
  std::vector<std::string>        qr_codes_;
  std::mutex                      qrcode_mtx_;

  laser_msgs::msg::UavControlDiagnostics        diagnostics_;
  std::vector<laser_msgs::msg::PointWithString> qrcode_vector_;

  // Servs
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_state_machine_;

  // Subs
  rclcpp::Subscription<laser_msgs::msg::UavControlDiagnostics>::ConstSharedPtr sub_diagnostics_;
  void                                                                         subControlManagerDiagnostics(const laser_msgs::msg::UavControlDiagnostics &msg);

  rclcpp::Subscription<laser_msgs::msg::PointWithStringArrayStamped>::ConstSharedPtr sub_qrcode_reader_;
  void subQrcodeReader(const laser_msgs::msg::PointWithStringArrayStamped &qrcode_reader);

  // pub
  rclcpp_lifecycle::LifecyclePublisher<laser_msgs::msg::TrajectoryPath>::SharedPtr pub_trajectory_;

  // Clt
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_land_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_takeoff_;

  bool land_request_{false};
  bool first_qrcode_{true};
  bool is_active_{false};
  bool first_pub_{true};
};
}  // namespace manager_node_cpp

#endif
