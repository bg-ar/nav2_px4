// ARI-Nav2-PX4-Bridge: A simple ROS 2 node to bridge Nav2 cmd_vel to PX4 offboard control mode.
// By BG Kang (github.com/bgkng), 2024. Open source under Apache 2.0 license.

#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std::chrono_literals;

static inline float clampf(float v, float lo, float hi) {
  return std::min(std::max(v, lo), hi);
}

class Nav2Px4Bridge : public rclcpp::Node
{
public:
  Nav2Px4Bridge() : Node("nav2_px4_bridge")
  {
    // Parameters
    px4_ns_            = declare_parameter<std::string>("px4_namespace", "");
    cmd_vel_topic_     = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    publish_rate_hz_   = declare_parameter<double>("publish_rate_hz", 50.0);
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);

    // If your PX4 topics are like ".../trajectory_setpoint_v1", set: topic_suffix: "_v1"
    topic_suffix_      = declare_parameter<std::string>("topic_suffix", "");

    // If true: interpret cmd_vel in BODY FLU, rotate to ENU using heading; then convert ENU->NED
    cmd_vel_is_body_flu_ = declare_parameter<bool>("cmd_vel_is_body_flu", true);

    max_vxy_     = declare_parameter<double>("max_vxy", 2.0);
    max_vz_      = declare_parameter<double>("max_vz", 1.0);
    max_yawrate_ = declare_parameter<double>("max_yawrate", 1.5);

    stream_always_     = declare_parameter<bool>("stream_always", true);
    auto_offboard_arm_ = declare_parameter<bool>("auto_offboard_and_arm", false);
    warmup_cycles_     = declare_parameter<int>("warmup_cycles", 50);

    // QoS: PX4 uXRCE-DDS often uses best_effort; some outputs can be transient_local
    auto px4_in_qos  = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto px4_out_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    // Topic name helper (namespace + optional suffix like _v1)
    const std::string ns = px4_ns_.empty() ? "" : ("/" + px4_ns_);
    const std::string in_base  = ns + "/fmu/in/";
    const std::string out_base = ns + "/fmu/out/";

    offboard_topic_ = in_base  + "offboard_control_mode" + topic_suffix_;
    traj_topic_     = in_base  + "trajectory_setpoint"   + topic_suffix_;
    cmd_topic_      = in_base  + "vehicle_command"       + topic_suffix_;
    localpos_topic_ = out_base + "vehicle_local_position"+ topic_suffix_;

    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_topic_, px4_in_qos);
    traj_pub_     = create_publisher<px4_msgs::msg::TrajectorySetpoint>(traj_topic_, px4_in_qos);
    cmd_pub_      = create_publisher<px4_msgs::msg::VehicleCommand>(cmd_topic_, px4_in_qos);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&Nav2Px4Bridge::cmdVelCb, this, std::placeholders::_1));

    localpos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      localpos_topic_, px4_out_qos,
      std::bind(&Nav2Px4Bridge::localPosCb, this, std::placeholders::_1));

    arm_srv_ = create_service<std_srvs::srv::Trigger>(
      "arm", std::bind(&Nav2Px4Bridge::armSrv, this, std::placeholders::_1, std::placeholders::_2));
    disarm_srv_ = create_service<std_srvs::srv::Trigger>(
      "disarm", std::bind(&Nav2Px4Bridge::disarmSrv, this, std::placeholders::_1, std::placeholders::_2));
    offboard_srv_ = create_service<std_srvs::srv::Trigger>(
      "set_offboard", std::bind(&Nav2Px4Bridge::offboardSrv, this, std::placeholders::_1, std::placeholders::_2));
    land_srv_ = create_service<std_srvs::srv::Trigger>(
      "land", std::bind(&Nav2Px4Bridge::landSrv, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Nav2Px4Bridge::timerCb, this));

    last_cmd_time_ = now();

    RCLCPP_INFO(get_logger(),
      "Nav2->PX4 bridge ready.\n"
      "  cmd_vel: %s\n"
      "  px4 in : %s | %s | %s\n"
      "  px4 out: %s",
      cmd_vel_topic_.c_str(),
      offboard_topic_.c_str(), traj_topic_.c_str(), cmd_topic_.c_str(),
      localpos_topic_.c_str());
  }

private:
  void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    last_cmd_ = *msg;
    last_cmd_time_ = now();
  }

  void localPosCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    heading_ned_ = msg->heading; // yaw in local NED
    have_heading_ = true;
  }

  void timerCb()
  {
    if (!stream_always_ && !stream_enabled_) return;

    publishOffboardControlModeVelocity();
    traj_pub_->publish(computeTrajectorySetpointFromCmdVel());

    if (auto_offboard_arm_) {
      if (warmup_counter_ < warmup_cycles_) {
        warmup_counter_++;
      } else if (!auto_done_) {
        setOffboardMode();
        arm();
        auto_done_ = true;
      }
    }
  }

  void publishOffboardControlModeVelocity()
  {
    px4_msgs::msg::OffboardControlMode m{};
    m.position = false;
    m.velocity = true;
    m.acceleration = false;
    m.attitude = false;
    m.body_rate = false;
    m.thrust_and_torque = false;
    m.direct_actuator = false;
    m.timestamp = nowMicros();
    offboard_pub_->publish(m);
  }

  px4_msgs::msg::TrajectorySetpoint computeTrajectorySetpointFromCmdVel()
  {
    geometry_msgs::msg::Twist cmd{};
    rclcpp::Time cmd_t;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      cmd = last_cmd_;
      cmd_t = last_cmd_time_;
    }

    const bool stale = ((now() - cmd_t).seconds() > cmd_vel_timeout_s_);

    float vx = stale ? 0.f : static_cast<float>(cmd.linear.x);
    float vy = stale ? 0.f : static_cast<float>(cmd.linear.y);
    float vz = stale ? 0.f : static_cast<float>(cmd.linear.z);
    float yawrate_enu = stale ? 0.f : static_cast<float>(cmd.angular.z);

    vx = clampf(vx, -static_cast<float>(max_vxy_), static_cast<float>(max_vxy_));
    vy = clampf(vy, -static_cast<float>(max_vxy_), static_cast<float>(max_vxy_));
    vz = clampf(vz, -static_cast<float>(max_vz_),  static_cast<float>(max_vz_));
    yawrate_enu = clampf(yawrate_enu, -static_cast<float>(max_yawrate_), static_cast<float>(max_yawrate_));

    // BODY FLU -> WORLD ENU (optional)
    float vx_enu = vx, vy_enu = vy, vz_enu = vz;

    if (cmd_vel_is_body_flu_ && have_heading_) {
      // heading_ned -> yaw_enu
      const float yaw_enu = static_cast<float>(M_PI_2) - heading_ned_;
      const float c = std::cos(yaw_enu);
      const float s = std::sin(yaw_enu);

      const float v_fwd  = vx;
      const float v_left = vy;

      vx_enu = v_fwd * c - v_left * s;
      vy_enu = v_fwd * s + v_left * c;
      vz_enu = vz;
    }

    // WORLD ENU -> WORLD NED
    const float vx_ned = vy_enu;    // North
    const float vy_ned = vx_enu;    // East
    const float vz_ned = -vz_enu;   // Down

    // yaw rate ENU(+CCW about +Up) -> NED(+ about Down): negate
    const float yawspeed_ned = -yawrate_enu;

    px4_msgs::msg::TrajectorySetpoint sp{};
    const float NaN = std::numeric_limits<float>::quiet_NaN();

    sp.position = {NaN, NaN, NaN};
    sp.velocity = {vx_ned, vy_ned, vz_ned};
    sp.acceleration = {NaN, NaN, NaN};
    sp.jerk = {NaN, NaN, NaN};

    sp.yaw = have_heading_ ? heading_ned_ : NaN;
    sp.yawspeed = yawspeed_ned;

    sp.timestamp = nowMicros();
    return sp;
  }

  void publishVehicleCommand(uint16_t command,
                             float p1 = 0.f, float p2 = 0.f, float p3 = 0.f, float p4 = 0.f,
                             float p5 = 0.f, float p6 = 0.f, float p7 = 0.f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1 = p1; msg.param2 = p2; msg.param3 = p3; msg.param4 = p4;
    msg.param5 = p5; msg.param6 = p6; msg.param7 = p7;

    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = nowMicros();

    cmd_pub_->publish(msg);
  }

  void setOffboardMode()
  {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);
    stream_enabled_ = true;
  }

  void arm()    { publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f); stream_enabled_ = true; }
  void disarm() { publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f); }
  void land()   { publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

  uint64_t nowMicros() const { return static_cast<uint64_t>(this->now().nanoseconds() / 1000ULL); }

  void armSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    arm();
    res->success = true;
    res->message = "ARM sent";
  }

  void disarmSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    disarm();
    res->success = true;
    res->message = "DISARM sent";
  }

  void offboardSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // Enable streaming now; PX4 requires stream before switching
    stream_enabled_ = true;
    warmup_counter_ = 0;
    auto_done_ = false;

    setOffboardMode();
    res->success = true;
    res->message = "OFFBOARD mode command sent (streaming enabled)";
  }

  void landSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    land();
    res->success = true;
    res->message = "LAND sent";
  }

private:
  std::string px4_ns_;
  std::string cmd_vel_topic_;
  std::string topic_suffix_;

  double publish_rate_hz_{50.0};
  double cmd_vel_timeout_s_{0.5};
  bool cmd_vel_is_body_flu_{true};

  double max_vxy_{2.0}, max_vz_{1.0}, max_yawrate_{1.5};

  bool stream_always_{true};
  bool auto_offboard_arm_{false};
  int warmup_cycles_{50};

  std::string offboard_topic_, traj_topic_, cmd_topic_, localpos_topic_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr localpos_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_, disarm_srv_, offboard_srv_, land_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex cmd_mtx_;
  geometry_msgs::msg::Twist last_cmd_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  bool have_heading_{false};
  float heading_ned_{0.f};

  bool stream_enabled_{false};
  int warmup_counter_{0};
  bool auto_done_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2Px4Bridge>());
  rclcpp::shutdown();
  return 0;
}
