#include "sdnova_simulation/drive.hpp"

#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gazebo/common/Time.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sdnova_simulation/itf.hpp"
#include "sdquadx/options.h"
#include "sdquadx/robot.h"
#include "tf2_ros/transform_broadcaster.h"

namespace sdnova {

namespace {
template <typename T>
T Square(T a) {
  return a * a;
}
// Ctrl duration in seconds.
double const kCtrlSec = 0.002;
}  // namespace

class QuadDriveImpl {
 public:
  bool Init(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  // SdQuadX
  sdquadx::RobotCtrl::SharedPtr robot_ctrl_;
  sdquadx::drive::DriveCtrl::SharedPtr drive_ctrl_;
  sdquadx::drive::Twist drive_twist_;
  sdquadx::drive::Pose drive_pose_;

  std::shared_ptr<LegImpl> leg_itf_;
  std::shared_ptr<ImuImpl> imu_itf_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// Covariance in odometry
  double covariance_[3];

  /// To publish odometry msg
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Last update time.
  gazebo::common::Time last_ctrl_at_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_set_callback_;

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(gazebo::common::UpdateInfo const &info);

  /// Callback when a velocity command is received.
  /// \param[in] msg Twist command message.
  void OnCmdVel(geometry_msgs::msg::Twist::ConstSharedPtr const &msg);

  /// Callback when a pose command is received.
  /// \param[in] msg Pose command message.
  void OnCmdPose(geometry_msgs::msg::Pose::ConstSharedPtr const &msg);

  /// Update odometry.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometry(const gazebo::common::Time &_current_time);

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time &_current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time &_current_time);

  rcl_interfaces::msg::SetParametersResult OnNodeParmasChanged(std::vector<rclcpp::Parameter> const &params);

 private:
  void LoadSdquadxOptions(sdquadx::Options::SharedPtr opts, sdf::ElementPtr sdf);
};

QuadDrive::QuadDrive() : impl_(std::make_unique<QuadDriveImpl>()) {}

QuadDrive::~QuadDrive() {}

void QuadDrive::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) { impl_->Init(model, sdf); }

bool QuadDriveImpl::Init(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;
  last_ctrl_at_ = model->GetWorld()->SimTime();

  // Initialize ROS node
  ros_node_ = gazebo_ros::Node::Get(sdf);
  ros_node_->declare_parameter<int>("mode", 0);
  ros_node_->declare_parameter<int>("state", 0);
  ros_node_->declare_parameter<int>("gait", 0);
  ros_node_->declare_parameter<double>("step_height", 0.1);
  param_set_callback_ = ros_node_->add_on_set_parameters_callback(
      [this](std::vector<rclcpp::Parameter> const &params) { return this->OnNodeParmasChanged(params); });
  gazebo_ros::QoS const &qos = ros_node_->get_qos();

  // Interface
  auto imu_topic = sdf->Get<std::string>("imu_topic", "/imu").first;
  imu_itf_ = std::make_shared<ImuImpl>();
  imu_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, qos.get_publisher_qos("imu", rclcpp::SensorDataQoS()),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr const msg) { this->imu_itf_->OnMsg(msg); });

  leg_itf_ = std::make_shared<LegImpl>(model);

  // Options
  auto opts = std::make_shared<sdquadx::Options>();
  LoadSdquadxOptions(opts, sdf->GetElement("sdquadx"));
  sdquadx::RobotCtrl::Build(robot_ctrl_, opts, leg_itf_, imu_itf_);

  // Drive Ctrl
  drive_ctrl_ = robot_ctrl_->GetDriveCtrl();

  auto drive_twist_topic = sdf->Get<std::string>("drive_twist_topic", "/cmd_vel").first;
  cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      drive_twist_topic, qos.get_publisher_qos("drive_twist", rclcpp::ServicesQoS()),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr const msg) { this->OnCmdVel(msg); });

  auto drive_pose_topic = sdf->Get<std::string>("drive_pose_topic", "/cmd_pose").first;
  cmd_pose_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Pose>(
      drive_pose_topic, qos.get_publisher_qos("drive_pose", rclcpp::ServicesQoS()),
      [this](geometry_msgs::msg::Pose::ConstSharedPtr const msg) { this->OnCmdPose(msg); });

  // Odometry
  odometry_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
  robot_base_frame_ = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  publish_odom_ = sdf->Get<bool>("publish_odom", false).first;
  publish_odom_tf_ = sdf->Get<bool>("publish_odom_tf", false).first;
  if (publish_odom_) {
    odometry_pub_ =
        ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));
  }
  if (publish_odom_tf_) {
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
  }

  covariance_[0] = sdf->Get<double>("covariance_x", 0.01).first;
  covariance_[1] = sdf->Get<double>("covariance_y", 0.01).first;
  covariance_[2] = sdf->Get<double>("covariance_yaw", 0.01).first;

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](gazebo::common::UpdateInfo const &info) { this->OnUpdate(info); });

  return true;
}

void QuadDriveImpl::LoadSdquadxOptions(sdquadx::Options::SharedPtr opts, sdf::ElementPtr sdf) {
  opts->ctrl_sec = kCtrlSec;

  std::unordered_map<std::string, sdquadx::logging::Level> loglevelmap = {
      {"debug", sdquadx::logging::Level::Debug},
      {"info", sdquadx::logging::Level::Info},
      {"warn", sdquadx::logging::Level::Warn},
      {"err", sdquadx::logging::Level::Err},
      {"critical", sdquadx::logging::Level::Critical}};
  std::unordered_map<std::string, sdquadx::logging::Target> logtargetmap = {
      {"console", sdquadx::logging::Target::Console},
      {"file", sdquadx::logging::Target::File},
      {"rotate_file", sdquadx::logging::Target::RotateFile}};
  auto log = sdf->GetElement("log");
  opts->log_level = loglevelmap[log->Get<std::string>("level", "warn").first];
  opts->log_target = logtargetmap[log->Get<std::string>("target", "console").first];
  std::strncpy(opts->log_filename, log->Get<std::string>("filename", "log/sdquadx.log").first.c_str(),
               sizeof(opts->log_filename) - 1);

  auto model = sdf->GetElement("model");
  opts->model.mass_total = model->Get<double>("mass_total", 41.).first;
}

void QuadDriveImpl::OnUpdate(gazebo::common::UpdateInfo const &info) {
  leg_itf_->RunOnce(info);

  double seconds_since_last_ctrl = (info.simTime - last_ctrl_at_).Double();
  if (seconds_since_last_ctrl < kCtrlSec) {
    return;
  }
  robot_ctrl_->RunOnce();

  UpdateOdometry(info.simTime);
  if (publish_odom_) PublishOdometryMsg(info.simTime);
  if (publish_odom_tf_) PublishOdometryTf(info.simTime);

  last_ctrl_at_ = info.simTime;
}

void QuadDriveImpl::OnCmdVel(geometry_msgs::msg::Twist::ConstSharedPtr const &msg) {
  drive_twist_.lvel_x = msg->linear.x;
  drive_twist_.lvel_y = msg->linear.y;
  drive_twist_.avel_z = msg->angular.z;

  drive_ctrl_->UpdateTwist(drive_twist_);
}

void QuadDriveImpl::OnCmdPose(geometry_msgs::msg::Pose::ConstSharedPtr const &msg) {
  drive_pose_.height = msg->position.z;
  // q: [w, x, y, z]
  sdquadx::SdVector4f q = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
  double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);

  drive_pose_.yaw =
      std::atan2(2 * (q[1] * q[2] + q[0] * q[3]), Square(q[0]) + Square(q[1]) - Square(q[2]) - Square(q[3]));
  drive_pose_.pitch = std::asin(as);
  drive_pose_.roll =
      std::atan2(2 * (q[2] * q[3] + q[0] * q[1]), Square(q[0]) - Square(q[1]) - Square(q[2]) + Square(q[3]));

  drive_ctrl_->UpdatePose(drive_pose_);
}

void QuadDriveImpl::UpdateOdometry(const gazebo::common::Time &_current_time) {
  auto const &est = robot_ctrl_->GetEstimatState();

  odom_.pose.pose.position.x = est.pos[0];
  odom_.pose.pose.position.x = est.pos[0];
  odom_.pose.pose.position.x = est.pos[0];

  odom_.pose.pose.orientation.w = est.ori[0];
  odom_.pose.pose.orientation.x = est.ori[1];
  odom_.pose.pose.orientation.y = est.ori[2];
  odom_.pose.pose.orientation.z = est.ori[3];

  odom_.twist.twist.angular.z = est.avel_robot[2];
  odom_.twist.twist.linear.x = est.lvel_robot[0];
  odom_.twist.twist.linear.y = est.lvel_robot[1];
}

void QuadDriveImpl::PublishOdometryTf(const gazebo::common::Time &_current_time) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void QuadDriveImpl::PublishOdometryMsg(const gazebo::common::Time &_current_time) {
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}

rcl_interfaces::msg::SetParametersResult QuadDriveImpl::OnNodeParmasChanged(
    std::vector<rclcpp::Parameter> const &params) {
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &param : params) {
    auto name = param.get_name();
    if (name == "mode" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      drive_ctrl_->UpdateMode(static_cast<sdquadx::drive::Mode>(param.as_int()));
    else if (name == "state" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      drive_ctrl_->UpdateState(static_cast<sdquadx::drive::State>(param.as_int()));
    else if (name == "gait" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      drive_ctrl_->UpdateGait(static_cast<sdquadx::drive::Gait>(param.as_int()));
    else if (name == "step_height" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      drive_ctrl_->UpdateStepHeight(param.as_double());
  }
  result.successful = true;
  return result;
}

GZ_REGISTER_MODEL_PLUGIN(QuadDrive)

}  // namespace sdnova
