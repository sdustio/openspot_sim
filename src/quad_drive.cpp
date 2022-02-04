#include "sdnova_simulation/quad_drive.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>

#include "sdnova_simulation/itf.hpp"
#include "sdquadx/options.h"
#include "sdquadx/robot.h"

namespace sdnova {

namespace {
template <typename T>
T Square(T a) {
  return a * a;
}
}  // namespace

class QuadDriveImpl {
 public:
  bool Init(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  // SdQuadX
  sdquadx::RobotCtrl::SharedPtr robotctrl_;
  sdquadx::drive::DriveCtrl::SharedPtr drive_ctrl_;
  sdquadx::drive::Twist drive_twist_;
  sdquadx::drive::Pose drive_pose_;

  std::shared_ptr<LegImpl> leg_itf_;
  std::shared_ptr<ImuImpl> imu_itf_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Ctrl duration in seconds.
  double ctrl_dt;

  /// Last update time.
  gazebo::common::Time last_ctrl_at_;

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(gazebo::common::UpdateInfo const &info);

  /// Callback when a velocity command is received.
  /// \param[in] msg Twist command message.
  void OnCmdVel(geometry_msgs::msg::Twist::ConstSharedPtr const &msg);

  /// Callback when a pose command is received.
  /// \param[in] msg Pose command message.
  void OnCmdPose(geometry_msgs::msg::Pose::ConstSharedPtr const &msg);

  rcl_interfaces::msg::SetParametersResult OnNodeParmasChanged(std::vector<rclcpp::Parameter> const &params);
};

QuadDrive::QuadDrive() : impl_(std::make_unique<QuadDriveImpl>()) {}

QuadDrive::~QuadDrive() {}

void QuadDrive::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) { impl_->Init(model, sdf); }

bool QuadDriveImpl::Init(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;
  ctrl_dt = sdf->Get<double>("ctrl_sec", 0.002).first;
  last_ctrl_at_ = model->GetWorld()->SimTime();

  // Initialize ROS node
  ros_node_ = gazebo_ros::Node::Get(sdf);
  ros_node_->declare_parameter<int>("mode", 0);
  ros_node_->declare_parameter<int>("state", 0);
  ros_node_->declare_parameter<int>("gait", 0);
  ros_node_->declare_parameter<double>("step_height", 0.1);
  auto cb = ros_node_->add_on_set_parameters_callback(
      [this](std::vector<rclcpp::Parameter> const &params) { return this->OnNodeParmasChanged(params); });

  imu_itf_ = std::make_shared<ImuImpl>();
  ros_node_->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr const msg) { this->imu_itf_->OnMsg(msg); });

  leg_itf_ = std::make_shared<LegImpl>(model);

  auto opts = std::make_shared<sdquadx::Options>();
  opts->ctrl_sec = ctrl_dt;
  opts->model.mass_total = sdf->Get<double>("mass_total", 41.0).first;

  sdquadx::RobotCtrl::Build(robotctrl_, opts, leg_itf_, imu_itf_);
  drive_ctrl_ = robotctrl_->GetDriveCtrl();

  ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::ServicesQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr const msg) { this->OnCmdVel(msg); });

  ros_node_->create_subscription<geometry_msgs::msg::Pose>(
      "cmd_pose", rclcpp::ServicesQoS(),
      [this](geometry_msgs::msg::Pose::ConstSharedPtr const msg) { this->OnCmdPose(msg); });

  gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](gazebo::common::UpdateInfo const &info) { this->OnUpdate(info); });

  return true;
}

void QuadDriveImpl::OnUpdate(gazebo::common::UpdateInfo const &info) {
  leg_itf_->RunOnce(info);

  double seconds_since_last_ctrl = (info.simTime - last_ctrl_at_).Double();
  if (seconds_since_last_ctrl < ctrl_dt) {
    return;
  }
  robotctrl_->RunOnce();
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
