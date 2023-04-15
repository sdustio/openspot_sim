#include "openspot/drive.hpp"

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/msgs/odometry.pb.h>

#include "openspot/itf.hpp"
#include "spotng/options.h"
#include "spotng/robot.h"


IGNITION_ADD_PLUGIN(
  openspot::QuadDrive,
  ignition::gazebo::System,
  openspot::QuadDrive::ISystemConfigure,
  openspot::QuadDrive::ISystemPreUpdate,
  openspot::QuadDrive::ISystemPostUpdate)

namespace openspot {

using namespace ignition;

namespace {
template <typename T>
T Square(T a) {
  return a * a;
}

void ExtractVectorf(std::vector<double> &out, std::string const &str, std::string delimiter = " ") {
  out.clear();
  std::size_t start = 0;
  std::size_t end = str.find(delimiter);
  while (end != std::string::npos) {
    out.push_back(std::stod(str.substr(start, end - start)));
    start = end + delimiter.size();
    end = str.find(delimiter, start);
  }
  out.push_back(std::stod(str.substr(start, str.size() - start)));
}
double const kCtrlSec = 0.002;
std::unordered_map<std::string, spotng::logging::Level> const kLogLevelMap = {
    {"debug", spotng::logging::Level::Debug},
    {"info", spotng::logging::Level::Info},
    {"warn", spotng::logging::Level::Warn},
    {"err", spotng::logging::Level::Err},
    {"critical", spotng::logging::Level::Critical}};
std::unordered_map<std::string, spotng::logging::Target> const kLogTargetMap = {
    {"console", spotng::logging::Target::Console},
    {"file", spotng::logging::Target::File},
    {"rotate_file", spotng::logging::Target::RotateFile}};
}  // namespace

class QuadDriveImpl {
 public:
  void LoadSpotngOptions(spotng::Options::SharedPtr opts, sdf::ElementConstPtr const &sdf);

  /// Callback when a velocity command is received.
  /// \param[in] msg Twist command message.
  void OnCmdVel(msgs::Twist const &msg);

  /// Callback when a pose command is received.
  /// \param[in] msg Pose command message.
  void OnCmdPose(msgs::Pose const &msg);

  void OnCmdMode(msgs::Int32 const &mode);
  void OnCmdState(msgs::Int32 const &state);
  void OnCmdGait(msgs::Int32 const &gait);
  void OnCmdStepHeight(msgs::Double const &step_height);

  /// Update odometry.
  void UpdateOdometry(gazebo::UpdateInfo const &info);

  // Spotng
  spotng::RobotCtrl::SharedPtr robot_ctrl;
  spotng::drive::DriveCtrl::SharedPtr drive_ctrl;
  spotng::drive::Twist drive_twist;
  spotng::drive::Pose drive_pose;

  std::shared_ptr<LegImpl> leg_itf;
  std::shared_ptr<ImuImpl> imu_itf;

  // Ignition communication node.
  transport::Node node;

  // Model interface
  gazebo::Model model{gazebo::kNullEntity};

  transport::Node::Publisher odom_pub;
  transport::Node::Publisher tf_pub;

  /// Odometry frame ID
  std::string odometry_frame;

  /// Robot base frame ID
  std::string robot_base_frame;

  std::chrono::steady_clock::duration last_time;
};

QuadDrive::QuadDrive() : impl_(std::make_unique<QuadDriveImpl>()) {}

void QuadDrive::Configure(gazebo::Entity const &entity, std::shared_ptr<const sdf::Element> const &sdf,
                          gazebo::EntityComponentManager &ecm, [[maybe_unused]] gazebo::EventManager &etm) {
  impl_->model = gazebo::Model(entity);

  // Interface
  auto default_topic = "/model/" + impl_->model.Name(ecm) + "/imu";
  auto imu_topic = sdf->Get<std::string>("imu_topic", default_topic).first;
  impl_->imu_itf = std::make_shared<ImuImpl>();
  impl_->node.Subscribe(imu_topic, &ImuImpl::OnMsg, impl_->imu_itf.get());

  impl_->leg_itf = std::make_shared<LegImpl>(ecm, impl_->model);

  // Options
  auto opts = std::make_shared<spotng::Options>();
  impl_->LoadSpotngOptions(opts, sdf);
  spotng::RobotCtrl::Build(impl_->robot_ctrl, opts, impl_->leg_itf, impl_->imu_itf);

  // QuadDrive Ctrl
  impl_->drive_ctrl = impl_->robot_ctrl->GetDriveCtrl();

  default_topic = "/model/" + impl_->model.Name(ecm) + "/cmd_vel";
  auto drive_twist_topic = sdf->Get<std::string>("drive_twist_topic", default_topic).first;
  impl_->node.Subscribe(drive_twist_topic, &QuadDriveImpl::OnCmdVel, impl_.get());

  default_topic = "/model/" + impl_->model.Name(ecm) + "/cmd_pose";
  auto drive_pose_topic = sdf->Get<std::string>("drive_pose_topic", default_topic).first;
  impl_->node.Subscribe(drive_pose_topic, &QuadDriveImpl::OnCmdPose, impl_.get());

  // Odometry
  impl_->odometry_frame = sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  default_topic = "/model/" + impl_->model.Name(ecm) + "/odometry";
  auto odom_topic = sdf->Get<std::string>("odom_topic", default_topic).first;
  impl_->odom_pub = impl_->node.Advertise<msgs::Odometry>(odom_topic);

  default_topic = "/model/" + impl_->model.Name(ecm) + "/tf";
  auto tf_topic = sdf->Get<std::string>("tf_topic", default_topic).first;
  impl_->tf_pub = impl_->node.Advertise<msgs::Pose>(tf_topic);
}

void QuadDrive::PreUpdate(ignition::gazebo::UpdateInfo const &info, ignition::gazebo::EntityComponentManager &ecm) {
  if (info.paused) return;

  std::chrono::duration<double> passed = info.simTime - impl_->last_time;
  if (passed.count() < kCtrlSec) return;

  impl_->leg_itf->RunOnce(ecm);
  impl_->robot_ctrl->RunOnce();

  impl_->last_time = info.simTime;
}

void QuadDrive::PostUpdate(ignition::gazebo::UpdateInfo const &info,
                          [[maybe_unused]] ignition::gazebo::EntityComponentManager const &ecm) {
  if (info.paused) return;
  impl_->UpdateOdometry(info);
}

void QuadDriveImpl::LoadSpotngOptions(spotng::Options::SharedPtr opts, sdf::ElementConstPtr const &sdf) {
  std::vector<double> v;

  // options
  opts->ctrl_sec = kCtrlSec;

  opts->log_level = kLogLevelMap.at(sdf->Get<std::string>("//log/level", "warn").first);
  opts->log_target = kLogTargetMap.at(sdf->Get<std::string>("//log/target", "console").first);
  std::strncpy(opts->log_filename, sdf->Get<std::string>("//log/filename", "log/spotng.log").first.c_str(),
               sizeof(opts->log_filename) - 1);

  // model options
  opts->model.body_length = sdf->Get<double>("//model/body_length", 0.58).first;
  opts->model.body_width = sdf->Get<double>("//model/body_width", 0.58).first;
  opts->model.body_height = sdf->Get<double>("//model/body_height", 0.58).first;

  opts->model.mass_body = sdf->Get<double>("//model/mass_body", 25.).first;
  opts->model.mass_abad = sdf->Get<double>("//model/mass_abad", 2.).first;
  opts->model.mass_hip = sdf->Get<double>("//model/mass_hip", 1.).first;
  opts->model.mass_knee = sdf->Get<double>("//model/mass_knee", 1.).first;
  opts->model.mass_total = sdf->Get<double>("//model/mass_total", 41.).first;

  opts->model.link_length_abad = sdf->Get<double>("//model/link_length_abad", 0.093).first;
  opts->model.link_length_hip = sdf->Get<double>("//model/link_length_hip", 0.284).first;
  opts->model.link_length_knee = sdf->Get<double>("//model/link_length_knee", 0.284).first;

  opts->model.basic_locomotion_height = sdf->Get<double>("//model/basic_locomotion_height", 0.4).first;
  opts->model.fast_locomotion_height = sdf->Get<double>("//model/fast_locomotion_height", 0.34).first;
  opts->model.foot_offsetx = sdf->Get<double>("//model/foot_offsetx", -0.03).first;
  opts->model.foot_offsety = sdf->Get<double>("//model/foot_offsety", -0.02).first;

  ExtractVectorf(v, sdf->Get<std::string>("//model/location_abad_fl", "0.29 0.066 0.0").first);
  std::copy_n(v.begin(), 3, opts->model.location_abad_fl.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//model/location_hip_fl", "0.05 0.093 0.0").first);
  std::copy_n(v.begin(), 3, opts->model.location_hip_fl.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//model/location_knee_fl", "0 0 -0.284").first);
  std::copy_n(v.begin(), 3, opts->model.location_knee_fl.begin());

  ExtractVectorf(v, sdf->Get<std::string>("//model/com_body", "0 0 0").first);
  std::copy_n(v.begin(), 3, opts->model.com_body.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//model/com_abad_fl", "0.05 0.021 0.0").first);
  std::copy_n(v.begin(), 3, opts->model.com_abad_fl.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//model/com_hip_fl", "0 0 -0.142").first);
  std::copy_n(v.begin(), 3, opts->model.com_hip_fl.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//model/com_knee_fl", "0 0 -0.142").first);
  std::copy_n(v.begin(), 3, opts->model.com_knee_fl.begin());

  ExtractVectorf(v, sdf->Get<std::string>("//model/inertia_body", "0.0567188 0 0 0 0.721252 0 0 0 0.737133").first);
  std::copy_n(v.begin(), 9, opts->model.inertia_body.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//model/inertia_abad", "0.002426 0 0 0 0.0025 0 0 0 0.002426").first);
  std::copy_n(v.begin(), 9, opts->model.inertia_abad.begin());
  ExtractVectorf(v,
                 sdf->Get<std::string>("//model/inertia_hip", "0.00679633 0 0 0 0.00682342 0 0 0 0.000177083").first);
  std::copy_n(v.begin(), 9, opts->model.inertia_hip.begin());
  ExtractVectorf(v,
                 sdf->Get<std::string>("//model/inertia_knee", "0.00679633 0 0 0 0.00682342 0 0 0 0.000177083").first);
  std::copy_n(v.begin(), 9, opts->model.inertia_knee.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//model/inertia_total", "0.07487 0 0 0 2.1566 0 0 0 2.1775").first);
  std::copy_n(v.begin(), 9, opts->model.inertia_total.begin());

  opts->model.max_com_height = sdf->Get<double>("//model/max_com_height", 0.55).first;
  opts->model.max_body_roll = sdf->Get<double>("//model/max_body_roll", 0.523).first;
  opts->model.max_body_pitch = sdf->Get<double>("//model/max_body_pitch", 0.785).first;
  opts->model.max_body_yaw = sdf->Get<double>("//model/max_body_yaw", 0.523).first;

  // ctrl options
  opts->ctrl.mpc_iters = sdf->Get<int>("//ctrl/mpc_iters", 15).first;
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/mpc_weights", "1.25 1.25 10 2 2 50 0 0 0.3 1.5 1.5 0.2 0").first);
  std::copy_n(v.begin(), 13, opts->ctrl.mpc_weights.begin());
  opts->ctrl.footskd_bonus_swing = sdf->Get<double>("//ctrl/footskd_bonus_swing", 0.).first;
  opts->ctrl.footskd_vkp = sdf->Get<double>("//ctrl/footskd_vkp", 0.1).first;
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kp_bodypos", "100 100 100").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kp_bodypos.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kd_bodypos", "10 10 20").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kd_bodypos.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kp_bodyori", "100 100 100").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kp_bodyori.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kd_bodyori", "10 10 10").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kd_bodyori.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kp_foot", "500 500 500").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kp_foot.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kd_foot", "60 60 60").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kd_foot.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kp_joint", "3 3 3").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kp_joint.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kd_joint", "1.0 0.2 0.2").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kd_joint.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kp_jpos", "80 80 80").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kp_jpos.begin());
  ExtractVectorf(v, sdf->Get<std::string>("//ctrl/kd_jpos", "1 1 1").first);
  std::copy_n(v.begin(), 3, opts->ctrl.kd_jpos.begin());
  ExtractVectorf(
      v, sdf->Get<std::string>("//ctrl/jpos_init", "-0.0 1.4 -2.7 0.0 1.4 -2.7 -0.0 1.4 -2.7 0.0 1.4 -2.7").first);
  std::copy_n(v.begin(), 12, opts->ctrl.jpos_init.front().begin());
  ExtractVectorf(
      v, sdf->Get<std::string>("//ctrl/jpos_fold", "-0.0 1.4 -2.7 0.0 1.4 -2.7 -0.0 1.4 -2.7 0.0 1.4 -2.7").first);
  std::copy_n(v.begin(), 12, opts->ctrl.jpos_fold.front().begin());
  ExtractVectorf(
      v, sdf->Get<std::string>("//ctrl/jpos_stand", "-0.0 0.8 -1.6 0.0 0.8 -1.6 -0.0 0.9 -1.5 0.0 0.9 -1.5").first);
  std::copy_n(v.begin(), 12, opts->ctrl.jpos_stand.front().begin());
  ExtractVectorf(
      v, sdf->Get<std::string>("//ctrl/jpos_rolling", "1.5 1.6 -2.77 1.3 3.1 -2.77 1.5 1.6 -2.77 1.3 3.1 -2.77").first);
  std::copy_n(v.begin(), 12, opts->ctrl.jpos_rolling.front().begin());
  opts->ctrl.max_trot_lvel_x = sdf->Get<double>("//ctrl/max_trot_lvel_x", 1.8).first;
  opts->ctrl.min_trot_lvel_x = sdf->Get<double>("//ctrl/min_trot_lvel_x", -0.8).first;
  opts->ctrl.max_trot_lvel_y = sdf->Get<double>("//ctrl/max_trot_lvel_y", 0.4).first;
  opts->ctrl.max_trot_avel_z = sdf->Get<double>("//ctrl/max_trot_avel_z", 0.8).first;
}

void QuadDriveImpl::OnCmdVel(msgs::Twist const &msg) {
  drive_twist.lvel_x = msg.linear().x();
  drive_twist.lvel_y = msg.linear().y();
  drive_twist.avel_z = msg.angular().z();

  drive_ctrl->UpdateTwist(drive_twist);
}

void QuadDriveImpl::OnCmdPose(msgs::Pose const &msg) {
  drive_pose.height = msg.position().z();
  // q: [w, x, y, z]
  spotng::SdVector4f q = {msg.orientation().w(), msg.orientation().x(), msg.orientation().y(), msg.orientation().z()};
  double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);

  drive_pose.yaw =
      std::atan2(2 * (q[1] * q[2] + q[0] * q[3]), Square(q[0]) + Square(q[1]) - Square(q[2]) - Square(q[3]));
  drive_pose.pitch = std::asin(as);
  drive_pose.roll =
      std::atan2(2 * (q[2] * q[3] + q[0] * q[1]), Square(q[0]) - Square(q[1]) - Square(q[2]) + Square(q[3]));

  drive_ctrl->UpdatePose(drive_pose);
}

void QuadDriveImpl::OnCmdMode(msgs::Int32 const &mode){
  drive_ctrl->UpdateMode(static_cast<spotng::drive::Mode>(mode.data()));
}
void QuadDriveImpl::OnCmdState(msgs::Int32 const &state){
  drive_ctrl->UpdateState(static_cast<spotng::drive::State>(state.data()));
}
void QuadDriveImpl::OnCmdGait(msgs::Int32 const &gait){
  drive_ctrl->UpdateGait(static_cast<spotng::drive::Gait>(gait.data()));
}
void QuadDriveImpl::OnCmdStepHeight(msgs::Double const &step_height){
  drive_ctrl->UpdateStepHeight(step_height.data());
}

void QuadDriveImpl::UpdateOdometry(gazebo::UpdateInfo const &info) {
  auto const &est = robot_ctrl->GetEstimatState();

  msgs::Odometry msg;

  msg.mutable_pose()->mutable_position()->set_x(est.pos[0]);
  msg.mutable_pose()->mutable_position()->set_y(est.pos[1]);
  msg.mutable_pose()->mutable_position()->set_z(est.pos[2]);

  msg.mutable_pose()->mutable_orientation()->set_w(est.ori[0]);
  msg.mutable_pose()->mutable_orientation()->set_x(est.ori[1]);
  msg.mutable_pose()->mutable_orientation()->set_y(est.ori[2]);
  msg.mutable_pose()->mutable_orientation()->set_z(est.ori[3]);

  msg.mutable_twist()->mutable_linear()->set_x(est.lvel_robot[0]);
  msg.mutable_twist()->mutable_linear()->set_y(est.lvel_robot[1]);
  msg.mutable_twist()->mutable_angular()->set_z(est.avel_robot[2]);

  msg.mutable_header()->mutable_stamp()->CopyFrom(ignition::msgs::Convert(info.simTime));

  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(odometry_frame);

  auto childFrame = msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(robot_base_frame);

  msgs::Pose tf_msg;
  tf_msg.mutable_header()->CopyFrom(msg.header());
  tf_msg.mutable_position()->CopyFrom(msg.pose().position());
  tf_msg.mutable_orientation()->CopyFrom(msg.pose().orientation());

  odom_pub.Publish(msg);
  tf_pub.Publish(tf_msg);
}

}  // namespace openspot
