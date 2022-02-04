#include "sdnova_simulation/itf.hpp"

namespace sdnova {
bool ImuImpl::ReadTo(sdquadx::sensor::ImuData &data) const {
  data = imudata_;
  return true;
}

bool ImuImpl::OnMsg(sensor_msgs::msg::Imu::ConstSharedPtr const &msg) {
  imudata_.quat = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
  imudata_.gyro = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
  imudata_.acc = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
  return true;
}

LegImpl::LegImpl(gazebo::physics::ModelPtr model) {
  std::array<std::string, 12> jns = {"fr_abac", "fr_hip", "fr_knee", "hr_abac", "hr_hip", "hr_knee",
                                     "fl_abac", "fl_hip", "fl_knee", "hl_abac", "hl_hip", "hl_knee"};
  for (size_t i = 0; i < jns.size(); i++) joints_[i] = model->GetJoint(jns[i]);
}

bool LegImpl::ReadTo(sdquadx::sensor::LegDatas &data) const {
  data = legdatas_;
  return true;
}

bool LegImpl::WriteFrom(sdquadx::interface::LegCmds const &cmds) {
  legcmds_ = cmds;
  return true;
}

bool LegImpl::RunOnce(gazebo::common::UpdateInfo const &info) {
  for (size_t i = 0; i < 12; i++) {
    auto joint = joints_[i];
    auto q = joint->Position(0);
    auto qd = joint->GetVelocity(0);

    size_t l = i / 3;
    size_t j = i % 3;
    legdatas_[l].q[j] = q;
    legdatas_[l].qd[j] = qd;

    auto tt = legcmds_[l].tau[j] + legcmds_[l].kp[j] * (legcmds_[l].q_des[j] - q) +
              legcmds_[l].kd[j] * (legcmds_[l].qd_des[j] - qd);
    joint->SetForce(0, tt);
  }

  return true;
}

}  // namespace sdnova
