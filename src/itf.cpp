#include "openspot/itf.hpp"

namespace openspot {

namespace consts {
std::array<std::string const, 12> const kJointNames = {
    "fr_abad_joint", "fr_hip_joint", "fr_knee_joint", "fl_abad_joint", "fl_hip_joint", "fl_knee_joint",
    "hr_abad_joint", "hr_hip_joint", "hr_knee_joint", "hl_abad_joint", "hl_hip_joint", "hl_knee_joint"};
std::array<double const, 12> const kJointPositions = {-0.785398, 1.40115, -2.953097, 0.785398, 1.40115, -2.953097,
                                                      -0.785398, 1.40115, -2.953097, 0.785398, 1.40115, -2.953097};
}  // namespace consts

bool ImuImpl::ReadTo(spotng::sensor::ImuData &data) const {
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
  for (size_t i = 0; i < consts::kJointNames.size(); i++) {
    auto joint = model->GetJoint(consts::kJointNames[i]);
    // joint->SetPosition(0, consts::kJointPositions[i]);
    joints_[i] = joint;
  }
}

bool LegImpl::ReadTo(spotng::sensor::LegDatas &data) const {
  data = legdatas_;
  return true;
}

bool LegImpl::WriteFrom(spotng::interface::LegCmds const &cmds) {
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

}  // namespace openspot
