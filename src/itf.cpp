#include "openspot/itf.hpp"
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>

namespace openspot {

using namespace ignition;

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

void ImuImpl::OnMsg(msgs::IMU const &msg) {
  imudata_.quat = {msg.orientation().w(), msg.orientation().x(), msg.orientation().y(), msg.orientation().z()};
  imudata_.gyro = {msg.angular_velocity().x(), msg.angular_velocity().y(), msg.angular_velocity().z()};
  imudata_.acc = {msg.linear_acceleration().x(), msg.linear_acceleration().y(), msg.linear_acceleration().z()};
}

LegImpl::LegImpl(ignition::gazebo::EntityComponentManager const &ecm, gazebo::Model &model) {
  for (size_t i = 0; i < consts::kJointNames.size(); i++) {
    auto joint = model.JointByName(ecm, consts::kJointNames[i]);
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

bool LegImpl::RunOnce(gazebo::EntityComponentManager &ecm) {
  for (size_t i = 0; i < 12; i++) {
    auto joint = joints_[i];

    auto q = ecm.Component<gazebo::components::JointPosition>(joint)->Data().at(0);
    auto qd = ecm.Component<gazebo::components::JointVelocity>(joint)->Data().at(0);

    size_t l = i / 3;
    size_t j = i % 3;
    legdatas_[l].q[j] = q;
    legdatas_[l].qd[j] = qd;

    auto tt = legcmds_[l].tau[j] + legcmds_[l].kp[j] * (legcmds_[l].q_des[j] - q) +
              legcmds_[l].kd[j] * (legcmds_[l].qd_des[j] - qd);

    auto jf = ecm.Component<gazebo::components::JointForceCmd>(joint);
    if(jf == nullptr) ecm.CreateComponent(joint, gazebo::components::JointForceCmd({tt}));
    else *jf = gazebo::components::JointForceCmd({tt});
  }

  return true;
}

}  // namespace openspot
