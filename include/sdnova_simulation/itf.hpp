#pragma once

#include <array>

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"
#include "sdquadx/interface.h"
#include "sdquadx/sensor.h"
#include "sensor_msgs/msg/imu.hpp"

namespace sdnova {
class ImuImpl : public sdquadx::interface::Imu {
 public:
  bool ReadTo(sdquadx::sensor::ImuData &data) const override;
  bool OnMsg(sensor_msgs::msg::Imu::ConstSharedPtr const &msg);

 private:
  sdquadx::sensor::ImuData imudata_;
};

class LegImpl : public sdquadx::interface::Leg {
 public:
  LegImpl(gazebo::physics::ModelPtr model);
  bool ReadTo(sdquadx::sensor::LegDatas &data) const override;
  bool WriteFrom(sdquadx::interface::LegCmds const &cmds) override;
  bool RunOnce(gazebo::common::UpdateInfo const &info);

 private:
  std::array<gazebo::physics::JointPtr, 12> joints_;
  sdquadx::sensor::LegDatas legdatas_;
  sdquadx::interface::LegCmds legcmds_;
};
}  // namespace sdnova
