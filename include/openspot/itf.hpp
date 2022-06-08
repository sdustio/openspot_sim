#pragma once

#include <array>

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"
#include "spotng/interface.h"
#include "spotng/sensor.h"
#include "sensor_msgs/msg/imu.hpp"

namespace openspot {
class ImuImpl : public spotng::interface::Imu {
 public:
  bool ReadTo(spotng::sensor::ImuData &data) const override;
  bool OnMsg(sensor_msgs::msg::Imu::ConstSharedPtr const &msg);

 private:
  spotng::sensor::ImuData imudata_;
};

class LegImpl : public spotng::interface::Leg {
 public:
  LegImpl(gazebo::physics::ModelPtr model);
  bool ReadTo(spotng::sensor::LegDatas &data) const override;
  bool WriteFrom(spotng::interface::LegCmds const &cmds) override;
  bool RunOnce(gazebo::common::UpdateInfo const &info);

 private:
  std::array<gazebo::physics::JointPtr, 12> joints_;
  spotng::sensor::LegDatas legdatas_;
  spotng::interface::LegCmds legcmds_;
};
}  // namespace openspot
