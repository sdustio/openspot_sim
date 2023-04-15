#pragma once

#include <array>

#include "spotng/interface.h"
#include "spotng/sensor.h"

#include <ignition/msgs/imu.pb.h>
#include <ignition/gazebo/Model.hh>

namespace openspot {
class ImuImpl : public spotng::interface::Imu {
 public:
  bool ReadTo(spotng::sensor::ImuData &data) const override;
  void OnMsg(ignition::msgs::IMU const &msg);

 private:
  spotng::sensor::ImuData imudata_;
};

class LegImpl : public spotng::interface::Leg {
 public:
  LegImpl(ignition::gazebo::EntityComponentManager const &ecm, ignition::gazebo::Model &model);
  bool ReadTo(spotng::sensor::LegDatas &data) const override;
  bool WriteFrom(spotng::interface::LegCmds const &cmds) override;
  bool RunOnce(ignition::gazebo::EntityComponentManager &ecm);

 private:
  std::array<ignition::gazebo::Entity, 12> joints_;
  spotng::sensor::LegDatas legdatas_;
  spotng::interface::LegCmds legcmds_;
};
}  // namespace openspot
