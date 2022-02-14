#pragma once

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace sdnova {

class QuadDriveImpl;

class QuadDrive : public gazebo::ModelPlugin {
 public:
  QuadDrive();

  virtual ~QuadDrive();

 protected:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  std::unique_ptr<QuadDriveImpl> impl_;
};

}  // namespace sdnova
