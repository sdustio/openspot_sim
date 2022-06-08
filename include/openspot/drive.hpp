#pragma once

#include <memory>

#include "gazebo/common/Plugin.hh"

namespace openspot {

class QuadDriveImpl;

class Drive : public gazebo::ModelPlugin {
 public:
  Drive();

  virtual ~Drive();

 protected:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  std::unique_ptr<QuadDriveImpl> impl_;
};

}  // namespace openspot
