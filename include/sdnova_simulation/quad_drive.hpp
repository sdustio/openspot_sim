#ifndef SDNOVA_SIMULATION__QUAD_DRIVE_HPP_
#define SDNOVA_SIMULATION__QUAD_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace sdnova_simulation {

class QuadDrivePrivate;

class QuadDrive : public gazebo::ModelPlugin {
 public:
  QuadDrive();

  virtual ~QuadDrive();

 protected:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void Reset() override;

 private:
  std::unique_ptr<QuadDrivePrivate> impl_;
};

}  // namespace sdnova_simulation

#endif  // SDNOVA_SIMULATION__QUAD_DRIVE_HPP_
