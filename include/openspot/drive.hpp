#pragma once

#include <ignition/gazebo/System.hh>
#include <memory>

namespace openspot {

class QuadDriveImpl;

class QuadDrive : public ignition::gazebo::System,
                  public ignition::gazebo::ISystemConfigure,
                  public ignition::gazebo::ISystemPreUpdate,
                  public ignition::gazebo::ISystemPostUpdate {
 public:
  QuadDrive();

  ~QuadDrive() override = default;

  void Configure(ignition::gazebo::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                 ignition::gazebo::EntityComponentManager &ecm, ignition::gazebo::EventManager &eventMgr) override;

  void PreUpdate(ignition::gazebo::UpdateInfo const &info, ignition::gazebo::EntityComponentManager &ecm) override;

  void PostUpdate(ignition::gazebo::UpdateInfo const &info,
                  ignition::gazebo::EntityComponentManager const &ecm) override;

 private:
  std::unique_ptr<QuadDriveImpl> impl_;
};

}  // namespace openspot
