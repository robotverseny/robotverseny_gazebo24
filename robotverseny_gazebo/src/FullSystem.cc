/*
 * Example FullSystem Gazebo Harmonic plugin
 */

#include <string>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include "robotverseny_gazebo/FullSystem.hh"

namespace robotverseny_gazebo 
{

void FullSystem::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  igndbg << "[FullSystem] Configure on entity: " << _entity << std::endl;
}

void FullSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &/*_ecm*/)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "[FullSystem] PreUpdate" << std::endl;
  }
}

void FullSystem::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &/*_ecm*/)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "[FullSystem] Update" << std::endl;
  }
}

void FullSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &/*_ecm*/) 
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "[FullSystem] PostUpdate" << std::endl;
  }
}

}  // namespace robotverseny_gazebo

// 🔑 Plugin registration must be placed AFTER the namespace
GZ_ADD_PLUGIN(
    robotverseny_gazebo::FullSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemUpdate,
    gz::sim::ISystemPostUpdate)
