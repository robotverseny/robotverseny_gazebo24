/*
 * Example Gazebo Harmonic system plugin
 * Author: robotverseny_gazebo
 */

#include <string>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include "robotverseny_gazebo/BasicSystem.hh"

namespace robotverseny_gazebo
{

void BasicSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                             const gz::sim::EntityComponentManager &/*_ecm*/)
{
  // Example: Print a message every 1000 iterations if simulation is not paused
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    igndbg << "[BasicSystem] PostUpdate tick at iteration "
           << _info.iterations << std::endl;
  }
}

}  // namespace robotverseny_gazebo

// 🔑 Plugin registration (must come after class + namespace)
GZ_ADD_PLUGIN(
    robotverseny_gazebo::BasicSystem,
    gz::sim::System,
    gz::sim::ISystemPostUpdate)
