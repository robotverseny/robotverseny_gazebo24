/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <ignition/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <ignition/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "robotverseny_gazebo/BasicSystem.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    robotverseny_gazebo::BasicSystem,
    ignition::gazebo::System,
    robotverseny_gazebo::BasicSystem::ISystemPostUpdate)

namespace robotverseny_gazebo 
{

void BasicSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                             const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    ignmsg << "robotverseny_gazebo::BasicSystem::PostUpdate" << std::endl;
  }
}

}  // namespace robotverseny_gazebo
