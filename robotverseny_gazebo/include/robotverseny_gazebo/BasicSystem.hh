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

#ifndef ROBOTVERSENY_GAZEBO__BASIC_SYSTEM_HH_
#define ROBOTVERSENY_GAZEBO__BASIC_SYSTEM_HH_

// Include the System interface header from Ignition Gazebo Citadel.
#include <ignition/gazebo/System.hh>

namespace robotverseny_gazebo
{
  // Main plugin class. For Citadel, it should inherit from `System` and
  // `ISystemPostUpdate`. Other interfaces can also be added based on the
  // plugin's purpose.
  class BasicSystem:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPostUpdate
  {
    // Plugins inheriting `ISystemPostUpdate` must implement the `PostUpdate`
    // method. This method is called every simulation iteration after physics
    // updates the world. `_info` provides information such as the current time,
    // and `_ecm` provides an interface to all entities and components in
    // simulation.
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
  };
}
#endif
