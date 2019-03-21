/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef DESCARTES_LIGHT_KINEMATIC_INTERFACE_H
#define DESCARTES_LIGHT_KINEMATIC_INTERFACE_H

#include <Eigen/Geometry>
#include <vector>
#include <memory>

namespace descartes_light
{

class KinematicsInterface
{
public:
  virtual ~KinematicsInterface() = default;

  virtual bool ik(const Eigen::Isometry3d& p, std::vector<double>& solution_set) const = 0;
  virtual bool fk(const double* pose, Eigen::Isometry3d& solution) const = 0;
};

using KinematicsInterfacePtr = std::shared_ptr<KinematicsInterface>;

}

#endif
