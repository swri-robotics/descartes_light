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

#include "descartes_light/impl/collision/tesseract_collision_checker.h"

descartes_light::TesseractCollision::TesseractCollision(tesseract::BasicEnvConstPtr collision_env,
                                                        const std::string& group_name)
  : collision_env_(std::move(collision_env))
  , contact_manager_(collision_env_->getDiscreteContactManager())
{
  kin_group_ = collision_env_->getManipulator(group_name);

  if (!kin_group_)
  {
    throw std::runtime_error("Group " + group_name + " not found in scene");
  }

  contact_manager_->setActiveCollisionObjects(kin_group_->getLinkNames());
  contact_manager_->setIsContactAllowedFn(std::bind(&TesseractCollision::isContactAllowed, this, std::placeholders::_1,
                                                    std::placeholders::_2));
  }

bool descartes_light::TesseractCollision::validate(const double* pos, std::size_t size)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  Eigen::Map<const Eigen::VectorXd> joint_angles(pos, long(size));
  const auto& joint_names = kin_group_->getJointNames();
  tesseract::EnvStatePtr env_state = collision_env_->getState(joint_names, joint_angles);

  // 2. Update the scene
  contact_manager_->setCollisionObjectsTransform(env_state->transforms);

  // 3. Ask the contact manager to go nuts
  tesseract::ContactResultMap results;
  contact_manager_->contactTest(results, tesseract::ContactTestType::FIRST);

  // 4. Analyze results
  const bool no_contacts = results.empty();

//#define DEBUG_PRINT
#ifdef DEBUG_PRINT
  for (const auto& contact : results)
  {
    std::cout << "Contact: " << contact.first.first << " - " << contact.first.second << "\n";
  }
#endif

  return no_contacts;
}

descartes_light::TesseractCollision* descartes_light::TesseractCollision::clone() const
{
  auto ptr = new TesseractCollision(*this);
  ptr->contact_manager_= this->contact_manager_->clone();
  return ptr;
}

bool descartes_light::TesseractCollision::isContactAllowed(const std::string &a, const std::string &b) const
{
  return collision_env_->getAllowedCollisionMatrix()->isCollisionAllowed(a ,b);
}
