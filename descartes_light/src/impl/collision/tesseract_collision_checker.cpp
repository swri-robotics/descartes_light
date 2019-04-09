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
                                                        const std::vector<std::string>& active_links,
                                                        const std::vector<std::string>& joint_names)
  : collision_env_(std::move(collision_env))
  , active_link_names_(active_links)
  , joint_names_(joint_names)
  , contact_manager_(collision_env_->getDiscreteContactManager())
{
  contact_manager_->setActiveCollisionObjects(active_links);
  contact_manager_->setIsContactAllowedFn(std::bind(&TesseractCollision::isContactAllowed, this, std::placeholders::_1,
                                                    std::placeholders::_2));
  }

bool descartes_light::TesseractCollision::validate(const double* pos, std::size_t size)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  Eigen::Map<const Eigen::VectorXd> joint_angles(pos, long(size));
  tesseract::EnvStatePtr env_state = collision_env_->getState(joint_names_, joint_angles);

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

std::shared_ptr<descartes_light::CollisionInterface> descartes_light::TesseractCollision::clone() const
{
  auto sptr = std::make_shared<TesseractCollision>(*this);
  sptr->contact_manager_= this->contact_manager_->clone();
  return sptr;
}

bool descartes_light::TesseractCollision::isContactAllowed(const std::string &a, const std::string &b) const
{
  return collision_env_->getAllowedCollisionMatrix()->isCollisionAllowed(a ,b);
}
