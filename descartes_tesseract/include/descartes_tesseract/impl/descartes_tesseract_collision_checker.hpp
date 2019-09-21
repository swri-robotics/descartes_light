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
#ifndef DESCARTES_TESSERACT_IMPL_DESCARTES_TESSERACT_COLLISION_CHECKER_HPP
#define DESCARTES_TESSERACT_IMPL_DESCARTES_TESSERACT_COLLISION_CHECKER_HPP

#include <descartes_tesseract/descartes_tesseract_collision_checker.h>

namespace descartes_light
{
template <typename FloatType>
bool TesseractCollision<FloatType>::isContactAllowed(const std::string& a, const std::string& b) const
{
  return acm_.isCollisionAllowed(a, b);
}

template <typename FloatType>
TesseractCollision<FloatType>::TesseractCollision(const tesseract_environment::Environment::ConstPtr &collision_env,
                                                  const std::vector<std::string>& active_links,
                                                  const std::vector<std::string>& joint_names)
  : state_solver_(collision_env->getStateSolver())
  , acm_(*(collision_env->getAllowedCollisionMatrix()))
  , active_link_names_(active_links)
  , joint_names_(joint_names)
  , contact_manager_(collision_env->getDiscreteContactManager())
{
  contact_manager_->setActiveCollisionObjects(active_links);
  contact_manager_->setIsContactAllowedFn(std::bind(&descartes_light::TesseractCollision<FloatType>::isContactAllowed,
                                                    this,
                                                    std::placeholders::_1,
                                                    std::placeholders::_2));
}

template <typename FloatType>
TesseractCollision<FloatType>::TesseractCollision(const TesseractCollision& collision_interface)
  : state_solver_(collision_interface.state_solver_->clone())
  , acm_(collision_interface.acm_)
  , active_link_names_(collision_interface.active_link_names_)
  , joint_names_(collision_interface.joint_names_)
  , contact_manager_(collision_interface.contact_manager_->clone())
{
  contact_manager_->setActiveCollisionObjects(active_link_names_);
  contact_manager_->setIsContactAllowedFn(std::bind(&descartes_light::TesseractCollision<FloatType>::isContactAllowed,
                                                    this,
                                                    std::placeholders::_1,
                                                    std::placeholders::_2));
}

template <typename FloatType>
bool TesseractCollision<FloatType>::validate(const FloatType* pos, const std::size_t size)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  Eigen::Map<const Eigen::VectorXd> joint_angles(reinterpret_cast<const double*>(pos), long(size));
  tesseract_environment::EnvState::Ptr env_state = state_solver_->getState(joint_names_, joint_angles);

  // 2. Update the scene
  contact_manager_->setCollisionObjectsTransform(env_state->transforms);

  // 3. Ask the contact manager to go nuts
  tesseract_collision::ContactResultMap results;
  contact_manager_->contactTest(results, tesseract_collision::ContactTestType::FIRST);

  // 4. Analyze results
  const bool no_contacts = results.empty();

  //#ifndef NDEBUG
  //  for (const auto& contact : results)
  //  {
  //    std::cout << "Contact: " << contact.first.first << " - " << contact.first.second << " Distance: " <<
  //    contact.second[0].distance << "\n";
  //  }
  //#endif

  return no_contacts;
}

template <typename FloatType>
FloatType TesseractCollision<FloatType>::distance(const FloatType* pos, const std::size_t size)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  Eigen::Map<const Eigen::VectorXd> joint_angles(reinterpret_cast<const double*>(pos), long(size));
  tesseract_environment::EnvState::Ptr env_state = state_solver_->getState(joint_names_, joint_angles);

  // 2. Update the scene
  contact_manager_->setCollisionObjectsTransform(env_state->transforms);

  // 3. Ask the contact manager to go nuts
  tesseract_collision::ContactResultMap results;
  contact_manager_->contactTest(results, tesseract_collision::ContactTestType::CLOSEST);

#ifndef NDEBUG
  std::cout << "Called descartes_light::TesseractCollision::distance\n";
  for (const auto& contact : results)
  {
    std::cout << "Contact: " << contact.first.first << " - " << contact.first.second
              << " Distance: " << contact.second[0].distance << "\n";
  }
#endif

  if (results.empty())
    return static_cast<FloatType>(contact_manager_->getContactDistanceThreshold());
  else
    return static_cast<FloatType>(results.begin()->second[0].distance);
}

template <typename FloatType>
typename CollisionInterface<FloatType>::Ptr TesseractCollision<FloatType>::clone() const
{
  return std::make_shared<TesseractCollision>(*this);
}

}  // namespace descartes_light

#endif  // DESCARTES_TESSERACT_IMPL_DESCARTES_TESSERACT_COLLISION_CHECKER_HPP
