#include "descartes_light/collision_checker.h"

descartes_light::TesseractCollision::TesseractCollision(tesseract::BasicEnvPtr collision_env)
  : collision_env_(std::move(collision_env))
  , contact_manager_(collision_env_->getDiscreteContactManager())
{
  tesseract::ContactRequest cr;
  cr.link_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  cr.type = tesseract::ContactRequestTypes::FIRST;

  contact_manager_->setContactRequest(cr);
}

bool descartes_light::TesseractCollision::validate(const double* pos, std::size_t size)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  Eigen::Map<const Eigen::VectorXd> joint_angles(pos, long(size));
  const auto& link_names = contact_manager_->getContactRequest().link_names;
  tesseract::EnvStatePtr env_state = collision_env_->getState(link_names, joint_angles);

  // 2. Update the scene
  contact_manager_->setCollisionObjectsTransform(env_state->transforms);

  // 3. Ask the contact manager to go nuts
  tesseract::ContactResultMap results;
  contact_manager_->contactTest(results);

  // 4. Analyze results
  return results.empty();
}

descartes_light::TesseractCollision* descartes_light::TesseractCollision::clone() const
{
  auto ptr = new TesseractCollision(*this);
  ptr->contact_manager_= this->contact_manager_->clone();
  return ptr;
}
