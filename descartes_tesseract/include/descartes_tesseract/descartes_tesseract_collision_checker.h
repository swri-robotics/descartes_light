#ifndef DESCARTES_TESSERACT_TESSERACT_COLLISION_CHECKER_H
#define DESCARTES_TESSERACT_TESSERACT_COLLISION_CHECKER_H

#include <memory>
#include <descartes_light/interface/collision_interface.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_collision/core/discrete_contact_manager.h>

namespace descartes_light
{
template <typename FloatType>
class TesseractCollision : public CollisionInterface<FloatType>
{
public:
  /**
   * @brief TesseractCollision
   * @param collision_env The collision Environment
   * @param active_links The list of active links
   * @param joint_names The list of joint names in the order that the data will be provided to the validate function.
   */
  TesseractCollision(const tesseract_environment::Environment::ConstPtr& collision_env,
                     const std::vector<std::string>& active_links,
                     const std::vector<std::string>& joint_names);

  /**
   * @brief Copy constructor that clones the object
   * @param collision_interface Object to copy/clone
   */
  TesseractCollision(const TesseractCollision& collision_interface);

  bool validate(const FloatType* pos, const std::size_t size) override;

  FloatType distance(const FloatType* pos, const std::size_t size) override;

  typename CollisionInterface<FloatType>::Ptr clone() const override;

private:
  bool isContactAllowed(const std::string& a, const std::string& b) const;

  tesseract_environment::StateSolver::Ptr state_solver_;
  tesseract_scene_graph::AllowedCollisionMatrix acm_;
  std::vector<std::string> active_link_names_;
  std::vector<std::string> joint_names_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
};

using TesseractCollisionF = TesseractCollision<float>;
using TesseractCollisionD = TesseractCollision<double>;

}  // namespace descartes_light

#endif  // DESCARTES_TESSERACT_TESSERACT_COLLISION_CHECKER_H
