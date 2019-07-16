#ifndef DESCARTES_TESSERACT_TESSERACT_COLLISION_CHECKER_H
#define DESCARTES_TESSERACT_TESSERACT_COLLISION_CHECKER_H

#include <memory>
#include <descartes_light/interface/collision_interface.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_collision/core/discrete_contact_manager.h>

namespace descartes_light
{

template<typename FloatType>
class TesseractCollision : public CollisionInterface<FloatType>
{
public:
  /**
   * @brief TesseractCollision
   * @param collision_env The collision Environment
   * @param active_links The list of active links
   * @param joint_names The list of joint names in the order that the data will be provided to the validate function.
   */
  TesseractCollision(tesseract_environment::EnvironmentConstPtr collision_env,
                     const std::vector<std::string>& active_links,
                     const std::vector<std::string>& joint_names);

  bool validate(const FloatType* pos, const std::size_t size) override;

  FloatType distance(const FloatType* pos, const std::size_t size) override;

  typename CollisionInterface<FloatType>::Ptr clone() const override;

  tesseract_environment::EnvironmentConstPtr& environment() { return collision_env_; }
  const tesseract_environment::EnvironmentConstPtr& environment() const { return collision_env_; }

private:
  bool isContactAllowed(const std::string& a, const std::string& b) const;

  tesseract_environment::EnvironmentConstPtr collision_env_;
  std::vector<std::string> active_link_names_;
  std::vector<std::string> joint_names_;
  tesseract_collision::DiscreteContactManagerPtr contact_manager_;
};

using TesseractCollisionF = TesseractCollision<float>;
using TesseractCollisionD = TesseractCollision<double>;

} // namespace descartes_light

#endif // DESCARTES_TESSERACT_TESSERACT_COLLISION_CHECKER_H
