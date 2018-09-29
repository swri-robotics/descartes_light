#ifndef DESCARTES_LIGHT_COLLISION_CHECKER_H
#define DESCARTES_LIGHT_COLLISION_CHECKER_H

#include <memory>
#include <tesseract_core/basic_env.h>

namespace descartes_light
{

class CollisionInterface
{
public:
  virtual ~CollisionInterface() {}

  virtual bool validate(const double* pos, std::size_t size) = 0;

  /** You assume ownership of return value */
  virtual CollisionInterface* clone() const = 0;
};

class TesseractCollision : public CollisionInterface
{
public:
  TesseractCollision(tesseract::BasicEnvConstPtr collision_env, const std::string& group_name);

  bool validate(const double* pos, std::size_t size) override;

  TesseractCollision* clone() const override;

  tesseract::BasicEnvConstPtr& environment() { return collision_env_; }
  const tesseract::BasicEnvConstPtr& environment() const { return collision_env_; }

private:
  bool isContactAllowed(const std::string& a, const std::string& b) const;

  tesseract::BasicEnvConstPtr collision_env_;
  tesseract::BasicKinConstPtr kin_group_;
  tesseract::DiscreteContactManagerBasePtr contact_manager_;
};

using CollisionInterfacePtr = std::shared_ptr<CollisionInterface>;

}

#endif
