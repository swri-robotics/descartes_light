#ifndef DESCARTES_LIGHT_COLLISION_CHECKER_H
#define DESCARTES_LIGHT_COLLISION_CHECKER_H

#include <memory>

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

  bool validate(const double* pos, std::size_t size) override;

  CollisionInterface* clone() const override;
};

}

#endif
