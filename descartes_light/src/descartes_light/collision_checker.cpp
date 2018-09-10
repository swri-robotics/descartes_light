#include "descartes_light/collision_checker.h"

bool descartes_light::TesseractCollision::validate(const double* pos, std::size_t size)
{
  return false;
}

descartes_light::CollisionInterface* descartes_light::TesseractCollision::clone() const
{
  return nullptr;
}
