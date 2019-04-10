#ifndef DESCARTES_LIGHT_SAMPLERS_UTILS_H
#define DESCARTES_LIGHT_SAMPLERS_UTILS_H

#include <memory>
#include <functional>

namespace descartes_light
{

typedef std::function<bool(const double* pose)> IsWithinLimitsFn;

}
#endif // DESCARTES_LIGHT_SAMPLERS_UTILS_H
