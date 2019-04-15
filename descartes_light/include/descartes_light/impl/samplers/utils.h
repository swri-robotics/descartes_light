#ifndef DESCARTES_LIGHT_SAMPLERS_UTILS_H
#define DESCARTES_LIGHT_SAMPLERS_UTILS_H

#include <memory>
#include <functional>

namespace descartes_light
{

template<typename FloatType>
using IsWithinLimitsFn = std::function<bool(const FloatType* pose)>;

}
#endif // DESCARTES_LIGHT_SAMPLERS_UTILS_H
