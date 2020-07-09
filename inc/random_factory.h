#if !defined(__RANDOM_FACTORY_H__)
#define __RANDOM_FACTORY_H__
#include "vec3.h"
#include <functional>
#include <random>
namespace rtc
{
namespace random
{

inline float_t UnitFloat()
{
    static std::uniform_real_distribution<double> distribution(0.0, 1.0);
    static std::mt19937 generator;
    static std::function<double()> rand_generator =
    std::bind(distribution, generator);
    return rand_generator();
}

inline Vector3 unitSphere() 
{
    Vector3 p;
    do
    {
        p = 2.0 * Vector3{UnitFloat(), UnitFloat(), UnitFloat()} - Vector3{1,1,1};
    } while (p.norm() > 1);
    return p;
}

} // namespace random_factory
} // namespace rtc
#endif // __RANDOM_FACTORY_H__
