#if !defined(_RAY_TRACER_H_)
#define _RAY_TRACER_H_

#include "hittable.h"
#include "material.h"
#include "rigid_body.h"
#include <iostream>

namespace rtc
{

inline Pixel skyBox(const Ray& ray)
{
    FloatType t = 0.5 * (ray.direction()(1) + 1.);
    return static_cast<Vec>(
        (1- t) * Pixel({1,1,1}) + t * Pixel({0.5, 0.7, 1.}));
}

inline Pixel trace(const HitManager& manager, const Ray& ray, int depth, std::vector<Ray>* ray_record=nullptr)
{
    auto p_record = manager.hit(ray);

    if(ray_record) ray_record->push_back(ray);

    if (nullptr != p_record)
    {
        // std::cout << "hit!" << std::endl;
        if (depth > 0)
        {

            return static_cast<Vec>(
                p_record->attenuation * trace(manager, p_record->scattered, depth - 1, ray_record));

        }
        else {
            std::cout << ".";
            return Pixel({1, 0, 0});
        }
    }
    return skyBox(ray);
}

} // namespace rtc
#endif // _RAY_TRACER_H_
