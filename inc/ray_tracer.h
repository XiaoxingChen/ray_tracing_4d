#if !defined(_RAY_TRACER_H_)
#define _RAY_TRACER_H_

#include "hittable.h"
#include "material.h"
#include "rigid_body.h"
#include <iostream>
#include "interpolation.h"

namespace rtc
{

inline Pixel skyBox(const Ray& ray)
{
    if(0)
    {
        FloatType t = 0.5 * (ray.direction()(1) + 1.);
        return (1- t) * Pixel({1,1,1}) + t * Pixel({0.5, 0.7, 1.});
    }

    static auto img = loadImage("assets/sky_box/star_night.jpeg");
    FloatType x = 0.5 * (ray.direction()(1) + 1.) * img.shape(0);
    FloatType y = 0.5 * (ray.direction()(0) + 1.) * img.shape(1);
    size_t i = static_cast<size_t>(x);
    size_t j = static_cast<size_t>(y);
    if(i >= img.shape(0) - 1 || j >= img.shape(1) - 1)
    {
        return img(i,j);
    }
    return interp::bilinearUnitSquare(
        Vec({x - i, y - j}) , img(Block({i, i+2},{j, j+2})));
}

inline Pixel trace(const HitManager& manager, Ray& ray, int depth, std::vector<Ray>* ray_record=nullptr)
{
    auto p_record = manager.hit(ray);

    if(ray_record) ray_record->push_back(ray);

    if (nullptr != p_record)
    {
        // std::cout << "hit!" << std::endl;
        if (depth > 0)
        {
            auto new_ray = p_record->scattered;
            // std::cout << "hit!" << std::endl;
            return p_record->attenuation * trace(manager, new_ray, depth - 1, ray_record);

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
