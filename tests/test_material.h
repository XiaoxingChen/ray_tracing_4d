#if !defined(_TEST_MATERIAL_H_)
#define _TEST_MATERIAL_H_

#include "material.h"
using namespace rtc;

inline void testMaterial()
{
    auto m = Material::choose(Material::METAL);
    RigidBodyHitRecord hit(3);
    if((m->attenuation(hit) - Pixel({0.8, 0.8, 0.8})).norm() > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

#endif // _TEST_MATERIAL_H_
