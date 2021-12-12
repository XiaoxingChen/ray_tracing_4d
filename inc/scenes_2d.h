#if !defined(_SCENES_2D_H_)
#define _SCENES_2D_H_

#include "material.h"
#include "rigid_body.h"
#include "hittable.h"
#include "gltf_utils.h"
#include "mxm/rigid_transform.h"

namespace rtc
{

namespace scene
{
inline HitManager<2> simple2D_001()
{
    HitManager<2> manager;
    size_t dim = 2;

    manager.addHittables(
        RigidBody<2>::choose(RigidBody<2>::SPHERE, Vec({6.5, 10}), Rotation<float, 2>::identity(), {1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody<2>::choose(RigidBody<2>::SPHERE, Vec({1.5,13}), Rotation<float, 2>::identity(), {1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    manager.addHittables(
        RigidBody<2>::choose(RigidBody<2>::SPHERE, Vec({4.5,15}), Rotation<float, 2>::identity(), {1}),
        Material::choose(Material::METAL));
    manager.addHittables(
    RigidBody<2>::choose(RigidBody<2>::SPHERE, Vec({-100, 30}), Rotation<float, 2>::identity(), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    return manager;
}
} // namespace scene

} // namespace rtc


#endif // _SCENES_2D_H_
