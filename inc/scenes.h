#if !defined(_SCENES_H_)
#define _SCENES_H_

#include "material.h"
#include "rigid_body.h"
#include "hittable.h"

namespace rtc
{
namespace scene
{

inline HitManager simple2D_001()
{
    HitManager manager;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 2, {6.5, 10, 1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 2, {1.5,13, 1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 2, {4.5,15, 1}),
        Material::choose(Material::METAL));
    manager.addHittables(
    RigidBody::choose(RigidBody::SPHERE, 2, {-100, 30, 100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    return manager;
}

inline HitManager simple3D_001()
{
    HitManager manager;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 3, {0, -.5, 5, 1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 3, {1.5,-1,7, 1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 3, {-1.5,-1,7, 1}),
        Material::choose(Material::METAL));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 3, {0.,-100,30, 100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, 3, {0.,0, 15, 2,2,2, 1,0, 0,1, 0,1, 0.9}),
        Material::choose(Material::METAL));

    return manager;
}

inline HitManager simple4D_001()
{
    HitManager manager;
    size_t dim = 4;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, dim, {5, 0, 0, 18,  3}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    // manager.addHittables(
    //     RigidBody::choose(RigidBody::SPHERE, dim, {1.5,-1,0, 17,  1}),
    //     Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    // manager.addHittables(
    //     RigidBody::choose(RigidBody::SPHERE, dim, {-1.5,-1,0, 17,  1}),
    //     Material::choose(Material::METAL));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, dim, {0.,-105,0, 30, 100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, dim, {-3,0,0,15, 2,2,2,0.0001, 1,0, 0,0, 0,0, 0,1, M_PI / 3}),
        Material::choose(Material::METAL));

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, dim, {-3,0,0,15, 1,1,1,0.0002, 1,0, 0,0, 0,0, 0,1, M_PI / 3}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.2, 0.6, 0.2})));

    return manager;
}


} // namespace scene
} // namespace rtc
#endif // _SCENES_H_
