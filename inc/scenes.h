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
    size_t dim = 2;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({6.5, 10}), Rotation::Identity(dim), {1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({1.5,13}), Rotation::Identity(dim), {1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({4.5,15}), Rotation::Identity(dim), {1}),
        Material::choose(Material::METAL));
    manager.addHittables(
    RigidBody::choose(RigidBody::SPHERE, Vec({-100, 30}), Rotation::Identity(dim), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    return manager;
}

inline HitManager simple3D_001()
{
    HitManager manager;

    size_t dim = 3;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({0, -.5, 5}), Rotation::Identity(dim), {1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({1.5,-1,7}), Rotation::Identity(dim), {1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({-1.5,-1,7}), Rotation::Identity(dim), {1}),
        Material::choose(Material::METAL));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({0.,-100,30}), Rotation::Identity(dim), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, Vec({0.,0, 15}), Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,1}), 0.9), {2,2,2}),
        Material::choose(Material::METAL));

    return manager;
}

inline HitManager simple4D_001()
{
    HitManager manager;
    size_t dim = 4;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({5, 0, 0, 18}), Rotation::Identity(dim), {3}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({0.,-105,0, 30}), Rotation::Identity(dim), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    Rotation rec_ori(Rotation::fromPlaneAngle(Vec({1,0,0,0}), Vec({0,0,0,1}), M_PI / 3)); 
    Vec rec_pos({-3, 0, 0, 15});

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, rec_pos, rec_ori, {2,2,2,0.0001}),
        Material::choose(Material::METAL));

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, rec_pos, rec_ori, {1,1,1,0.0002}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.2, 0.6, 0.2})));

    return manager;
}


} // namespace scene
} // namespace rtc
#endif // _SCENES_H_
