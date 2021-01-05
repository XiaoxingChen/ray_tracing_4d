#include "io.h"
#include <vector>
#include <iostream>
#include "camera.h"
#include "random_factory.h"
#include "ThreadPool.h"
#include "material.h"
#include "rigid_body.h"
#include "hittable.h"

using namespace rtc;

Pixel color(const HitManager& manager, const Ray& ray, int depth)
{
    auto p_record = manager.hit(ray);
    std::cout << "ori: " << ray.origin().T().str() << ", dir: " << ray.direction().T().str() << std::endl;
    if (nullptr != p_record)
    {
        // std::cout << "hit!" << std::endl;
        if (depth < 10)
        {

            return static_cast<Vec>(
                p_record->attenuation * color(manager, p_record->scattered, depth + 1));

        }
        else {
            std::cout << ".";
            return Pixel({1, 0, 0});
        }
    }
    FloatType t = 0.5 * (ray.direction()(1) + 1.);
    std::cout << "#" << std::flush;
    return static_cast<Vec>(
        (1- t) * Pixel({1,1,1}) + t * Pixel({0.5, 0.7, 1.}));
}

int main(int argc, char const *argv[])
{
    HitManager manager;

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, 3, {0.,0, 15, 2,2,2, 1,0, 0,1, 0,0, 0}),
        Material::choose(Material::METAL));


    size_t dim = 3;

    // std::vector<uint32_t> dir_raw{0xbcd280fa,0x3e19d468,0x3f7d0268};
    // std::vector<uint32_t> ori_raw{0xbead0e58,0x3ffced93,0x41500002};

    std::vector<uint32_t> ori_raw{0,0,0};
    std::vector<uint32_t> dir_raw{0xbcd280fb,0x3e19d468,0x3f7d0269};


    std::vector<FloatType> dir_f;
    std::vector<FloatType> ori_f;
    for(size_t i = 0; i < dim ; i++)
    {
        float dir_val = *(float*)&dir_raw[i];
        float ori_val = *(float*)&ori_raw[i];
        dir_f.push_back(dir_val);
        ori_f.push_back(ori_val);
    }
    Ray ray(ori_f, dir_f);
    std::cout << "ray in: "  << ray.direction().T().str() << std::endl << std::flush;
    auto px = color(manager, ray, 0);
    std::cout << "done" << std::endl;
    return 0;
}
