#include "io.h"
#include <vector>
#include <iostream>
#include "mxm/model_camera.h"
#include "mxm/random.h"
#include "ThreadPool.h"
#include "material.h"
#include "rigid_body.h"
#include "hittable.h"
#include "ray_tracer.h"
// #include "scenes.h"
#include "utils.h"

using namespace rtc;


int main(int argc, char const *argv[])
{
    const size_t DIM = 3;
    HitManager<DIM> manager;

    manager.addHittables(
        RigidBody<DIM>::choose(RigidBody<DIM>::RECTANGLE, 3, {0.,0, 15, 2,2,2, 1,0, 0,1, 0,0, 0}),
        Material::choose(Material::METAL));


    // size_t dim = 3;
    size_t recursion_depth = 20;

    // std::vector<uint32_t> dir_raw{0xbcd280fa,0x3e19d468,0x3f7d0268};
    // std::vector<uint32_t> ori_raw{0xbead0e58,0x3ffced93,0x41500002};

    std::vector<uint32_t> ori_raw{0,0,0};
    std::vector<uint32_t> dir_raw{0xbcd280fb,0x3e19d468,0x3f7d0269};


    std::vector<FloatType> dir_f;
    std::vector<FloatType> ori_f;
    for(size_t i = 0; i < DIM ; i++)
    {
        float dir_val = *(float*)&dir_raw[i];
        float ori_val = *(float*)&ori_raw[i];
        dir_f.push_back(dir_val);
        ori_f.push_back(ori_val);
    }
    Ray ray(ori_f, dir_f);
    std::cout << "ray in: "  << mxm::to_string(ray.direction().T()) << std::endl << std::flush;
    auto px = trace(manager, ray, recursion_depth);
    std::cout << "done" << std::endl;
    return 0;
}
