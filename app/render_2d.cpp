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
#include "scenes_2d.h"
#include "utils.h"

using namespace rtc;
using namespace mxm;

int main(int argc, char const *argv[])
{
    const size_t DIM = 2;
    size_t nx = 640;
    size_t height_repeat = 300;

    size_t sample_num = 100;
    size_t recursion_depth = 10;
    Camera<float, DIM> cam;
    cam.setOrientation(Rotation<float, DIM>::fromAngle(0.1)).setFocalLength({500}).setResolution({nx});
    // Camera<float, DIM> cam(
    //     Vec(dim), Rotation<float, DIM>::fromAngle(0.1),
    //     Vec(std::vector<FloatType>(1, 500)), Vec(std::vector<FloatType>(1, nx)));
    std::vector<Pixel> img;

    HitManager<DIM> manager = rtc::scene::simple2D_001();


    bool multi_process = 1;
    auto ppm_coord = PPMCoordinateSequence(nx, 1);
    if(multi_process)
    {
        int thread_num = std::thread::hardware_concurrency() * 0.9;
        ThreadPool pool(thread_num);
        std::vector< std::future<std::vector<Pixel>> > pool_results;

        int step_len = 10000;
        for(int i = 0; i < ppm_coord.size(); i += step_len)
        {
            pool_results.emplace_back(
                pool.enqueue(
                    threadFunc<DIM>, cam, &manager, sample_num, recursion_depth, ppm_coord.begin() + i,
                    i + step_len >= ppm_coord.size() ? ppm_coord.end() : ppm_coord.begin() + i + step_len)
                );
        }

        for(auto & vec: pool_results)
        {
            for(auto & px: vec.get())
                img.push_back(px);

            if(img.size() % 1000 == 0)
                std::cout << "process: " << 100. * img.size() / ppm_coord.size() << "%" << std::endl;
        }
    }
    else
    {
        img = threadFunc<DIM>(cam, &manager, sample_num, recursion_depth, ppm_coord.begin(), ppm_coord.end());
    }

    decltype(img) img0(img);

    for(int i = 1; i < height_repeat ; i++)
    {
        img.insert(img.end(), img0.begin(), img0.begin() + nx);
    }

    writeToPPM("image_2d.ppm", nx, height_repeat, img);
    return 0;
}
