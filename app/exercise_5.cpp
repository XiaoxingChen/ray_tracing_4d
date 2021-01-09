#include "io.h"
#include <vector>
#include <iostream>
#include "camera.h"
#include "random_factory.h"
#include "ThreadPool.h"
#include "material.h"
#include "rigid_body.h"
#include "hittable.h"
#include "ray_tracer.h"
#include "scenes.h"
#include "utils.h"


using namespace rtc;


int main(int argc, char const *argv[])
{
    size_t nx = 640;
    size_t ny = 480;
    // size_t nx = 320;
    // size_t ny = 240;
    size_t sample_num = 5;
    size_t recursion_depth = 10;
    Camera cam(Vec(3), Rotation::fromAxisAngle(Vec({0,0,1}), 0), Vec({500, 500}), Vec({(FloatType)nx/2, (FloatType)ny/2}));
    std::vector<Pixel> img;

    auto manager = rtc::scene::simple3D_002();

    bool multi_process = 1;
    auto ppm_coord = PPMCoordinateSequence(nx, ny);
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
                    threadFunc, cam, manager, sample_num, recursion_depth, ppm_coord.begin() + i,
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
        img = threadFunc(cam, manager, sample_num, recursion_depth, ppm_coord.begin(), ppm_coord.end());
    }

    writeToPPM("exercise_5.ppm", nx, ny, img);
    return 0;
}
