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
    if (nullptr != p_record)
    {
        // std::cout << "hit!" << std::endl;
        if (depth < 50)
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
    return static_cast<Vec>(
        (1- t) * Pixel({1,1,1}) + t * Pixel({0.5, 0.7, 1.}));
}

std::vector<Pixel> threadFunc(
    const OrientationFixedCamera& cam,
    const HitManager& manager,
    int sample_num,
    PixelCoordinates::const_iterator begin,
    PixelCoordinates::const_iterator end)
{
    std::vector<Pixel> ret;
    for(auto it = begin; it != end; it++)
    {
        auto & uv = *it;
        Pixel col;
        auto r = cam.pixelRay({uv.at(0)});
        for (int s = 0; s < sample_num; s++)
        {
            col += color(manager, r, 0);
        }
        col *= (1./float(sample_num));
        for(int i = 0; i < 3; i++) col(i) = sqrt(col(i));
        ret.push_back(col);
    }
    return ret;
}

int main(int argc, char const *argv[])
{
    size_t nx = 640;
    size_t height_repeat = 50;
    size_t dim = 2;
    size_t sample_num = 10;
    OrientationFixedCamera cam(Vec(dim), Rotation(0.1), Vec(std::vector<FloatType>(1, 500)), Vec(std::vector<FloatType>(1, nx/2.)));
    std::vector<Pixel> img;

    HitManager manager;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 2, {0, 10, 1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 2, {1.5,13, 1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, 2, {-1.5,15, 1}),
        Material::choose(Material::METAL));
    manager.addHittables(
    RigidBody::choose(RigidBody::SPHERE, 2, {-100, 30, 100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    bool multi_process = 1;
    auto ppm_coord = PPMCoordinateSequence(nx, height_repeat);
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
                    threadFunc, cam, manager, sample_num, ppm_coord.begin() + i,
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
        img = threadFunc(cam, manager, sample_num, ppm_coord.begin(), ppm_coord.end());
    }

    writeToPPM("exercise_5_2d.ppm", nx, height_repeat, img);
    return 0;
}
