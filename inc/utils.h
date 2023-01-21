#if !defined(_UTILS_H_)
#define _UTILS_H_

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
#include "stb_image.h"
#include "stb_image_write.h"
using namespace mxm;
namespace rtc
{
template<size_t DIM>
std::vector<Pixel> threadFunc(
    const Camera<float, DIM>& cam,
    HitManagerPtr<DIM> p_manager,
    int sample_num,
    size_t recursion_depth,
    PixelCoordinates::const_iterator begin,
    PixelCoordinates::const_iterator end)
{
    std::vector<Pixel> ret;
    for(auto it = begin; it != end; it++)
    {
        auto & uv = *it;
        Pixel col;
        // auto r = cam.pixelRay(uv);
        // Ray<> r(cam.pose().translation(), cam.pixelDirection(Mat<size_t>({DIM,1}, {uv})))
        Ray<> r;
        r.origin() = cam.pose().translation();
        if(2 == DIM)
        {
            r.setDirection(cam.pixelDirection(Matrix<size_t>({DIM-1,1}, {uv.at(0)})));
        }
        else if(3 == DIM)
        {
            r.setDirection(cam.pixelDirection(Matrix<size_t>({DIM-1,1}, {uv.at(0), uv.at(1)}, COL)));
        }
        else
        {
            assert(false);
        }

        for (int s = 0; s < sample_num; s++)
        {
            std::vector<Ray<>> ray_record;
            Ray<> input_ray(r);
            // col += trace(manager, r, 0, &ray_record);
            col += trace(*p_manager, input_ray, recursion_depth, nullptr);
            if(ray_record.size() >= recursion_depth - 1)
            {
                for(auto & ray: ray_record)
                    std::cout << "ori: " << mxm::to_string(ray.origin().T()) << " dir: " << mxm::to_string(ray.direction().T()) << std::endl;
                // exit(0);
            }
        }
        col *= (1./float(sample_num));
        for(int i = 0; i < 3; i++) col(i) = sqrt(col(i));
        ret.push_back(col);
    }
    return ret;
}

template<size_t DIM>
class RenderSample
{
public:
    enum Mode{
        eNAIVE,
        eMULTITHREADING,
        eSINGLE_RAY
    };
    RenderSample(): test_ray_(DIM){}
    using ThisType = RenderSample<DIM>;
    ThisType& setCamera(const Camera<float, DIM>& cam) { cam_ = cam; return *this; }
    ThisType& setSampleNum(size_t n) { sample_num_ = n; return *this; }
    ThisType& setRecursionDepth(size_t n) { recursion_depth_ = n; return *this; }
    ThisType& setMode(Mode mode) { mode_ = mode; return *this; }
    ThisType& setOutputFilename(const std::string& filename) { output_filename_ = filename; return *this; }
    ThisType& setScene(HitManagerPtr<DIM> scene) { scene_ = scene; return *this; }
    // ThisType& setTargetPixel(const std::vector<size_t>& px) { target_pixel_ = px; return *this; }
    ThisType& setTestRay(const Ray<>& r) { test_ray_ = r; return *this; }
    ThisType& enableRayStack(bool enable) { enable_ray_stack_ = enable; return *this; }


    void run()
    {
        auto ppm_coord = (cam_.pose().dim() - 1) == 2 ?
            PPMCoordinateSequence(cam_.resolution(0), cam_.resolution(1)) :
            PPMCoordinateSequence(cam_.resolution(0), cam_.resolution(1), cam_.resolution(2));
        std::vector<Ray<>> ray_stack;
        std::vector<Ray<>>* p_ray_stack = enable_ray_stack_ ? &ray_stack : nullptr;
        std::vector<Pixel> img;
        if(!scene_)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        if(mode_ == eNAIVE)
        {
            img = threadFunc<DIM>(cam_, scene_, sample_num_, recursion_depth_, ppm_coord.begin(), ppm_coord.end());
        }
        else if(mode_ == eMULTITHREADING)
        {
            int thread_num = std::thread::hardware_concurrency() - 1;
            ThreadPool pool(thread_num);
            std::vector< std::future<std::vector<Pixel>> > pool_results;

            int step_len = 1000;
            for(int i = 0; i < ppm_coord.size(); i += step_len)
            {
                pool_results.emplace_back(
                    pool.enqueue(
                        threadFunc<DIM>, cam_, scene_, sample_num_, recursion_depth_, ppm_coord.begin() + i,
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
        else if(mode_ == eSINGLE_RAY)
        {
            // auto ray = cam_.pixelRay(target_pixel_);
            Ray<> ray(test_ray_);
            auto px = trace(*scene_, ray, recursion_depth_, p_ray_stack);
            for(auto & r: ray_stack)
            {
                std::cout << "o: " << mxm::to_string( r.origin().T()) << "d: " << mxm::to_string( r.direction().T()) << std::endl;
            }
        }

        if(mode_ != eSINGLE_RAY)
        {
            // writeToPPM(output_filename_, resolution.at(0), resolution.at(1), img);
            size_t stride =  cam_.resolution(0) * cam_.resolution(1);
            std::vector<uint8_t> img_data(stride * img.at(0).size());
            for(size_t i = 0; i < img.size(); i+= stride)
            {
                std::string output_name = std::string("build/") + output_filename_ + "_" + std::to_string(i / stride) + ".png";
                for(size_t i = 0; i < stride; i++)
                {
                    img_data.at(img.at(0).size()* i + 0) = img.at(i).rU8();
                    img_data.at(img.at(0).size()* i + 1) = img.at(i).gU8();
                    img_data.at(img.at(0).size()* i + 2) = img.at(i).bU8();
                }
                stbi_write_png(output_name.c_str(), cam_.resolution(0), cam_.resolution(1), img.at(i).size(), img_data.data(), cam_.resolution(0) * img.at(i).size());
                // writeToPPM(output_name, cam_.resolution(0), cam_.resolution(1), img.cbegin() + i, img.cbegin() + i + stride);
            }
        }
    }

private:
    Camera<float,DIM> cam_;
    size_t sample_num_;
    size_t recursion_depth_;
    Mode mode_;
    bool enable_ray_stack_;
    // std::vector<size_t> target_pixel_;
    Ray<> test_ray_;
    HitManagerPtr<DIM> scene_;
    std::string output_filename_;

};

} // namespace rtc


#endif // _UTILS_H_
