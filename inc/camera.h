#ifndef _CAMERA_H_
#define _CAMERA_H_
// #include "vec3.h"
#include "ray.h"

namespace rtc{
#if 0
class OrientationFixedCamera
{
    public:
        OrientationFixedCamera(VecIn position=V3(), VecIn focus=V3{500, 500, 0}, VecIn c=V3{320, 240, 0})
        :position_(position), f_(focus), c_(c) {}
        Ray pixelRay(int u, int v) const
        {
            auto dir = Vector3{(u - c_.x())/f_.x(),(v - c_.y())/f_.y(), 1.};
            return Ray(position_, dir);
        }
    private:
        Vector3 position_;
        Vector3 f_;
        Vector3 c_;
};
#endif

class Camera
{
public:
    Camera(size_t dim=3):position_(dim), f_(Vec::ones(dim) * 500.), c_(Vec::ones(dim) * 300.)
    {}
    Camera(VecIn position=Vec(3), VecIn focus=Vec({500, 500, 0}), VecIn c=Vec({320, 240, 0}))
        :position_(position), f_(focus), c_(c)
    {
        checkDimension(position_);
        checkDimension(f_);
        checkDimension(c_);
    }

    Ray pixelRay(const std::vector<size_t>& pixel_coordinate) const
    {
        if(pixel_coordinate.size() + 1 != position_.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        // auto dir = Vector3{(u - c_.x())/f_.x(),(v - c_.y())/f_.y(), 1.};
        Vec dir(position_.size());
        for(size_t i = 0; i < pixel_coordinate.size(); i++)
        {
            dir(i) = (pixel_coordinate.at(i) - c_(i)) / f_(i);
        }
        dir(pixel_coordinate.size()) = 1.;
        return Ray(position_, dir);
    }

private:
    template <typename T>
    const Camera& checkDimension(const T& v) const
    {
        if(position_.size() != v.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
    Vec position_;
    Vec f_;
    Vec c_;
};

using OrientationFixedCamera = Camera;
}//rtc
#endif