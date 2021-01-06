#ifndef _CAMERA_H_
#define _CAMERA_H_
// #include "vec3.h"
#include "ray.h"
#include "rotation.h"

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
    Camera(size_t dim=3):position_(dim), orientation_(Rotation::Identity(dim)), f_(Vec::ones(dim) * 500.), c_(Vec::ones(dim) * 300.)
    {}
    Camera(VecIn position=Vec(3), const Rotation& r=Rotation::Identity(3), VecIn focus=Vec({500, 500}), VecIn c=Vec({320, 240}))
        :position_(position), orientation_(r), f_(focus), c_(c)
    {
        checkDimension();
    }

    Ray pixelRay(const std::vector<size_t>& pixel_coordinate) const
    {
        if(position_.size() != pixel_coordinate.size() + 1)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        Vec dir(position_.size());
        for(size_t i = 0; i < pixel_coordinate.size(); i++)
        {
            dir(i) = (pixel_coordinate.at(i) - c_(i)) / f_(i);
        }
        dir(pixel_coordinate.size()) = 1.;
        dir = orientation_.apply(dir);
        return Ray(position_, dir);
    }

    const Vec& position() const { return position_; }
    Camera& setPosition(const Vec& pos)
    {
        position_ = pos;
        checkDimension();
        return *this;
    }
    Rotation& orientation() { return orientation_; }

private:
    const Camera& checkDimension() const
    {
        if(position_.size() != orientation_.dim())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        if(position_.size() != f_.size() + 1)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        if(position_.size() != c_.size() + 1)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        return *this;
    }
    Vec position_;
    Rotation orientation_;
    Vec f_;
    Vec c_;
};

using OrientationFixedCamera = Camera;
}//rtc
#endif