#ifndef _CAMERA_H_
#define _CAMERA_H_
// #include "vec3.h"
#include "ray.h"
#include "rotation.h"

namespace rtc{


class Camera
{
public:
    Camera(size_t dim=3):position_(dim), orientation_(Rotation::Identity(dim)), f_(Vec::ones(dim - 1) * 500.), c_(Vec::ones(dim - 1) * 300.)
    {}
    Camera(VecIn position=Vec(3), const Rotation& r=Rotation::Identity(3), VecIn focus=Vec({500, 500}), VecIn resolution=Vec({640, 480}))
        :position_(position), orientation_(r), f_(focus), c_(resolution * 0.5)
    {
        checkDimension();
    }

    void operator=(const Camera& rhs)
    {
        position_ = rhs.position_;
        orientation_ = rhs.orientation_;
        f_ = rhs.f_;
        c_ = rhs.c_;
    }

    Ray pixelRay(const std::vector<size_t>& pixel_coordinate) const
    {
        if(position_.size() > pixel_coordinate.size() + 1)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        Vec dir(position_.size());
        for(size_t i = 0; i < position_.size() - 1; i++)
        {
            dir(i) = (pixel_coordinate.at(i) - c_(i)) / f_(i);
        }
        dir(position_.size() - 1) = 1.;
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
    std::vector<size_t> resolution() const
    {
        std::vector<size_t> res;
        for(size_t i = 0; i < c_.size(); i++)
            res.push_back( static_cast<size_t>(2 * c_(i) + 0.5) );
        return res;
    }

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

}//rtc
#endif