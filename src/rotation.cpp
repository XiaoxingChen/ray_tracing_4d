#include "rotation.h"
#include "vec3.h"
#include <cmath>
#include <initializer_list>

namespace rtc
{
    using M3 = Mat3;

    Rotation::Rotation(const M3& R)
    {
        float_t rx = R(2, 1) - R(1, 2);
        float_t ry = R(0, 2) - R(2, 0);
        float_t rz = R(1, 0) - R(0, 1);

        float_t theta, s, c;
        s = std::sqrt((rx*rx + ry*ry + rz*rz)*0.25);
        c = (R(0, 0) + R(1, 1) + R(2, 2) - 1)*0.5;
        c = c > 1. ? 1. : c < -1. ? -1. : c;
        theta = acos(c);
        V3 r{rx, ry, rz};

        if( s < 1e-5 )
        {
            float_t t;

            if( c > 0 )
                r = V3();
            else
            {
                t = (R(0, 0) + 1)*0.5;
                r.at(0) = std::sqrt(std::max(t,(float_t)0));
                t = (R(1, 1) + 1)*0.5;
                r.at(1) = std::sqrt(std::max(t,(float_t)0))*(R(0, 1) < 0 ? -1. : 1.);
                t = (R(2, 2) + 1)*0.5;
                r.at(2) = std::sqrt(std::max(t,(float_t)0))*(R(0, 2) < 0 ? -1. : 1.);
                if( fabs(r.at(0)) < fabs(r.at(1)) && fabs(r.at(0)) < fabs(r.at(2)) && (R(1, 2) > 0) != (r.at(1)*r.at(2) > 0) )
                    r.at(2) = -r.at(2);
                theta /= r.norm();
                r *= theta;
            }
        }
        else
        {
            float_t vth = 1/(2*s);
            vth *= theta;
            r *= vth;
        }
        angle_ = r.norm();
        axis_ = UnitVector3(r);
    }

    M3 Rotation::rodrigues() const
    {
        float_t c = cos(angle_);
        float_t s = sin(angle_);
        float_t c1 = 1. - c;
        float_t itheta = angle_ ? 1./angle_ : 0.;
        const float_t & rx = axis_.x() * itheta;
        const float_t & ry = axis_.y() * itheta;
        const float_t & rz = axis_.z() * itheta;

        M3 rrt{
            V3{rx*rx, rx*ry, rx*rz},
            V3{rx*ry, ry*ry, ry*rz},
            V3{rx*rz, ry*rz, rz*rz}};

        M3 r_x{
            V3{0,  -rz,  ry},
            V3{rz,   0, -rx},
            V3{-ry, rx,   0}};

        return M3::Identity() * c + rrt * c1 + r_x * s;
    }

    Rotation Rotation::operator*(const ThisType& rhs) const
    {
        return Rotation(rodrigues().dot(rhs.rodrigues())); 
    }

    V3 Rotation::apply(const V3& vector)
    {
        return rodrigues().dot(vector);
    }
} // namespace rtc
