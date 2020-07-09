#include "rotation.h"
#include "vec3.h"
#include <cmath>

namespace rtc
{
    using M3 = std::array<V3, 3>;
    Rotation::Rotation(const M3& R)
    {
        float_t rx = R.at(2).at(1) - R.at(1).at(2);
        float_t ry = R.at(0).at(2) - R.at(2).at(0);
        float_t rz = R.at(1).at(0) - R.at(0).at(1);

        float_t theta, s, c;
        s = std::sqrt((rx*rx + ry*ry + rz*rz)*0.25);
        c = (R.at(0).at(0) + R.at(1).at(1) + R.at(2).at(2) - 1)*0.5;
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
                t = (R.at(0).at(0) + 1)*0.5;
                r.at(0) = std::sqrt(std::max(t,(float_t)0));
                t = (R.at(1).at(1) + 1)*0.5;
                r.at(1) = std::sqrt(std::max(t,(float_t)0))*(R.at(0).at(1) < 0 ? -1. : 1.);
                t = (R.at(2).at(2) + 1)*0.5;
                r.at(2) = std::sqrt(std::max(t,(float_t)0))*(R.at(0).at(2) < 0 ? -1. : 1.);
                if( fabs(r.at(0)) < fabs(r.at(1)) && fabs(r.at(0)) < fabs(r.at(2)) && (R.at(1).at(2) > 0) != (r.at(1)*r.at(2) > 0) )
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

        M3 c1_rrt{
            c1 * V3{rx*rx, rx*ry, rx*rz},
            c1 * V3{rx*ry, ry*ry, ry*rz},
            c1 * V3{rx*rz, ry*rz, rz*rz}};

        M3 s_r_x{
            s * V3{0,  -rz,  ry},
            s * V3{rz,   0, -rx},
            s * V3{-ry, rx,   0}};

        return M3{
            V3{c, 0, 0} + c1_rrt[0] + s_r_x[0],
            V3{0, c, 0} + c1_rrt[1] + s_r_x[1],
            V3{0, 0, c} + c1_rrt[2] + s_r_x[2],
        };
    }
} // namespace rtc
