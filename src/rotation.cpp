#include "rotation.h"
// #include "vec3.h"
#include "linalg.h"
#include <cmath>

namespace rtc
{
    using M3 = Mat;
#if 1
    Rotation Rotation::fromMatrix(const Mat& R)
    {
        float_t rx = R(2, 1) - R(1, 2);
        float_t ry = R(0, 2) - R(2, 0);
        float_t rz = R(1, 0) - R(0, 1);

        float_t theta, s, c;
        s = std::sqrt((rx*rx + ry*ry + rz*rz)*0.25);
        c = (R(0, 0) + R(1, 1) + R(2, 2) - 1)*0.5;
        c = c > 1. ? 1. : c < -1. ? -1. : c;
        theta = acos(c);
        Vec r({rx, ry, rz});

        if( s < 1e-5 )
        {
            float_t t;

            if( c > 0 )
                r = Vec(3);
            else
            {
                t = (R(0, 0) + 1)*0.5;
                r(0) = std::sqrt(std::max(t,(float_t)0));
                t = (R(1, 1) + 1)*0.5;
                r(1) = std::sqrt(std::max(t,(float_t)0))*(R(0, 1) < 0 ? -1. : 1.);
                t = (R(2, 2) + 1)*0.5;
                r(2) = std::sqrt(std::max(t,(float_t)0))*(R(0, 2) < 0 ? -1. : 1.);
                if( fabs(r(0)) < fabs(r(1)) && fabs(r(0)) < fabs(r(2)) && (R(1, 2) > 0) != (r(1)*r(2) > 0) )
                    r(2) = -r(2);
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
        // angle_ = r.norm();
        // axis_ = UnitVector3(r);
        Mat plane = orthogonalComplement(r);
        return Rotation(plane, r.norm());
    }
#endif
    Mat Rotation::asMatrix() const
    {
        return rodrigues();
    }

    M3 Rotation::rodrigues() const
    {
        Vec axis(orthogonalComplement(plane_));
        axis.normalize();
        float_t c = cos(angle_);
        float_t s = sin(angle_);
        float_t c1 = 1. - c;
        float_t itheta = angle_ ? (1./angle_) : 0.;
        const float_t & rx = axis(0);
        const float_t & ry = axis(1);
        const float_t & rz = axis(2);

        M3 rrt({3,3},{
            rx*rx, rx*ry, rx*rz,
            rx*ry, ry*ry, ry*rz,
            rx*rz, ry*rz, rz*rz});

        M3 r_x({3,3},{
            0,  -rz,  ry,
            rz,   0, -rx,
            -ry, rx,   0});

        return M3::Identity(3) * c + rrt * c1 + r_x * s;
    }

    Rotation Rotation::operator*(const ThisType& rhs) const
    {
        return fromMatrix(rodrigues().dot(rhs.rodrigues()));
    }

    Mat Rotation::apply(const Mat& vectors) const
    {
        return rodrigues().matmul(vectors);
        // return rodrigues();
    }
} // namespace rtc
