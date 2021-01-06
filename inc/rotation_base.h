#if !defined(_ROTATION_BASE_H_)
#define _ROTATION_BASE_H_

#include "linalg.h"

namespace rtc
{

inline Mat rodrigues2D(FloatType angle)
{
    FloatType c = cos(angle);
    FloatType s = sin(angle);
    return Mat({2,2}, {c, s, -s, c});
}

inline Mat rodrigues3D(UnitVecIn axis, FloatType angle)
{
    FloatType c = cos(angle);
    FloatType s = sin(angle);
    FloatType c1 = 1. - c;
    FloatType itheta = angle ? (1./angle) : 0.;
    const FloatType & rx = axis(0);
    const FloatType & ry = axis(1);
    const FloatType & rz = axis(2);

    Mat rrt({3,3},{
        rx*rx, rx*ry, rx*rz,
        rx*ry, ry*ry, ry*rz,
        rx*rz, ry*rz, rz*rz});

    Mat r_x({3,3},{
        0,  -rz,  ry,
        rz,   0, -rx,
        -ry, rx,   0});

    return Mat::Identity(3) * c + rrt * c1 + r_x * s;
}

inline void matrixToAxisAngle3D(const Mat& R, UnitVec& axis, FloatType& angle)
{
    FloatType rx = R(2, 1) - R(1, 2);
    FloatType ry = R(0, 2) - R(2, 0);
    FloatType rz = R(1, 0) - R(0, 1);

    FloatType theta, s, c;
    s = std::sqrt((rx*rx + ry*ry + rz*rz)*0.25);
    c = (R(0, 0) + R(1, 1) + R(2, 2) - 1)*0.5;
    c = c > 1. ? 1. : c < -1. ? -1. : c;
    theta = acos(c);
    Vec r({rx, ry, rz});

    if( s < 1e-5 )
    {
        FloatType t;

        if( c > 0 )
            r = Vec(3);
        else
        {
            t = (R(0, 0) + 1)*0.5;
            r(0) = std::sqrt(std::max(t,(FloatType)0));
            t = (R(1, 1) + 1)*0.5;
            r(1) = std::sqrt(std::max(t,(FloatType)0))*(R(0, 1) < 0 ? -1. : 1.);
            t = (R(2, 2) + 1)*0.5;
            r(2) = std::sqrt(std::max(t,(FloatType)0))*(R(0, 2) < 0 ? -1. : 1.);
            if( fabs(r(0)) < fabs(r(1)) && fabs(r(0)) < fabs(r(2)) && (R(1, 2) > 0) != (r(1)*r(2) > 0) )
                r(2) = -r(2);
            theta /= r.norm();
            r *= theta;
        }
    }
    else
    {
        FloatType vth = 1/(2*s);
        vth *= theta;
        r *= vth;
    }
    angle = r.norm();
    axis = UnitVec(r);
}

inline Mat reflection(UnitVecIn u)
{
    return Mat::Identity(u.size()) - u.matmul(u.T());
}

inline std::array<UnitVec, 2> planeAngleToBivector(UnitVecIn u, UnitVecIn v, FloatType angle)
{
    UnitVec v_perpend(v - u.dot(v));
    UnitVec v_new(cos(angle) * u + sin(angle) * v_perpend);
    return {u,v_new};
}

inline Mat bivectorToRotationMatrix(UnitVecIn u, UnitVecIn v)
{
    return reflection(v).matmul(reflection(u));
}
} // namespace rtc


#endif // _ROTATION_BASE_H_