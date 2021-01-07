#if !defined(_TEST_ROTATION_H_)
#define _TEST_ROTATION_H_

#include "rotation.h"
#include <iostream>

using namespace rtc;

inline void rotationTestCase1()
{
    Rotation r1 = Rotation::fromAxisAngle(Vec({0,0,1}), 0.4);
    Rotation r2 = Rotation::fromAxisAngle(Vec({0,0,1}), 0.2);
    Vec v({1,0,0});
    auto result = (r1 * r2).apply(v);
    Vec expect({0.82533561, 0.56464247, 0.});
    if((result - expect).norm() > eps())
    {
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

inline void rotationTestCase2()
{
    Vec u1({1, 0, 0.1});
    Vec u2({1, 1, 0.1});
    auto r = Rotation::fromMatrix(rtc::bivectorToRotationMatrix(u1, u2));

    Vec v({1,0,1});
    Vec v1(r.apply(v));

    Vec expect({-0.08369046, 1.09452736, 0.89163095});
    if((v1 - expect).norm() > 10*eps())
    {
        // std::cout << r.asMatrix().str() << std::endl;
        std::cout << v1.T().str() << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

inline void testRotation()
{
    rotationTestCase1();
    rotationTestCase2();
}

#endif // _TEST_ROTATION_H_
