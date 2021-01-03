#if !defined(_TEST_ROTATION_H_)
#define _TEST_ROTATION_H_

#include "rotation.h"
#include <iostream>

using namespace rtc;

inline void testRotation()
{
    Rotation r1(orthogonalComplement(Vec({0,0,1})), 0.4);
    Rotation r2(orthogonalComplement(Vec({0,0,1})), 0.2);
    Vec v({1,0,0});
    auto result = (r1 * r2).apply(v);
    Vec expect({0.82533561, 0.56464247, 0.});
    if((result - expect).norm() > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

#endif // _TEST_ROTATION_H_
