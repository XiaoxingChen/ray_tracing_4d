#if !defined(_TEST_RIGID_TRANSFORM_H)
#define _TEST_RIGID_TRANSFORM_H

#include "rigid_transform.h"
using namespace rtc;

inline void testRigidTransform()
{
    size_t dim = 3;
    RigidTrans pose(Vec::zeros(dim), Rotation::Identity(dim));
    if((pose.asMatrix() - Mat::Identity(dim + 1)).norm() > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}


#endif // _TEST_RIGID_TRANSFORM_H
