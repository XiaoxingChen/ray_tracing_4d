#if !defined(_TEST_LINALG_H)
#define _TEST_LINALG_H

#include <iostream>
#include "linalg.h"

using namespace rtc;
float_t eps(1e-7);

void testLinearAlgebra()
{
    Mat m1({3,3},{1,1,1, 2,2,2, 3,3,3});
    Mat v1({3,1}, {1,1,1});
    Mat expected({3,1}, {3, 6, 9});

    if((m1.dot(v1) - expected).norm() > eps)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

#endif // _TEST_LINALG_H
